#!/usr/bin/env python3
import asyncio
import websockets
import json
import serial
import os
import pty
import time
import subprocess
import signal
import sys
import tty
import select
import threading
import queue
import ssl
import traceback
import fcntl

REAL_SERIAL_PORT = "/dev/serial_twistctl"
VIRTUAL_SERIAL_TX = "/tmp/virtual_twist_tx"
VIRTUAL_SERIAL_RX = "/tmp/virtual_twist_rx"
BAUDRATE = 115200

connected_clients = set()
ps2_connected = False
last_phone_cmd_time = 0
phone_vcx = 0.0
phone_wc = 0.0
serial_cmd_queue = queue.Queue()
active_mode = None
active_subscriptions = {}
phone_motor_enable = False

PHONE_ACTIVE_TIMEOUT_S = 0.5   # joystick considered idle after this long without a message
PHONE_SEND_PERIOD_S = 0.05     # 20 Hz while the phone joystick is active
IDLE_SEND_PERIOD_S = 0.1       # 10 Hz heartbeat (STM32 watchdog times out after 500 ms)
STATUS_PRINT_PERIOD_S = 1.0    # diagnostic summary rate
ZERO_DISABLE_CMD = b"vcx=0.000,wc=0.000,en=0\n"
ZERO_ENABLE_CMD = b"vcx=0.000,wc=0.000,en=1\n"

# Telemetry field 17 (index 16) reported by the STM32 firmware (see Serial_Output in main.c):
#   0 = serial control active, 1 = PS2 joystick mode, 2 = motors disabled / braking
CTRL_MODE_NAMES = {0: "serial", 1: "joystick", 2: "disabled"}

def create_pty(path):
    master, slave = pty.openpty()
    tty.setraw(slave)
    # Set master to non-blocking to prevent thread deadlocks
    flags = fcntl.fcntl(master, fcntl.F_GETFL)
    fcntl.fcntl(master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    if os.path.exists(path):
        try:
            os.remove(path)
        except OSError:
            pass
    os.symlink(os.ttyname(slave), path)
    return master, slave

def open_real_serial():
    """Block until the real STM32 serial port can be opened."""
    while True:
        try:
            ser = serial.Serial(REAL_SERIAL_PORT, BAUDRATE, timeout=0.01)
            print(f"Opened real serial port {REAL_SERIAL_PORT} @ {BAUDRATE}")
            return ser
        except Exception as e:
            print(f"Waiting for real serial port {REAL_SERIAL_PORT}: {e}")
            time.sleep(1.0)

def read_pty(fd):
    """Non-blocking read from a PTY master. Returns b'' if nothing/EIO."""
    try:
        data = os.read(fd, 1024)
    except OSError:
        # BlockingIOError (nothing to read) or EIO (peer closed); avoid busy-looping
        time.sleep(0.01)
        return b""
    if not data:
        time.sleep(0.01)
    return data

def serial_proxy_thread(loop):
    global ps2_connected
    try:
        master_tx, slave_tx = create_pty(VIRTUAL_SERIAL_TX)
        master_rx, slave_rx = create_pty(VIRTUAL_SERIAL_RX)
    except Exception as e:
        print(f"Failed to create PTYs: {e}")
        return

    ser = open_real_serial()

    serial_buffer = b""
    last_phone_send = 0.0
    last_idle_send = 0.0
    last_ros_cmd_time = 0.0

    # Diagnostics (summarised once per STATUS_PRINT_PERIOD_S)
    last_status_print = time.time()
    last_misc_print = 0.0
    tx_count = 0
    ros_fwd_count = 0
    echo_count = 0
    telemetry_count = 0
    last_tx_cmd = ""
    last_echo = ""
    last_fields = 0
    last_ctrl_mode = None
    last_ps2_key = None
    last_ps2_flag = None

    def send(cmd_bytes):
        # SerialException propagates to the outer handler, which reopens the port
        nonlocal tx_count, last_tx_cmd
        ser.write(cmd_bytes)
        tx_count += 1
        last_tx_cmd = cmd_bytes.decode("ascii", errors="replace").strip()

    while True:
        try:
            r, _, _ = select.select([master_tx, ser.fileno()], [], [], 0.05)
            now = time.time()
            phone_active = (now - last_phone_cmd_time <= PHONE_ACTIVE_TIMEOUT_S)

            # 1. Read from REAL serial, forward to RX PTY (for ROS) and parse telemetry
            if ser.fileno() in r:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    try:
                        os.write(master_rx, data)
                    except OSError:
                        # BlockingIOError when no ROS reader drains the PTY, EIO if it closed.
                        # Drop the telemetry for ROS rather than blocking the control loop.
                        pass

                    serial_buffer += data
                    if len(serial_buffer) > 8192 and b"\n" not in serial_buffer:
                        serial_buffer = b""  # garbage guard
                    while b"\n" in serial_buffer:
                        line, serial_buffer = serial_buffer.split(b"\n", 1)
                        text = line.decode("ascii", errors="ignore").strip()
                        if not text:
                            continue
                        parts = text.split(",")
                        if len(parts) >= 16 and not text.startswith("Received"):
                            # Telemetry CSV: 16 floats + ctrl_mode + ps2_key [+ ps2_connected]
                            telemetry_count += 1
                            last_fields = len(parts)
                            try:
                                if len(parts) >= 17:
                                    last_ctrl_mode = int(parts[16])
                                if len(parts) >= 18:
                                    last_ps2_key = int(parts[17])
                                if len(parts) >= 19:
                                    last_ps2_flag = int(parts[18])
                                    new_ps2_status = (last_ps2_flag == 1)
                                    if new_ps2_status != ps2_connected:
                                        ps2_connected = new_ps2_status
                                        print(f"[STM32] PS2 connected: {ps2_connected}")
                                        asyncio.run_coroutine_threadsafe(broadcast_state(), loop)
                            except ValueError:
                                pass  # corrupted line (TX DMA collision on the STM32)
                        elif text.startswith("Received"):
                            # Firmware echoes every UART frame it receives: "Received: <frame>"
                            echo_count += 1
                            last_echo = text
                        elif now - last_misc_print > 1.0:
                            print(f"[STM32] non-telemetry line: {text[:120]!r}")
                            last_misc_print = now

            # 2. Explicit one-shot commands from the WebSocket handler (enable/disable)
            sent_explicit = False
            while True:
                try:
                    cmd_bytes = serial_cmd_queue.get_nowait()
                except queue.Empty:
                    break
                print(f"Explicit motor command -> serial: {cmd_bytes.strip()}")
                send(cmd_bytes)
                sent_explicit = True
            if sent_explicit:
                # The STM32 frames commands by UART idle gaps; a second command written
                # back-to-back would merge into the same frame and be ignored. Delay the
                # periodic sends to the next loop pass.
                last_phone_send = now
                last_idle_send = now

            # 3. Control logic & multiplexing
            if len(connected_clients) == 0 and not ps2_connected:
                # No phone and no PS2: force zero speed + disable, discard ROS commands
                if now - last_idle_send > IDLE_SEND_PERIOD_S:
                    send(ZERO_DISABLE_CMD)
                    last_idle_send = now
                if master_tx in r:
                    read_pty(master_tx)
            elif phone_active:
                # Phone joystick active -> override ROS
                if now - last_phone_send > PHONE_SEND_PERIOD_S:
                    en_val = 1 if phone_motor_enable else 0
                    cmd = f"vcx={phone_vcx:.3f},wc={phone_wc:.3f},en={en_val}\n"
                    send(cmd.encode())
                    last_phone_send = now
                if master_tx in r:
                    read_pty(master_tx)  # discard ROS while the phone has control
            else:
                # Phone idle -> let ROS (TX PTY) through
                ros_sent = False
                if master_tx in r:
                    data = read_pty(master_tx)
                    if data:
                        ser.write(data)
                        ros_fwd_count += 1
                        last_ros_cmd_time = now
                        ros_sent = True

                # No ROS traffic: heartbeat so the STM32 watchdog (500 ms) stays fed
                if not ros_sent and (now - last_ros_cmd_time > PHONE_ACTIVE_TIMEOUT_S):
                    if now - last_idle_send > IDLE_SEND_PERIOD_S:
                        en_val = 1 if phone_motor_enable else 0
                        send(f"vcx=0.000,wc=0.000,en={en_val}\n".encode())
                        last_idle_send = now

            # 4. Diagnostics summary
            if now - last_status_print >= STATUS_PRINT_PERIOD_S:
                if telemetry_count == 0:
                    stm = "NO TELEMETRY from STM32"
                else:
                    mode = CTRL_MODE_NAMES.get(last_ctrl_mode, str(last_ctrl_mode))
                    stm = (f"fields={last_fields} ctrl_mode={mode} "
                           f"ps2_key={last_ps2_key} ps2={last_ps2_flag}")
                    if last_fields < 19:
                        stm += " [OLD FIRMWARE? <19 fields: no en= / ps2_connected support]"
                print(
                    f"[STATUS] {stm} | tx={tx_count}/s last_tx='{last_tx_cmd}' "
                    f"ros_fwd={ros_fwd_count}/s | echoes={echo_count}/s last_echo='{last_echo[:60]}' | "
                    f"clients={len(connected_clients)} phone_active={phone_active} "
                    f"enable={phone_motor_enable} vcx={phone_vcx:.3f} wc={phone_wc:.3f}"
                )
                last_status_print = now
                tx_count = ros_fwd_count = echo_count = telemetry_count = 0

        except serial.SerialException as e:
            print(f"Serial port error: {e} -- reopening {REAL_SERIAL_PORT}")
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.0)
            ser = open_real_serial()
            serial_buffer = b""
        except Exception as e:
            # Never let one bad iteration kill the control thread
            print(f"ERROR in serial_proxy_thread loop: {e}")
            traceback.print_exc()
            time.sleep(0.1)

async def handle_websocket(websocket, path=None):
    global connected_clients, phone_vcx, phone_wc, last_phone_cmd_time, phone_motor_enable
    if len(connected_clients) >= 8:
        await websocket.close(1008, "Max connections reached")
        return
    
    connected_clients.add(websocket)
    await broadcast_state()
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                # Joystick messages arrive at high rate; they are summarised in the
                # serial thread's 1 Hz [STATUS] line instead of printed individually.
                if data.get("type") != "joystick":
                    print(f"Received WS: {data}")

                if data.get("type") == "joystick":
                    phone_vcx = float(data.get("linear", 0.0) or 0.0)
                    phone_wc = float(data.get("angular", 0.0) or 0.0)
                    last_phone_cmd_time = time.time()
                elif data.get("type") == "motor_state":
                    phone_motor_enable = bool(data.get("enable"))
                    print(f"Received motor_state: enable={phone_motor_enable}")
                    if phone_motor_enable:
                        serial_cmd_queue.put(ZERO_ENABLE_CMD)
                    else:
                        serial_cmd_queue.put(ZERO_DISABLE_CMD)
                elif data.get("type") == "greeting":
                    msg = data.get("message", "")
                    print(f"Received greeting from client: {msg}")
                elif data.get("type") == "launch":
                    mode = data.get("mode")
                    asyncio.create_task(run_make_command(f"launch-{mode}"))
                elif data.get("type") == "subscribe":
                    topic = data.get("topic")
                    if topic:
                        task = asyncio.create_task(run_ros2_topic_echo(websocket, topic))
                        active_subscriptions[websocket] = task
                elif data.get("type") == "unsubscribe":
                    if websocket in active_subscriptions:
                        active_subscriptions[websocket].cancel()
                        del active_subscriptions[websocket]
            except Exception as e:
                print(f"Error handling message: {message}, Error: {e}")
                traceback.print_exc()
    except Exception as e:
        print(f"Websocket connection error: {e}")
        traceback.print_exc()
    finally:
        if websocket in active_subscriptions:
            active_subscriptions[websocket].cancel()
            del active_subscriptions[websocket]
        connected_clients.discard(websocket)
        if len(connected_clients) == 0:
            # Safety: last phone left -> forget its enable state and speeds
            phone_motor_enable = False
            phone_vcx = 0.0
            phone_wc = 0.0
            last_phone_cmd_time = 0
            serial_cmd_queue.put(ZERO_DISABLE_CMD)
            print("Last client disconnected: motors disabled, phone state reset")
        await broadcast_state()

async def broadcast_state():
    state = json.dumps({
        "type": "state",
        "connections": len(connected_clients),
        "ps2_connected": ps2_connected,
        "active_mode": active_mode
    })
    if connected_clients:
        await asyncio.gather(*(ws.send(state) for ws in connected_clients), return_exceptions=True)

async def run_make_command(target):
    repo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    process = await asyncio.create_subprocess_exec(
        "make", target,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
        cwd=repo_dir
    )
    while True:
        line = await process.stdout.readline()
        if not line:
            break
        log_msg = json.dumps({"type": "log", "data": line.decode(errors='replace')})
        if connected_clients:
            await asyncio.gather(*(ws.send(log_msg) for ws in connected_clients), return_exceptions=True)
    await process.wait()

async def run_ros2_topic_echo(websocket, topic):
    process = None
    try:
        cmd = f"source /opt/ros/humble/setup.bash && ros2 topic echo {topic}"
        process = await asyncio.create_subprocess_exec(
            "bash", "-c", cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT
        )
        while True:
            line = await process.stdout.readline()
            if not line:
                break
            msg = json.dumps({
                "type": "topic_data", 
                "topic": topic, 
                "data": line.decode(errors='replace').strip()
            })
            await websocket.send(msg)
    except asyncio.CancelledError:
        if process:
            try:
                process.terminate()
            except:
                pass
    except Exception:
        pass

async def monitor_active_mode():
    global active_mode
    while True:
        try:
            output = subprocess.check_output(["pgrep", "-a", "-f", "make launch-"]).decode()
            current_mode = None
            for line in output.strip().split('\n'):
                if 'make launch-' in line and not 'pgrep' in line:
                    parts = line.split('make launch-')
                    if len(parts) > 1:
                        current_mode = parts[1].split()[0]
                        break
            
            if current_mode != active_mode:
                active_mode = current_mode
                await broadcast_state()
        except subprocess.CalledProcessError:
            if active_mode is not None:
                active_mode = None
                await broadcast_state()
        except Exception:
            pass
        
        await asyncio.sleep(2.0)

async def close_all_connections():
    for ws in list(connected_clients):
        await ws.close(1000, "Force disconnected by make kill-phones")

def handle_sigusr1(signum, frame):
    if loop is not None:
        asyncio.run_coroutine_threadsafe(close_all_connections(), loop)

loop = None

async def main():
    global loop
    loop = asyncio.get_running_loop()
    signal.signal(signal.SIGUSR1, handle_sigusr1)

    proxy_thread = threading.Thread(target=serial_proxy_thread, args=(loop,), daemon=True)
    proxy_thread.start()
    
    loop.create_task(monitor_active_mode())

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(
        certfile="/home/badger/frogboard-cert/fullchain.cer",
        keyfile="/home/badger/frogboard-cert/robotfrogboard.yoonzh.com.key"
    )

    async with websockets.serve(
        handle_websocket, 
        "192.168.100.100",
        # "192.168.100.102", # updated 2026/10/19
        # "100.79.128.21",
        9090, 
        ssl=ssl_context
    ):
        print("FrogBoard Server started at wss://192.168.100.100:9090")
        # print("FrogBoard Server started at wss://100.79.128.21:9090")
        
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass