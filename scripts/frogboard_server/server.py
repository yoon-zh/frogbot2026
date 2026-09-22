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

def create_pty(path):
    master, slave = pty.openpty()
    tty.cfmakeraw(slave)
    if os.path.exists(path):
        try:
            os.remove(path)
        except OSError:
            pass
    os.symlink(os.ttyname(slave), path)
    return master, slave

def serial_proxy_thread(loop):
    global ps2_connected
    try:
        master_tx, slave_tx = create_pty(VIRTUAL_SERIAL_TX)
        master_rx, slave_rx = create_pty(VIRTUAL_SERIAL_RX)
    except Exception as e:
        print(f"Failed to create PTYs: {e}")
        return

    while True:
        try:
            ser = serial.Serial(REAL_SERIAL_PORT, BAUDRATE, timeout=0.01)
            break
        except Exception as e:
            print(f"Waiting for real serial port {REAL_SERIAL_PORT}: {e}")
            time.sleep(1.0)

    serial_buffer = b""
    last_phone_send = time.time()
    last_idle_send = time.time()

    while True:
        try:
            r, w, x = select.select([master_tx, ser.fileno()], [], [], 0.05)
        except Exception as e:
            time.sleep(0.1)
            continue

        now = time.time()
        phone_active = (now - last_phone_cmd_time <= 0.5)

        # 1. Read from REAL serial, forward to RX PTY and parse telemetry
        if ser.fileno() in r:
            try:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    try:
                        os.write(master_rx, data)
                    except OSError as e:
                        if e.errno != 5: # Ignore EIO if reader node closed it
                            pass
                    
                    serial_buffer += data
                    while b'\n' in serial_buffer:
                        line, serial_buffer = serial_buffer.split(b'\n', 1)
                        try:
                            parts = line.decode('ascii', errors='ignore').strip().split(',')
                            if len(parts) >= 19:
                                new_ps2_status = (int(parts[18]) == 1)
                                if new_ps2_status != ps2_connected:
                                    ps2_connected = new_ps2_status
                                    asyncio.run_coroutine_threadsafe(broadcast_state(), loop)
                        except Exception:
                            pass
            except Exception as e:
                print(f"Serial read error: {e}")
        
        # Process explicit commands from WebSocket
        while not serial_cmd_queue.empty():
            cmd_bytes = serial_cmd_queue.get_nowait()
            try:
                ser.write(cmd_bytes)
            except OSError:
                pass

        # 2. Control Logic & Multiplexing
        if len(connected_clients) == 0 and not ps2_connected:
            # Force zero speed if no phone and no PS2
            if now - last_idle_send > 0.1:
                try:
                    ser.write(b"vcx=0.000,wc=0.000,en=0\n")
                except OSError:
                    pass
                last_idle_send = now
            # Drain TX PTY to prevent blocking
            if master_tx in r:
                try:
                    os.read(master_tx, 1024)
                except OSError as e:
                    if e.errno != 5: # Ignore EIO
                        pass
        else:
            if phone_active:
                # Phone joystick active -> override ROS
                if now - last_phone_send > 0.05:
                    cmd = f"vcx={phone_vcx:.3f},wc={phone_wc:.3f},en=1\n"
                    try:
                        ser.write(cmd.encode())
                    except OSError:
                        pass
                    last_phone_send = now
                # Drain TX PTY to prevent blocking
                if master_tx in r:
                    try:
                        os.read(master_tx, 1024)
                    except OSError as e:
                        if e.errno != 5:
                            pass
            else:
                # Phone idle -> let ROS (TX PTY) through
                if master_tx in r:
                    try:
                        data = os.read(master_tx, 1024)
                        if data:
                            ser.write(data)
                    except OSError as e:
                        if e.errno != 5: # Ignore EIO
                            pass

async def handle_websocket(websocket, path):
    global connected_clients, phone_vcx, phone_wc, last_phone_cmd_time
    if len(connected_clients) >= 8:
        await websocket.close(1008, "Max connections reached")
        return
    
    connected_clients.add(websocket)
    await broadcast_state()
    try:
        async for message in websocket:
            data = json.loads(message)
            if data.get("type") == "joystick":
                phone_vcx = float(data.get("linear", 0.0))
                phone_wc = float(data.get("angular", 0.0))
                last_phone_cmd_time = time.time()
            elif data.get("type") == "motor_state":
                enable = data.get("enable")
                if enable:
                    serial_cmd_queue.put(b"vcx=0.000,wc=0.000,en=1\n")
                else:
                    serial_cmd_queue.put(b"vcx=0.000,wc=0.000,en=0\n")
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
    except Exception:
        pass
    finally:
        if websocket in active_subscriptions:
            active_subscriptions[websocket].cancel()
            del active_subscriptions[websocket]
        connected_clients.remove(websocket)
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
    asyncio.run_coroutine_threadsafe(close_all_connections(), loop)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    signal.signal(signal.SIGUSR1, handle_sigusr1)

    proxy_thread = threading.Thread(target=serial_proxy_thread, args=(loop,), daemon=True)
    proxy_thread.start()
    
    loop.create_task(monitor_active_mode())

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(
        certfile="/home/badger/frogboard-cert/fullchain.cer",
        keyfile="/home/badger/frogboard-cert/robotfrogboard.yoonzh.com.key"
    )

    start_server = websockets.serve(
        handle_websocket, 
        "192.168.100.102",
        # "100.79.128.21",
        9090, 
        ssl=ssl_context
    )
    
    print("FrogBoard Server started at wss://192.168.100.102:9090")
    # print("FrogBoard Server started at wss://100.79.128.21:9090")
    
    loop.run_until_complete(start_server)
    loop.run_forever()