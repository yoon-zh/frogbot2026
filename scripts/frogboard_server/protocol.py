"""Pure (I/O free) helpers for the FrogBoard <-> STM32 serial protocol.

Kept separate from server.py so the logic can be unit tested without
websockets / pyserial / a robot attached.

Downlink frame (one per UART idle gap, newline terminated):

    vcx=<m/s>,wc=<rad/s>,en=<code>\\n

``en`` codes understood by the firmware (see drive_control.h):

    0  EN_DISABLE  falling edge -> active-brake and disarm (serial source only)
    1  EN_NORMAL   no arming action (what ROS sends; it can never arm the robot)
    2  EN_ARM      arm request; only honoured when the commanded speed is ~0,
                   the robot is disarmed, the serial source is selected and no
                   e-stop is latched. Repeated by the server for a short window
                   so one dropped UART frame cannot lose the request.
"""

import threading

EN_DISABLE = 0
EN_NORMAL = 1
EN_ARM = 2

# How long an Enable / Disable press keeps being re-sent at the phone rate.
ARM_WINDOW_S = 0.5
DISABLE_WINDOW_S = 0.5
# Grace after the arm window before telemetry saying "disabled" is believed.
CHASSIS_SYNC_GRACE_S = 0.5

CTRL_MODE_SERIAL = 0
CTRL_MODE_JOYSTICK = 1
CTRL_MODE_DISABLED = 2


def format_command(vcx, wc, en):
    """Return the exact bytes to write to the STM32 (real newline, not '\\\\n')."""
    return f"vcx={vcx:.3f},wc={wc:.3f},en={int(en)}\n".encode("ascii")


class PhoneControlState:
    """Tracks the phone's motor-enable button and derives the ``en`` code."""

    def __init__(self):
        self.enabled = False
        self.arm_until = 0.0
        self.disable_until = 0.0

    def set_enabled(self, enabled, now):
        enabled = bool(enabled)
        if enabled:
            self.arm_until = now + ARM_WINDOW_S
            self.disable_until = 0.0
        else:
            self.disable_until = now + DISABLE_WINDOW_S
            self.arm_until = 0.0
        self.enabled = enabled

    def reset(self, now):
        """Safety reset (last client left): behave as an explicit Disable."""
        self.set_enabled(False, now)

    def in_arm_window(self, now):
        return self.enabled and now < self.arm_until

    def override_active(self, now):
        """True while an Enable/Disable press is still being (re)sent."""
        return now < self.arm_until or now < self.disable_until

    def en_code(self, now):
        if not self.enabled:
            return EN_DISABLE
        if self.in_arm_window(now):
            return EN_ARM
        return EN_NORMAL

    def velocities(self, vcx, wc, now):
        """Speed to send: zero until the robot is armed (arming needs ~0)."""
        if not self.enabled or self.in_arm_window(now):
            return 0.0, 0.0
        return vcx, wc

    def build_command(self, vcx, wc, now):
        vcx, wc = self.velocities(vcx, wc, now)
        return format_command(vcx, wc, self.en_code(now))

    def sync_from_chassis(self, ctrl_mode, now):
        """Drop the enabled flag if the STM32 reports it is disabled.

        Happens when a PS2 brake / e-stop / R2 takeover overrides the phone, or
        an arm request was refused. Returns True if the flag changed.
        """
        if (
            self.enabled
            and ctrl_mode == CTRL_MODE_DISABLED
            and now >= self.arm_until + CHASSIS_SYNC_GRACE_S
        ):
            self.enabled = False
            return True
        return False


class LinkStats:
    """Thread-safe per-interval statistics for diagnosing joystick latency.

    The asyncio thread calls ``on_joystick`` for every websocket joystick
    message, the serial thread calls ``on_phone_send`` when it writes a phone
    frame, and a monitor task calls ``on_loop_lag``. ``snapshot_and_reset``
    returns (and clears) the numbers for the 1 Hz status line.
    """

    # Gaps longer than this are the user pausing, not link jitter.
    ACTIVE_GAP_LIMIT_S = 1.0

    def __init__(self):
        self._lock = threading.Lock()
        self._last_rx = None
        self._pending_rx = None
        self._reset()

    def _reset(self):
        self._joy_count = 0
        self._max_gap = 0.0
        self._lat_sum = 0.0
        self._lat_n = 0
        self._lat_max = 0.0
        self._loop_lag_max = 0.0

    def on_joystick(self, now):
        """Record a joystick message; returns the gap since the previous one."""
        with self._lock:
            gap = None
            if self._last_rx is not None:
                gap = now - self._last_rx
                if gap < self.ACTIVE_GAP_LIMIT_S:
                    self._max_gap = max(self._max_gap, gap)
            self._last_rx = now
            self._pending_rx = now  # age of the newest value not yet written
            self._joy_count += 1
            return gap

    def on_phone_send(self, now):
        """Record that a phone frame was written; measures how stale its value was."""
        with self._lock:
            if self._pending_rx is None:
                return
            lat = now - self._pending_rx
            self._pending_rx = None
            self._lat_sum += lat
            self._lat_n += 1
            self._lat_max = max(self._lat_max, lat)

    def on_loop_lag(self, lag):
        with self._lock:
            self._loop_lag_max = max(self._loop_lag_max, lag)

    def snapshot_and_reset(self):
        with self._lock:
            snap = {
                "joy_count": self._joy_count,
                "max_gap_ms": self._max_gap * 1000.0,
                "ws_to_tx_avg_ms": (self._lat_sum / self._lat_n * 1000.0) if self._lat_n else 0.0,
                "ws_to_tx_max_ms": self._lat_max * 1000.0,
                "loop_lag_max_ms": self._loop_lag_max * 1000.0,
            }
            self._reset()
            return snap
