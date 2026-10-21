import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as p  # noqa: E402


class FormatCommandTests(unittest.TestCase):
    def test_uses_real_newline_and_en_field(self):
        self.assertEqual(
            p.format_command(0.5, -0.2, p.EN_NORMAL), b"vcx=0.500,wc=-0.200,en=1\n"
        )
        self.assertNotIn(b"\\n", p.format_command(0, 0, p.EN_ARM))
        self.assertTrue(p.format_command(0, 0, p.EN_ARM).endswith(b"en=2\n"))


class PhoneControlStateTests(unittest.TestCase):
    def test_default_is_disabled_and_zero(self):
        s = p.PhoneControlState()
        self.assertEqual(s.en_code(0.0), p.EN_DISABLE)
        self.assertEqual(s.build_command(0.4, 0.2, 0.0), b"vcx=0.000,wc=0.000,en=0\n")
        self.assertFalse(s.override_active(0.0))

    def test_enable_sends_arm_pulse_with_zero_speed_then_normal(self):
        s = p.PhoneControlState()
        s.set_enabled(True, 10.0)
        # Arm window: en=2 and forced zero speed even if the stick is deflected.
        self.assertEqual(s.en_code(10.1), p.EN_ARM)
        self.assertEqual(s.build_command(0.4, 0.2, 10.1), b"vcx=0.000,wc=0.000,en=2\n")
        self.assertTrue(s.override_active(10.1))
        # After the window: en=1 and the joystick passes through.
        after = 10.0 + p.ARM_WINDOW_S + 0.01
        self.assertEqual(s.en_code(after), p.EN_NORMAL)
        self.assertEqual(s.build_command(0.4, 0.2, after), b"vcx=0.400,wc=0.200,en=1\n")
        self.assertFalse(s.override_active(after))

    def test_disable_sends_en0_window_then_idle(self):
        s = p.PhoneControlState()
        s.set_enabled(True, 0.0)
        s.set_enabled(False, 5.0)
        self.assertEqual(s.build_command(0.4, 0.2, 5.1), b"vcx=0.000,wc=0.000,en=0\n")
        self.assertTrue(s.override_active(5.1))
        self.assertFalse(s.override_active(5.0 + p.DISABLE_WINDOW_S + 0.01))

    def test_disable_cancels_pending_arm_window(self):
        s = p.PhoneControlState()
        s.set_enabled(True, 1.0)
        s.set_enabled(False, 1.1)
        self.assertEqual(s.en_code(1.2), p.EN_DISABLE)

    def test_reset_behaves_like_disable(self):
        s = p.PhoneControlState()
        s.set_enabled(True, 0.0)
        s.reset(3.0)
        self.assertFalse(s.enabled)
        self.assertTrue(s.override_active(3.1))

    def test_chassis_disabled_clears_enable_after_grace(self):
        s = p.PhoneControlState()
        s.set_enabled(True, 0.0)
        # Telemetry lags the arm request: ignored during the grace period.
        self.assertFalse(s.sync_from_chassis(p.CTRL_MODE_DISABLED, 0.4))
        self.assertFalse(s.sync_from_chassis(p.CTRL_MODE_DISABLED, 0.9))
        self.assertTrue(s.enabled)
        # Still disabled well after arming -> PS2 brake / e-stop / refused arm.
        self.assertTrue(s.sync_from_chassis(p.CTRL_MODE_DISABLED, 1.1))
        self.assertFalse(s.enabled)

    def test_chassis_serial_or_joystick_keeps_enable(self):
        s = p.PhoneControlState()
        s.set_enabled(True, 0.0)
        self.assertFalse(s.sync_from_chassis(p.CTRL_MODE_SERIAL, 5.0))
        self.assertFalse(s.sync_from_chassis(p.CTRL_MODE_JOYSTICK, 5.0))
        self.assertTrue(s.enabled)


class LinkStatsTests(unittest.TestCase):
    def test_counts_gaps_and_ws_to_tx_latency(self):
        st = p.LinkStats()
        self.assertIsNone(st.on_joystick(10.00))
        self.assertAlmostEqual(st.on_joystick(10.05), 0.05)
        self.assertAlmostEqual(st.on_joystick(10.20), 0.15)
        st.on_phone_send(10.23)  # newest joystick value was 30 ms stale
        st.on_phone_send(10.30)  # nothing pending: not counted
        st.on_joystick(10.40)
        st.on_phone_send(10.41)  # 10 ms
        st.on_loop_lag(0.004)
        st.on_loop_lag(0.080)
        snap = st.snapshot_and_reset()
        self.assertEqual(snap["joy_count"], 4)
        self.assertAlmostEqual(snap["max_gap_ms"], 200.0, places=3)
        self.assertAlmostEqual(snap["ws_to_tx_avg_ms"], 20.0, places=3)
        self.assertAlmostEqual(snap["ws_to_tx_max_ms"], 30.0, places=3)
        self.assertAlmostEqual(snap["loop_lag_max_ms"], 80.0, places=3)

    def test_pauses_are_not_counted_as_jitter_and_reset_clears(self):
        st = p.LinkStats()
        st.on_joystick(1.0)
        st.on_joystick(5.0)  # user paused for 4 s
        snap = st.snapshot_and_reset()
        self.assertEqual(snap["max_gap_ms"], 0.0)
        self.assertEqual(st.snapshot_and_reset()["joy_count"], 0)


if __name__ == "__main__":
    unittest.main()
