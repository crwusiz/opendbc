#!/usr/bin/env python3
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket


class TestHyundaiRedundancy(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.hyundaiCanfd

  def _assert_copies_match(self):
    self.assertEqual(common.state_buffer("safety_a")[:], common.state_buffer("safety_b")[:])
    self.assertFalse(self.safety.safety_redundancy_fault)

  def _resume_msg(self):
    # The default CAN FD configuration sends CRUISE_BUTTONS on bus 2.
    return make_CANPacket(0x1CF, 2, b"\x00\x00\x01\x00\x00\x00\x00\x00")

  def test_configuration_copies_match(self):
    modes = (CarParams.SafetyModel.hyundai, CarParams.SafetyModel.hyundaiLegacy, CarParams.SafetyModel.hyundaiCanfd)
    for mode in modes:
      for param in range(1 << len(HyundaiSafetyFlags)):
        with self.subTest(mode=mode, param=param):
          self.assertEqual(self.safety.set_safety_hooks(mode, param), 0)
          self._assert_copies_match()

          setup = self.safety.safety_a.setup
          self.assertEqual(setup.hyundai_ev_gas_signal, bool(param & HyundaiSafetyFlags.EV_GAS))
          self.assertEqual(setup.hyundai_hybrid_gas_signal,
                           bool(param & HyundaiSafetyFlags.HYBRID_GAS) and not setup.hyundai_ev_gas_signal)
          self.assertEqual(setup.hyundai_longitudinal,
                           bool(param & HyundaiSafetyFlags.LONG) and mode != CarParams.SafetyModel.hyundaiLegacy)
          self.assertEqual(setup.hyundai_camera_scc,
                           bool(param & HyundaiSafetyFlags.CAMERA_SCC) and mode != CarParams.SafetyModel.hyundaiLegacy)
          if mode == CarParams.SafetyModel.hyundaiCanfd:
            self.assertEqual(setup.hyundai_canfd_angle_steer_msg, bool(param & HyundaiSafetyFlags.CANFD_ANGLE_STEER_MSG))
            self.assertEqual(setup.hyundai_canfd_lka_steer_msg_alt,
                             bool(param & (HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiSafetyFlags.CANFD_ANGLE_STEER_MSG)))

  def test_angle_flag_resets_on_mode_change(self):
    param = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.CANFD_ANGLE_STEER_MSG
    self.assertEqual(self.safety.set_safety_hooks(self.SAFETY_MODEL, param), 0)
    self.assertTrue(self.safety.safety_a.setup.hyundai_canfd_angle_steer_msg)
    self.assertTrue(self.safety.safety_b.setup.hyundai_canfd_angle_steer_msg)

    self.assertEqual(self.safety.set_safety_hooks(self.SAFETY_MODEL, 0), 0)
    for copy in (self.safety.safety_a, self.safety.safety_b):
      self.assertFalse(copy.setup.hyundai_canfd_angle_steer_msg)
      self.assertFalse(copy.setup.hyundai_canfd_lka_steer_msg_alt)

  def test_tx_timestamps_use_shared_time(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(1000000)
    self.safety.set_timer_step(1)
    self.assertTrue(self._tx(self._resume_msg()))

    for copy in (self.safety.safety_a, self.safety.safety_b):
      self.assertIn(1000001, copy.state.hyundai_canfd_bus0_timestamps)
      self.assertIn(1000001, copy.state.hyundai_canfd_bus2_timestamps)
    self._assert_copies_match()

  def test_forwarding_timeout_uses_shared_time(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(1000000)
    self.assertTrue(self._tx(self._resume_msg()))

    # With one timer snapshot, both copies see 39,999 us since TX. Separate
    # timer reads would straddle the 40,000 us expiry and disagree.
    self.safety.set_timer(1039998)
    self.safety.set_timer_step(1)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x1CF), -1)
    self._assert_copies_match()
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x1CF), 2)
    self._assert_copies_match()

  def test_recent_tx_blocks_both_directions_until_expiry(self):
    for tx_time in (1000000, 0xFFFFFFF0):
      with self.subTest(tx_time=tx_time):
        self.assertEqual(self.safety.set_safety_hooks(self.SAFETY_MODEL, 0), 0)
        self.safety.set_controls_allowed(True)
        self.safety.set_timer(tx_time)
        for bus, destination in ((0, 2), (2, 0)):
          self.assertEqual(self.safety.safety_fwd_hook(bus, 0x1CF), destination)
        self.assertTrue(self._tx(self._resume_msg()))

        for elapsed in (0, 39999, 40000):
          self.safety.set_timer((tx_time + elapsed) & 0xFFFFFFFF)
          for bus, destination in ((0, 2), (2, 0)):
            with self.subTest(elapsed=elapsed, bus=bus):
              expected = -1 if elapsed < 40000 else destination
              self.assertEqual(self.safety.safety_fwd_hook(bus, 0x1CF), expected)
              self._assert_copies_match()

  def test_forwarding_timestamp_corruption_fails_closed(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(1000000)
    self.assertTrue(self._tx(self._resume_msg()))

    timestamps = self.safety.safety_b.state.hyundai_canfd_bus2_timestamps
    index = list(timestamps).index(1000000)
    timestamps[index] = 960000
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x1CF), -1)
    self.assertTrue(self.safety.safety_redundancy_fault)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(common.state_buffer("safety_a")[:], common.state_buffer("safety_b")[:])
    self.safety.safety_redundancy_fault = False

  def test_longitudinal_angle_steering_is_checked(self):
    param = HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.CANFD_ANGLE_STEER_MSG
    self.assertEqual(self.safety.set_safety_hooks(self.SAFETY_MODEL, param), 0)
    self.safety.set_controls_allowed(True)

    dat = bytearray(32)
    dat[9] = 0x20  # Active angle control.
    self.assertTrue(self._tx(make_CANPacket(0x110, 1, dat)))
    desired_angle = 1801
    dat[10] = (desired_angle & 0x3F) << 2
    dat[11] = desired_angle >> 6
    self.assertFalse(self._tx(make_CANPacket(0x110, 1, dat)))


if __name__ == "__main__":
  unittest.main()
