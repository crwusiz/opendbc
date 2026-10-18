import math
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.navi_state import NAVI, NaviState
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiExFlags


HDA_50 = bytes.fromhex('4fe828000000000040329a1100000100')
STATUS_50 = bytes.fromhex('d0af0b4001000030000000323210f1000000000000000000')
STATUS_50_LOW = bytes.fromhex('3e38110401000000280000323210f1000000000000000000')
HDA_80 = bytes.fromhex('5ef6be000000000041509a1100000100')
SECTION_LOW = bytes.fromhex('80a5d30001000000000000325010f1000000000000000000')
SECTION_HIGH = bytes.fromhex('b99cdd0001000000000010325010f1000000000000000000')


def _parsers(monkeypatch, car=CAR.KIA_EV6):
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get_bool=lambda key: False))
  cp = structs.CarParams(carFingerprint=str(car), flags=int(car.config.flags))
  return CarState.__new__(CarState).get_can_parsers_canfd(cp)


def _step(state, parsers, *, timestamp=1_000_000_000, frames=(), alt_frames=(), speed_limit=0., is_metric=True):
  cp, alt = parsers[Bus.pt], parsers[Bus.alt]
  cp.update([(timestamp, [(address, data, cp.bus) for address, data in frames])])
  alt.update([(timestamp, [(address, data, alt.bus) for address, data in alt_frames])])
  ret = structs.CarState(speedLimit=speed_limit)
  warning = state.update(cp, ret, False, None if state.wrapped else cp.vl[NAVI.POSITION_MSG],
                         None if state.wrapped else cp.vl[NAVI.SEGMENT_MSG], cp.vl[state.profile_message],
                         profile_short=None if state.wrapped else cp.vl[NAVI.PROFILE_SHORT_MSG],
                         hda_info=cp.vl[NAVI.WRAPPED_HDA_MSG] if state.wrapped else None,
                         cp_alt=alt, status=alt.vl[NAVI.STATUS_MSG] if state.wrapped else None, is_metric=is_metric)
  return warning, ret


def test_all_canfd_platforms_register_optional_navigation_messages(monkeypatch):
  for car in CAR:
    if not car.config.flags & HyundaiFlags.CANFD:
      continue
    parsers = _parsers(monkeypatch, car)
    for name in (NAVI.POSITION_MSG, NAVI.SEGMENT_MSG, NAVI.PROFILE_SHORT_MSG, NAVI.PROFILE_MSG):
      if car == CAR.KIA_PV5:
        assert name not in parsers[Bus.pt].vl
        continue
      assert name in parsers[Bus.pt].vl, car
      assert parsers[Bus.pt].message_states[parsers[Bus.pt].dbc.name_to_msg[name].address].ignore_alive
    if car == CAR.KIA_PV5:
      assert NAVI.WRAPPED_HDA_MSG in parsers[Bus.pt].vl
      assert NAVI.WRAPPED_PROFILE_MSG in parsers[Bus.pt].vl
      assert NAVI.STATUS_MSG in parsers[Bus.alt].vl


@pytest.mark.parametrize('value,expected', [(0xD0, ('camera', 60, 0)), (6, ('bump', 0, 6)),
                                          (0x64291, ('camera', 40, 1)), (0x64270, ('camera', 30, 0))])
def test_real_long_profile_reaches_navigation_state(monkeypatch, value, expected):
  parsers = _parsers(monkeypatch)
  frame = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.PROFILE_MSG, 0,
            {'VALUE': value, 'OFFSET': 300, 'PROFILE_TYPE': 16, 'PATH_INDEX': 8})
  state = NaviState()
  warning, ret = _step(state, parsers, frames=[frame[:2]])
  assert state._classify_profile(state._decode_profile(parsers[Bus.pt].vl[NAVI.PROFILE_MSG])) == expected
  assert ret.naviAvailable and ret.naviActive
  assert warning == (expected[0] == 'camera')
  if expected[0] == 'camera':
    assert ret.speedLimit == expected[1]
    assert ret.speedLimitDistance == 300
  else:
    assert ret.speedBumpDistance == 300


def test_real_short_profile_reaches_curve_state(monkeypatch):
  parsers = _parsers(monkeypatch)
  frame = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.PROFILE_SHORT_MSG, 0,
            {'OFFSET': 313, 'DISTANCE': 5, 'VALUE_0': 599, 'PROFILE_TYPE': 1, 'PATH_INDEX': 8})
  _, ret = _step(NaviState(), parsers, frames=[frame[:2]])
  assert ret.naviCurveDistance == 313
  assert ret.naviCurveCurvature == pytest.approx(0.00112)
  assert ret.naviCurveSpeed == pytest.approx(math.sqrt(1.9 / 0.00112) * 3.6)


def test_position_range_average_enters_and_exits_section(monkeypatch):
  parsers = _parsers(monkeypatch)
  packer = CANPacker('hyundai_canfd_generated')
  state = NaviState()
  frame = packer.make_can_msg(NAVI.POSITION_MSG, 0, {'RangeAvgSpeed': 103})
  _, ret = _step(state, parsers, frames=[frame[:2]], speed_limit=100)
  assert ret.naviSectionActive and ret.naviSpeed == 100
  frame = packer.make_can_msg(NAVI.POSITION_MSG, 0, {'RangeAvgSpeed': 0})
  _, ret = _step(state, parsers, timestamp=1_100_000_000, frames=[frame[:2]], speed_limit=100)
  assert not ret.naviSectionActive


def test_segment_reset_clears_existing_events_and_curves(monkeypatch):
  parsers = _parsers(monkeypatch)
  packer = CANPacker('hyundai_canfd_generated')
  state = NaviState()
  profile = packer.make_can_msg(NAVI.PROFILE_MSG, 0, {'VALUE': 0xD0, 'OFFSET': 300, 'PROFILE_TYPE': 16})
  _step(state, parsers, frames=[profile[:2]])
  state.curves = [{'target': 300., 'curvature': 0.01, 'speed': 50.}]
  raw = (2 << 22).to_bytes(8, 'little')
  frame = packer.make_can_msg(NAVI.SEGMENT_MSG, 0, {f'BYTE_{i + 1}': byte for i, byte in enumerate(raw)})
  _, ret = _step(state, parsers, timestamp=1_100_000_000, frames=[frame[:2]])
  assert not state.events and not state.curves
  assert ret.naviCurveRouteState == 2 and ret.naviCurveSpeed == 0


def test_missing_navigation_frames_do_not_invalidate_optional_parser():
  names = (NAVI.POSITION_MSG, NAVI.SEGMENT_MSG, NAVI.PROFILE_SHORT_MSG, NAVI.PROFILE_MSG)
  parser = CANParser('hyundai_canfd_generated', [(name, math.nan) for name in names], 0)
  parser.update([(5_000_000_000, [])])
  assert parser.can_valid and not parser.bus_timeout


def test_navigation_on_another_bus_does_not_update_registered_parser(monkeypatch):
  parsers = _parsers(monkeypatch)
  cp = parsers[Bus.pt]
  frame = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.PROFILE_MSG, cp.bus + 1,
                                                         {'VALUE': 0xD0, 'OFFSET': 300, 'PROFILE_TYPE': 16})
  cp.update([(1_000_000_000, [frame])])
  assert NaviState._message_timestamp(cp, NAVI.PROFILE_MSG) == 0


@pytest.mark.parametrize('frame,expected', [(10, '0000808a32300100'), (20, '0000808a32300100'),
                                          (30, '0000808a32300100'), (40, 'ffffffffffffffff'),
                                          (50, 'ffffffffffffffff'), (60, 'ffffffffffffffff')])
def test_corner_radar_keepalive_retains_reference_payload(monkeypatch, frame, expected):
  monkeypatch.setattr(hyundaicanfd.SpeedLimiter, 'instance',
                      lambda: SimpleNamespace(recv=lambda: None, get_active=lambda: False))
  cs = SimpleNamespace(out=structs.CarState(), lfahda_cluster_info=None, cruise_buttons_msg=None,
                       adrv_msg_160=None, adrv_msg_1ea=None, ccnc_msg_161=None, ccnc_msg_162=None)
  cp = structs.CarParams(flags=int(HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CANFD_LKA_STEER_MSG))
  can = SimpleNamespace(ECAN=0, CAM=2)
  control = structs.CarControl()
  messages = hyundaicanfd.create_adrv_messages(CANPacker('hyundai_canfd_generated'), cp, control, cs,
                                             None, can, frame, 80, control.hudControl)
  assert messages == [(0x4B9, bytes.fromhex(expected), 2)]


@pytest.mark.parametrize('payload,value,offset', [
  ('5e0f24ffffffffffb0000000cce70814ffffffffffffffff', 0xB0, 1996),
  ('c27c44ffffffffff060000007ae30814ffffffffffffffff', 6, 890),
])
def test_pv5_logged_wrapped_profiles_decode(monkeypatch, payload, value, offset):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  state = NaviState(wrapped=True)
  _, ret = _step(state, parsers, frames=[(0x364, HDA_50), (0x093, bytes.fromhex(payload))], alt_frames=[(0x380, STATUS_50)])
  profile = state._decode_profile(parsers[Bus.pt].vl[NAVI.WRAPPED_PROFILE_MSG])
  assert profile == {'value': value, 'offset': offset, 'counter': 3, 'update': 1, 'profile_type': 16}
  assert ret.naviAvailable
  assert parsers[Bus.pt].vl[NAVI.WRAPPED_HDA_MSG]['CountryCode'] == 410


def test_pv5_section_latches_pulses_and_releases_after_missing_status(monkeypatch):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  state = NaviState(wrapped=True)
  def step(t, status):
    return _step(state, parsers, timestamp=int(t * 1e9), frames=[(0x364, HDA_80)],
                 alt_frames=[] if status is None else [(0x380, status)])[1]
  assert not step(1, SECTION_LOW).naviSectionActive
  assert step(1.1, SECTION_HIGH).naviSectionActive
  assert step(1.2, SECTION_LOW).naviSectionActive
  assert step(1.3, SECTION_HIGH).naviSpeed == 80
  assert not step(2.4, None).naviSectionActive
  assert not step(2.5, SECTION_HIGH).naviSectionActive
  assert not step(2.6, SECTION_LOW).naviSectionActive
  assert step(2.7, SECTION_HIGH).naviSectionActive


def test_pv5_map_warning_ends_camera_and_consumes_cached_profile(monkeypatch):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  state = NaviState(wrapped=True)
  profile = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.WRAPPED_PROFILE_MSG, 0,
                      {'PROLONG_VALUE': 0xB0, 'PROLONG_OFFSET': 200, 'PROLONG_PROFILE_TYPE': 16})
  warning, ret = _step(state, parsers, frames=[(0x364, HDA_50), profile[:2]], alt_frames=[(0x380, STATUS_50)])
  assert warning and ret.speedLimitDistance == 200
  ended = bytearray(HDA_50)
  ended[11] = 0x09  # MapSource 1 instead of 2, as in the reference end frames.
  warning, ret = _step(state, parsers, timestamp=1_100_000_000, frames=[(0x364, bytes(ended))],
                       alt_frames=[(0x380, STATUS_50_LOW)])
  assert not warning and not state.events and ret.speedLimitDistance == 0
  assert state.profile_timestamp == 1_000_000_000


def test_pv5_notification_pulse_cannot_restart_a_passed_camera(monkeypatch):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  state = NaviState(wrapped=True)
  profile = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.WRAPPED_PROFILE_MSG, 0,
                      {'PROLONG_VALUE': 0xB0, 'PROLONG_OFFSET': 200, 'PROLONG_PROFILE_TYPE': 16})
  assert _step(state, parsers, frames=[(0x364, HDA_50), profile[:2]], alt_frames=[(0x380, STATUS_50)])[0]
  warning, ret = _step(state, parsers, timestamp=1_050_000_000, frames=[(0x364, HDA_50)],
                       alt_frames=[(0x380, STATUS_50_LOW)])
  assert warning and ret.speedLimitDistance == 200
  state.total_distance = 201
  for t, status in ((1.1, STATUS_50_LOW), (1.2, STATUS_50)):
    warning, ret = _step(state, parsers, timestamp=int(t * 1e9), frames=[(0x364, HDA_50)], alt_frames=[(0x380, status)])
    assert not warning and not ret.naviActive and ret.speedLimitDistance == 0


@pytest.mark.parametrize('missing', ['hda', 'status', 'both'])
def test_pv5_stale_navigation_cancels_stored_camera(monkeypatch, missing):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  state = NaviState(wrapped=True)
  profile = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.WRAPPED_PROFILE_MSG, 0,
                      {'PROLONG_VALUE': 0xB0, 'PROLONG_OFFSET': 200, 'PROLONG_PROFILE_TYPE': 16})
  assert _step(state, parsers, frames=[(0x364, HDA_50), profile[:2]], alt_frames=[(0x380, STATUS_50)])[0]
  warning, ret = _step(state, parsers, timestamp=2_100_000_000,
                       frames=[] if missing in ('hda', 'both') else [(0x364, HDA_50)],
                       alt_frames=[] if missing in ('status', 'both') else [(0x380, STATUS_50)])
  assert not warning and not ret.naviActive and ret.speedLimitDistance == 0
  assert not state.events


def test_pv5_does_not_use_legacy_kind_seven_to_start_a_section(monkeypatch):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  state = NaviState(wrapped=True)
  profile = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.WRAPPED_PROFILE_MSG, 0,
                      {'PROLONG_VALUE': 0x117, 'PROLONG_OFFSET': 0, 'PROLONG_PROFILE_TYPE': 16})
  _, ret = _step(state, parsers, frames=[(0x364, HDA_80), profile[:2]], alt_frames=[(0x380, SECTION_LOW)])
  assert not ret.naviSectionActive and not state.speed_zone_active
  assert not state.events


def test_pv5_section_requires_matching_speed_limits(monkeypatch):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  state = NaviState(wrapped=True)
  status = CANPacker('hyundai_canfd_generated').make_can_msg(NAVI.STATUS_MSG, 1, {'SPEED_LIMIT': 70, 'SECTION_ALERT': 1})
  _, ret = _step(state, parsers, frames=[(0x364, HDA_80)], alt_frames=[status[:2]])
  assert not ret.naviSectionActive


def test_pv5_imperial_limits_are_converted_to_kph(monkeypatch):
  parsers = _parsers(monkeypatch, CAR.KIA_PV5)
  warning, ret = _step(NaviState(wrapped=True), parsers, frames=[(0x364, HDA_50)],
                       alt_frames=[(0x380, STATUS_50)], is_metric=False)
  assert warning
  assert ret.naviSpeed == pytest.approx(50 * 1.609344)
  assert ret.speedLimit == pytest.approx(50 * 1.609344)


@pytest.mark.parametrize('native,external,main,enabled,expected', [
  (True, False, True, True, 2),
  (True, False, True, False, 1),
  (True, False, False, False, 0),
  (False, False, True, True, 0),
  (False, True, True, True, 2),
])
def test_cluster_navigation_icon_uses_native_navigation_availability(monkeypatch, native, external, main, enabled, expected):
  monkeypatch.setattr(hyundaicanfd.SpeedLimiter, 'instance',
                      lambda: SimpleNamespace(recv=lambda: None, get_active=lambda: external))
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get=lambda *args, **kwargs: 0))
  hyundaicanfd.create_adrv_messages.__dict__.pop('_lane_line_check', None)
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['CCNC_0x161']
  out = structs.CarState(naviAvailable=native)
  out.cruiseState.available = main
  cs = SimpleNamespace(out=out, lfahda_cluster_info=None, cruise_buttons_msg=None, adrv_msg_160=None,
                       adrv_msg_1ea=None, ccnc_msg_162=None, ccnc_msg_161={key: 0 for key in definition.sigs},
                       left_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                       right_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                       left_lane_line_visibility=hyundaicanfd.LaneLineVisibility(),
                       right_lane_line_visibility=hyundaicanfd.LaneLineVisibility())
  cp = structs.CarParams(flags=int(HyundaiFlags.CANFD_CAMERA_SCC), exFlags=int(HyundaiExFlags.CCNC))
  control = structs.CarControl(enabled=enabled)
  messages = hyundaicanfd.create_adrv_messages(packer, cp, control, cs, None,
                                             SimpleNamespace(ECAN=0, CAM=2), 5, 80, control.hudControl)
  assert len(messages) == 1
  assert definition.sigs['NAV_ICON'].get_raw_value(messages[0][1]) == expected
