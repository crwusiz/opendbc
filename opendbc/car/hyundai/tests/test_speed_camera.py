from types import SimpleNamespace

from opendbc.car.hyundai.navi_state import NAVI_POSITION_TIMEOUT_NS, NaviState


def _navi_state():
  state = NaviState()
  state.school_zone_control = False
  state.position = {"RangeAvgSpeed": 103}
  return state


def _ret(speed_limit=100.0):
  return SimpleNamespace(speedLimit=speed_limit)


def test_navi_range_average_holds_section_until_zero():
  state = _navi_state()
  cp = SimpleNamespace(ts_nanos={"Hud_Navi_V2_POS_PE": {"RangeAvgSpeed": 1_000_000_000}},
                       _last_update_nanos=1_100_000_000)
  ret = _ret()

  assert not state._update_events(cp, ret, True)
  assert state.speed_zone_active
  assert ret.naviSectionActive
  assert ret.naviSpeed == 100

  state.position["RangeAvgSpeed"] = 0
  cp.ts_nanos["Hud_Navi_V2_POS_PE"]["RangeAvgSpeed"] = 1_200_000_000
  cp._last_update_nanos = 1_200_000_000
  assert not state._update_events(cp, ret, True)
  assert not state.speed_zone_active
  assert not ret.naviSectionActive


def test_navi_stale_range_average_releases_section():
  state = _navi_state()
  cp = SimpleNamespace(ts_nanos={"Hud_Navi_V2_POS_PE": {"RangeAvgSpeed": 1_000_000_000}},
                       _last_update_nanos=1_000_000_000)
  ret = _ret()

  assert not state._update_events(cp, ret, True)
  cp._last_update_nanos += NAVI_POSITION_TIMEOUT_NS + 1
  assert not state._update_events(cp, ret, True)
  assert not state.speed_zone_active
  assert not ret.naviSectionActive


def test_navi_range_average_does_not_keep_regular_cap_in_school_zone():
  state = _navi_state()
  state.speed_zone_active = True
  state.speed_zone_speed = 100
  state.position["RangeAvgSpeed"] = 25
  cp = SimpleNamespace(ts_nanos={"Hud_Navi_V2_POS_PE": {"RangeAvgSpeed": 1_000_000_000}},
                       _last_update_nanos=1_000_000_000)
  ret = _ret(30.0)

  assert not state._update_events(cp, ret, True)
  assert not state.speed_zone_active
  assert not ret.naviSectionActive
