from types import SimpleNamespace

from opendbc.car.hyundai.carstate import CarState, VEHICLE_NAVI_POSITION_TIMEOUT_NS


def _car_state():
  state = CarState.__new__(CarState)
  state.totalDistance = 0.0
  state.vehicleNaviCanControl = True
  state.vehicleNaviSchoolZoneControl = False
  state.vehicleNaviEvents = []
  state.vehicleNaviSegmentTimestamp = 0
  state.vehicleNaviProfileTimestamp = 0
  state.vehicleNaviAvailable = False
  state.vehicleNaviRouteResetTimestamp = 0
  state.vehicleNaviCameraTarget = None
  state.vehicleNaviSpeedZoneActive = False
  state.vehicleNaviSpeedZoneSpeed = 0.0
  state.vehicleNaviSchoolZoneActive = False
  state.vehicleNaviSchoolZoneStartDistance = 0.0
  state.vehicleNaviSchoolZoneUsesCameraStatus = False
  state.navi_position_4b4 = {"RangeAvgSpeed": 103}
  state.navi_segment_4b9 = None
  state.navi_profile_4be = None
  return state


def _ret(speed_limit=100.0):
  return SimpleNamespace(speedLimit=speed_limit)


def test_vehicle_navi_range_average_holds_section_until_zero():
  state = _car_state()
  cp = SimpleNamespace(ts_nanos={"Hud_Navi_V2_POS_PE": {"RangeAvgSpeed": 1_000_000_000}},
                       _last_update_nanos=1_100_000_000)
  ret = _ret()

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert state.vehicleNaviSpeedZoneActive
  assert ret.vehicleNaviSectionActive
  assert ret.vehicleNaviSpeed == 100

  state.navi_position_4b4["RangeAvgSpeed"] = 0
  cp.ts_nanos["Hud_Navi_V2_POS_PE"]["RangeAvgSpeed"] = 1_200_000_000
  cp._last_update_nanos = 1_200_000_000
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_stale_range_average_releases_section():
  state = _car_state()
  cp = SimpleNamespace(ts_nanos={"Hud_Navi_V2_POS_PE": {"RangeAvgSpeed": 1_000_000_000}},
                       _last_update_nanos=1_000_000_000)
  ret = _ret()

  assert not state._update_vehicle_navi_events(cp, ret, True)
  cp._last_update_nanos += VEHICLE_NAVI_POSITION_TIMEOUT_NS + 1
  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviSectionActive


def test_vehicle_navi_range_average_does_not_keep_regular_cap_in_school_zone():
  state = _car_state()
  state.vehicleNaviSpeedZoneActive = True
  state.vehicleNaviSpeedZoneSpeed = 100
  state.navi_position_4b4["RangeAvgSpeed"] = 25
  cp = SimpleNamespace(ts_nanos={"Hud_Navi_V2_POS_PE": {"RangeAvgSpeed": 1_000_000_000}},
                       _last_update_nanos=1_000_000_000)
  ret = _ret(30.0)

  assert not state._update_vehicle_navi_events(cp, ret, True)
  assert not state.vehicleNaviSpeedZoneActive
  assert not ret.vehicleNaviSectionActive
