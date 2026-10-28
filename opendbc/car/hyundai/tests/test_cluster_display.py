from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExFlags


@pytest.fixture
def cluster(monkeypatch):
  navigation = SimpleNamespace(active=False)
  monkeypatch.setattr(hyundaicanfd.SpeedLimiter, 'instance',
                      lambda: SimpleNamespace(recv=lambda: None, get_active=lambda: navigation.active))
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get=lambda *args, **kwargs: 0))
  for name in ('_gap_bars', '_gap_popup_until', '_lane_line_check'):
    monkeypatch.setattr(hyundaicanfd.create_adrv_messages, name, None, raising=False)
    monkeypatch.delattr(hyundaicanfd.create_adrv_messages, name)

  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['CCNC_0x161']
  stock = {key: 0 for key in definition.sigs}
  stock.update(ALERTS_1=11, ALERTS_3=18, LANE_HIGHLIGHT=5, LANE_HIGHLIGHT_DISTANCE=123.4)
  out = structs.CarState()
  out.cruiseState.available = True
  state = SimpleNamespace(out=out, ccnc_msg_161=stock, ccnc_msg_162=None, adrv_msg_160=None,
                          adrv_msg_1ea=None, lfahda_cluster_info=None, cruise_buttons_msg=None,
                          leftLnPosition=1.7, rightLnPosition=1.7, leftLnQualStat=2, rightLnQualStat=2,
                          left_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                          right_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                          left_lane_line_visibility=hyundaicanfd.LaneLineVisibility(),
                          right_lane_line_visibility=hyundaicanfd.LaneLineVisibility())
  control = structs.CarControl(enabled=True)
  cp = structs.CarParams(flags=int(HyundaiFlags.CANFD_CAMERA_SCC), exFlags=int(HyundaiExFlags.CCNC))

  def emit(frame=5):
    original = stock.copy()
    messages = hyundaicanfd.create_adrv_messages(packer, cp, control, state, None,
                                               SimpleNamespace(ECAN=0, CAM=2), frame, 80, control.hudControl)
    assert len(messages) == 1
    address, data, bus = messages[0]
    assert (address, bus) == (0x161, 0)
    assert stock == original
    values = {key: sig.get_raw_value(data) * sig.factor + sig.offset for key, sig in definition.sigs.items()}
    assert values['CHECKSUM'] == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))
    assert (values['ALERTS_1'], values['ALERTS_3']) == (11, 18)
    return values

  return SimpleNamespace(emit=emit, navigation=navigation, control=control, state=state)


@pytest.mark.parametrize('main,enabled,normal', [(True, True, 1), (True, False, 3), (False, False, 7), (False, True, 1)])
@pytest.mark.parametrize('left,right,flashing', [(False, False, None), (True, False, 4), (False, True, 4), (True, True, 6)])
def test_background_uses_native_flashing_for_turn_signals_and_hazards(cluster, main, enabled, normal, left, right, flashing):
  cluster.state.out.cruiseState.available = main
  cluster.control.enabled = enabled
  cluster.state.out.leftBlinker = left
  cluster.state.out.rightBlinker = right
  for frame in (5, 10, 55, 100, 105):
    assert cluster.emit(frame)['BACKGROUND'] == (normal if flashing is None else flashing)


@pytest.mark.parametrize('enabled,lateral,navigation,left_depart,right_depart,expected', [
  (True, False, False, False, False, 0),
  (True, False, True, False, False, 0),
  (True, False, True, True, False, 0),
  (True, True, False, False, False, 1),
  (False, True, True, False, False, 1),
  (True, True, True, False, False, 3),
  (True, True, False, True, False, 4),
  (True, True, False, False, True, 4),
  (True, True, True, True, False, 4),
  (True, True, True, False, True, 4),
  (True, True, True, True, True, 4),
])
def test_lane_highlight_follows_lateral_hdp_and_departure_state(
  cluster, enabled, lateral, navigation, left_depart, right_depart, expected,
):
  cluster.control.enabled = enabled
  cluster.control.latActive = lateral
  cluster.navigation.active = navigation
  cluster.control.hudControl.leftLaneDepart = left_depart
  cluster.control.hudControl.rightLaneDepart = right_depart
  values = cluster.emit()
  assert values['LANE_HIGHLIGHT'] == expected
  assert values['LANE_HIGHLIGHT_DISTANCE'] == (50.0 if expected else 0.0)


def test_background_and_highlight_return_to_normal_after_signals_and_warning_clear(cluster):
  cluster.control.latActive = True
  cluster.navigation.active = True
  for frame, left, right, departure, background, highlight in (
    (5, True, False, False, 4, 3),
    (10, True, True, True, 6, 4),
    (15, False, True, False, 4, 3),
    (20, False, False, False, 1, 3),
  ):
    cluster.state.out.leftBlinker = left
    cluster.state.out.rightBlinker = right
    cluster.control.hudControl.leftLaneDepart = departure
    values = cluster.emit(frame)
    assert (values['BACKGROUND'], values['LANE_HIGHLIGHT'], values['LANE_HIGHLIGHT_DISTANCE']) == (background, highlight, 50.0)

  cluster.control.latActive = False
  values = cluster.emit(25)
  assert (values['LANE_HIGHLIGHT'], values['LANE_HIGHLIGHT_DISTANCE']) == (0, 0.0)
