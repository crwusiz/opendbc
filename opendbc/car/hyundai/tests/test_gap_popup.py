from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExFlags


@pytest.fixture
def cluster(monkeypatch):
  navigation = SimpleNamespace(active=True)
  monkeypatch.setattr(hyundaicanfd.SpeedLimiter, 'instance',
                      lambda: SimpleNamespace(recv=lambda: None, get_active=lambda: navigation.active))
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get=lambda *args, **kwargs: 0))
  for name in ('_gap_bars', '_gap_popup_until', '_lane_line_check'):
    monkeypatch.setattr(hyundaicanfd.create_adrv_messages, name, None, raising=False)
    monkeypatch.delattr(hyundaicanfd.create_adrv_messages, name)

  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['CCNC_0x161']
  stock = {key: 0 for key in definition.sigs}
  out = structs.CarState()
  out.cruiseState.available = True
  state = SimpleNamespace(out=out, ccnc_msg_161=stock, ccnc_msg_162=None, adrv_msg_160=None,
                          adrv_msg_1ea=None, lfahda_cluster_info=None, cruise_buttons_msg=None,
                          left_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                          right_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                          left_lane_line_visibility=hyundaicanfd.LaneLineVisibility(),
                          right_lane_line_visibility=hyundaicanfd.LaneLineVisibility())
  control = structs.CarControl(enabled=True)
  cp = structs.CarParams(flags=int(HyundaiFlags.CANFD_CAMERA_SCC), exFlags=int(HyundaiExFlags.CCNC))

  def emit(frame, bars):
    control.hudControl.leadDistanceBars = bars
    original = stock.copy()
    messages = hyundaicanfd.create_adrv_messages(packer, cp, control, state, None,
                                               SimpleNamespace(ECAN=0, CAM=2), frame, 80, control.hudControl)
    assert len(messages) == 1
    address, data, bus = messages[0]
    assert (address, bus) == (0x161, 0)
    assert stock == original
    assert definition.sigs['CHECKSUM'].get_raw_value(data) == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))
    return definition.sigs['ALERTS_3'].get_raw_value(data)

  return SimpleNamespace(emit=emit, navigation=navigation, control=control, stock=stock)


@pytest.mark.parametrize('bars', (1, 2, 3, 4))
def test_first_gap_does_not_trigger_popup(cluster, bars):
  assert cluster.emit(5, bars) == 0
  assert cluster.emit(10, bars) == 0


@pytest.mark.parametrize('bars', (1, 2, 3, 4))
def test_gap_change_selects_stock_distance_popup(cluster, bars):
  assert cluster.emit(5, 3 if bars == 2 else 2) == 0
  assert cluster.emit(10, bars) == 6 + bars


def test_unchanged_gap_expires_after_two_seconds(cluster):
  assert cluster.emit(5, 2) == 0
  assert cluster.emit(10, 3) == 9
  for frame in range(15, 210, 5):
    assert cluster.emit(frame, 3) == 9
  assert cluster.emit(210, 3) == 0
  assert cluster.emit(215, 3) == 0


def test_repeated_change_restarts_two_second_popup(cluster):
  assert cluster.emit(5, 2) == 0
  assert cluster.emit(10, 3) == 9
  assert cluster.emit(110, 4) == 10
  assert cluster.emit(210, 4) == 10
  assert cluster.emit(305, 4) == 10
  assert cluster.emit(310, 4) == 0


@pytest.mark.parametrize('inactive', ('navigation', 'cruise'))
def test_gap_change_without_hdp_does_not_start_or_defer_popup(cluster, inactive):
  if inactive == 'navigation':
    cluster.navigation.active = False
  else:
    cluster.control.enabled = False
  assert cluster.emit(5, 2) == 0
  assert cluster.emit(10, 3) == 0
  cluster.navigation.active = True
  cluster.control.enabled = True
  assert cluster.emit(15, 3) == 0
  assert cluster.emit(20, 4) == 10


@pytest.mark.parametrize('alert', (2, 7, 10, 18, 21))
def test_existing_alert_keeps_priority_until_cleared(cluster, alert):
  cluster.stock['ALERTS_3'] = alert
  assert cluster.emit(5, 2) == alert
  assert cluster.emit(10, 3) == alert
  cluster.stock['ALERTS_3'] = 0
  assert cluster.emit(15, 3) == 9


@pytest.mark.parametrize('bars', (0, 5))
def test_invalid_gap_does_not_generate_distance_alert(cluster, bars):
  assert cluster.emit(5, 2) == 0
  assert cluster.emit(10, bars) == 0


def test_started_popup_finishes_its_timer_after_hdp_turns_off(cluster):
  assert cluster.emit(5, 2) == 0
  assert cluster.emit(10, 3) == 9
  cluster.navigation.active = False
  assert cluster.emit(15, 3) == 9
  assert cluster.emit(205, 3) == 9
  assert cluster.emit(210, 3) == 0
