import math
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExFlags


@pytest.fixture
def cluster(monkeypatch):
  params = {'LaneLineCheck': 0}
  monkeypatch.setattr(hyundaicanfd, 'Params', lambda: SimpleNamespace(get=lambda key, **kwargs: params[key]))
  monkeypatch.setattr(hyundaicanfd.SpeedLimiter, 'instance',
                      lambda: SimpleNamespace(recv=lambda: None, get_active=lambda: False))
  monkeypatch.delattr(hyundaicanfd.create_adrv_messages, '_lane_line_check', raising=False)
  packer = CANPacker('hyundai_canfd_generated')
  definition = packer.dbc.name_to_msg['CCNC_0x161']
  stock = {key: 0 for key in definition.sigs}
  out = structs.CarState(leftLaneLine=10, rightLaneLine=11)
  out.cruiseState.available = True
  state = SimpleNamespace(out=out, ccnc_msg_161=stock, ccnc_msg_162=None, adrv_msg_160=None,
                          adrv_msg_1ea=None, lfahda_cluster_info=None, cruise_buttons_msg=None)
  control = structs.CarControl(enabled=True, latActive=True)
  control.hudControl.leftLaneVisible = True
  control.hudControl.rightLaneVisible = True
  model = log.ModelDataV2.new_message(laneLineProbs=[0., 0.9, 0.9, 0.])
  return SimpleNamespace(packer=packer, definition=definition, stock=stock, original=stock.copy(),
                         state=state, control=control, model=model, params=params)


def _lane_values(cluster, frame=5):
  cp = structs.CarParams(flags=int(HyundaiFlags.CANFD_CAMERA_SCC), exFlags=int(HyundaiExFlags.CCNC))
  messages = hyundaicanfd.create_adrv_messages(cluster.packer, cp, cluster.control, cluster.state,
                                             cluster.model, SimpleNamespace(ECAN=0, CAM=2), frame, 80,
                                             cluster.control.hudControl)
  assert len(messages) == 1
  address, data, bus = messages[0]
  assert (address, bus) == (0x161, 0)
  assert cluster.stock == cluster.original
  assert cluster.definition.sigs['CHECKSUM'].get_raw_value(data) == hyundaicanfd.hkg_can_fd_checksum(address, None, bytearray(data))
  return tuple(cluster.definition.sigs[key].get_raw_value(data) for key in ('LANELINE_LEFT', 'LANELINE_RIGHT'))


@pytest.mark.parametrize('line,available,expected', [
  (10, False, 8), (10, True, 10), (15, False, 8), (15, True, 10),
  (11, False, 2), (11, True, 6), (14, False, 2), (14, True, 6),
  (12, False, 2), (12, True, 6), (30, False, 8), (30, True, 10),
])
def test_known_lane_type_selects_solid_or_dashed_style(cluster, line, available, expected):
  cluster.state.out.leftLaneLine = line
  cluster.model.meta.laneChangeAvailableLeft = available
  assert _lane_values(cluster) == (expected, 2)


def test_left_and_right_lane_styles_are_independent(cluster):
  cluster.state.out.leftLaneLine = 11
  cluster.state.out.rightLaneLine = 15
  cluster.model.meta.laneChangeAvailableLeft = True
  cluster.model.meta.laneChangeAvailableRight = True
  assert _lane_values(cluster) == (6, 10)
  cluster.state.out.rightBlindspot = True
  assert _lane_values(cluster) == (6, 4)


@pytest.mark.parametrize('lane_line,lane_prob,visible', [
  (-1, 0.9, True), (10, 0., True), (10, 0.5, True), (10, 0.9, False),
  (0, 0., True), (10, float('nan'), True), (10, float('inf'), True),
])
def test_missing_lane_is_hidden_even_with_stale_warning(cluster, lane_line, lane_prob, visible):
  cluster.state.out.leftLaneLine = lane_line
  cluster.model.laneLineProbs = [0., lane_prob, 0.9, 0.]
  cluster.model.meta.laneChangeAvailableLeft = True
  cluster.state.out.leftBlindspot = True
  cluster.control.hudControl.leftLaneVisible = visible
  cluster.control.hudControl.leftLaneDepart = True
  assert _lane_values(cluster, 5) == (1, 2)


def test_model_visibility_without_oem_type_uses_solid_style(cluster):
  cluster.state.out.leftLaneLine = 0
  assert _lane_values(cluster) == (2, 2)


def test_oem_detection_is_used_without_model_data(cluster):
  cluster.model = None
  assert _lane_values(cluster) == (8, 2)
  cluster.state.out.leftLaneLine = 0
  cluster.state.out.rightLaneLine = 0
  assert _lane_values(cluster) == (1, 1)


def test_missing_lane_releases_previous_dashed_style(cluster):
  cluster.model.meta.laneChangeAvailableLeft = True
  assert _lane_values(cluster) == (10, 2)
  cluster.model.laneLineProbs = [0., 0., 0.9, 0.]
  assert _lane_values(cluster) == (1, 2)
  cluster.model.laneLineProbs = [0., 0.9, 0.9, 0.]
  cluster.state.out.leftLaneLine = 11
  assert _lane_values(cluster) == (6, 2)


def test_visible_lane_retains_departure_blink(cluster):
  cluster.control.hudControl.leftLaneDepart = True
  assert _lane_values(cluster, 5) == (4, 2)
  assert _lane_values(cluster, 55) == (1, 2)


def test_lane_warning_overrides_dashed_style(cluster):
  cluster.state.out.leftLaneLine = 20
  cluster.model.meta.laneChangeAvailableLeft = True
  assert _lane_values(cluster) == (4, 2)


def test_type_check_mode_allows_green_dashed_yellow_line(cluster):
  cluster.params['LaneLineCheck'] = 1
  cluster.state.out.leftLaneLine = 20
  cluster.model.meta.laneChangeAvailableLeft = True
  assert _lane_values(cluster) == (10, 4)


@pytest.mark.parametrize('message,probability_signal', [('CAM_0x362', 'LANE_PROB'), ('CAM_0x2a4', 'LANE_LINE_PROB')])
@pytest.mark.parametrize('probability,color,lane_type,expected', [(31, 1, 0, 10), (31, 3, 5, 35),
                                                               (0, 1, 0, -1), (31, 0, 0, -1)])
def test_camera_lane_decoding_preserves_detection_and_type(message, probability_signal, probability, color, lane_type, expected):
  packer = CANPacker('hyundai_canfd_generated')
  parser = CANParser('hyundai_canfd_generated', [(message, math.nan)], 0)
  values = {f'LEFT_{probability_signal}': probability, 'LEFT_LANE_COLOR': color, 'LEFT_LANE_TYPE': lane_type,
            f'RIGHT_{probability_signal}': 0, 'RIGHT_LANE_COLOR': 1, 'RIGHT_LANE_TYPE': 1}
  frame = packer.make_can_msg(message, 0, values)
  assert parser.update([(1_000_000_000, [frame])])
  assert CarState._get_lane_line_info(parser.vl[message], 'LEFT') == expected
  assert CarState._get_lane_line_info(parser.vl[message], 'RIGHT') == -1
