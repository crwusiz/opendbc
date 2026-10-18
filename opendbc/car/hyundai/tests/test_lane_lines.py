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
                          adrv_msg_1ea=None, lfahda_cluster_info=None, cruise_buttons_msg=None,
                          left_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                          right_lane_line_warning=hyundaicanfd.LaneLineWarning(),
                          left_lane_line_visibility=hyundaicanfd.LaneLineVisibility(),
                          right_lane_line_visibility=hyundaicanfd.LaneLineVisibility())
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
def test_warning_is_preserved_even_when_lane_is_unseen(cluster, lane_line, lane_prob, visible):
  cluster.state.out.leftLaneLine = lane_line
  cluster.model.laneLineProbs = [0., lane_prob, 0.9, 0.]
  cluster.model.meta.laneChangeAvailableLeft = True
  cluster.state.out.leftBlindspot = True
  cluster.control.hudControl.leftLaneVisible = visible
  assert _lane_values(cluster, 5) == (4, 2)
  assert _lane_values(cluster, 105) == (4, 2)


@pytest.mark.parametrize('probability', [float('nan'), float('inf'), -0.1, 1.1])
def test_invalid_probability_without_warning_hides_lane(cluster, probability):
  cluster.model.laneLineProbs = [0., probability, 0.9, 0.]
  assert _lane_values(cluster) == (14, 2)


def test_hud_invisibility_without_warning_uses_ordinary_hidden_value(cluster):
  cluster.control.hudControl.leftLaneVisible = False
  assert _lane_values(cluster) == (14, 2)


def test_model_visibility_without_oem_type_uses_solid_style(cluster):
  cluster.state.out.leftLaneLine = 0
  assert _lane_values(cluster) == (2, 2)


def test_oem_detection_is_used_without_model_data(cluster):
  cluster.model = None
  assert _lane_values(cluster) == (8, 2)
  cluster.state.out.leftLaneLine = 0
  cluster.state.out.rightLaneLine = 0
  assert _lane_values(cluster) == (14, 14)


def test_low_probability_requires_one_second_before_hiding(cluster):
  assert _lane_values(cluster, 0) == (8, 2)
  cluster.model.laneLineProbs = [0., 0., 0.9, 0.]
  for frame in range(5, 105, 5):
    assert _lane_values(cluster, frame) == (8, 2)
  assert _lane_values(cluster, 105) == (14, 2)
  cluster.model.laneLineProbs = [0., 0.1, 0.9, 0.]
  assert _lane_values(cluster, 110) == (14, 2)
  cluster.model.laneLineProbs = [0., 0.12, 0.9, 0.]
  assert _lane_values(cluster, 115) == (8, 2)


def test_visibility_band_preserves_state_and_restarts_low_timer(cluster):
  cluster.model.laneLineProbs = [0., 0.05, 0.9, 0.]
  assert _lane_values(cluster, 5) == (8, 2)
  assert _lane_values(cluster, 100) == (8, 2)
  cluster.model.laneLineProbs = [0., 0.08, 0.9, 0.]
  assert _lane_values(cluster, 105) == (8, 2)
  cluster.model.laneLineProbs = [0., 0.05, 0.9, 0.]
  assert _lane_values(cluster, 110) == (8, 2)
  assert _lane_values(cluster, 205) == (8, 2)
  assert _lane_values(cluster, 210) == (14, 2)


def test_visibility_timers_are_independent(cluster):
  cluster.model.laneLineProbs = [0., 0.05, 0.9, 0.]
  assert _lane_values(cluster, 5) == (8, 2)
  cluster.model.laneLineProbs = [0., 0.05, 0.05, 0.]
  assert _lane_values(cluster, 20) == (8, 2)
  assert _lane_values(cluster, 105) == (14, 2)
  assert _lane_values(cluster, 120) == (14, 14)


@pytest.mark.parametrize('mode', [0, 1])
def test_unknown_oem_line_does_not_create_warning_or_green(cluster, mode):
  cluster.params['LaneLineCheck'] = mode
  cluster.state.out.leftLaneLine = -1
  cluster.model.meta.laneChangeAvailableLeft = True
  assert _lane_values(cluster)[0] == 2


def test_visible_lane_retains_departure_blink(cluster):
  cluster.control.hudControl.leftLaneDepart = True
  assert _lane_values(cluster, 5) == (4, 2)
  assert _lane_values(cluster, 55) == (1, 2)


def test_departure_warning_blinks_even_when_lane_is_unseen(cluster):
  cluster.model.laneLineProbs = [0., 0., 0.9, 0.]
  cluster.control.hudControl.leftLaneVisible = False
  cluster.control.hudControl.leftLaneDepart = True
  assert _lane_values(cluster, 105) == (4, 2)
  assert _lane_values(cluster, 155) == (1, 2)


def test_lane_warning_overrides_dashed_style(cluster):
  cluster.state.out.leftLaneLine = 20
  cluster.model.meta.laneChangeAvailableLeft = True
  assert _lane_values(cluster) == (4, 2)


def test_type_check_mode_allows_green_dashed_yellow_line(cluster):
  cluster.params['LaneLineCheck'] = 1
  cluster.state.out.leftLaneLine = 20
  cluster.model.meta.laneChangeAvailableLeft = True
  assert _lane_values(cluster) == (10, 4)


@pytest.mark.parametrize('mode,warning_line,blindspot,clear_line,available,expected', [
  (0, 21, False, 11, True, 6), (0, 21, False, 11, False, 2),
  (0, 21, False, 10, True, 10), (0, 21, False, 10, False, 8),
  (0, 11, True, 11, True, 6), (0, 10, True, 10, True, 10),
  (1, 21, False, 20, True, 10), (1, 11, False, 10, False, 8),
])
def test_warning_is_immediate_and_requires_700ms_clear(cluster, mode, warning_line, blindspot, clear_line, available, expected):
  cluster.params['LaneLineCheck'] = mode
  cluster.model.meta.laneChangeAvailableLeft = available
  cluster.state.out.leftLaneLine = clear_line
  assert _lane_values(cluster, 0)[0] == expected

  cluster.state.out.leftLaneLine = warning_line
  cluster.state.out.leftBlindspot = blindspot
  assert _lane_values(cluster, 5)[0] == 4

  cluster.state.out.leftLaneLine = clear_line
  cluster.state.out.leftBlindspot = False
  for frame in range(10, 80, 5):
    assert _lane_values(cluster, frame)[0] == 4
  assert _lane_values(cluster, 80)[0] == expected
  assert cluster.state.out.leftLaneLine == clear_line
  assert not cluster.state.out.leftBlindspot
  assert cluster.model.meta.laneChangeAvailableLeft == available


def test_recurring_warning_restarts_clear_timer(cluster):
  cluster.model.meta.laneChangeAvailableLeft = True
  cluster.state.out.leftLaneLine = 21
  assert _lane_values(cluster, 5) == (4, 2)
  cluster.state.out.leftLaneLine = 10
  for frame in range(10, 65, 5):
    assert _lane_values(cluster, frame) == (4, 2)

  cluster.state.out.leftBlindspot = True
  assert _lane_values(cluster, 65) == (4, 2)
  cluster.state.out.leftBlindspot = False
  for frame in range(70, 140, 5):
    assert _lane_values(cluster, frame) == (4, 2)
  assert _lane_values(cluster, 140) == (10, 2)


def test_left_and_right_warning_clear_timers_are_independent(cluster):
  cluster.model.meta.laneChangeAvailableLeft = True
  cluster.model.meta.laneChangeAvailableRight = True
  cluster.state.out.leftLaneLine = 20
  cluster.state.out.rightLaneLine = 21
  assert _lane_values(cluster, 5) == (4, 4)

  cluster.state.out.leftLaneLine = 10
  for frame in range(10, 105, 5):
    if frame == 30:
      cluster.state.out.rightLaneLine = 11
    assert _lane_values(cluster, frame) == (4 if frame < 80 else 10, 4 if frame < 100 else 6)


def test_all_warning_sources_must_clear_before_release_timer_starts(cluster):
  cluster.model.meta.laneChangeAvailableLeft = True
  cluster.state.out.leftLaneLine = 21
  assert _lane_values(cluster, 5) == (4, 2)
  cluster.state.out.leftLaneLine = 10
  cluster.state.out.leftBlindspot = True
  for frame in range(10, 80, 5):
    assert _lane_values(cluster, frame) == (4, 2)
  cluster.state.out.leftBlindspot = False
  for frame in range(80, 150, 5):
    assert _lane_values(cluster, frame) == (4, 2)
  assert _lane_values(cluster, 150) == (10, 2)


@pytest.mark.parametrize('missing_source', ['camera', 'model', 'hud'])
def test_missing_lane_does_not_suppress_warning_hold(cluster, missing_source):
  cluster.model.meta.laneChangeAvailableLeft = True
  cluster.state.out.leftLaneLine = 21
  assert _lane_values(cluster, 5) == (4, 2)
  cluster.state.out.leftLaneLine = 10
  assert _lane_values(cluster, 10) == (4, 2)

  if missing_source == 'camera':
    cluster.state.out.leftLaneLine = -1
  elif missing_source == 'model':
    cluster.model.laneLineProbs = [0., 0., 0.9, 0.]
  else:
    cluster.control.hudControl.leftLaneVisible = False
  assert _lane_values(cluster, 15) == (4, 2)

  cluster.state.out.leftLaneLine = 10
  cluster.model.laneLineProbs = [0., 0.9, 0.9, 0.]
  cluster.control.hudControl.leftLaneVisible = True
  assert _lane_values(cluster, 20) == (4, 2)
  for frame in range(25, 80, 5):
    assert _lane_values(cluster, frame) == (4, 2)
  assert _lane_values(cluster, 80) == (10, 2)


def test_warning_clear_delay_expires_into_hidden_lane(cluster):
  cluster.state.out.leftLaneLine = 21
  cluster.model.laneLineProbs = [0., 0., 0.9, 0.]
  assert _lane_values(cluster, 5) == (4, 2)
  assert _lane_values(cluster, 105) == (4, 2)
  cluster.state.out.leftLaneLine = 10
  for frame in range(110, 180, 5):
    assert _lane_values(cluster, frame) == (4, 2)
  assert _lane_values(cluster, 180) == (14, 2)


def test_model_availability_and_cluster_colors_follow_warning_and_detection_timers(cluster, monkeypatch):
  from openpilot.selfdrive.controls.lib import desire_helper

  monkeypatch.setattr(desire_helper, 'Params', lambda: SimpleNamespace(get=lambda *args, **kwargs: 0,
                                                                      get_bool=lambda *args: False))
  helper = desire_helper.DesireHelper()
  radar = log.RadarState.new_message()
  cluster.state.out.vEgo = 20.
  cluster.model.laneLineProbs = [0.9] * 4
  cluster.model.roadEdgeStds = [1., 1.]
  for field, positions in (('laneLines', (-5.4, -1.8, 1.8, 5.4)), ('roadEdges', (-8., 8.))):
    for line, y in zip(cluster.model.init(field, len(positions)), positions, strict=True):
      line.x, line.y = [0., 30., 60.], [y] * 3

  def step(frame):
    model_data = {key: getattr(cluster.model, key) for key in ('laneLineProbs', 'roadEdgeStds', 'laneLines', 'roadEdges')}
    helper.update(cluster.state.out, True, 0., model_data, radar)
    cluster.model.meta.laneChangeAvailableLeft = helper.lane_change_available_left
    cluster.model.meta.laneChangeAvailableRight = helper.lane_change_available_right
    return _lane_values(cluster, frame)[0]

  for frame in range(0, 100, 5):
    color = step(frame)
  assert color == 10
  cluster.state.out.leftBlindspot = True
  assert step(100) == 4
  cluster.state.out.leftBlindspot = False
  for frame in range(105, 300, 5):
    assert step(frame) == (4 if frame < 175 else 8)
  assert step(300) == 10

  radar.leadsLeft = [{'present': True, 'dRel': 4., 'vRel': 0.}]
  assert step(305) == 8
  radar.leadsLeft = []
  for frame in range(310, 335, 5):
    assert step(frame) == 8
  assert step(335) == 10

  cluster.model.laneLineProbs = [0.9, 0., 0.9, 0.9]
  for frame in range(340, 440, 5):
    assert step(frame) == 8
  assert step(440) == 14
  cluster.model.laneLineProbs = [0.9, 0.1, 0.9, 0.9]
  assert step(445) == 14
  cluster.model.laneLineProbs = [0.9, 0.12, 0.9, 0.9]
  assert step(450) == 8


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
