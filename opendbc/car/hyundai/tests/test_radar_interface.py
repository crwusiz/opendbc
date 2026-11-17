import math
import unittest
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd, radar_interface
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiExFlags


class TestHyundaiRadarInterface(unittest.TestCase):
  def setUp(self):
    self.nanos = 1_000_000_000
    self.packers = {}

  def _interface(self, *, tracks=True, canfd=False, group1=False, unavailable=False):
    car = CAR.KIA_EV6 if canfd else CAR.HYUNDAI_SONATA_DN8
    cp = structs.CarParams(carFingerprint=str(car), flags=int(HyundaiFlags.CANFD) if canfd else 0,
                          exFlags=int(HyundaiExFlags.RADAR_GROUP1) if group1 else 0,
                          radarUnavailable=unavailable, safetyConfigs=[{}])
    with patch.object(radar_interface, 'Params') as params, patch.object(hyundaicanfd, 'Params') as bus_params:
      params.return_value.get_bool.side_effect = lambda key: tracks and key == 'RadarTrackEnable'
      bus_params.return_value.get_bool.return_value = False
      return radar_interface.RadarInterface(cp)

  def _message(self, parser, name, values):
    if parser.dbc_name not in self.packers:
      self.packers[parser.dbc_name] = CANPacker(parser.dbc_name)
    return self.packers[parser.dbc_name].make_can_msg(name, parser.bus, values)

  def _send(self, interface, messages=()):
    self.nanos += 10_000_000
    return interface.update([(self.nanos, list(messages))])

  def _tracks(self, interface, values):
    return [self._message(interface.rcp, f'RADAR_TRACK_{addr:x}', values if addr == interface.radar_start_addr else {})
            for addr in range(interface.radar_start_addr, interface.radar_start_addr + interface.radar_msg_count)]

  def _scc_message(self, interface, *, distance=40, speed=-2, status=1):
    if interface.canfd:
      name, values = 'SCC_CONTROL', {'ObjectDistance': distance, 'ObjectRelativeSpeed': speed}
    else:
      name, values = 'SCC11', {'ACC_ObjDist': distance, 'ACC_ObjRelSpd': speed, 'ACC_ObjLatPos': 2,
                              'ACC_ObjStatus': status}
    return self._message(interface.rcp_scc, name, values)

  def _scc_cycle(self, interface, **values):
    for _ in range(4):
      self.assertIsNone(self._send(interface, [self._scc_message(interface, **values)]))
    return self._send(interface, [self._scc_message(interface, **values)])

  def test_unavailable_radar_skips_parsers_and_publishes_at_20_hz(self):
    for tracks in (False, True):
      for canfd in (False, True):
        with self.subTest(tracks=tracks, canfd=canfd):
          with patch.object(radar_interface, 'get_radar_can_parser') as tracks_parser, \
               patch.object(radar_interface, 'get_radar_can_parser_scc') as scc_parser:
            interface = self._interface(tracks=tracks, canfd=canfd, unavailable=True)
          tracks_parser.assert_not_called()
          scc_parser.assert_not_called()
          self.assertIsNone(interface.rcp)
          self.assertIsNone(interface.rcp_scc)
          for frame in range(1, 11):
            result = self._send(interface)
            if frame % 5:
              self.assertIsNone(result)
            else:
              self.assertIsNotNone(result)
              self.assertEqual(len(result.points), 0)
              self.assertFalse(result.errors.canError)
          self.assertEqual(interface.frame, 10)

  def test_classic_tracks_accumulate_until_trigger_and_clear_invalid_points(self):
    interface = self._interface()
    messages = self._tracks(interface, {'STATE': 3, 'LONG_DIST': 40, 'AZIMUTH': 10, 'REL_SPEED': -2})
    self.assertIsNone(self._send(interface, messages[:-1]))
    self.assertIn(interface.radar_start_addr, interface.updated_messages)
    result = self._send(interface, messages[-1:])
    self.assertFalse(result.errors.canError)
    self.assertFalse(interface.updated_messages)
    self.assertEqual(len(result.points), 1)
    point = result.points[0]
    self.assertEqual(point.trackId, 32)
    self.assertAlmostEqual(point.dRel, math.cos(math.radians(10)) * 40, places=5)
    self.assertAlmostEqual(point.yRel, -0.5 * math.sin(math.radians(10)) * 40, places=5)
    self.assertAlmostEqual(point.vRel, -2)

    result = self._send(interface, self._tracks(interface, {'STATE': 0}))
    self.assertEqual(len(result.points), 0)

  def test_canfd_track_groups_decode_and_clear_invalid_points(self):
    for group1 in (False, True):
      with self.subTest(group1=group1):
        interface = self._interface(canfd=True, group1=group1)
        if group1:
          values = {'VALID_CNT1': 11, 'LONG_DIST1': 40, 'LAT_DIST1': -2, 'REL_SPEED1': -1,
                    'VALID_CNT2': 11, 'LONG_DIST2': 60, 'LAT_DIST2': 3, 'REL_SPEED2': 2}
          expected = {32: (40, -2, -1), 48: (60, 3, 2)}
        else:
          values = {'VALID_CNT': 11, 'LONG_DIST': 40, 'LAT_DIST': -2, 'REL_SPEED': -1}
          expected = {32: (40, -2, -1)}
        result = self._send(interface, self._tracks(interface, values))
        self.assertFalse(result.errors.canError)
        points = {point.trackId: point for point in result.points}
        self.assertEqual(points.keys(), expected.keys())
        for track_id, (distance, lateral, speed) in expected.items():
          self.assertAlmostEqual(points[track_id].dRel, distance)
          self.assertAlmostEqual(points[track_id].yRel, lateral)
          self.assertAlmostEqual(points[track_id].vRel, speed)

        invalid = {'VALID_CNT1': 10, 'VALID_CNT2': 10} if group1 else {'VALID_CNT': 10}
        result = self._send(interface, self._tracks(interface, invalid))
        self.assertEqual(len(result.points), 0)

  def test_scc_only_cadence_and_filters(self):
    for canfd in (False, True):
      with self.subTest(canfd=canfd):
        interface = self._interface(tracks=False, canfd=canfd)
        self.assertIsNone(interface.rcp)
        self.assertEqual(len(self._scc_cycle(interface).points), 0)
        result = self._scc_cycle(interface)
        self.assertFalse(result.errors.canError)
        self.assertEqual(len(result.points), 1)
        point = result.points[0]
        self.assertEqual(point.trackId, 0)
        self.assertAlmostEqual(point.dRel, 40)
        self.assertAlmostEqual(point.yRel, 0 if canfd else -2)
        self.assertAlmostEqual(point.vRel, -2)

        self.assertEqual(len(self._scc_cycle(interface, distance=44).points), 0)
        self.assertEqual(len(self._scc_cycle(interface, distance=44).points), 1)
        self.assertEqual(len(self._scc_cycle(interface, distance=44, speed=0).points), 0)
        self.assertEqual(len(self._scc_cycle(interface, distance=44, speed=0).points), 1)
        self.assertEqual(len(self._scc_cycle(interface, distance=150, speed=0).points), 0)
        self.assertEqual(len(self._scc_cycle(interface, distance=150, speed=0).points), 0)
        if not canfd:
          self.assertEqual(len(self._scc_cycle(interface, distance=44, speed=0, status=0).points), 0)

  def test_missing_can_messages_report_errors(self):
    for tracks in (False, True):
      with self.subTest(tracks=tracks):
        interface = self._interface(tracks=tracks)
        if tracks:
          result = self._send(interface, [self._message(interface.rcp, interface.trigger_msg, {})])
        else:
          for _ in range(4):
            self.assertIsNone(self._send(interface))
          result = self._send(interface)
        self.assertIsNotNone(result)
        self.assertTrue(result.errors.canError)

  def test_tracks_publish_together_with_scc(self):
    for canfd in (False, True):
      with self.subTest(canfd=canfd):
        interface = self._interface(canfd=canfd)
        values = ({'VALID_CNT': 11, 'LONG_DIST': 45, 'LAT_DIST': -2, 'REL_SPEED': -1} if canfd else
                  {'STATE': 3, 'LONG_DIST': 45, 'AZIMUTH': 0, 'REL_SPEED': -1})
        for expected_ids in ({32}, {0, 32}):
          messages = self._tracks(interface, values) + [self._scc_message(interface)]
          result = self._send(interface, messages)
          self.assertFalse(result.errors.canError)
          self.assertEqual({point.trackId for point in result.points}, expected_ids)
          self.assertFalse(interface.updated_scc)

  def test_update_callback_returns_radar_data(self):
    interface = self._interface()
    messages = self._tracks(interface, {'STATE': 4, 'LONG_DIST': 40, 'REL_SPEED': -2})
    interface.rcp.update([(self.nanos, messages)])
    result = interface._update(interface.rcp, {interface.trigger_msg})
    self.assertIsNotNone(result)
    self.assertFalse(result.errors.canError)
    self.assertEqual([point.trackId for point in result.points], [32])

  def test_scc_parser_updates_once_while_waiting_for_tracks(self):
    interface = self._interface()
    with patch.object(interface.rcp_scc, 'update', wraps=interface.rcp_scc.update) as update_scc:
      for _ in range(5):
        message = self._message(interface.rcp_scc, 'SCC11', {'ACC_ObjDist': 40, 'ACC_ObjStatus': 1})
        self.assertIsNone(self._send(interface, [message]))
    self.assertEqual(update_scc.call_count, 5)


if __name__ == '__main__':
  unittest.main()
