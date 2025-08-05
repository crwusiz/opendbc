import copy
import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExFlags

from openpilot.common.params import Params
from openpilot.selfdrive.controls.neokii.navi_controller import SpeedLimiter
from opendbc.car.common.conversions import Conversions as CV


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering and not Params().get_bool("CameraSccEnable"):  #배선개조는 무조건 Bus0가 ECAN임.
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CC, CS, CAN, frame, lat_active, apply_torque, apply_angle, angle_max_torque):
  enabled = CC.enabled
  angle_control = CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
  camera_scc = CP.flags & HyundaiFlags.CANFD_CAMERA_SCC

  common_values = {
    "LKA_MODE": 2,
    "LKA_ICON": 2 if enabled else 1,
    "TORQUE_REQUEST": apply_torque,
    "LKA_ASSIST": 0,
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_MODE": 0,
    "HAS_LANE_SAFETY": 0,  # hide LKAS settings
    "NEW_SIGNAL_2": 0,
    "DAMP_FACTOR": 100,  # can potentially tuned for better perf [3, 200]
    "LKA_AVAILABLE": 0,
  }

  lfa_values = copy.copy(common_values)

  lkas_values = copy.copy(common_values)

  # For cars with an ADAS ECU (commonly HDA2), by sending LKAS actuation messages we're
  # telling the ADAS ECU to forward our steering and disable stock LFA lane centering.
  ret = []

  values = copy.copy(CS.mdps_info)
  if angle_control:
    if CS.lfa_alt_info is not None:
      values["ADAS_ActiveStat_Lv2"] = CS.lfa_alt_info["ADAS_AngleActiveStat_Lv2"]
  else:
    if CS.lfa_info is not None:
      values["LKA_ACTIVE"] = 1 if CS.lfa_info["STEER_REQ"] == 1 else 0

  if frame % 1000 < 40:
    values["OutTorque"] += 220
  ret.append(packer.make_can_msg("MDPS", CAN.CAM, values))

  if frame % 10 == 0:
    if CP.exFlags & HyundaiExFlags.HOD:
      values = copy.copy(CS.hod_info)
      if frame % 1000 < 40:
        values["TOUCH_DETECT"] = 3
        values["TOUCH1"] = 50
        values["TOUCH2"] = 50
        values["CHECKSUM_"] = 0
        dat = packer.make_can_msg("HANDS_ON_DETECTION", 0, values)[1]
        values["CHECKSUM_"] = hyundai_crc8(dat[1:8])

      ret.append(packer.make_can_msg("HANDS_ON_DETECTION", CAN.CAM, values))

  if angle_control:
    if camera_scc:
      lfa_values |= {
        "LKA_MODE": 0,  # TODO: not used by the stock system
        "TORQUE_REQUEST": 0,  # we don't use torque
        "STEER_REQ": 0,  # we don't use torque
        # this goes 0 when LFA lane changes, 3 when LKA_ICON is >=green
        "LKA_AVAILABLE": 3 if lat_active else 0,
        #"ADAS_AngleReq": 0,
        #"ADAS_AngleActiveStat_Lv2": 0,
        #"ADAS_AngleTorqueGain": 0,
      }

      values = {
        "ADAS_AngleReq": apply_angle,
        "ADAS_AngleActiveStat_Lv2": 2 if lat_active else 1,
        "ADAS_AngleTorqueGain": angle_max_torque if lat_active else 0,
      }
      ret.append(packer.make_can_msg("LFA_ALT", CAN.ECAN, values))

    else:
      lkas_values |= {
        "LKA_MODE": 0,  # TODO: not used by the stock system
        "TORQUE_REQUEST": 0,  # we don't use torque
        "STEER_REQ": 0,  # we don't use torque
        # this goes 0 when LFA lane changes, 3 when LKA_ICON is >=green
        "LKA_AVAILABLE": 3 if lat_active else 0,
        "ADAS_AngleReq": apply_angle,
        "ADAS_AngleActiveStat_Lv2": 2 if lat_active else 1,
        "ADAS_AngleTorqueGain": angle_max_torque if lat_active else 0,
      }

    if CP.flags & HyundaiFlags.CANFD_LKA_STEERING:
      lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT else "LKAS"
      if CP.openpilotLongitudinalControl:
        ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))
      if not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC):
        ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, lkas_values))
    else:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))

  return ret


def create_suppress_lfa(packer, CP, CC, CS, CAN):
  enabled = CC.enabled
  #lfa_block_msg = CS.lfa_block_msg
  #lka_steering_alt = CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT
  #suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"
  #msg_bytes = 32 if lka_steering_alt else 24
  #values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}

  if CS.msg_0x362 is not None:
    suppress_msg = "CAM_0x362"
    lfa_block_msg = CS.msg_0x362
  elif CS.msg_0x2a4 is not None:
    suppress_msg = "CAM_0x2a4"
    lfa_block_msg = CS.msg_0x2a4
  else:
    return []

  values = copy.copy(lfa_block_msg)
  values["COUNTER"] = lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0 if enabled else 3
  values["RIGHT_LANE_LINE"] = 0 if enabled else 3
  return [packer.make_can_msg(suppress_msg, CAN.ACAN, values)]


def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def create_buttons_canfd_alt(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt % 256,
    "CRUISE_BUTTONS": btn,
  }
  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS_ALT", bus, values)


def create_acc_cancel(packer, CP, CS, CAN):
  cruise_info_copy = CS.cruise_info
  camera_scc = CP.flags & HyundaiFlags.CANFD_CAMERA_SCC

  # TODO: why do we copy different values here?
  if camera_scc:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "SysFailStat",
      "MainStat",
      "OperationStat",
      "TakeoverReq",
      "InfoDisplay",
      "AlertDisplay",
      "DistanceGapSet",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "OperationStat",
      "VSetDis",
      "InfoDisplay",
    ]}
  values.update({
    "OperationStat": 4,
    "AccelRequestRaw": 0.0,
    "AccelRequest": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_lfahda_cluster(packer, CC, CS, CAN):
  if CS.lfahda_cluster_info is not None:
    values = {}
    values["HDA_ICON"] = 1 if CC.long_active else 0
    values["LFA_ICON"] = 2 if CC.lat_active else 0
  else:
    return []
  return [packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)]


def create_acc_control(packer, CP, CC, CS, CAN, accel_last, accel, stopping, set_speed, hud):
  enabled = CC.enabled
  gas_override = CC.cruiseControl.override
  camera_scc = CP.flags & HyundaiFlags.CANFD_CAMERA_SCC

  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)

  if camera_scc:
    values = copy.copy(CS.cruise_info)
    values |= {
      "OperationStat": 0 if not enabled else (2 if gas_override else 1),
      "MainStat": 1,
      "StopReq": 1 if stopping else 0,
      "AccelRequest": a_val,
      "AccelRequestRaw": a_raw,
      "VSetDis": set_speed,
      "JerkUpperLimit": jerk if enabled else 1,
      "JerkLowerLimit": 3.0,

      #"ObjectDistance": 1,
      "NSCC_MainStat": 2,
      "SET_ME_TMP_64": 0x64,
      "DistanceGapSet": hud.leadDistanceBars,
      "InfoDisplay": 4 if stopping and CS.out.aEgo > -0.3 else 0,

      "TargetDistance": CS.out.vEgo + 4.0,
      "AlertDisplay": 0,
      "TakeoverReq": 0,
      "AccelLimitBandUpper": 0,
      "AccelLimitBandLower": 0,
      "SysFailStat": 0,
    }

    hud_lead_info = 0
    if hud.leadVisible:
      hud_lead_info = 1 if values["ObjectRelativeSpeed"] > 0 else 2
    values["ObjectStat"] = hud_lead_info

  else:
    values = {
      "OperationStat": 0 if not enabled else (2 if gas_override else 1),
      "MainStat": 1,
      "StopReq": 1 if stopping else 0,
      "AccelRequest": a_val,
      "AccelRequestRaw": a_raw,
      "VSetDis": set_speed,
      "JerkUpperLimit": jerk if enabled else 1,
      "JerkLowerLimit": 3.0,

      "ObjectStat": 2,
      "NSCC_MainStat": 2,
      "ObjectRelativeSpeed": 0,
      "SET_ME_TMP_64": 0x64,
      "DistanceGapSet": hud.leadDistanceBars,
      "InfoDisplay": 4 if stopping and CS.out.cruiseState.standstill else 0,
    }

    # fixes auto regen stuck on max for hybrids, should probably apply to all cars
    values.update(
      {"ObjectDistance": 1} if CS.cruise_info is None else {s: CS.cruise_info[s] for s in ["ObjectDistance", "ObjectRelativeSpeed"]})

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CC, CAN):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if CC.leftBlinker:
    blink = 3
  elif CC.rightBlinker:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_fca_warning_light(packer, CP, CAN, frame):
  ret = []
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    return ret

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xFF,
      'SET_ME_FC': 0xFC,
      'SET_ME_9': 0x9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret


def create_adrv_messages(packer, CP, CC, CS, CAN, frame, set_speed, hud):
  main_enabled = CS.out.cruiseState.available
  cruise_enabled = CC.enabled
  lat_active = CC.latActive
  ccnc = CP.exFlags & HyundaiExFlags.CCNC
  nav_active = SpeedLimiter.instance().get_active()
  hdp_active = cruise_enabled and nav_active

  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
    if frame % 5 == 0 and CS.ccnc_msg_161 is not None and ccnc:
      values = copy.copy(CS.ccnc_msg_161)
      values |= {
        "SETSPEED": 6 if hdp_active else 3 if main_enabled else 0,
        "SETSPEED_HUD": 5 if hdp_active else 2 if cruise_enabled else 1,
        "vSetDis": set_speed,

        "DISTANCE": 4 if hdp_active else hud.leadDistanceBars,
        "DISTANCE_LEAD": 2 if cruise_enabled and hud.leadVisible else 0,
        "DISTANCE_CAR": 3 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0,
        "DISTANCE_SPACING": 5 if hdp_active else 1 if cruise_enabled else 0,

        "TARGET": 1 if cruise_enabled else 0,
        "TARGET_DISTANCE": int(hud.leadDistance),

        "BACKGROUND": 1 if cruise_enabled else 3 if main_enabled else 7,
        "CENTERLINE": 1 if lat_active else 0,
        "CAR_CIRCLE": 2 if hdp_active else 1 if lat_active else 0,

        "NAV_ICON": 2 if nav_active else 1,
        "HDA_ICON": 5 if hdp_active else 2 if lat_active else 1,
        "LFA_ICON": 5 if hdp_active else 2 if lat_active else 1,
        "LKA_ICON": 4 if lat_active else 3,
        "FCA_ALT_ICON": 0,
        "DAW_ICON": 0,

        "LCA_LEFT_ARROW": 2 if CS.out.leftBlinker else 0,
        "LCA_RIGHT_ARROW": 2 if CS.out.rightBlinker else 0,

        "LCA_LEFT_ICON": 1 if CS.out.leftBlindspot else 2,
        "LCA_RIGHT_ICON": 1 if CS.out.rightBlindspot else 2,

        "SOUNDS_2": 0,
      }

      alerts_disable_map = {
        "ALERTS_2": [1, 2, 5],
        "ALERTS_3": [17, 26],
        "ALERTS_5": [1, 4, 5],
      }

      for key, reset_values in alerts_disable_map.items():
        if values.get(key) in reset_values:
          values[key] = 0

      curvature = round(CS.out.steeringAngleDeg / 3)

      values["LANELINE_CURVATURE"] = (min(abs(curvature), 15) + (-1 if curvature < 0 else 0)) if lat_active else 0
      values["LANELINE_CURVATURE_DIRECTION"] = 1 if curvature < 0 and lat_active else 0

      def get_lane_value(depart, visible, frame):
        if depart:
          return 4 if (frame // 50) % 2 == 0 else 1
        return 2 if visible else 0

      values["LANELINE_LEFT"] = get_lane_value(hud.leftLaneDepart, hud.leftLaneVisible, frame)
      values["LANELINE_RIGHT"] = get_lane_value(hud.rightLaneDepart, hud.rightLaneVisible, frame)

      """
      if lat_active and (CS.out.leftBlinker or CS.out.rightBlinker):
        msg_1b5 = copy.copy(CS.ccnc_msg_1b5)
        leftlaneraw, rightlaneraw = msg_1b5["LeftLnPosition"], msg_1b5["RightLnPosition"]

        scale_per_m = 15 / 1.7
        leftlane = abs(int(round(15 + (leftlaneraw - 1.7) * scale_per_m)))
        rightlane = abs(int(round(15 + (rightlaneraw - 1.7) * scale_per_m)))

        if msg_1b5["LeftLnQualStat"] not in (2, 3):
          leftlane = 0
        if msg_1b5["RightLnQualStat"] not in (2, 3):
          rightlane = 0

        if leftlaneraw == -2.0248375:
          leftlane = 30 - rightlane
        if rightlaneraw == 2.0248375:
          rightlane = 30 - leftlane

        if leftlaneraw == rightlaneraw == 0:
          leftlane = rightlane = 15
        elif leftlaneraw == 0:
          leftlane = 30 - rightlane
        elif rightlaneraw == 0:
          rightlane = 30 - leftlane

        total = leftlane + rightlane
        if total == 0:
          leftlane = rightlane = 15
        else:
          leftlane = round((leftlane / total) * 30)
          rightlane = 30 - leftlane

        values["LANELINE_LEFT_POSITION"] = leftlane
        values["LANELINE_RIGHT_POSITION"] = rightlane
      """

      ret.append(packer.make_can_msg("CCNC_0x161", CAN.ECAN, values))

    if frame % 5 == 0 and CS.ccnc_msg_162 is not None and ccnc:
      values = copy.copy(CS.ccnc_msg_162)
      for f in {"FAULT_FCA", "FAULT_LSS", "FAULT_DAS", "FAULT_LFA"}:
        values[f] = 0

      sensors = [
        ('ff', 'FF_DETECT'),
        ('lf', 'LF_DETECT'),
        ('rf', 'RF_DETECT'),
        ('lr', 'LR_DETECT'),
        ('rr', 'RR_DETECT')
      ]

      for sensor_key, detect_key in sensors:
        distance = getattr(CS, f"{sensor_key}_distance")
        if distance > 0:
          values[detect_key] = 3 if distance > 30 else 4

      if hud.leftLaneDepart or hud.rightLaneDepart:
        values["VIBRATE"] = 1

      ret.append(packer.make_can_msg("CCNC_0x162", CAN.ECAN, values))

    #if frame % 2 == 0 and CS.adrv_msg_160 is not None:
    #  values = CS.adrv_msg_160
    #  values |= {
    #  }
    #  ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

    if frame % 5 == 0 and CS.adrv_msg_200 is not None:
      values = copy.copy(CS.adrv_msg_200)
      values |= {
        "TauGapSet": hud.leadDistanceBars,
      }
      ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

    if frame % 5 == 0 and CS.adrv_msg_1ea is not None:
      values = copy.copy(CS.adrv_msg_1ea)
      values |= {
        "HDA_MODE1": 0x8,
        "HDA_MODE2": 0x1,
      }
      ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

    if frame % 20 == 0 and CS.hda_msg_4a3 is not None:
      values = copy.copy(CS.hda_msg_4a3)
      values |= {
        "SIGNAL_0": 5,
        "NEW_SIGNAL_1": 4,
        "SPEED_LIMIT": 80,
        "NEW_SIGNAL_3": 154,
        "NEW_SIGNAL_4": 9,
        "NEW_SIGNAL_5": 0,
        "NEW_SIGNAL_6": 256,
      }
      ret.append(packer.make_can_msg("hda_msg_4a3", CAN.CAM, values))

    return ret

  else:

    ret.extend(create_fca_warning_light(packer, CP, CAN, frame))
    if frame % 5 == 0:
      values = {
        'HDA_MODE1': 0x8,
        'HDA_MODE2': 0x1,
        'SET_ME_FF': 0xFF,
        'SET_ME_TMP_F': 0xF,
        'SET_ME_TMP_F_2': 0xf,
      }
      ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

      values = {
        'SET_ME_E1': 0xE1,
        'TauGapSet': 0x1,
        'NEW_SIGNAL_2': 0x3,
      }
      ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

    if frame % 20 == 0:
      values = {
        'SET_ME_15': 0x15,
      }
      ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

    if frame % 100 == 0:
      values = {
        'SET_ME_22': 0x22,
        'SET_ME_41': 0x41,
      }
      ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

    return ret


def hkg_can_fd_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc


def hyundai_crc8(data: bytes) -> int:
  poly = 0x2F
  crc = 0xFF

  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc = (crc << 1) & 0xFF

  return crc ^ 0xFF
