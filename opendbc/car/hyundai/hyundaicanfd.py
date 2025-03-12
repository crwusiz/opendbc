import copy
import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExFlags

from openpilot.common.params import Params
from openpilot.selfdrive.controls.neokii.navi_controller import SpeedLimiter


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering and not Params().get_bool("HyundaiCameraSCC"):  #배선개조는 무조건 Bus0가 ECAN임.
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


def create_steering_messages(packer, CP, CC, CS, CAN, lat_active, apply_torque, apply_angle, angle_max_torque):
  enabled = CC.enabled
  angle_control = CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING

  """
  common_values = {
    "LKA_MODE": 2,
    "LKA_ICON": 2 if enabled else 1,
    "TORQUE_REQUEST": apply_torque,
    "LKA_ASSIST": 0,
    "STEER_REQ": 1 if lat_active else 0,
    "STEER_MODE": 0,
    "HAS_LANE_SAFETY": 0,  # hide LKAS settings
    "NEW_SIGNAL_2": 0,
  }

  lkas_values = copy.copy(common_values)
  lkas_values["LKA_AVAILABLE"] = 0

  # Angle control doesn't support using LFA yet
  if angle_control:
    # TODO: HAS_LANE_SAFETY isn't used by the stock system
    lkas_values |= {
      "LKA_MODE": 0,  # TODO: not used by the stock system
      "TORQUE_REQUEST": 0,  # we don't use torque
      "STEER_REQ": 0,  # we don't use torque
      # this goes 0 when LFA lane changes, 3 when LKA_ICON is >=green
      "LKA_AVAILABLE": 3 if lat_active else 0,
      "LKAS_ANGLE_CMD": apply_angle,
      "LKAS_ANGLE_ACTIVE": 2 if lat_active else 1,
      "LKAS_ANGLE_MAX_TORQUE": angle_max_torque if lat_active else 0,
    }

  lfa_values = copy.copy(common_values)
  lfa_values["NEW_SIGNAL_1"] = 0
  """

  # For cars with an ADAS ECU (commonly HDA2), by sending LKAS actuation messages we're
  # telling the ADAS ECU to forward our steering and disable stock LFA lane centering.
  ret = []

  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    if angle_control:
      apply_angle = np.clip(apply_angle, -119, 119)

      values = {
        "LKAS_ANGLE_ACTIVE": 2 if abs(CS.out.steeringAngleDeg) < 110.0 and lat_active else 1,
        "LKAS_ANGLE_CMD": apply_angle,
        "LKAS_ANGLE_MAX_TORQUE": angle_max_torque if lat_active else 0,
      }
      ret.append(packer.make_can_msg("LFA_ALT", CAN.ECAN, values))

      values = CS.lfa_info
      values["LKA_MODE"] = 0
      values["LKA_ICON"] = 2 if enabled else 1
      values["TORQUE_REQUEST"] = 0  # apply_torque
      values["LKA_ASSIST"] = 0
      values["STEER_REQ"] = 0  # 1 if lat_active else 0
      values["HAS_LANE_SAFETY"] = 0  # hide LKAS settings
      values["LKA_AVAILABLE"] = 3 if lat_active else 0  # this changes sometimes, 3 seems to indicate engaged
      values["STEER_MODE"] = 0
      values["LKAS_ANGLE_CMD"] = apply_angle if lat_active else 0
      values["LKAS_ANGLE_ACTIVE"] = 0  # 2 if lat_active else 1
      values["LKAS_ANGLE_MAX_TORQUE"] = 0  # angle_max_torque if lat_active else 0
      values["LKAS_SIGNAL_1"] = 10
      #values["NEW_SIGNAL_3"] = 9
      #values["LKAS_SIGNAL_2"] = 1
      #values["LKAS_SIGNAL_3"] = 1
      #values["LKAS_SIGNAL_4"] = 1
      #values["LKAS_SIGNAL_5"] = 1

    else:
      values = {
        "LKA_MODE": 2,
        "LKA_ICON": 2 if lat_active else 1,
        "TORQUE_REQUEST": apply_torque,
        "STEER_REQ": 1 if lat_active else 0,
        "HAS_LANE_SAFETY": 0,
        "LKA_AVAILABLE": 0,
        "STEER_MODE": 0,
        #"LKA_ASSIST": 0,
        #"VALUE104": 3 if lat_active else 100,
      }

    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  else:
    if angle_control:
      values = {
        "LKA_MODE": 0,
        "LKA_ICON": 2 if enabled else 1,
        "TORQUE_REQUEST": 0,  # apply_torque,
        "LKA_ASSIST": 0,
        "STEER_REQ": 0,  # 1 if lat_active else 0,
        "STEER_MODE": 0,
        "HAS_LANE_SAFETY": 0,  # hide LKAS settings
        "LKA_AVAILABLE": 3 if lat_active else 0,  # this changes sometimes, 3 seems to indicate engaged
        "LKAS_ANGLE_CMD": apply_angle if lat_active else 0,
        "LKAS_ANGLE_ACTIVE": 2 if lat_active else 1,
        "LKAS_ANGLE_MAX_TORQUE": angle_max_torque if lat_active else 0,
        "LKAS_SIGNAL_1": 10,
        "NEW_SIGNAL_3": 9,
        "LKAS_SIGNAL_2": 1,
        "LKAS_SIGNAL_3": 1,
        "LKAS_SIGNAL_4": 1,
        "LKAS_SIGNAL_5": 1,
      }

    else:
      values = {
        "LKA_MODE": 2,
        "LKA_ICON": 2 if enabled else 1,
        "TORQUE_REQUEST": apply_torque,
        "VALUE104": 3 if enabled else 100,
        "STEER_REQ": 1 if lat_active else 0,
        "HAS_LANE_SAFETY": 0,  # hide LKAS settings
        "LKA_ASSIST": 0,
        "STEER_MODE": 0,
      }

    if CP.flags & HyundaiFlags.CANFD_LKA_STEERING:
      lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT else "LKAS"
      if CP.openpilotLongitudinalControl:
        ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
      if not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC):
        ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, values))
    else:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret


def create_suppress_lfa(packer, CP, CC, CS, CAN):
  enabled = CC.enabled
  lfa_block_msg = CS.lfa_block_msg
  lka_steering_alt = CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT

  suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"
  msg_bytes = 32 if lka_steering_alt else 24

  values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0 if enabled else 3
  values["RIGHT_LANE_LINE"] = 0 if enabled else 3
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)


def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def create_buttons_canfd_alt(packer, CP, CAN, button, canfd_buttons):
  try:
    values = copy.copy(canfd_buttons)
    values["CRUISE_BUTTONS"] = button
    values["COUNTER"] = (values["COUNTER"] + 1) % 256
    bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEERING else CAN.CAM
    return packer.make_can_msg("CRUISE_BUTTONS_ALT", bus, values)
  except:
    return None


def create_acc_cancel(packer, CP, CS, CAN):
  cruise_info_copy = CS.cruise_info

  # TODO: why do we copy different values here?
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_lfahda_cluster(packer, CC, CAN):
  enabled = CC.enabled

  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": 2 if enabled else 0,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def create_acc_control(packer, CP, CC, CS, CAN, accel_last, accel, stopping, set_speed, hud_control, jerk_u, jerk_l):
  enabled = CC.enabled
  gas_override = CC.cruiseControl.override

  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)

  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = CS.cruise_info
    values["ACCMode"] = 0 if not enabled else (2 if gas_override else 1)
    values["MainMode_ACC"] = 1
    values["StopReq"] = 1 if stopping else 0
    values["aReqValue"] = a_val
    values["aReqRaw"] = a_raw
    values["VSetDis"] = set_speed
    #values["JerkLowerLimit"] = jerk if enabled else 1
    #values["JerkUpperLimit"] = 3.0
    values["JerkLowerLimit"] = jerk_l if enabled else 1
    values["JerkUpperLimit"] = jerk_u
    values["DISTANCE_SETTING"] = hud_control.leadDistanceBars # + 5

    values["SET_ME_2"] = 4
    values["SET_ME_TMP_64"] = 100
    values["NEW_SIGNAL_3"] = 1 if hud_control.leadVisible else 0 #0  # 1이되면 차선이탈방지 알람이 뜬다고...  => 앞에 차가 있으면, 1또는 2가 됨. 전방두부?
    values["ZEROS_5"] = 0
    values["TARGET_DISTANCE"] = CS.out.vEgo * 1.0 + 4.0
    values["CRUISE_STANDSTILL"] = 1 if stopping and CS.out.aEgo > -0.1 else 0
    values["NEW_SIGNAL_2"] = 0  # 이것이 켜지면 가속을 안하는듯함.

  else:
    values = {
      "ACCMode": 0 if not enabled else (2 if gas_override else 1),
      "MainMode_ACC": 1,
      "StopReq": 1 if stopping else 0,
      "aReqValue": a_val,
      "aReqRaw": a_raw,
      "VSetDis": set_speed,
      #"JerkLowerLimit": jerk if enabled else 1,
      #"JerkUpperLimit": 3.0,
      "JerkLowerLimit": jerk_l if enabled else 1,
      "JerkUpperLimit": jerk_u,

      "ACC_ObjDist": 1,
      "SET_ME_2": 4,
      "SET_ME_TMP_64": 100,
      "DISTANCE_SETTING": hud_control.leadDistanceBars, # + 5,
      "CRUISE_STANDSTILL": 1 if stopping and CS.out.cruiseState.standstill else 0,
    }

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
      'AEB_SETTING': 1,  # show AEB disabled icon
      'SET_ME_2': 2,
      'SET_ME_FF': 255,
      'SET_ME_FC': 252,
      'SET_ME_9': 9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret


def create_adrv_messages(packer, CP, CC, CS, CAN, frame, hud_control, disp_angle, left_lane_warning, right_lane_warning):
  main_enabled = CS.out.cruiseState.available
  cruise_enabled = CC.enabled
  lat_active = CC.latActive
  ccnc = CP.exFlags & HyundaiExFlags.CCNC
  nav_active = SpeedLimiter.instance().get_active()
  hdp_active = cruise_enabled and nav_active

  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []
  values = {}
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
    if frame % 5 == 0 and CS.ccnc_info_161 is not None and ccnc:
      values = CS.ccnc_info_161

      values["SETSPEED"] = 6 if hdp_active else 3 if main_enabled else 0
      values["SETSPEED_HUD"] = 5 if hdp_active else 2 if cruise_enabled else 1
      values["vSetDis"] = int(hud_control.setSpeed * 3.6 + 0.5)

      values["DISTANCE"] = 4 if hdp_active else hud_control.leadDistanceBars
      values["DISTANCE_LEAD"] = 2 if cruise_enabled and hud_control.leadVisible else 0
      values["DISTANCE_CAR"] = 3 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
      values["DISTANCE_SPACING"] = 5 if hdp_active else 1 if cruise_enabled else 0

      values["TARGET"] = 1 if cruise_enabled else 0
      values["TARGET_DISTANCE"] = int(hud_control.leadDistance)

      values["BACKGROUND"] = 1 if cruise_enabled else 3 if main_enabled else 7
      values["CENTERLINE"] = 1 if lat_active else 0
      values["CAR_CIRCLE"] = 2 if hdp_active else 1 if lat_active else 0

      values["NAV_ICON"] = 2 if nav_active else 1
      values["HDA_ICON"] = 5 if hdp_active else 2 if lat_active else 1
      values["LFA_ICON"] = 5 if hdp_active else 2 if lat_active else 1
      values["LKA_ICON"] = 4 if lat_active else 3

      values["FCA_ALT_ICON"] = 0

      # SETSPEED 0 "HIDDEN" 1 "GRAY" 2 "GREEN" 3 "WHITE" 6 "CYAN";
      # SETSPEED_HUD 0 "HIDDEN" 1 "GRAY" 2 "GREEN" 3 "WHITE" 5 "CYAN";
      # DISTANCE_LEAD 0 "HIDDEN" 1 "GRAY" 2 "WHITE";
      # DISTANCE_CAR 0 "HIDDEN" 1 "GRAY" 2 "WHITE" 3 "CYAN";
      # DISTANCE_SPACING 0 "HIDDEN" 1 "BLUE" 3 "WHITE" 5 "CYAN";
      # TARGET 0 "HIDDEN" 1 "BLUE" 3 "WHITE";
      # BACKGROUND 0 "HIDDEN" 1 "BLUE" 3 "ORANGE" 4 "FLASHING ORANGE" 6 "FLASHING RED" 7 "GRAY";
      # CAR_CIRCLE 0 "HIDDEN" 1 "GRAY" 2 "CYAN";
      # NAV_ICON 0 "HIDDEN" 1 "GRAY" 2 "GREEN" 4 "WHITE";
      # HDA_ICON 0 "HIDDEN" 1 "GRAY" 2 "GREEN" 3 "WHITE" 5 "CYAN HDP";
      # LFA_ICON 0 "HIDDEN" 1 "GRAY" 2 "GREEN" 3 "WHITE" 5 "CYAN";
      # LKA_ICON 0 "HIDDEN" 1 "ORANGE" 3 "GRAY" 4 "GREEN";
      # DAW_ICON 0 "HIDDEN" 1 "ORANGE";
      # FCA_ALT_ICON 0 "HIDDEN" 1 "ORANGE" 3 "RED";

      if values["ALERTS_2"] == 5:
        values["ALERTS_2"] = 0
        values["SOUNDS_2"] = 0
        values["DAW_ICON"] = 0

      if values["ALERTS_3"] in [17, 26]:
        values["ALERTS_3"] = 0

      if values["ALERTS_5"] in [1, 4, 5]:
        values["ALERTS_5"] = 0

      curvature = {
        i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i
        for i in range(-15, 16)
      }
      values["LANELINE_CURVATURE"] = curvature.get(max(-15, min(int(disp_angle / 3), 15)), 14) if lat_active else 15
      if hud_control.leftLaneDepart:
        values["LANELINE_LEFT"] = 4 if (frame // 50) % 2 == 0 else 1
      else:
        values["LANELINE_LEFT"] = 2 if hud_control.leftLaneVisible else 0
      if hud_control.rightLaneDepart:
        values["LANELINE_RIGHT"] = 4 if (frame // 50) % 2 == 0 else 1
      else:
        values["LANELINE_RIGHT"] = 2 if hud_control.rightLaneVisible else 0

      values["LANELINE_LEFT_POSITION"] = 15
      values["LANELINE_RIGHT_POSITION"] = 15

      values["LCA_LEFT_ARROW"] = 2 if CS.out.leftBlinker else 0
      values["LCA_RIGHT_ARROW"] = 2 if CS.out.rightBlinker else 0

      values["LCA_LEFT_ICON"] = 1 if CS.out.leftBlindspot else 2
      values["LCA_RIGHT_ICON"] = 1 if CS.out.rightBlindspot else 2

      # LCA_LEFT_ARROW 0 "HIDDEN" 2 "VISIBLE";
      # LCA_RIGHT_ARROW 0 "HIDDEN" 2 "VISIBLE";
      # LCA_LEFT_ICON 0 "HIDDEN" 1 "GRAY" 2 "GREEN" 4 "WHITE";
      # LCA_RIGHT_ICON 0 "HIDDEN" 1 "GRAY" 2 "GREEN" 4 "WHITE";

      ret.append(packer.make_can_msg("CCNC_0x161", CAN.ECAN, values))

    if frame % 5 == 0 and CS.ccnc_info_162 is not None and ccnc:
      values = CS.ccnc_info_162
      values["FAULT_FCA"] = 0
      values["FAULT_LSS"] = 0
      values["FAULT_LFA"] = 0
      values["FAULT_LCA"] = 0
      values["FAULT_DAS"] = 0
      values["FAULT_HDA"] = 0

      #values["FAULT_FSS"] = 0
      #values["FAULT_SLA"] = 0
      #values["FAULT_DAW"] = 0
      #values["FAULT_SCC"] = 0

      # FAULT_FCA 0 "HIDDEN" 1 "CHECK_FORWARD_SIDE_SAFETY_SYSTEM" 2 "FORWARD_SIDE_SAFETY_SYSTEM_LIMITED_CAMERA_OBSCURED" 3 "FORWARD_SIDE_SAFETY_SYSTEM_LIMITED_RADAR_BLOCKED";
      # FAULT_LSS 0 "HIDDEN" 1 "CHECK_LANE_SAFETY_SYSTEM" 2 "LANE_SAFETY_SYSTEM_DISABLED_CAMERA_OBSCURED";
      # FAULT_LFA 0 "HIDDEN" 1 "CHECK_LANE_FOLLOWING_SYSTEM_ASSIST_SYSTEM";
      # FAULT_LCA 0 "HIDDEN" 1 "CHECK_LANE_CHANGE_ASSIST_FUNCTION" 2 "LANE_CHANGE_ASSIST_FUNCTION_DISABLED_CAMERA_OBSCURED" 3 "LANE_CHANGE_ASSIST_FUNCTION_DISABLED_RADAR_BLOCKED";
      # FAULT_DAS 0 "HIDDEN" 1 "CHECK_DRIVER_ASSISTANCE_SYSTEM" 2 "DRIVER_ASSISTANCE_SYSTEM_LIMITED_CAMERA_OBSCURED" 3 "DRIVER_ASSISTANCE_SYSTEM_LIMITED_RADAR_BLOCKED" 4 "DRIVER_ASSISTANCE_SYSTEM_LIMITED_CAMERA_OBSCURED_AND_RADAR_BLOCKED";
      # FAULT_HDA 0 "HIDDEN" 1 "CHECK_HIGHWAY_DRIVING_ASSIST_SYSTEM";

      # FAULT_FSS 0 "HIDDEN" 1 "CHECK_FORWARD_SAFETY_SYSTEM" 2 "FORWARD_SAFETY_SYSTEM_LIMITED_CAMERA_OBSCURED" 3 "FORWARD_SAFETY_SYSTEM_LIMITED_RADAR_BLOCKED";
      # FAULT_SLA 0 "HIDDEN" 1 "CHECK_SPEED_LIMIT_SYSTEM" 2 "SPEED_LIMIT_SYSTEM_DISABLED_CAMERA_OBSCURED";
      # FAULT_DAW 0 "HIDDEN" 1 "CHECK_INATTENTIVE_DRIVING_WARNING_SYSTEM" 2 "INATTENTIVE_DRIVING_WARNING_SYSTEM_DISABLED_CAMERA_OBSCURED";
      # FAULT_SCC 0 "HIDDEN" 1 "CHECK_SMART_CRUISE_CONTROL_SYSTEM" 2 "SMART_CRUISE_CONTROL_DISABLED_RADAR_BLOCKED";

      if left_lane_warning or right_lane_warning:
        values["VIBRATE"] = 1
      ret.append(packer.make_can_msg("CCNC_0x162", CAN.ECAN, values))

    if frame % 2 == 0 and CS.adrv_info_160 is not None:
      values = CS.adrv_info_160
      values["NEW_SIGNAL_1"] = 0  # steer_temp관련없음, 계기판에러
      values["SET_ME_9"] = 17  # steer_temp관련없음, 계기판에러
      values["SET_ME_2"] = 0  # 커멘트해도 steer_temp에러남, 2값은 콤마에서 찾은거니...
      values["DATA102"] = 0  # steer_temp관련없음
      ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

    if frame % 5 == 0 and CS.adrv_info_200 is not None:
      values = CS.adrv_info_200
      values["TauGapSet"] = hud_control.leadDistanceBars
      ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

    if frame % 5 == 0 and CS.adrv_info_1ea is not None:
      values = CS.adrv_info_1ea
      values["HDA_MODE1"] = 8
      values["HDA_MODE2"] = 1
      ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

    """
    if frame % 20 == 0 and CS.hda_info_4a3 is not None:
      values = CS.hda_info_4a3
      # SIGNAL_4: 7, SIGNAL_0: 0 으로 해도 .. 옆두부는 나오기도 함.. 아오5
      values["SIGNAL_4"] = 10 if CC.enabled else 0   # 0, 5(고속도로진입), 10(고속도로), 7,5(국도에서 간혹), 0,10(카니발)      , 5(고속도로진입,EV6), 11(고속도로,EV6)
      values["SIGNAL_0"] = 5 if CC.enabled else 0  # 0, 2(고속도로진입), 1(고속도로),                      5(카니발은 항상)  , 2(고속도로진입,EV6), 1(고속도로,EV6)
      values["NEW_SIGNAL_1"] = 4
      values["NEW_SIGNAL_2"] = 0
      values["NEW_SIGNAL_3"] = 154
      values["NEW_SIGNAL_4"] = 9
      values["NEW_SIGNAL_5"] = 0
      values["NEW_SIGNAL_6"] = 256
      values["NEW_SIGNAL_7"] = 0
      ret.append(packer.make_can_msg("HDA_INFO_0x4a3", CAN.CAM, values))
    if frame % 10 == 0 and CS.hda_info_4b4 is not None:
      values = CS.hda_info_4b4
      values["NEW_SIGNAL_1"] = 8
      values["NEW_SIGNAL_3"] = (frame / 100) % 10
      values["NEW_SIGNAL_4"] = 146
      values["NEW_SIGNAL_5"] = 68
      values["NEW_SIGNAL_6"] = 76
      ret.append(packer.make_can_msg("HDA_INFO_0x4b4", CAN.CAM, values))
    """

    return ret

  else:
    values = {}

    ret.extend(create_fca_warning_light(packer, CP, CAN, frame))
    if frame % 5 == 0:
      values = {
        'HDA_MODE1': 8,
        'HDA_MODE2': 1,
        'SET_ME_FF': 255,
      }
      ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

      values = {
        'SET_ME_E1': 225,
        'TauGapSet' : 1,
        'NEW_SIGNAL_2': 3,
      }
      ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

    if frame % 20 == 0:
      values = {
        'SET_ME_15': 21,
      }
      ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

    if frame % 100 == 0:
      values = {
        'SET_ME_22': 34,
        'SET_ME_41': 65,
      }
      ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

    return ret
