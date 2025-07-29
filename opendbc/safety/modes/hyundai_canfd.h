#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus)                \
  {0x1CF, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(bus)                \
  {0x1AA, bus, 16, .check_relay = false},  /* CRUISE_BUTTON ALT */  \

#define HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(a_can, e_can)     \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x50,  a_can, 16, .check_relay = (a_can) == 0},  /* LKAS */      \
  {0x2A4, a_can, 24, .check_relay = (a_can) == 0},  /* CAM_0x2A4 */ \

#define HYUNDAI_CANFD_LKA_STEERING_ALT_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(e_can)                    \
  {0x110, a_can, 32, .check_relay = (a_can) == 0},  /* LKAS_ALT */  \
  {0x362, a_can, 32, .check_relay = (a_can) == 0},  /* CAM_0x362 */ \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)                 \
  {0x12A, e_can, 16, .check_relay = (e_can) == 0},  /* LFA */            \
  {0x1E0, e_can, 16, .check_relay = (e_can) == 0},  /* LFAHDA_CLUSTER */ \

#define HYUNDAI_CANFD_LFA_STEERING_ALT_TX_MSGS(e_can)               \
  {0xCB, e_can, 24, .check_relay = (e_can) == 0},  /* LFA_ALT */    \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS_DUAL(e1, e2) \
  HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e1)                \
  HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e2)                \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal)   \
  {0x1A0, e_can, 32, .check_relay = (longitudinal)},  /* SCC_CONTROL */ \

#define HYUNDAI_CANFD_ADRV_TX_MSGS(e_can)                       \
  {0x51,  e_can, 32, .check_relay = false},  /* ADRV_0x51 */    \
  {0x1EA, e_can, 32, .check_relay = false},  /* ADRV_0x1ea */   \
  {0x200, e_can,  8, .check_relay = false},  /* ADRV_0x200 */   \
  {0x345, e_can,  8, .check_relay = false},  /* ADRV_0x345 */   \
  {0x1DA, e_can, 32, .check_relay = false},  /* ADRV_0x1da */   \

#define HYUNDAI_CANFD_ADRV_TX_MSGS_DUAL(e1, e2) \
  HYUNDAI_CANFD_ADRV_TX_MSGS(e1)                \
  HYUNDAI_CANFD_ADRV_TX_MSGS(e2)                \

// *** Addresses checked in rx hook ***
// EV, ICE, HYBRID: ACCELERATOR (0x35), ACCELERATOR_BRAKE_ALT (0x100), ACCELERATOR_ALT (0x105)
#define HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                       \
  {.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                                            \
           {0x100, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                                           \
           {0x105, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}}},                                         \
  {.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},                            \
  {.msg = {{0x125, (pt_bus), 16, 100U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(pt_bus)                                                                               \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                          \
  {.msg = {{0x1cf, (pt_bus), 8, 50U, .ignore_checksum = true, .max_counter = 0xfU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(pt_bus)                                                                                 \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                            \
  {.msg = {{0x1aa, (pt_bus), 16, 50U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                                               \
  {.msg = {{0x1a0, (scc_bus), 32, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_lka_steering_alt = false;
static bool hyundai_canfd_angle_steering = false;

static unsigned int hyundai_canfd_get_lka_addr(void) {
  return hyundai_canfd_lka_steering_alt ? 0x110U : 0x50U;
}

#define CANFD_TX_ENTRIES_SIZE 20

typedef struct {
  int addr;
  uint32_t timestamp;
} CanFdTxEntry;

CanFdTxEntry tx_bus_2[CANFD_TX_ENTRIES_SIZE] = {
  [0]  = { .addr = 0x160,  .timestamp = 0 },  // ADRV_0x160
  [1]  = { .addr = 0x161,  .timestamp = 0 },  // CCNC_0x161
  [2]  = { .addr = 0x162,  .timestamp = 0 },  // CCNC_0x162
  [3]  = { .addr = 0x1A0,  .timestamp = 0 },  // SCC_CONTROL
  [4]  = { .addr = 0x1DA,  .timestamp = 0 },  // ADRV_0x1DA
  [5]  = { .addr = 0x1EA,  .timestamp = 0 },  // ADRV_0x1EA
  [6]  = { .addr = 0x200,  .timestamp = 0 },  // ADRV_0x200
  [7]  = { .addr = 0x345,  .timestamp = 0 },  // ADRV_0x345
  [8]  = { .addr = 0x12A,  .timestamp = 0 },  // LFA
  [9]  = { .addr = 0xCB,   .timestamp = 0 },  // LFA_ALT
  [10] = { .addr = 0x1E0,  .timestamp = 0 },  // LFAHDA_CLUSTER
  [11] = { .addr = 0x110,  .timestamp = 0 },  // LKAS_ALT
  [12] = { .addr = 0x50,   .timestamp = 0 },  // LKAS
  [13] = { .addr = 0x362,  .timestamp = 0 },  // CAM_0x362
  [14] = { .addr = 0x2A4,  .timestamp = 0 },  // CAM_0x2A4
  [15] = { .addr = 0x51,   .timestamp = 0 },  // ADRV_0x51
};

CanFdTxEntry tx_bus_0[CANFD_TX_ENTRIES_SIZE] = {
  [0] = { .addr = 0x4A3,  .timestamp = 0 },  // HDA_INFO_0x4A3
  [1] = { .addr = 0x2AF,  .timestamp = 0 },  // HANDS_ON_DETECTION
  [2] = { .addr = 0xEA,   .timestamp = 0 },  // MDPS
  [3] = { .addr = 0x7C4,  .timestamp = 0 },  // VEHICLE DIAGNOSTICS
};

static void update_canfd_entry(CanFdTxEntry *entries, int size, int addr, bool tx) {
  for (int i = 0; i < size; i++) {
    if (entries[i].addr == 0) break;

    if (entries[i].addr == addr) {
      entries[i].timestamp = tx ? microsecond_timer_get() : 0;
      break;
    }
  }
}

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *msg) {
  uint8_t ret = 0;
  if (GET_LEN(msg) == 8U) {
    ret = msg->data[1] >> 4;
  } else {
    ret = msg->data[2];
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *msg) {
  uint32_t chksum = msg->data[0] | (msg->data[1] << 8);
  return chksum;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {

  const unsigned pt_bus = (hyundai_canfd_lka_steering && !hyundai_camera_scc) ? 1U : 0U;
  const unsigned int scc_bus = hyundai_camera_scc ? 2U : pt_bus;

  if (msg->bus == pt_bus) {
    // driver torque
    if (msg->addr == 0xeaU) {
      int torque_driver_new = ((msg->data[11] & 0x1fU) << 8U) | msg->data[10];
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);
    }

    // steering angle
    if (msg->addr == 0x125U) {
      int angle_meas_new = (msg->data[4] << 8) | msg->data[3];
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // cruise buttons
    const unsigned int button_addr = hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
    if (msg->addr == button_addr) {
      bool main_button = false;
      int cruise_button = 0;
      bool lfa_button = false;
      if (msg->addr == 0x1cfU) {
        cruise_button = msg->data[2] & 0x7U;
        main_button = GET_BIT(msg, 19U);
        lfa_button = GET_BIT(msg, 23U);
      } else {
        cruise_button = (msg->data[4] >> 4) & 0x7U;
        main_button = GET_BIT(msg, 34U);
        lfa_button = GET_BIT(msg, 39U);
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button, lfa_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((msg->addr == 0x35U) && hyundai_ev_gas_signal) {
      gas_pressed = msg->data[5] != 0U;
    } else if ((msg->addr == 0x105U) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 103U) || (msg->data[13] != 0U) || GET_BIT(msg, 112U);
    } else if ((msg->addr == 0x100U) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 176U);
    } else {
    }

    // brake press
    if (msg->addr == 0x175U) {
      brake_pressed = GET_BIT(msg, 81U);
    }

    // vehicle moving
    if (msg->addr == 0xa0U) {
      uint32_t fl = (GET_BYTES(msg, 8, 2)) & 0x3FFFU;
      uint32_t fr = (GET_BYTES(msg, 10, 2)) & 0x3FFFU;
      uint32_t rl = (GET_BYTES(msg, 12, 2)) & 0x3FFFU;
      uint32_t rr = (GET_BYTES(msg, 14, 2)) & 0x3FFFU;
      vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);

      // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
      UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 * KPH_TO_MS);
    }
  }

  gas_pressed = brake_pressed = false;

  if (msg->bus == scc_bus) {
    // cruise state
    if ((msg->addr == 0x1a0U) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((msg->data[8] >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
    }
  }
}

static bool hyundai_canfd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_CANFD_STEERING_LIMITS = {
    .max_torque = 270,
    .max_rt_delta = 112,
    .max_rate_up = 2,
    .max_rate_down = 3,
    .driver_torque_allowance = 250,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,

    // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
    // we allow setting torque actuation bit to 0 while maintaining the requested torque value for two consecutive frames
    .min_valid_request_frames = 89,
    .max_invalid_request_frames = 2,
    .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
    .has_steer_req_tolerance = true,
  };

  const AngleSteeringLimits HYUNDAI_CANFD_ANGLE_STEERING_LIMITS = {
    .max_angle = 1800,
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      //{5., 25., 25.},
      //{0.3, 0.15, 0.15}
      {0, 5., 25.},
      {1.0, 0.6, 0.2}
    },
    .angle_rate_down_lookup = {
      //{5., 25., 25.},
      //{0.36, 0.26, 0.26}
      {0, 5., 25.},
      {1.5, 0.9, 0.3}
    },
  };
  bool tx = true;

  // steering
  const unsigned steer_addr = hyundai_canfd_lka_steering ? hyundai_canfd_get_lka_addr() : 0x12aU;
  if (msg->addr == steer_addr) {
    if (hyundai_canfd_angle_steering) {
      const int lkas_angle_active = (msg->data[9] >> 4) & 0x3U;
      const bool steer_angle_req = lkas_angle_active != 1;

      int desired_angle = (msg->data[11]) << 6) | (msg->data[10]) >> 2);
      desired_angle = to_signed(desired_angle, 14);

      if (steer_angle_cmd_checks(desired_angle, steer_angle_req, HYUNDAI_CANFD_ANGLE_STEERING_LIMITS)) {
        tx = false;
      }
    } else {
      int desired_torque = (((msg->data[6] & 0xFU) << 7U) | (msg->data[5] >> 1U)) - 1024U;
      bool steer_req = GET_BIT(msg, 52U);

      if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_STEERING_LIMITS)) {
        tx = false;
      }
    }
  }

  // cruise buttons check
  if (msg->addr == 0x1cfU) {
    int button = msg->data[2] & 0x7U;
    bool is_cancel = (button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (button == HYUNDAI_BTN_RESUME);
    bool is_set = (button == HYUNDAI_BTN_SET);

    bool allowed = (is_cancel && cruise_engaged_prev) || (is_resume && controls_allowed) || (is_set && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (((msg->addr == 0x730U) && hyundai_canfd_lka_steering) || ((msg->addr == 0x7D0U) && !hyundai_camera_scc)) {
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (msg->addr == 0x1a0U) {
    int desired_accel_raw = (((msg->data[17] & 0x7U) << 8) | msg->data[16]) - 1023U;
    int desired_accel_val = ((msg->data[18] << 4) | (msg->data[17] >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      const int acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (acc_mode != 4) {
        violation = true;
      }

      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  update_canfd_entry(tx_bus_2, CANFD_TX_ENTRIES_SIZE, addr, tx);
  update_canfd_entry(tx_bus_0, CANFD_TX_ENTRIES_SIZE, addr, tx);

  return tx;
}

static bool should_block_msg(CanFdTxEntry *entries, int size, int addr, uint32_t now, uint32_t timeout) {
  for (int i = 0; i < size; i++) {
    if (entries[i].msg->addr == 0) break;

    if (entries[i].msg->addr == addr && (now - entries[i].timestamp) < timeout) {
      return true;
    }
  }
  return false;
}

static bool hyundai_canfd_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;
  uint32_t OP_CAN_SEND_TIMEOUT = 100000; // 1sec
  uint32_t now = microsecond_timer_get();

  if (bus_num == 0) {
    block_msg = should_block_msg(tx_bus_0, CANFD_TX_ENTRIES_SIZE, addr, now, OP_CAN_SEND_TIMEOUT);
  } else if (bus_num == 2) {
    block_msg = should_block_msg(tx_bus_2, CANFD_TX_ENTRIES_SIZE, addr, now, OP_CAN_SEND_TIMEOUT);
  }

  return block_msg;
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const int HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT = 128;
  const int HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;
  const int HYUNDAI_PARAM_CANFD_ANGLE_STEERING = 1024;

  static const CanMsg HYUNDAI_CANFD_LKA_STEERING_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEERING_ALT_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEERING_ALT_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(1, 1)
    HYUNDAI_CANFD_LKA_STEERING_ALT_COMMON_TX_MSGS(1, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS_DUAL(0, 1)
    HYUNDAI_CANFD_ADRV_TX_MSGS_DUAL(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_ALT_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, false)
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 0, 16, .check_relay = false},  // ADRV_0x160
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x161, 0, 32, .check_relay = false},  // CCNC_0x161
    {0x162, 0, 32, .check_relay = false},  // CCNC_0x162
    {0x4A3, 2,  8, .check_relay = false},  // HDA_INFO_0x4a3
    {0xEA,  2, 24, .check_relay = false},  // MDPS
    {0x2AF, 2,  8, .check_relay = false},  // HANDS_ON_DETECTION
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true)
    HYUNDAI_CANFD_LFA_STEERING_ALT_TX_MSGS(0)
    {0x7D0, 0,  8, .check_relay = false},  // tester present for radar ECU disable
    {0x160, 1, 16, .check_relay = true},   // ADRV_0x160
  };

#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)                          \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)                    \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal))     \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \
    {0x161, 0, 32, .check_relay = (longitudinal)}, /* CCNC_0x161 */ \
    {0x162, 0, 32, .check_relay = (longitudinal)}, /* CCNC_0x162 */ \
    {0xEA,  2, 24, .check_relay = (longitudinal)}, /* MDPS       */ \
    {0x7C4, 2,  8, .check_relay = (longitudinal)}, /* 0x7C4      */ \

  hyundai_common_init(param);

  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_angle_steering = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ANGLE_STEERING);
  hyundai_canfd_lka_steering_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT) || hyundai_canfd_angle_steering;

  safety_config ret;
  if (hyundai_longitudinal) {
    if (hyundai_canfd_lka_steering) {
      static RxCheck hyundai_canfd_lka_steering_long_rx_checks_camera_scc[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      };
      static RxCheck hyundai_canfd_lka_steering_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
      };

      ret = hyundai_camera_scc ?
        BUILD_SAFETY_CFG(hyundai_canfd_lka_steering_long_rx_checks_camera_scc, HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS) : \
        BUILD_SAFETY_CFG(hyundai_canfd_lka_steering_long_rx_checks, HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS);
    } else {
      // Longitudinal checks for LFA steering
      static RxCheck hyundai_canfd_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(true)
      };

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_long_rx_checks, ret);
      }

      if (hyundai_camera_scc) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS, ret);
      }
    }

  } else {
    if (hyundai_canfd_lka_steering) {
      // *** LKA steering checks ***
      // E-CAN is on bus 1, SCC messages are sent on cars with ADRV ECU.
      // Does not use the alt buttons message
      static RxCheck hyundai_canfd_lka_steering_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      SET_RX_CHECKS(hyundai_canfd_lka_steering_rx_checks, ret);
      if (hyundai_canfd_lka_steering_alt) {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEERING_ALT_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEERING_TX_MSGS, ret);
      }

    } else if (!hyundai_camera_scc) {
      // Radar sends SCC messages on these cars instead of camera
      static RxCheck hyundai_canfd_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_TX_MSGS, ret);

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_radar_scc_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_radar_scc_rx_checks, ret);
      }

    } else {
      // *** LFA steering checks ***
      // Camera sends SCC messages on LFA steering cars.
      // Both button messages exist on some platforms, so we ensure we track the correct one using flag
      static RxCheck hyundai_canfd_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
      };

      SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_rx_checks, ret);
      }
    }
  }

  return ret;
}

const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .fwd = hyundai_canfd_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
