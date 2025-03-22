#pragma once

#include "safety_declarations.h"
#include "safety_hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus) \
  {0x1CF, bus, 8, false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(bus) \
  {0x1AA, bus, 16, false},  /* CRUISE_BUTTON ALT */  \

#define HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                    \
  {0x50,  a_can, 16, (a_can) == 0},  /* LKAS */                 \
  {0x2A4, a_can, 24, false},         /* CAM_0x2A4 */            \

#define HYUNDAI_CANFD_LKA_STEERING_ALT_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(e_can)                    \
  {0x110, a_can, 32, (a_can) == 0},  /* LKAS_ALT */                 \
  {0x362, a_can, 32, false},         /* CAM_0x362 */                \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)  \
  {0x12A, e_can, 16, (e_can) == 0},  /* LFA */            \
  {0x1E0, e_can, 16, false},         /* LFAHDA_CLUSTER */ \

#define HYUNDAI_CANFD_LFA_STEERING_ALT_TX_MSGS(e_can) \
  {0xCB, e_can, 24, (e_can) == 0},  /* LFA_ALT */     \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS_DUAL(e1, e2) \
  HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e1)                \
  HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e2)                \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal)   \
  {0x1A0, e_can, 32, (longitudinal)},  /* SCC_CONTROL */                \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS_DUAL(e1, e2) \
  HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e1, true)          \
  HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e2, true)          \

#define HYUNDAI_CANFD_ADRV_TX_MSGS(e_can)        \
  {0x51,  e_can, 32, false},  /* ADRV_0x51 */    \
  {0x1EA, e_can, 32, false},  /* ADRV_0x1ea */   \
  {0x200, e_can,  8, false},  /* ADRV_0x200 */   \
  {0x345, e_can,  8, false},  /* ADRV_0x345 */   \
  {0x1DA, 1,     32, false},  /* ADRV_0x1da */   \

#define HYUNDAI_CANFD_ADRV_TX_MSGS_DUAL(e1, e2) \
  HYUNDAI_CANFD_ADRV_TX_MSGS(e1)                \
  HYUNDAI_CANFD_ADRV_TX_MSGS(e2)                \

// *** Addresses checked in rx hook ***
// EV, ICE, HYBRID: ACCELERATOR (0x35), ACCELERATOR_BRAKE_ALT (0x100), ACCELERATOR_ALT (0x105)
#define HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                       \
  {.msg = {{0x35, (pt_bus), 32, .max_counter = 0xffU, .frequency = 100U},                                            \
           {0x100, (pt_bus), 32, .max_counter = 0xffU, .frequency = 100U},                                           \
           {0x105, (pt_bus), 32, .max_counter = 0xffU, .frequency = 100U}}},                                         \
  {.msg = {{0x175, (pt_bus), 24, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}},                            \
  {.msg = {{0xa0, (pt_bus), 24, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},                            \
  {.msg = {{0xea, (pt_bus), 24, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},                            \
  {.msg = {{0x125, (pt_bus), 16, .ignore_checksum = true, .max_counter = 0xffU, .frequency = 100U}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(pt_bus)                                                               \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                          \
  {.msg = {{0x1cf, (pt_bus), 8, .ignore_checksum = true, .max_counter = 0xfU, .frequency = 50U}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(pt_bus)                                                                 \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                            \
  {.msg = {{0x1aa, (pt_bus), 16, .ignore_checksum = true, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}},  \

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                               \
  {.msg = {{0x1a0, (scc_bus), 32, .max_counter = 0xffU, .frequency = 50U}, { 0 }, { 0 }}},  \

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_lka_steering_alt = false;
static bool hyundai_canfd_angle_steering = false;

static int hyundai_canfd_get_lka_addr(void) {
  return hyundai_canfd_lka_steering_alt ? 0x110 : 0x50;
}

#define CANFD_TX_ENTRIES_SIZE 25
#define MAX_ADDR_LIST_SIZE 128
#define OP_CAN_SEND_TIMEOUT 100000

typedef struct {
  int addr;
  uint32_t timestamp;
} CanFdTxEntry;

CanFdTxEntry canfd_tx_entries[CANFD_TX_ENTRIES_SIZE] = {
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
  [11] = { .addr = 0xEA,   .timestamp = 0 },  // MDPS
  [12] = { .addr = 0x110,  .timestamp = 0 },  // LKAS_ALT
  [13] = { .addr = 0x50,   .timestamp = 0 },  // LKAS
  [14] = { .addr = 0x362,  .timestamp = 0 },  // CAM_0x362
  [15] = { .addr = 0x2A4,  .timestamp = 0 },  // CAM_0x2A4
  [16] = { .addr = 0x51,   .timestamp = 0 },  // ADRV_0x51
  //[] = { .addr = 0x1BA,  .timestamp = 0 },  // BLINDSPOTS_REAR_CORNERS
  //[] = { .addr = 0x1E5,  .timestamp = 0 },  // BLINDSPOTS_FRONT_CORNER_1
  //[] = { .addr = 0x1B5,  .timestamp = 0 },  // CCNC_0x1B5
  //[] = { .addr = 0x1FA,  .timestamp = 0 },  // CLUSTER_SPEED_LIMIT
  //[] = { .addr = 0x4A3,  .timestamp = 0 },  // HDA_INFO_0x4A3
  //[] = { .addr = 0x4B4,  .timestamp = 0 },  // HDA_INFO_0x4B4
};

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *to_push) {
  uint8_t ret = 0;
  if (GET_LEN(to_push) == 8U) {
    ret = GET_BYTE(to_push, 1) >> 4;
  } else {
    ret = GET_BYTE(to_push, 2);
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *to_push) {
  uint32_t chksum = GET_BYTE(to_push, 0) | (GET_BYTE(to_push, 1) << 8);
  return chksum;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  const int pt_bus = (hyundai_canfd_lka_steering && !hyundai_camera_scc) ? 1 : 0;
  const int scc_bus = hyundai_camera_scc ? 2 : pt_bus;

  if (bus == pt_bus) {
    // driver torque
    if (addr == 0xea) {
      int torque_driver_new = ((GET_BYTE(to_push, 11) & 0x1fU) << 8U) | GET_BYTE(to_push, 10);
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);
    }

    // steering angle
    if (addr == 0x125) {
      int angle_meas_new = (GET_BYTE(to_push, 4) << 8) | GET_BYTE(to_push, 3);
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // cruise buttons
    const int button_addr = hyundai_canfd_alt_buttons ? 0x1aa : 0x1cf;
    if (addr == button_addr) {
      bool main_button = false;
      int cruise_button = 0;
      if (addr == 0x1cf) {
        cruise_button = GET_BYTE(to_push, 2) & 0x7U;
        main_button = GET_BIT(to_push, 19U);
      } else {
        cruise_button = (GET_BYTE(to_push, 4) >> 4) & 0x7U;
        main_button = GET_BIT(to_push, 34U);
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((addr == 0x35) && hyundai_ev_gas_signal) {
      gas_pressed = GET_BYTE(to_push, 5) != 0U;
    } else if ((addr == 0x105) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(to_push, 103U) || (GET_BYTE(to_push, 13) != 0U) || GET_BIT(to_push, 112U);
    } else if ((addr == 0x100) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(to_push, 176U);
    } else {
    }

    // brake press
    if (addr == 0x175) {
      brake_pressed = GET_BIT(to_push, 81U);
    }

    // vehicle moving
    if (addr == 0xa0) {
      uint32_t fl = (GET_BYTES(to_push, 8, 2)) & 0x3FFFU;
      uint32_t fr = (GET_BYTES(to_push, 10, 2)) & 0x3FFFU;
      uint32_t rl = (GET_BYTES(to_push, 12, 2)) & 0x3FFFU;
      uint32_t rr = (GET_BYTES(to_push, 14, 2)) & 0x3FFFU;
      vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);

      // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
      UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 / 3.6);
    }
  }

  gas_pressed = brake_pressed = false;

  if (bus == scc_bus) {
    // cruise state
    if ((addr == 0x1a0) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((GET_BYTE(to_push, 8) >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
    }
  }

  const int steer_addr = hyundai_canfd_lka_steering ? hyundai_canfd_get_lka_addr() : 0x12a;
  bool stock_ecu_detected = (addr == steer_addr) && (bus == 0);
  if (hyundai_longitudinal) {
    // on LKA steering cars, ensure ADRV ECU is still knocked out
    // on others, ensure accel msg is blocked from camera
    stock_ecu_detected = stock_ecu_detected || ((addr == 0x1a0) && (bus == pt_bus));
  }
  generic_rx_checks(stock_ecu_detected);
}

static bool hyundai_canfd_tx_hook(const CANPacket_t *to_send) {
  const TorqueSteeringLimits HYUNDAI_CANFD_STEERING_LIMITS = {
    .max_steer = 512,
    .max_rt_delta = 112,
    .max_rt_interval = 250000,
    .max_rate_up = 10,
    .max_rate_down = 10,
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
      {0, 5., 25.},
      //{0.3, 0.15, 0.15}
      {1.2, 0.8, 0.2}
    },
    .angle_rate_down_lookup = {
      //{5., 25., 25.},
      {0, 5., 25.},
      //{0.36, 0.26, 0.26}
      {1.8, 1.2, 0.3}
    },
  };
  bool tx = true;
  int addr = GET_ADDR(to_send);

  // steering
  //const int steer_addr = (hyundai_canfd_lka_steering && !hyundai_longitudinal) ? hyundai_canfd_get_lka_addr() : 0x12a;
  const int steer_addr = hyundai_canfd_lka_steering ? hyundai_canfd_get_lka_addr() : 0x12a;
  if (addr == steer_addr) {
    if (hyundai_canfd_angle_steering) {
      const int lkas_angle_active = (GET_BYTE(to_send, 9) >> 4) & 0x3U;
      const bool steer_angle_req = lkas_angle_active != 1;

      int desired_angle = (GET_BYTE(to_send, 11) << 6) | (GET_BYTE(to_send, 10) >> 2);
      desired_angle = to_signed(desired_angle, 14);

      if (steer_angle_cmd_checks(desired_angle, steer_angle_req, HYUNDAI_CANFD_ANGLE_STEERING_LIMITS)) {
        tx = false;
      }
    } else {
      int desired_torque = (((GET_BYTE(to_send, 6) & 0xFU) << 7U) | (GET_BYTE(to_send, 5) >> 1U)) - 1024U;
      bool steer_req = GET_BIT(to_send, 52U);

      if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_STEERING_LIMITS)) {
        tx = false;
      }
    }
  }

  // cruise buttons check
  if (addr == 0x1cf) {
    int button = GET_BYTE(to_send, 2) & 0x7U;
    bool is_cancel = (button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (button == HYUNDAI_BTN_RESUME);
    bool is_set = (button == HYUNDAI_BTN_SET);

    bool allowed = (is_cancel && cruise_engaged_prev) || (is_resume && controls_allowed) || (is_set && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (((addr == 0x730) && hyundai_canfd_lka_steering) || ((addr == 0x7D0) && !hyundai_camera_scc)) {
    if ((GET_BYTES(to_send, 0, 4) != 0x00803E02U) || (GET_BYTES(to_send, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (addr == 0x1a0) {
    int desired_accel_raw = (((GET_BYTE(to_send, 17) & 0x7U) << 8) | GET_BYTE(to_send, 16)) - 1023U;
    int desired_accel_val = ((GET_BYTE(to_send, 18) << 4) | (GET_BYTE(to_send, 17) >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      const int acc_mode = (GET_BYTE(to_send, 8) >> 4) & 0x7U;
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

  for (int i = 0; i < CANFD_TX_ENTRIES_SIZE; i++) {
    if (canfd_tx_entries[i].addr == 0) {
      break;
    }
    if (addr == canfd_tx_entries[i].addr) {
      canfd_tx_entries[i].timestamp = tx ? microsecond_timer_get() : 0;
      break;
    }
  }

  return tx;
}

typedef struct {
  int addrs[MAX_ADDR_LIST_SIZE];
  int count;
} AddrList;

static bool add_addr_to_list(AddrList *list, int addr) {
  if (list->count >= MAX_ADDR_LIST_SIZE) {
    return false; // List is full
  }

  for (int i = 0; i < list->count; i++) {
    if (list->addrs[i] == addr) {
      return false; // Already exists
    }
  }

  list->addrs[list->count++] = addr;
  return true;
}

static void print_hex(uint32_t num) {
  const char hex_digits[] = "0123456789abcdef";
  char buf[9] = {0};
  for (int i = 7; i >= 0; i--) {
    buf[i] = hex_digits[num & 0xFU];
    num >>= 4;
  }
  int start = 0;
  while (start < 7 && buf[start] == '0') start++;
  print(&buf[start]);
}

static void print_addr_list(const char *prefix, const AddrList *list, int bus_num, uint32_t timestamp) {
  print(prefix);
  print("Bus=");
  putui((uint32_t)bus_num);
  print(", ts=");
  putui(timestamp);
  print(", Addrs=[");
  for (int j = 0; j < list->count; j++) {
    putui((uint32_t)list->addrs[j]);  // Dec
    print("(0x");
    print_hex((uint32_t)list->addrs[j]);  // Hex
    print(")");
    if (j < (list->count - 1)) {
      print(", ");
    }
  }
  print("]\n");
}

static bool hyundai_canfd_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;
  uint32_t now = microsecond_timer_get();
  static AddrList addr_list = {{0}, 0};

  // LKAS for cars with LKAS and LFA messages, LFA for cars with no LKAS messages
  //int lfa_block_addr = hyundai_canfd_lka_steering_alt ? 0x362 : 0x2a4;
  //bool is_lka_msg = ((addr == hyundai_canfd_get_lka_addr()) || (addr == lfa_block_addr)) && hyundai_canfd_lka_steering;
  //bool is_lfa_msg = ((addr == 0x12a) && !hyundai_canfd_lka_steering);

  // HUD icons
  //bool is_lfahda_msg = ((addr == 0x1e0) && !hyundai_canfd_lka_steering);

  // CCNC messages
  bool is_ccnc_msg = (addr == 0x161) || (addr == 0x162);

  // SCC_CONTROL and ADRV_0x160 for camera SCC cars, we send our own longitudinal commands and to show FCA light
  //bool is_scc_msg = (((addr == 0x1a0) || (!is_ccnc_msg && (addr == 0x160))) && hyundai_longitudinal && !hyundai_canfd_lka_steering);

  if (bus_num == 0) {
    block_msg = ((is_ccnc_msg) && (((addr) == 0xEA) || ((addr) == 0x7C4))); // mdps || vehicle diagnostics
  } else if (bus_num == 2) {
    //block_msg = is_lka_msg || is_lfa_msg || is_lfahda_msg || is_scc_msg || is_ccnc_msg;

    for (int i = 0; i < CANFD_TX_ENTRIES_SIZE; i++) {
      if (canfd_tx_entries[i].addr == 0) {
        break;
      }
      if (addr == canfd_tx_entries[i].addr && (now - canfd_tx_entries[i].timestamp) < OP_CAN_SEND_TIMEOUT) {
        block_msg = true;
        break;
      }
    }
  }

  if (add_addr_to_list(&addr_list, addr)) {
    print_addr_list("Debug: Bus Addr List - ", &addr_list, bus_num, now);
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
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS_DUAL(0,1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS_DUAL(0,1)
    HYUNDAI_CANFD_ADRV_TX_MSGS_DUAL(0,1)
    HYUNDAI_CANFD_LFA_STEERING_ALT_TX_MSGS(0)
    {0x730, 1,  8, false},  // tester present for ADAS ECU disable
    {0x160, 0, 16, false},  // ADRV_0x160
    {0x160, 1, 16, false},  // ADRV_0x160
    {0x161, 0, 32, false},  // CCNC_0x161
    {0x162, 0, 32, false},  // CCNC_0x162
    //{0x4A3, 2,  8, false},  // HDA_INFO_0x4a3
    //{0x4B4, 2,  8, false},  // HDA_INFO_0x4b4
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
    {0x7D0, 0,  8, false},  // tester present for radar ECU disable
    {0x160, 1, 16, false},  // ADRV_0x160
  };

#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, false},  /* ADRV_0x160 */ \
    {0x161, 0, 32, false},  /* CCNC_0x161 */ \
    {0x162, 0, 32, false},  /* CCNC_0x162 */ \

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
