#include "safety_forwards.h"

const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {5., .8, .25}};

const struct lookup_t TESLA_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {5., 3.5, .8}};

const int TESLA_DEG_TO_CAN = 10;

const uint32_t TIME_TO_ENGAGE = 500000; //0.5s wait for AP
uint32_t time_cruise_engaged = 0;

//for safetyParam parsing
const uint16_t TESLA_HAS_AP_HARDWARE = 1;
const uint16_t TESLA_HAS_ACC = 2;
const uint16_t TESLA_OP_LONG_CONTROL = 4;
const uint16_t TESLA_HUD_INTEGRATION = 8;
const uint16_t TESLA_BODY_CONTROLS = 16;

bool has_ap_hardware = false;
bool has_acc = false;
bool has_op_long_control = false;
bool has_hud_integration = false;
bool has_body_controls = false;
int last_acc_status = -1;

const CanMsg TESLA_TX_MSGS[] = {
  {0x488, 0, 4},  // DAS_steeringControl - Lat Control
  {0x2B9, 0, 8},  // DAS_control - Long Control
  {0x209, 0, 8},  // DAS_longControl - Long Control
  {0x45,  0, 8},  // STW_ACTN_RQ - ACC Control
  {0x45,  2, 8},  // STW_ACTN_RQ - ACC Control
  {0x399, 0, 8},  // DAS_status - HUD
  {0x389, 0, 8},  // DAS_status2 - HUD
  {0x239, 0, 8},  // DAS_lanes - HUD
  {0x309, 0, 8},  // DAS_object - HUD
  {0x3A9, 0, 8},  // DAS_telemetry - HUD
  {0x3E9, 0, 8},  // DAS_bodyControls - Car Integration for turn signal on ALCA
};

AddrCheckStruct tesla_rx_checks[] = {
  {.msg = {{0x370, 0, 8, .expected_timestep = 40000U}}},   // EPAS_sysStatus (25Hz)
  {.msg = {{0x108, 0, 8, .expected_timestep = 10000U}}},   // DI_torque1 (100Hz)
  {.msg = {{0x118, 0, 6, .expected_timestep = 10000U}}},   // DI_torque2 (100Hz)
  {.msg = {{0x155, 0, 8, .expected_timestep = 20000U}}},   // ESP_B (50Hz)
  {.msg = {{0x20a, 0, 8, .expected_timestep = 20000U}}},   // BrakeMessage (50Hz)
  {.msg = {{0x368, 0, 8, .expected_timestep = 100000U}}},  // DI_state (10Hz)
  {.msg = {{0x318, 0, 8, .expected_timestep = 100000U}}},  // GTW_carState (10Hz)
  // {.msg = {{0x399, 2, 8, .expected_timestep = 500000U}}},  // AutopilotStatus (2Hz)
};
#define TESLA_RX_CHECK_LEN (sizeof(tesla_rx_checks) / sizeof(tesla_rx_checks[0]))

CanMsgFwd tesla_fwd_modded[] = {
  {.msg = {0x488,2,4},.fwd_to_bus=0,.expected_timestep = 50000U,.counter_mask_H=0x00000000,.counter_mask_L=0x000F0000}, // DAS_steeringControl - Lat Control - 20Hz
  {.msg = {0x2B9,2,8},.fwd_to_bus=0,.expected_timestep = 25000U,.counter_mask_H=0x00E00000,.counter_mask_L=0x00000000}, // DAS_control - Long Control - 40Hz
  {.msg = {0x209,2,8},.fwd_to_bus=0,.expected_timestep = 25000U,.counter_mask_H=0x00E00000,.counter_mask_L=0x00000000}, // DAS_longControl - Long Control - 40Hz
};
#define TESLA_FWD_CHECK_LEN (sizeof(tesla_fwd_modded) / sizeof(tesla_fwd_modded[0]))

bool autopilot_enabled = false;

static uint8_t tesla_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = (uint8_t)(addr) + (uint8_t)((unsigned int)(addr) >> 8U);
  for (int i = 0; i < (len - 1); i++) {
    checksum += (uint8_t)GET_BYTE(to_push, i);
  }
  return checksum;
}

static bool tesla_compute_fwd_checksum(CAN_FIFOMailBox_TypeDef *to_fwd) {
    uint8_t checksum = tesla_compute_checksum(to_fwd); 
    bool valid = false;
    int addr = GET_ADDR(to_fwd);

    if (addr == 0x488) {
      to_fwd->RDLR = (to_fwd->RDLR | (checksum << 24));
      valid = true;
    }

    if ((addr == 0x209) || (addr == 0x2B9)) {
      to_fwd->RDHR = (to_fwd->RDHR | (checksum << 24));
      valid = true;
    }

    return valid;
}

static bool tesla_compute_fwd_should_mod(CAN_FIFOMailBox_TypeDef *to_fwd) {
    bool valid = false;
    int addr = GET_ADDR(to_fwd);

    if (addr == 0x488) {
      valid = !autopilot_enabled;
    }

    if (addr == 0x209) {
      valid = !autopilot_enabled;
    }

    if (addr == 0x2B9) {
      valid = !autopilot_enabled;      
    }

    return valid;
}

static int tesla_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  //update gmlan for giraffe control
  if (board_has_gmlan())
  {
    //we're still in tesla safety mode, reset the timeout counter and make sure our output is enabled
    set_gmlan_digital_output(GMLAN_HIGH);
    reset_gmlan_switch_timeout(); 
  };

  bool valid = addr_safety_check(to_push, tesla_rx_checks, TESLA_RX_CHECK_LEN,
                                 NULL, NULL, NULL);

  if(valid) {
    int bus = GET_BUS(to_push);
    int addr = GET_ADDR(to_push);

    if(bus == 0) {
      if(addr == 0x370) {
        // Steering angle: (0.1 * val) - 819.2 in deg.
        // Store it 1/10 deg to match steering request
        int angle_meas_new = (((GET_BYTE(to_push, 4) & 0x3F) << 8) | GET_BYTE(to_push, 5)) - 8192;
        update_sample(&angle_meas, angle_meas_new);
      }

      if(addr == 0x155) {
        // Vehicle speed: (0.01 * val) * KPH_TO_MPS
        vehicle_speed = ((GET_BYTE(to_push, 5) << 8) | (GET_BYTE(to_push, 6))) * 0.01 / 3.6;
        vehicle_moving = vehicle_speed > 0.;
      }

      if(addr == 0x108) {
        // Gas pressed
        gas_pressed = (GET_BYTE(to_push, 6) != 0);
      }

      if(addr == 0x20a) {
        // Brake pressed
        brake_pressed = ((GET_BYTE(to_push, 0) & 0x0C) >> 2 != 1);
      }

      if(addr == 0x368) {
        // Cruise state
        int cruise_state = (GET_BYTE(to_push, 1) >> 4);
        bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                              (cruise_state == 3) ||  // STANDSTILL
                              (cruise_state == 4) ||  // OVERRIDE
                              (cruise_state == 6) ||  // PRE_FAULT
                              (cruise_state == 7);    // PRE_CANCEL

        if(cruise_engaged && !cruise_engaged_prev && !autopilot_enabled) {
          time_cruise_engaged = TIM2->CNT;
        }
        
        if((time_cruise_engaged !=0) && (get_ts_elapsed(TIM2->CNT,time_cruise_engaged) >= TIME_TO_ENGAGE)) {
          if (!autopilot_enabled) {
            controls_allowed = 1;
          }
          time_cruise_engaged = 0;
        }
        
        if(!cruise_engaged) {
          controls_allowed = 0;
        }

        cruise_engaged_prev = cruise_engaged;
      }
    }

    if (bus == 2) {
      if (addr == 0x399) {
        // Autopilot status
        int autopilot_status = (GET_BYTE(to_push, 0) & 0xF);
        autopilot_enabled = (autopilot_status == 3) ||  // ACTIVE_1
                            (autopilot_status == 4) ||  // ACTIVE_2
                            (autopilot_status == 5);    // ACTIVE_NAVIGATE_ON_AUTOPILOT
        if (autopilot_enabled) {
          controls_allowed = 0;
        }
      }

      if (addr == 0x2B9) {
        //AP1 DAS_control
        last_acc_status = ((GET_BYTE(to_push, 1)>> 4) & 0xF);
      }
    }

    // 0x488: DAS_steeringControl should not be received on bus 0
    generic_rx_checks((addr == 0x488) && (bus == 0));
  }

  return valid;
}


static int tesla_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  if(!msg_allowed(to_send, TESLA_TX_MSGS, sizeof(TESLA_TX_MSGS) / sizeof(TESLA_TX_MSGS[0]))) {
    tx = 0;
  }

  if(relay_malfunction) {
    tx = 0;
  }

  //do not allow long control if not enabled
  if ((!has_op_long_control) && ((addr == 0x2B9) || (addr == 0x209))) {
    //{0x2B9, 0, 8},  // DAS_control - Long Control
    //{0x209, 0, 8},  // DAS_longControl - Long Control
    tx = 0;
  }

  //do not allow body controls if not enabled
  if ((!has_body_controls) && (addr == 0x3E9)) {
    //{0x3E9, 0, 8},  // DAS_bodyControls
    tx = 0;
  }

  //do not allow hud integration messages if not enabled
  if ((!has_hud_integration) && ((addr == 0x399) || (addr == 0x389) || (addr == 0x239) ||(addr == 0x309) || (addr == 0x3A9))) {
    //{0x399, 0, 8},  // DAS_status - HUD
    //{0x389, 0, 8},  // DAS_status2 - HUD
    //{0x239, 0, 8},  // DAS_lanes - HUD
    //{0x309, 0, 8},  // DAS_object - HUD
    //{0x3A9, 0, 8},  // DAS_telemetry - HUD
    tx = 0;
  }

  if(addr == 0x488) {
    // Steering control: (0.1 * val) - 1638.35 in deg.
    // We use 1/10 deg as a unit here
    int raw_angle_can = (((GET_BYTE(to_send, 0) & 0x7F) << 8) | GET_BYTE(to_send, 1));
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = GET_BYTE(to_send, 2) >> 6;
    bool steer_control_enabled = (steer_control_type != 0) &&  // NONE
                                 (steer_control_type != 3);    // DISABLED

    // Rate limit while steering
    if(controls_allowed && steer_control_enabled) {
      // Add 1 to not false trigger the violation
      float delta_angle_float;
      delta_angle_float = (interpolate(TESLA_LOOKUP_ANGLE_RATE_UP, vehicle_speed) * TESLA_DEG_TO_CAN) + 1.;
      int delta_angle_up = (int)(delta_angle_float);
      delta_angle_float =  (interpolate(TESLA_LOOKUP_ANGLE_RATE_DOWN, vehicle_speed) * TESLA_DEG_TO_CAN) + 1.;
      int delta_angle_down = (int)(delta_angle_float);
      int highest_desired_angle = desired_angle_last + ((desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
      int lowest_desired_angle = desired_angle_last - ((desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

      // Check for violation;
      violation |= max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);
    }
    desired_angle_last = desired_angle;

    // Angle should be the same as current angle while not steering
    if(!controls_allowed && ((desired_angle < (angle_meas.min - 1)) || (desired_angle > (angle_meas.max + 1)))) {
      violation = true;
    }

    // No angle control allowed when controls are not allowed
    if(!controls_allowed && steer_control_enabled) {
      violation = true;
    }
  }

  if(addr == 0x45) {
    // No button other than cancel can be sent by us
    int control_lever_status = (GET_BYTE(to_send, 0) & 0x3F);
    if((control_lever_status != 0) && (control_lever_status != 1)) {
      violation = true;
    }
  }

  if(violation) {
    controls_allowed = 0;
    tx = 0;
  }

  if (fwd_data_message(to_send,tesla_fwd_modded,TESLA_FWD_CHECK_LEN,violation)) {
    //do not send if the message is in the forwards
    tx = 0;
  }

  return tx;
}

static int tesla_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  //we check to see first if these are modded forwards
  int fwd_modded = fwd_modded_message(to_fwd,tesla_fwd_modded,TESLA_FWD_CHECK_LEN,
            tesla_compute_fwd_should_mod,tesla_compute_fwd_checksum);
  if (fwd_modded != -2) {
    //it's a forward modded message, so just forward now
    return fwd_modded;
  }

  if(bus_num == 0) {
    // Chassis to autopilot

    //we need to modify EPAS_sysStatus->EPAS_eacStatus from 2 to 1 otherwise we can never 
    //engage AutoPilot. Once we send the steering commands from OP the status
    //changes from 1-AVAILABLE to 2-ACTIVE and AutoPilot becomes unavailable
    //The condition has to be:
    // IF controls_allowed AND EPAS_eacStatus = 2 THEN EPAS_eacStatus = 1
    if ((addr == 0x370) && (controls_allowed == 1) && (!autopilot_enabled)) {
      int epas_eacStatus = ((GET_BYTE(to_fwd, 6) & 0xE0) >> 5);
      //we only change from 2 to 1 leaving all other values alone
      if (epas_eacStatus == 2) {
        to_fwd->RDHR = (to_fwd->RDHR & 0x001FFFFF) | 0X00200000;
        to_fwd->RDHR = (to_fwd->RDHR | (tesla_compute_checksum(to_fwd) << 24));
      }
    }
    bus_fwd = 2;
  }

  if(bus_num == 2) {
    // Autopilot to chassis
    //0x488 DAS_steeringControl - Lat Control
    //0x2B9 DAS_control - Long Control
    //0x209 DAS_longControl - Long Control
    //0x399 DAS_status - HUD
    //0x389 DAS_status2 - HUD
    //0x239 DAS_lanes - HUD
    //0x309 DAS_object - HUD
    //0x3A9 DAS_telemetry - HUD
    //0x3E9 DAS_bodyControls - Car Integration for turn signal on ALCA
    bool is_lkas_msg = (addr == 0x488);
    bool is_acc_msg = has_op_long_control && ((addr == 0x2B9) || (addr == 0x209));
    bool is_hud_msg = has_hud_integration && ((addr == 0x399) || (addr == 0x389) || (addr == 0x239) || (addr == 0x309) || (addr == 0x3A9));
    bool is_bodyControl_msg = has_body_controls && (addr == 0x3E9);
    bool block_msg = controls_allowed && (is_lkas_msg || is_acc_msg || is_hud_msg || is_bodyControl_msg );
    if(!block_msg) {
      bus_fwd = 0;
    }
  }

  if(relay_malfunction) {
    bus_fwd = -1;
  }

  return bus_fwd;
}

static void tesla_init(int16_t param) {
  controls_allowed = 0;
  //init gmlan for giraffe control
  if (board_has_gmlan())
  {
    gmlan_switch_init(1);
  };
  relay_malfunction_reset();
  has_ap_hardware = GET_FLAG(param, TESLA_HAS_AP_HARDWARE);
  has_acc = GET_FLAG(param, TESLA_HAS_ACC);
  has_op_long_control = GET_FLAG(param, TESLA_OP_LONG_CONTROL);
  has_hud_integration = GET_FLAG(param, TESLA_HUD_INTEGRATION);
  has_body_controls = GET_FLAG(param, TESLA_BODY_CONTROLS);
}

const safety_hooks tesla_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .tx = tesla_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = tesla_fwd_hook,
  .addr_check = tesla_rx_checks,
  .addr_check_len = TESLA_RX_CHECK_LEN,
};
