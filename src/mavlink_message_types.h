#ifndef MAVLINK_MESSAGE_TYPES_H
#define MAVLINK_MESSAGE_TYPES_H

// MAVLink message ID definitions
const char* getMessageType(uint8_t msgId) {
  switch (msgId) {
    case 0:   return "HEARTBEAT";
    case 1:   return "SYS_STATUS";
    case 2:   return "SYSTEM_TIME";
    case 4:   return "PING";
    case 20:  return "PARAM_REQUEST_READ";
    case 21:  return "PARAM_REQUEST_LIST";
    case 22:  return "PARAM_VALUE";
    case 23:  return "PARAM_SET";
    case 24:  return "GPS_RAW_INT";
    case 27:  return "RAW_IMU";
    case 29:  return "SCALED_PRESSURE";
    case 30:  return "ATTITUDE";
    case 31:  return "ATTITUDE_QUATERNION";
    case 32:  return "LOCAL_POSITION_NED";
    case 33:  return "GLOBAL_POSITION_INT";
    case 36:  return "SERVO_OUTPUT_RAW";
    case 42:  return "MISSION_CURRENT";
    case 43:  return "AHRS2";
    case 62:  return "NAV_CONTROLLER_OUTPUT";
    case 65:  return "RC_CHANNELS";
    case 66:  return "GPS_STATUS";
    case 67:  return "SET_MODE";
    case 70:  return "RC_CHANNELS_OVERRIDE";
    case 74:  return "VFR_HUD";
    case 75:  return "COMMAND_INT";
    case 76:  return "COMMAND_LONG";
    case 77:  return "COMMAND_ACK";
    case 82:  return "VELOCITY_TARGET_LOCAL_NED";
    case 84:  return "POSITION_TARGET_LOCAL_NED";
    case 87:  return "POSITION_TARGET_GLOBAL_INT";
    case 88:  return "VELOCITY_TARGET_GLOBAL_INT";
    case 100: return "GLOBAL_VISION_POSITION_ESTIMATE";
    case 101: return "LOCAL_VISION_POSITION_ESTIMATE";
    case 102: return "VISION_POSITION_ESTIMATE";
    case 103: return "VISION_SPEED_ESTIMATE";
    case 109: return "RADIO_STATUS";
    case 110: return "FILE_TRANSFER_PROTOCOL";
    case 111: return "TIMESYNC";
    case 115: return "RC_CHANNELS";
    case 116: return "SCALED_IMU2";
    case 121: return "LOG_ERASE";
    case 125: return "POWER_STATUS";
    case 129: return "SCALED_IMU3";
    case 136: return "TERRAIN_REPORT";
    case 137: return "SCALED_PRESSURE2";
    case 147: return "BATTERY_STATUS";
    case 148: return "AUTOPILOT_VERSION";
    case 150: return "EKF_STATUS_REPORT";
    case 156: return "MISSION_REQUEST";
    case 159: return "MISSION_ACK";
    case 210: return "NAV_CONTROLLER_OUTPUT";
    case 241: return "VIBRATION";
    case 242: return "HOME_POSITION";
    case 250: return "LOG_REQUEST_DATA";
    case 253: return "STATUSTEXT";
    default:  return "UNKNOWN_MESSAGE_TYPE";
  }
}

#endif // MAVLINK_MESSAGE_TYPES_H
