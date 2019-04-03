#ifndef COMMAND_CODE_HEADER
#define COMMAND_CODE_HEADER

typedef char CommandCode;

const CommandCode COMM_CODE_REQUEST_DATA = '?';
const CommandCode COMM_CODE_GET_TORQUE_SETPOINT = 'D';
const CommandCode COMM_CODE_SET_TORQUE_SETPOINT = 'F';
const CommandCode COMM_CODE_START_TRIAL = 'E';
const CommandCode COMM_CODE_END_TRIAL = 'G';
const CommandCode COMM_CODE_CALIBRATE_TORQUE = 'H';
const CommandCode COMM_CODE_GET_PID_PARAMS = 'K';
const CommandCode COMM_CODE_CALIBRATE_FSR = 'L';
const CommandCode COMM_CODE_SET_PID_PARAMS = 'M';
const CommandCode COMM_CODE_CHECK_BLUETOOTH = 'N';
const CommandCode COMM_CODE_CHECK_MEMORY = '<';
const CommandCode COMM_CODE_CLEAR_MEMORY = '>';
const CommandCode COMM_CODE_SET_KF = '_';
const CommandCode COMM_CODE_GET_KF = '`';
const CommandCode COMM_CODE_SET_SMOOTHING_PARAMS = ')';
const CommandCode COMM_CODE_GET_SMOOTHING_PARAMS = '(';
const CommandCode COMM_CODE_SET_FREQ_BASELINE = 'P';
const CommandCode COMM_CODE_ADJ_N3 = 'O';
const CommandCode COMM_CODE_GET_FSR_THRESHOLD = 'Q';
const CommandCode COMM_CODE_SET_FSR_THRESHOLD = 'R';
const CommandCode COMM_CODE_SET_PERC = 'S';
const CommandCode COMM_CODE_CLEAN_BLUETOOTH_BUFFER = 'C';
const CommandCode COMM_CODE_STOP_N3_ADJ = 'T';
const CommandCode COMM_CODE_STOP_TORQUE_ADJ = 'I';
const CommandCode COMM_CODE_SAVE_EXP_PARAMS = '!';
const CommandCode COMM_CODE_MODIFY_ZERO = 'W';
const CommandCode COMM_CODE_GET_GAIN = '{';
const CommandCode COMM_CODE_SET_GAIN = '}';
const CommandCode COMM_CODE_ACTIVATE_PROP_CTRL = '+';
const CommandCode COMM_CODE_DEACTIVATE_PROP_CTRL = '=';
const CommandCode COMM_CODE_ACTIVATE_AUTO_KF = '.';
const CommandCode COMM_CODE_DEACTIVATE_AUTO_KF = ';';
const CommandCode COMM_CODE_ACTIVATE_PROP_PIVOT_CTRL = '#';
const CommandCode COMM_CODE_DEACTIVATE_PROP_PIVOT_CTRL = '^';
const CommandCode COMM_CODE_GET_BASELINE = 'B';
const CommandCode COMM_CODE_CALC_BASELINE = 'b';
const CommandCode COMM_CODE_CALC_BALANCE_BASELINE = '&';

#endif
