#include <cstdlib>
#include "Commander.h"
#include <ctype.h>
#include "usbd_cdc_if.h"

#define F(msg) (msg)

Commander *commander_ptr = nullptr;

Commander::Commander(Print &serial, char eol, bool echo) {
    com_port = &serial;
    this->eol = eol;
    this->echo = echo;
    commander_ptr = this;
}

Commander::Commander(char eol, bool echo) {
    this->eol = eol;
    this->echo = echo;
    commander_ptr = this;
}

void Commander::add(char id, CommandCallback onCommand, char *label) {
    call_list[call_count] = onCommand;
    call_ids[call_count] = id;
    call_label[call_count] = label;
    call_count++;
}

void Commander::run() {
    if (!com_port) return;
    run(*com_port, eol);
}

void Commander::run(Print &serial, char eol) {
    com_port = &serial;
//  Print* tmp = com_port; // save the serial instance
//  char eol_tmp = this->eol;
//  this->eol = eol;
//  com_port = &serial;
//
//  // a string to hold incoming data
//  while (serial.available()) {
//    // get the new byte:
//    int ch = serial.read();
//    received_chars[rec_cnt++] = (char)ch;
//    // end of user input
//    if(echo)
//      print((char)ch);
//    if (isSentinel(ch)) {
//      // execute the user command
//      run(received_chars);
//
//      // reset the command buffer
//      received_chars[0] = 0;
//      rec_cnt=0;
//    }
//    if (rec_cnt>=MAX_COMMAND_LENGTH) { // prevent buffer overrun if message is too long
//        received_chars[0] = 0;
//        rec_cnt=0;
//    }
//  }
//
//  com_port = tmp; // reset the instance to the internal value
//  this->eol = eol_tmp;
}

void Commander::run(char *user_input) {
    // execute the user command
    char id = user_input[0];
    switch (id) {
        case CMD_SCAN: //!< modified by Yang Luo
            for (int i = 0; i < call_count; i++) {
                send_buffer += call_ids[i];
                send_buffer += ":";
                if (call_label[i])
                    send_buffer += call_label[i];
                send_buffer += "\r\n";
            }
            printBuffer();
            break;
        case CMD_VERBOSE: //!< modified by Yang Luo
            if (!isSentinel(user_input[1])) verbose = (VerboseMode) atoi(&user_input[1]);
            send_buffer += getVerbose("Verb:");
            switch (verbose) {
                case VerboseMode::nothing:
                    send_buffer += "off!";
                    break;
                case VerboseMode::on_request:
                case VerboseMode::user_friendly:
                    send_buffer += "on!";
                    break;
            }
            send_buffer += "\r\n";
            printBuffer();
            break;
        case CMD_DECIMAL: //!< modified by Yang Luo
            if (!isSentinel(user_input[1])) decimal_places = atoi(&user_input[1]);
            send_buffer += getVerbose("Decimal:");
            send_buffer += std::to_string(decimal_places);
            send_buffer += "\r\n";
            printBuffer();
            break;
        default:
            for (int i = 0; i < call_count; i++) {
                if (id == call_ids[i]) {
                    call_list[i](&user_input[1]);
                    break;
                }
            }
            break;
    }
}

/// \brief Parse motor command
/// \param motor
/// \param user_command for example, if the <b>command</b> is "MMG6", then the <b>user_command</b> is "MG6"
void Commander::motor(FOCMotor *motor, char *user_command) {

    // if target setting
    if (isdigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+') {
        target(motor, user_command);
        return;
    }

    // parse command letter
    char cmd = user_command[0]; //!< MG6->M, T1->T
    char sub_cmd = user_command[1]; //!< MG6->G, T1->1
    // check if there is a subcommand or not
    int value_index = (sub_cmd >= 'A' && sub_cmd <= 'Z') ? 2 : 1; //!< if still alphabet, then value is from index 2
    // check if get command
    bool GET = isSentinel(user_command[value_index]); //!< some sub_command is set if has value, or is get.
    // parse command values
    float value = atof(&user_command[value_index]);


    // a bit of optimisation of variable memory for Arduino UNO (atmega328)
    switch (cmd) {
        case CMD_C_Q_PID:      //
            send_buffer += getVerbose(F("PID curr q| "));
            if (sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
            else pid(&motor->PID_current_q, &user_command[1]);
            break;
        case CMD_C_D_PID:      //
            send_buffer += getVerbose(F("PID curr d| "));
            if (sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
            else pid(&motor->PID_current_d, &user_command[1]);
            break;
        case CMD_V_PID:      //
            send_buffer += getVerbose(F("PID vel| "));
            if (sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
            else pid(&motor->PID_velocity, &user_command[1]);
            break;
        case CMD_A_PID:      //
            send_buffer += getVerbose(F("PID angle| "));
            if (sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
            else pid(&motor->P_angle, &user_command[1]);
            break;
        case CMD_LIMITS:      //
            send_buffer += getVerbose(F("Limits| "));
            switch (sub_cmd) {
                case SCMD_LIM_VOLT:      // voltage limit change
                    send_buffer += getVerbose(F("volt: "));
                    if (!GET) {
                        motor->voltage_limit = value;
                        motor->PID_current_d.limit = value;
                        motor->PID_current_q.limit = value;
                        // change velocity pid limit if in voltage mode and no phase resistance set
                        if (!_isset(motor->phase_resistance) &&
                            motor->torque_controller == TorqueControlType::voltage)
                            motor->PID_velocity.limit = value;
                    }
//          println(motor->voltage_limit);
                    send_buffer += std::to_string(motor->voltage_limit);
                    printBufferln();
                    break;
                case SCMD_LIM_CURR:      // current limit
                    send_buffer += getVerbose(F("curr: "));
                    if (!GET) {
                        motor->current_limit = value;
                        // if phase resistance specified or the current control is on set the current limit to the velocity PID
                        if (_isset(motor->phase_resistance) ||
                            motor->torque_controller != TorqueControlType::voltage)
                            motor->PID_velocity.limit = value;
                    }
//          println(motor->current_limit);
                    send_buffer += std::to_string(motor->current_limit);
                    printBufferln();
                    break;
                case SCMD_LIM_VEL:      // velocity limit
                    send_buffer += getVerbose(F("vel: "));
                    if (!GET) {
                        motor->velocity_limit = value;
                        motor->P_angle.limit = value;
                    }
//          println(motor->velocity_limit);
                    send_buffer += std::to_string(motor->velocity_limit);
                    printBufferln();
                    break;
                default:
                    printError();
                    break;
            }
            break;
        case CMD_MOTION_TYPE:
        case CMD_TORQUE_TYPE:
        case CMD_STATUS:
            motion(motor, &user_command[0]);
            break;
        case CMD_PWMMOD:
            // PWM modulation change
            send_buffer += getVerbose(F("PWM Mod | "));
            switch (sub_cmd) {
                case SCMD_PWMMOD_TYPE:      // zero offset
                    send_buffer += getVerbose(F("type: "));
                    if (!GET) motor->foc_modulation = (FOCModulationType) value;
                    switch (motor->foc_modulation) {
                        case FOCModulationType::SinePWM:
//              println(F("SinePWM"));
                            send_buffer += "SinePWM";
                            printBufferln();
                            break;
                        case FOCModulationType::SpaceVectorPWM:
//              println(F("SVPWM"));
                            send_buffer += "SVPWM";
                            printBufferln();
                            break;
                        case FOCModulationType::Trapezoid_120:
//              println(F("Trap 120"));
                            send_buffer += "Trap 120";
                            printBufferln();
                            break;
                        case FOCModulationType::Trapezoid_150:
//              println(F("Trap 150"));
                            send_buffer += "Trap 150";
                            printBufferln();
                            break;
                    }
                    break;
                case SCMD_PWMMOD_CENTER:      // centered modulation
                    send_buffer += getVerbose(F("center: "));
                    if (!GET) motor->modulation_centered = value;
//          println(motor->modulation_centered);
                    send_buffer += std::to_string(motor->modulation_centered);
                    printBufferln();
                    break;
                default:
                    printError();
                    break;
            }
            break;
        case CMD_RESIST:
            send_buffer += getVerbose(F("R phase: "));
            if (!GET) {
                motor->phase_resistance = value;
                if (motor->torque_controller == TorqueControlType::voltage)
                    motor->PID_velocity.limit = motor->current_limit;
            }
            if (_isset(motor->phase_resistance)) {
//                println(motor->phase_resistance);
                send_buffer += std::to_string(motor->phase_resistance);
                printBufferln();
            }
            else {
//                println(0);
                send_buffer += std::to_string(0);
                printBufferln();
            }
            break;
        case CMD_KV_RATING:
            send_buffer += getVerbose(F("Motor KV: "));
            if (!GET) {
                motor->KV_rating = value;
            }
            if (_isset(motor->KV_rating)) {
//                println(motor->KV_rating);
                send_buffer += std::to_string(motor->KV_rating);
                printBufferln();
            }
            else {
//                println(0);
                send_buffer += std::to_string(0);
                printBufferln();
            }
            break;
        case CMD_SENSOR:
            // Sensor zero offset
            send_buffer += getVerbose(F("Sensor | "));
            switch (sub_cmd) {
                case SCMD_SENS_MECH_OFFSET:      // zero offset
                    send_buffer += getVerbose(F("offset: "));
                    if (!GET) motor->sensor_offset = value;
//                    println(motor->sensor_offset);
                    send_buffer += std::to_string(motor->sensor_offset);
                    printBufferln();
                    break;
                case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
                    send_buffer += getVerbose(F("el. offset: "));
                    if (!GET) motor->zero_electric_angle = value;
//                    println(motor->zero_electric_angle);
                    send_buffer += std::to_string(motor->zero_electric_angle);
                    printBufferln();
                    break;
                default:
                    printError();
                    break;
            }
            break;
        case CMD_MONITOR:     // get current values of the state variables
            send_buffer += getVerbose(F("Monitor | "));
            switch (sub_cmd) {
                case SCMD_GET:      // get command
                    switch ((uint8_t) value) {
                        case 0: // get target
                            send_buffer += getVerbose(F("target: "));
//                            println(motor->target);
                            send_buffer += std::to_string(motor->target);
                            printBufferln();
                            break;
                        case 1: // get voltage q
                            send_buffer += getVerbose(F("Vq: "));
//                            println(motor->voltage.q);
                            send_buffer += std::to_string(motor->voltage.q);
                            printBufferln();
                            break;
                        case 2: // get voltage d
                            send_buffer += getVerbose(F("Vd: "));
//                            println(motor->voltage.d);
                            send_buffer += std::to_string(motor->voltage.d);
                            printBufferln();
                            break;
                        case 3: // get current q
                            send_buffer += getVerbose(F("Cq: "));
//                            println(motor->current.q);
                            send_buffer += std::to_string(motor->current.q);
                            printBufferln();
                            break;
                        case 4: // get current d
                            send_buffer += getVerbose(F("Cd: "));
//                            println(motor->current.d);
                            send_buffer += std::to_string(motor->current.d);
                            printBufferln();
                            break;
                        case 5: // get velocity
                            send_buffer += getVerbose(F("vel: "));
//                            println(motor->shaft_velocity);
                            send_buffer += std::to_string(motor->shaft_velocity);
                            printBufferln();
                            break;
                        case 6: // get angle
                            send_buffer += getVerbose(F("angle: "));
//                            println(motor->shaft_angle);
                            send_buffer += std::to_string(motor->shaft_angle);
                            printBufferln();
                            break;
                        case 7: // get all states
                            send_buffer += getVerbose(F("all: "));
                            send_buffer += std::to_string(motor->shaft_angle);
                            send_buffer += ";";
                            send_buffer += std::to_string(motor->voltage.q);
                            send_buffer += ";";
                            send_buffer += std::to_string(motor->voltage.d);
                            send_buffer += ";";
                            send_buffer += std::to_string(motor->current.q);
                            send_buffer += ";";
                            send_buffer += std::to_string(motor->current.d);
                            send_buffer += ";";
                            send_buffer += std::to_string(motor->shaft_velocity);
                            send_buffer += ";";
                            send_buffer += std::to_string(motor->shaft_angle);
                            printBufferln();
                            break;
                        default:
                            printError();
                            break;
                    }
                    break;
                case SCMD_DOWNSAMPLE:
                    send_buffer += getVerbose(F("downsample: "));
                    if (!GET) motor->monitor_downsample = value;
                    send_buffer += std::to_string(motor->monitor_downsample);
                    printBufferln();
//                    println((int) motor->monitor_downsample);
                    break;
                case SCMD_CLEAR:
                    motor->monitor_variables = (uint8_t) 0;
//                    println(F("clear"));
                    send_buffer += "clear";
                    printBufferln();
                    break;
                case SCMD_SET:
                    if (!GET) motor->monitor_variables = (uint8_t) 0;
                    for (int i = 0; i < 7; i++) {
                        if (isSentinel(user_command[value_index + i])) break;
                        if (!GET) motor->monitor_variables |= (user_command[value_index + i] - '0') << (6 - i);
//                        print((user_command[value_index + i] - '0'));
                        send_buffer += std::to_string((user_command[value_index + i] - '0'));
                    }
//                    println("");
                    printBufferln();
                    break;
                default:
                    printError();
                    break;
            }
            break;
        default:  // unknown cmd
            send_buffer += getVerbose(F("unknown cmd "));
            printError();
    }
}

void Commander::motion(FOCMotor *motor, char *user_cmd, char *separator) {
    char cmd = user_cmd[0];
    char sub_cmd = user_cmd[1];
    bool GET = isSentinel(user_cmd[1]);
    float value = atof(&user_cmd[(sub_cmd >= 'A' && sub_cmd <= 'Z') ? 2 : 1]);

    switch (cmd) {
        case CMD_MOTION_TYPE:
            send_buffer += getVerbose(F("Motion:"));
            switch (sub_cmd) {
                case SCMD_DOWNSAMPLE:
                    send_buffer += getVerbose(F(" downsample: "));
                    if (!GET) motor->motion_downsample = value;
//                    println((int) motor->motion_downsample);
                    send_buffer += std::to_string(motor->motion_downsample);
                    printBufferln();
                    break;
                default:
                    // change control type
                    if (!GET && value >= 0 && (int) value < 5) // if set command
                        motor->controller = (MotionControlType) value;
                    switch (motor->controller) {
                        case MotionControlType::torque:
//                            println(F("torque"));
                            send_buffer += "torque";
                            printBufferln();
                            break;
                        case MotionControlType::velocity:
//                            println(F("vel"));
                            send_buffer += "vel";
                            printBufferln();
                            break;
                        case MotionControlType::angle:
//                            println(F("angle"));
                            send_buffer += "angle";
                            printBufferln();
                            break;
                        case MotionControlType::velocity_openloop:
//                            println(F("vel open"));
                            send_buffer += "vel open";
                            printBufferln();
                            break;
                        case MotionControlType::angle_openloop:
//                            println(F("angle open"));
                            send_buffer += "angle open";
                            printBufferln();
                            break;
                    }
                    break;
            }
            break;
        case CMD_TORQUE_TYPE:
            // change control type
            send_buffer += getVerbose(F("Torque: "));
            if (!GET && (int8_t) value >= 0 && (int8_t) value < 3)// if set command
                motor->torque_controller = (TorqueControlType) value;
            switch (motor->torque_controller) {
                case TorqueControlType::voltage:
//                    println(F("volt"));
                    send_buffer += "volt";
                    printBufferln();
                    // change the velocity control limits if necessary
                    if (!_isset(motor->phase_resistance)) motor->PID_velocity.limit = motor->voltage_limit;
                    break;
                case TorqueControlType::dc_current:
//                    println(F("dc curr"));
                    send_buffer += "dc curr";
                    printBufferln();
                    // change the velocity control limits if necessary
                    motor->PID_velocity.limit = motor->current_limit;
                    break;
                case TorqueControlType::foc_current:
//                    println(F("foc curr"));
                    send_buffer += "foc curr";
                    printBufferln();
                    // change the velocity control limits if necessary
                    motor->PID_velocity.limit = motor->current_limit;
                    break;
            }
            break;
        case CMD_STATUS:
            // enable/disable
            send_buffer += getVerbose(F("Status: "));
            if (!GET) (bool) value ? motor->enable() : motor->disable();
//            println(motor->enabled);
            send_buffer += std::to_string(motor->enabled);
            printBufferln();
            break;
        default:
            target(motor, user_cmd, separator);
            break;
    }
}

void Commander::pid(PIDController *pid, char *user_cmd) {
    char cmd = user_cmd[0];
    bool GET = isSentinel(user_cmd[1]);
    float value = atof(&user_cmd[1]);

    switch (cmd) {
        case SCMD_PID_P:      // P gain change
            send_buffer += getVerbose("P: ");
            if (!GET) pid->P = value;
//            println(pid->P);
            send_buffer += std::to_string(pid->P);
            printBufferln();
            break;
        case SCMD_PID_I:      // I gain change
            send_buffer += getVerbose("I: ");
            if (!GET) pid->I = value;
//            println(pid->I);
            send_buffer += std::to_string(pid->I);
            printBufferln();
            break;
        case SCMD_PID_D:      // D gain change
            send_buffer += getVerbose("D: ");
            if (!GET) pid->D = value;
//            println(pid->D);
            send_buffer += std::to_string(pid->D);
            printBufferln();
            break;
        case SCMD_PID_RAMP:      //  ramp change
            send_buffer += getVerbose("ramp: ");
            if (!GET) pid->output_ramp = value;
//            println(pid->output_ramp);
            send_buffer += std::to_string(pid->output_ramp);
            printBufferln();
            break;
        case SCMD_PID_LIM:      //  limit change
            send_buffer += getVerbose("limit: ");
            if (!GET) pid->limit = value;
//            println(pid->limit);
            send_buffer += std::to_string(pid->limit);
            printBufferln();
            break;
        default:
            printError();
            break;
    }
}

void Commander::lpf(LowPassFilter *lpf, char *user_cmd) {
    char cmd = user_cmd[0];
    bool GET = isSentinel(user_cmd[1]);
    float value = atof(&user_cmd[1]);

    switch (cmd) {
        case SCMD_LPF_TF:      // Tf value change
            printVerbose(F("Tf: "));
            if (!GET) lpf->Tf = value;
//            println(lpf->Tf);
            send_buffer += std::to_string(lpf->Tf);
            printBufferln();
            break;
        default:
            printError();
            break;
    }
}

void Commander::scalar(float *value, char *user_cmd) {
    bool GET = isSentinel(user_cmd[0]);
    if (!GET) *value = atof(user_cmd);
    println(*value);
}


void Commander::target(FOCMotor *motor, char *user_cmd, char *separator) {
    // if no values sent
    if (isSentinel(user_cmd[0])) return;

    float pos, vel, torque;
    char *next_value;
    switch (motor->controller) {
        case MotionControlType::torque: // setting torque target
            torque = atof(strtok(user_cmd, separator));
            motor->target = torque;
            break;
        case MotionControlType::velocity: // setting velocity target + torque limit
            // set the target
            vel = atof(strtok(user_cmd, separator));
            motor->target = vel;

            // allow for setting only the target velocity without chaning the torque limit
            next_value = strtok(NULL, separator);
            if (next_value) {
                torque = atof(next_value);
                motor->PID_velocity.limit = torque;
                // torque command can be voltage or current
                if (!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage)
                    motor->voltage_limit = torque;
                else motor->current_limit = torque;
            }
            break;
        case MotionControlType::angle: // setting angle target + torque, velocity limit
            // setting the target position
            pos = atof(strtok(user_cmd, separator));
            motor->target = pos;

            // allow for setting only the target position without chaning the velocity/torque limits
            next_value = strtok(NULL, separator);
            if (next_value) {
                vel = atof(next_value);
                motor->velocity_limit = vel;
                motor->P_angle.limit = vel;

                // allow for setting only the target position and velocity limit without the torque limit
                next_value = strtok(NULL, separator);
                if (next_value) {
                    torque = atof(next_value);
                    motor->PID_velocity.limit = torque;
                    // torque command can be voltage or current
                    if (!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage)
                        motor->voltage_limit = torque;
                    else motor->current_limit = torque;
                }
            }
            break;
        case MotionControlType::velocity_openloop: // setting velocity target + torque limit
            // set the target
            vel = atof(strtok(user_cmd, separator));
            motor->target = vel;
            // allow for setting only the target velocity without chaning the torque limit
            next_value = strtok(NULL, separator);
            if (next_value) {
                torque = atof(next_value);
                // torque command can be voltage or current
                if (!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
                else motor->current_limit = torque;
            }
            break;
        case MotionControlType::angle_openloop: // setting angle target + torque, velocity limit
            // set the target
            pos = atof(strtok(user_cmd, separator));
            motor->target = pos;

            // allow for setting only the target position without chaning the velocity/torque limits
            next_value = strtok(NULL, separator);
            if (next_value) {
                vel = atof(next_value);
                motor->velocity_limit = vel;
                // allow for setting only the target velocity without chaning the torque limit
                next_value = strtok(NULL, separator);
                if (next_value) {
                    torque = atof(next_value);
                    // torque command can be voltage or current
                    if (!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
                    else motor->current_limit = torque;
                }
            }
            break;
    }
    send_buffer += getVerbose(F("Target: "));
    send_buffer += std::to_string(motor->target);
    printBufferln();
//    println(motor->target);
}


bool Commander::isSentinel(char ch) {
    if (ch == eol)
        return true;
    else if (ch == '\r') {
        printVerbose(F("Warn: \\r detected! \n"));
    }
    return false;
}

void Commander::print(const int number) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->print(number);
}

void Commander::print(const float number) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->print((float) number, (int) decimal_places);
}

void Commander::print(const char *message) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->print(message);
}

//void Commander::print(const __FlashStringHelper *message){
//  if(!com_port || verbose == VerboseMode::nothing ) return;
//  com_port->print(message);
//}
void Commander::print(const char message) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->print(message);
}

void Commander::println(const int number) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->println(number);
}

void Commander::println(const float number) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->println((float) number, (int) decimal_places);
}

void Commander::println(const char *message) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->println(message);
}

//void Commander::println(const __FlashStringHelper *message){
//  if(!com_port || verbose == VerboseMode::nothing ) return;
//  com_port->println(message);
//}
void Commander::println(const char message) {
    if (!com_port || verbose == VerboseMode::nothing) return;
    com_port->println(message);
}


void Commander::printVerbose(const char *message) {
    if (verbose == VerboseMode::user_friendly) print(message);
}

//void Commander::printVerbose(const __FlashStringHelper *message){
//  if(verbose == VerboseMode::user_friendly) print(message);
//}

std::string Commander::getVerbose(const char *message) {
    std::string s;
    if (verbose == VerboseMode::user_friendly)
        s += message;

    return s;
}

void Commander::printError() {
    println(F("err"));
}

Commander::~Commander() {
    commander_ptr = nullptr;
}

void Commander::run(uint8_t *Buf, uint32_t *Len) {

    memset(received_chars, 0, MAX_COMMAND_LENGTH);

    if (*Len >= MAX_COMMAND_LENGTH) { // prevent buffer overrun if message is too long
        return;
    }

    memcpy(received_chars, Buf, *Len);

    if (echo)
        print(received_chars);
    if (isSentinel(received_chars[*Len - 1])) { //!< Judge the last character is '\n'
        // execute the user command
//        usb_printf("correct");
        run(received_chars);
    }
}

void Commander::printBuffer() {
    print(send_buffer.c_str());
    send_buffer.clear();
}

void Commander::printBufferln() {
    send_buffer += "\r\n";
    printBuffer();
}

void UsbdCdcRecieveFsCallback(uint8_t *Buf, uint32_t *Len) {
    if (commander_ptr) {
        commander_ptr->run(Buf, Len);
    }
}