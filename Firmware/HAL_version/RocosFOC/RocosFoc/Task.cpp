//
// Created by think on 2022/10/7.
//

#include "Task.h"
#include "usbd_cdc_if.h"
#include <SimpleFOC.h>

const char* target_string = "target velocity";
const char* motor_string = "motor cmd";

//!<<<<<<<<<<<<<<Variable Definition>>>>>>>>>>>>>>>>>!//
MagneticSensorSPI sensor;

BLDCMotor motor = BLDCMotor(14);

BLDCDriver3PWM driver = BLDCDriver3PWM();

//目标变量
float target_velocity = 10;

Commander commander = Commander(SerialUSB);

void doMotor(char* cmd){ commander.motor(&motor, cmd); }
void doTarget(char* cmd) { commander.scalar(&target_velocity, cmd); }



//!<<<<<<<<<<<<<<Function Definition>>>>>>>>>>>>>>>>>!//

uint16_t SPI_AS5048A_ReadData(void)
{
    uint16_t angle_value;
    uint16_t command = 0xFFFF;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //cs鐗囬??
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&command, (uint8_t*)&angle_value, 1, 100);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //cs鍙栨秷鐗囬??
    angle_value = angle_value & 0x3FFF;

    return angle_value;
}


//!<<<<<<<<<<<<<<<<<<<<Setup>>>>>>>>>>>>>>>>>>>//
/**
 * @description TaskSetup Wrap c/cpp for initialization, used before while(1)
 */
void TaskSetup(void) {
    sensor.init();

    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 24.0;
    driver.init();
    motor.linkDriver(&driver);

//    motor.controller = MotionControlType::velocity_openloop;
//    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;


    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 1;
    motor.PID_velocity.D = 0;

    motor.velocity_limit = 50;

    motor.voltage_limit = 16.8;

    motor.useMonitoring(SerialUSB);

//    motor.monitor_downsample = 10; // default is 10
    motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;

    motor.init();
    motor.initFOC(); //align sensor and start FOC

    commander.add('T', doTarget, const_cast<char *>(target_string));
    commander.add('M', doMotor, const_cast<char *>(motor_string));

//    SerialUSB.println(F("Double motor sketch ready."));

    _delay(100);

}

//!<<<<<<<<<<<<<<<<<<<<Loop>>>>>>>>>>>>>>>>>>>//
/**
 * @description TaskDo Wrap c/cpp for cycling call, used inside while(1)
 */
void TaskDo(void) {
//      if(get_data_flag) {
//          usb_printf("get data:\r\n%s %d\r\n", buf, data_nums);
//          get_data_flag = 0;
//      }
//      sensorSpi.update();
//      float angle = sensorSpi.getAngle();
//      usb_printf("Angle1 is: %.4f\r\n", angle);
//      angle = SPI_AS5048A_ReadData() /16383.0 * 360.0;
//      usb_printf("Angle2 is: %.4f\r\n", angle);
//
//      HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
//
//      _writeDutyCycle3PWM(0.5, 0.2, 0.3);
//      HAL_Delay(1000);

    // main FOC algorithm function
    // the faster you run this function the better
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz
    motor.loopFOC();
    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    // You can also use motor.move() and set the motor.target in the code
    motor.move(target_velocity);

    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!
//     motor.monitor();


}