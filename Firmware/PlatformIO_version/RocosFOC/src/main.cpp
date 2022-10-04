#include "Arduino.h"
#include "SimpleFOC.h"
#include "USBSerial.h"
#include "SPI.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"

#define SPI_MISO   PA6
#define SPI_MOSI   PA7
#define SPI_SCLK   PA5
#define SPI_CS     PA4

#define PWM_A      PB4
#define PWM_B      PB5
#define PWM_C      PB0
#define DRV_EN     PC15

#define SENSOR_A   PA0
#define SENSOR_B   PA1

// MagneticSensorAS5048A sensor1(SPI_CS, false); // false - 是否开启快速读取模式
// MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, SPI_CS);
// SPIClass spi_1(SPI_MOSI, SPI_MISO, SPI_SCLK);

//电机
BLDCMotor motor1 = BLDCMotor(14);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PB4, PB5, PB0, DRV_EN);

// InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, SENSOR_A, SENSOR_B);

//目标变量
float target_velocity = 5;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void onMotor(char* cmd){ command.motor(&motor1,cmd); }      // 串口控制指令：电机


void setup() {
  // put your setup code here, to run once:
  // spi_1.begin();
  // sensor1.init(&spi_1);

  // motor1.linkSensor(&sensor1);

  driver1.voltage_power_supply = 16.8;
  driver1.init();

  motor1.linkDriver(&driver1);

  // 电流检测
  // current_sense1.init();
  // current_sense1.gain_b *= -1;
  // // current_sense1.skip_align = true;
  // motor1.linkCurrentSense(&current_sense1);

  motor1.voltage_limit = 16.8;   // [V]
  motor1.velocity_limit = 40; // [rad/s]


  //FOC模型选择
  // motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // 控制环
  motor1.controller = MotionControlType::velocity_openloop;
  // motor1.torque_controller = TorqueControlType::foc_current; 
  // motor1.controller = MotionControlType::torque;


  // FOC电流控制PID参数
  // motor1.PID_current_q.P = 5;
  // motor1.PID_current_q.I= 100;
  // motor1.PID_current_d.P= 5;
  // motor1.PID_current_d.I = 100;
  // motor1.LPF_current_q.Tf = 0.002; // 1ms default
  // motor1.LPF_current_d.Tf = 0.002; // 1ms default

  
  // // maximal voltage to be set to the motor
  // motor1.voltage_limit = 24;


  // // 速度环PID参数
  // motor1.PID_velocity.P = 0.2;
  // motor1.PID_velocity.I = 10;
  // motor1.PID_velocity.D = 0;

  // // angle loop controller
  // motor1.P_angle.P = 20;

  // // velocity low pass filtering time constant
  // motor1.LPF_velocity.Tf = 0.01;




  // // 速度限制
  // motor1.velocity_limit = 20;

  // //速度低通滤波时间常数
  // motor1.LPF_velocity.Tf = 0.01;



  // monitor接口设置
  Serial.begin();
  // comment out if not needed
  // motor1.useMonitoring(Serial);

  // monitor相关设置
  // motor1.monitor_downsample = 0;
  // motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;

  //电机初始化
  motor1.init();
  // align encoder and start FOC
  // motor1.initFOC(); 

  // 映射电机到commander
  command.add('T', doTarget, "target velocity");
  command.add('M',onMotor,"my motor"); 

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));

  // _delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
    // sensor1.update();
  // pos = sensor1.getAngle();
  // float vel = sensor1.getVelocity();

  // // Serial.printf("Angle is: %.3f\r\n", pos);
  // Serial.println(pos, 3);
  // Serial.println(vel, 3);
  // delay(10);

  // iterative setting FOC phase voltage
  // motor1.loopFOC();

  // iterative function setting the outter loop target
  motor1.move(target_velocity);

  // user communication
  motor1.monitor();

  command.run();

}  