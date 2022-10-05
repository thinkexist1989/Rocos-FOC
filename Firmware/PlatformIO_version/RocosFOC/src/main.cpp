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

InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, SENSOR_A, SENSOR_B);

//目标变量
float target_velocity = 5;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void onMotor(char* cmd){ command.motor(&motor1,cmd); }      // 串口控制指令：电机


void setup() {
  //
  driver1.pwm_frequency = 20000; // 20kHz
  //
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = 12;

  driver1.init();

  driver1.enable();

  _delay(1000);



}

void loop() {
  // put your main code here, to run repeatedly:

  // iterative setting FOC phase voltage
  // motor1.loopFOC();

  // iterative function setting the outter loop target
  driver1.setPwm(3, 1, 5);

  // user communication
  // motor1.monitor();

  // command.run();

}  