#include "BLDCDriver3PWM.h"

BLDCDriver3PWM::BLDCDriver3PWM(){
  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
    // set zero to PWM
    setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
    HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

}

// init hardware pins
int BLDCDriver3PWM::init() {
  // sanity check for the voltage limit configuration
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  initialized = true;
  return true;
}



// Set voltage to the pwm pin
void BLDCDriver3PWM::setPhaseState(int s) {
  // disable if needed
//    if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
//        digitalWrite(enableA_pin, sa == _HIGH_IMPEDANCE ? LOW : HIGH);
//        digitalWrite(enableB_pin, sb == _HIGH_IMPEDANCE ? LOW : HIGH);
//        digitalWrite(enableC_pin, sc == _HIGH_IMPEDANCE ? LOW : HIGH);
//    HAL_GPIO_WritePin(DRV_EN_GPIO_Port,
//                      DRV_EN_Pin,
//                      s == _HIGH_IMPEDANCE ? GPIO_PIN_RESET : GPIO_PIN_SET);

}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c);
}
