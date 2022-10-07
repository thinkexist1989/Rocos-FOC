
#include "MagneticSensorSPI.h"
#include <math.h>
#include "main.h"


MagneticSensorSPI::MagneticSensorSPI() {

}

void MagneticSensorSPI::init(bool faseMode) {

    fast_mode = faseMode;
    if(fast_mode)
        read_pointer = &MagneticSensorSPI::read;
    else
        read_pointer = &MagneticSensorSPI::faseRead;

    cpr = pow(2, bit_resolution);

    this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSPI::getSensorAngle() {
    return (getRawCount() / (float) cpr) * _2PI;
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorSPI::getRawCount() {
    return (int) (this->*read_pointer)(angle_register);
}

// SPI functions 
/**
 * Utility function used to calculate even parity of word
 */
byte MagneticSensorSPI::spiCalcEvenParity(word value) {
    byte cnt = 0;
    byte i;

    for (i = 0; i < 16; i++) {
        if (value & 0x1) cnt++;
        value >>= 1;
    }
    return cnt & 0x1;
}

/**
* Read a register from the sensor
* Takes the address of the register as a 16 bit word
* Returns the value of the register
*/
uint16_t MagneticSensorSPI::read(uint16_t angle_register) {

    uint16_t command = angle_register;

    uint16_t angle_value;

    //Send the command
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //cs selected
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &command, (uint8_t *) &angle_value, 1, 100);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //cs deselected

    delay_us(1);

    //Now read the response
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //cs selected
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &command, (uint8_t *) &angle_value, 1, 100);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //cs deselected


    angle_value = angle_value & 0x3FFF;

    return angle_value;
}

/**
* Fast read a register from the sensor
* Takes the address of the register as a 16 bit word
* Returns the value of the register
*/
uint16_t MagneticSensorSPI::faseRead(uint16_t angle_register) {
    uint16_t command = angle_register;

    uint16_t angle_value;

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //cs selected
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &command, (uint8_t *) &angle_value, 1, 100);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //cs deselected
    angle_value = angle_value & 0x3FFF;

    return angle_value;
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void MagneticSensorSPI::close() {
//	spi->end();
}


