#ifndef MAGNETICSENSORSPI_LIB_H
#define MAGNETICSENSORSPI_LIB_H


#include "../common/base_classes/Sensor.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"

#define DEF_ANGLE_REGISTER 0x3FFF

typedef uint32_t word;
typedef uint8_t byte ;

class MagneticSensorSPI: public Sensor{
 public:
    /**
     *  MagneticSensorSPI class constructor
     */
    MagneticSensorSPI();

    /** sensor initialise pins */
    void init(bool faseMode = false);

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;
    

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    // spi variables
    uint16_t angle_register = DEF_ANGLE_REGISTER; //!< SPI angle register to read

    bool fast_mode;
    int bit_resolution = 14; //!< 14 bit valid
    // spi functions
    /** Stop SPI communication */
    void close(); 
    /** Read one SPI register value */
    uint16_t read(uint16_t angle_register);
    /** Fast Read one SPI register value */
    uint16_t fastRead(uint16_t angle_register);
    /** Calculate parity value  */
    byte spiCalcEvenParity(word value);

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    uint16_t (MagneticSensorSPI::*read_pointer)(uint16_t); // function pointer


};


#endif
