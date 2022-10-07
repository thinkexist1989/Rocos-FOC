//
// Created by think on 2022/10/7.
//

#include "Task.h"
#include "usbd_cdc_if.h"
#include "sensors/MagneticSensorSPI.h"

//!<<<<<<<<<<<<<<Variable Definition>>>>>>>>>>>>>>>>>!//
MagneticSensorSPI sensorSpi;

//!<<<<<<<<<<<<<<Function Definition>>>>>>>>>>>>>>>>>!//

uint16_t SPI_AS5048A_ReadData(void)
{
    uint16_t angle_value;
    uint16_t command = 0xFFFF;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //cs片�??
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&command, (uint8_t*)&angle_value, 1, 100);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //cs取消片�??
    angle_value = angle_value & 0x3FFF;

    return angle_value;
}


//!<<<<<<<<<<<<<<<<<<<<Setup>>>>>>>>>>>>>>>>>>>//
/**
 * @description TaskSetup Wrap c/cpp for initialization, used before while(1)
 */
void TaskSetup(void) {
    sensorSpi.init();
}

/**
 * @description TaskDo Wrap c/cpp for cycling call, used inside while(1)
 */
void TaskDo(void) {
      if(get_data_flag) {
          usb_printf("get data:\r\n%s %d\r\n", buf, data_nums);
          get_data_flag = 0;
      }
      sensorSpi.update();
      float angle = sensorSpi.getAngle();
      usb_printf("Angle1 is: %.4f\r\n", angle);
      angle = SPI_AS5048A_ReadData() /16383.0 * 360.0;
      usb_printf("Angle2 is: %.4f\r\n", angle);

      HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);

      _writeDutyCycle3PWM(0.5, 0.2, 0.3);
      HAL_Delay(1000);
}