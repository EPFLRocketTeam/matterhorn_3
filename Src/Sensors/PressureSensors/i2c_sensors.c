/*
 * abs_p.c
 *
 *  Created on: 17 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

/*
 * Code based on this library: https://github.com/freetronics/BaroSensor/blob/master/BaroSensor.cpp
 */

#include <Sensors/i2c_sensors.h>
#include "stm32f4xx_hal.h"
#include "Misc/Common.h"

#include <Sensors/BME280/bme280.h>
#if(SIMULATION == 1)
#include <Misc/SimData.h>
#endif
#include <Misc/rocket_constants.h>

extern I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef* hi2c;
extern osSemaphoreId i2cSensorsSemHandle;
extern BARO_data BARO_buffer[];

#define I2C_TIMEOUT 100 //ms

#define MAX_TEMPERATURE 85
#define MIN_TEMPERATURE -40

#define MAX_PRESSURE 1200
#define MIN_PRESSURE 100

int8_t stm32_i2c_read (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  HAL_I2C_Mem_Read_DMA(hi2c, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len);
  return osSemaphoreWait (i2cSensorsSemHandle, I2C_TIMEOUT);
}

int8_t stm32_i2c_write (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  HAL_I2C_Mem_Write (hi2c, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
}

void stm32_delay_ms (uint32_t period)
{
  osDelay (period);
}

extern UART_HandleTypeDef huart2;

void print_sensor_data (struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
  static char buf[100];
  sprintf (buf, "temp %0.2f, p %0.2f, hum %0.2f\r\n", (float) comp_data->temperature, (float) comp_data->pressure, comp_data->humidity);
  HAL_UART_Transmit_DMA(&huart2, buf, strlen(buf));
#else
  sprintf(buf, "temp %ld, p %ld, hum %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

void initI2cDevices ()
{
  hi2c = &hi2c1;

  struct bme280_dev dev;
  int8_t rslt = BME280_OK;

  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = &stm32_i2c_read;
  dev.write = &stm32_i2c_write;
  dev.delay_ms = &stm32_delay_ms;

  rslt = bme280_init (&dev);

  uint8_t settings_sel;
  struct bme280_data comp_data;

  /* Recommended mode of operation: Indoor navigation */
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

  rslt = bme280_set_sensor_settings (settings_sel, &dev);

  while (1)
    {
      rslt = bme280_set_sensor_mode (BME280_FORCED_MODE, &dev);
      /* Wait for the measurement to complete and print data @25Hz */
      HAL_Delay (40);
      rslt = bme280_get_sensor_data (BME280_ALL, &comp_data, &dev);
      print_sensor_data (&comp_data);
    }

}

uint32_t failedReading = 0;

extern int startSimulation;

void TK_fetch_i2c ()
{
#if(SIMULATION == 1)

  while (!startSimulation)
    {
      osDelay (10);
    }

  // Save initialization time to synchronize program clock with data
  float32_t initial_sim_time = SimData[0][SIM_TIMESTAMP] - HAL_GetTick ();
  uint32_t sensorCounter = 0;

  // Populate first sensor structure
  BARO_data* newBaroData = &BARO_buffer[(currentBaroSeqNumber + 1) % CIRC_BUFFER_SIZE];
  newBaroData->temperature = 0;
  newBaroData->pressure = 0;
  newBaroData->altitude = SimData[sensorCounter][SIM_ALTITUDE];

  PITOT_buffer[(currentPitotSeqNumber + 1) % CIRC_BUFFER_SIZE] = 0.0f;
  currentBaroSeqNumber++;
  sensorCounter++;
  currentPitotSeqNumber++;

  for (;;)
    {
      if ((sensorCounter < SIM_TAB_HEIGHT - 1)
          && ((HAL_GetTick () - (SimData[sensorCounter][SIM_TIMESTAMP] - initial_sim_time)) > 0))
        {
          // change sensor data
          // create artificial sensor structure
          BARO_data* newBaroData = &BARO_buffer[(currentBaroSeqNumber + 1) % CIRC_BUFFER_SIZE];
          //populate data
          newBaroData->temperature = 0;
          newBaroData->pressure = SimData[sensorCounter][SIM_PRESSURE];
          newBaroData->altitude = SimData[sensorCounter][SIM_ALTITUDE];

          PITOT_buffer[(currentPitotSeqNumber++) % CIRC_BUFFER_SIZE] = SimData[sensorCounter][SIM_VELOCITYX];
          //increment counters
          currentBaroSeqNumber++;
          currentBaroTimestamp = HAL_GetTick();

          sensorCounter++;
        }
      osDelay (SimData[sensorCounter][SIM_TIMESTAMP] - SimData[sensorCounter - 1][SIM_TIMESTAMP] - 1);
    }

# else
  osSemaphoreWait (i2cSensorsSemHandle, 1); // make sure the semaphore is taken at the start of the loop

  initI2cDevices ();

  uint32_t samplingStart, elapsed;

  for (;;)
    {

      samplingStart = HAL_GetTick ();

    }

#endif
}

inline float altitudeFromPressure (float pressure_hPa)
{
  float altitude = 44330 * (1.0 - pow (pressure_hPa / ADJUSTED_SEA_LEVEL_PRESSURE, 0.1903));

  if (altitude < 100 || altitude > 5000)
    {
      return 0;
    }
  else
    {
      return altitude;
    }
}


/*
 void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c)
 {
 osSemaphoreRelease (pressureSensorI2cSemHandle);
 }
 */

/*void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c)
{
  osSemaphoreRelease (i2cSensorsSemHandle);
}
*/

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
  osSemaphoreRelease (i2cSensorsSemHandle);
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  osSemaphoreRelease (i2cSensorsSemHandle);
}

/*
void HAL_I2C_ErrorCallback (I2C_HandleTypeDef *hi2c)
{
  osSemaphoreRelease (i2cSensorsSemHandle);
}

void HAL_I2C_AbortCpltCallback (I2C_HandleTypeDef *hi2c)
{
  //osSemaphoreRelease (i2cSensorsSemHandle);
}
*/
