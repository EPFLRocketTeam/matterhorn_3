/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#include "cmsis_os.h"

#include "Misc/Common.h"

#include <Misc/data_handling.h>

extern IMU_data IMU_buffer[];
extern BARO_data BARO_buffer[];

extern osMessageQId xBeeQueueHandle;

extern int startSimulation;
extern volatile float32_t high_range_pressure;

void TK_data (void const * args)
{

  uint32_t lastImuSeqNumber = 0, lastBaroSeqNumber = 0, telemetrySeqNumber = 0;

  osDelay (800); //Wait for the first values to be written by the IMU and barometer.

  for (;;)
    {
      uint32_t measurement_time = HAL_GetTick ();

      IMU_data* imu_data = &IMU_buffer[currentImuSeqNumber % CIRC_BUFFER_SIZE];
      BARO_data* baro_data = &BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];

      lastImuSeqNumber = currentImuSeqNumber;
      lastBaroSeqNumber = currentBaroSeqNumber;

      //TODO: log sensor value here
      /*Telemetry_Message m = createTelemetryDatagram (imu_data, baro_data, pitot_press, measurement_time,
                                                     telemetrySeqNumber++);
      osMessagePut (xBeeQueueHandle, (uint32_t) &m, 50);
       */

      osDelay (15 - (HAL_GetTick () - measurement_time));
    }
}

