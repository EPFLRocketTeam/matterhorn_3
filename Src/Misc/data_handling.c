/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#include <Misc/Common.h>
#include <Misc/sd_sync.h>

extern IMU_data IMU_buffer[];
extern BARO_data BARO_buffer[];

extern osMessageQId sdLoggingQueueHandle;
char buffer[2048];

void TK_data (void const * args)
{
  osDelay (5000);

  uint32_t lastImuSeqNumber = 0, lastBaroSeqNumber = 0, telemetrySeqNumber = 0;

  for (;;)
    {

      uint32_t measurement_time = HAL_GetTick ();

      IMU_data* imu_data = &IMU_buffer[currentImuSeqNumber % CIRC_BUFFER_SIZE];
      BARO_data* baro_data = &BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];

      lastImuSeqNumber = currentImuSeqNumber;
      lastBaroSeqNumber = currentBaroSeqNumber;

      sprintf (buffer, "%lu, %lu, "
               "%f, %f, %f, "
               "%f, %f, %f, "
               "%f, %f, %f, "
               "%f, %f, %f\r\n",
               telemetrySeqNumber++, measurement_time, baro_data->pressure, baro_data->temperature, baro_data->altitude,
               imu_data->acceleration.x, imu_data->acceleration.y, imu_data->acceleration.z, imu_data->gyro_rps.x,
               imu_data->gyro_rps.y, imu_data->gyro_rps.z, imu_data->mag_uT.x, imu_data->mag_uT.y, imu_data->mag_uT.z);

      int message_length = strlen (buffer);
      if (buffer > 0)
        {
          void* message = pvPortMalloc (message_length + 1);
          if (message != NULL)
            {
              memcpy (message, buffer, message_length + 1); //include the trailing NULL char
              String_Message mess =
                { .ptr = message, .size = message_length };
              osMessagePut (sdLoggingQueueHandle, (uint32_t) &mess, 50);
            }
        }

      uint32_t elapsed = HAL_GetTick () - measurement_time;
      if (elapsed < 10)
        {
          osDelay (10 - elapsed);
        }
    }
}

