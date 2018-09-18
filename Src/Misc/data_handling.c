/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#include "cmsis_os.h"
#include "fatfs.h"

#include <Misc/Common.h>
#include <Misc/data_handling.h>

#define MAX_FOLDER_NUMBER 100

extern IMU_data IMU_buffer[];
extern BARO_data BARO_buffer[];

extern osMessageQId xBeeQueueHandle;

extern int startSimulation;
extern volatile float32_t high_range_pressure;

osStatus initSdFile ()
{

  FRESULT res; /* FatFs function common result code */
  uint32_t byteswritten, bytesread; /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  uint8_t rtext[100];
  DIR currentDir;

  /*##-2- Register the file system object to the FatFs module ##############*/
  if (f_mount (&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      SD_Error ();
    }
  else
    {
      TCHAR dir[20];
      for (int i = 0; i < MAX_FOLDER_NUMBER; i++)
        {
          sprintf (dir, "DATAERT%d", i);
          FILINFO info;
          if (f_stat (dir, &info) != FR_OK)
            {
              f_mkdir (dir);
              break;
            }
        }
      TCHAR path[100];

      sprintf (path, "%s" "/" "ERT.txt", dir);
      if (f_open (&SDFile, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
        {
          /* 'STM32.TXT' file Open for write Error */
          SD_Error ();
        }
      else
        {
          /*##-5- Write data to the text file ################################*/
          res = f_write (&SDFile, wtext, sizeof(wtext), (void *) &byteswritten);

          if ((byteswritten == 0) || (res != FR_OK))
            {
              /* 'STM32.TXT' file Write or EOF Error */
              SD_Error ();
            }
          else
            {
              /*##-6- Close the open text file #################################*/
              f_close (&SDFile);

              /*##-7- Open the text file object with read access ###############*/
              if (f_open (&SDFile, "STM32.TXT", FA_READ) != FR_OK)
                {
                  /* 'STM32.TXT' file Open for read Error */
                  SD_Error ();
                }
              else
                {
                  /*##-8- Read data from the text file ###########################*/
                  res = f_read (&SDFile, rtext, sizeof(rtext), (UINT*) &bytesread);

                  if ((bytesread == 0) || (res != FR_OK))
                    {
                      /* 'STM32.TXT' file Read or EOF Error */
                      SD_Error ();
                    }
                  else
                    {
                      /*##-9- Close the open text file #############################*/
                      f_close (&SDFile);

                      /*##-10- Compare read data with the expected data ############*/
                      if ((bytesread != byteswritten))
                        {
                          /* Read data is different from the expected data */
                          //SD_Error();
                        }
                      else
                        {
                          /*
                           HAL_GPIO_TogglePin (BUZZER_GPIO_Port, BUZZER_Pin);
                           osDelay (500);
                           HAL_GPIO_TogglePin (BUZZER_GPIO_Port, BUZZER_Pin);
                           */
                        }
                    }
                }
            }
        }
    }

}

void SD_Error ()
{
  HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  osDelay (500);
  HAL_GPIO_WritePin (BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  osDelay (portMAX_DELAY);
}

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

