/*
 * data_handling.h
 *
 *  Created on: 19 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#include <Misc/datastructs.h>
#include "cmsis_os.h"

#ifndef DATA_HANDLING_H_
#define DATA_HANDLING_H_

#ifdef __cplusplus
extern "C"
  {
#endif

void TK_sd_sync (const void* args);

osStatus initSdFile ();


#ifdef __cplusplus
}
#endif

#endif //DATA_HANDLING_H_
