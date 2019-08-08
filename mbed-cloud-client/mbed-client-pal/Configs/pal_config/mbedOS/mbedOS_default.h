/*******************************************************************************
 * Copyright 2016, 2017 ARM Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#ifndef PAL_MBEDOS_CONFIGURATION_H_

#include "cmsis_os.h"

#if (defined(MBED_DEBUG) && !defined(DEBUG))
    #define DEBUG
#endif

/*!
 * \brief This file is for more specific definitions (per board, if needed).
 *        if this file is defined it will be included from pal_configuration.h
 *        if not, the default file will be included - if needed
 */
#ifndef PAL_BOARD_SPECIFIC_CONFIG
    #if defined(TARGET_K64F)
        #include "K64F_default.h"
    #elif defined(TARGET_NUCLEO_F429ZI)
        #include "NUCLEO_default.h"
    #elif defined(TARGET_UBLOX_EVK_ODIN_W2)
        #include "ODIN_default.h"
    #endif
#endif


#ifndef PAL_RTOS_WAIT_FOREVER
    #define PAL_RTOS_WAIT_FOREVER osWaitForever
#endif

#ifndef PAL_NUMBER_OF_PARTITIONS
    #define PAL_NUMBER_OF_PARTITIONS 1
#endif


#ifndef PAL_FS_MOUNT_POINT_PRIMARY
    #if (PAL_NUMBER_OF_PARTITIONS == 2)
        #define PAL_FS_MOUNT_POINT_PRIMARY    "/sd"           //!< User should change this for the his working folder
    #else
        #define PAL_FS_MOUNT_POINT_PRIMARY    "/sd"
    #endif
#endif

#ifndef PAL_FS_MOUNT_POINT_SECONDARY
    #if (PAL_NUMBER_OF_PARTITIONS == 2)
        #define PAL_FS_MOUNT_POINT_SECONDARY    "/sd2"
    #else
        #define PAL_FS_MOUNT_POINT_SECONDARY    "/sd"         //!< User should change this for the his working folder
    #endif
#endif

#ifndef PAL_NUM_OF_THREAD_INSTANCES
    #define PAL_NUM_OF_THREAD_INSTANCES 1
#endif

#ifndef PAL_MAX_SEMAPHORE_COUNT
    #define PAL_MAX_SEMAPHORE_COUNT 1024
#endif



#endif /* PAL_MBEDOS_CONFIGURATION_H_ */
