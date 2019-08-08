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


#ifndef _PAL_COFIGURATION_H
#define _PAL_COFIGURATION_H
#include "limits.h"


#ifdef PAL_USER_DEFINED_CONFIGURATION
    #include PAL_USER_DEFINED_CONFIGURATION
#else
    #include "sotp_fs.h"
#endif


/*! \brief If needed any board specific configuration please set this define
*/
#ifdef PAL_BOARD_SPECIFIC_CONFIG
    #include PAL_BOARD_SPECIFIC_CONFIG
#endif


/*! \brief let the user choose its platform configuration file.
    \note if the user does not specify a platform configuration file,
    \note PAL uses a default configuration set that can be found at \b Configs/pal_config folder
  */

#ifdef PAL_PLATFORM_DEFINED_CONFIGURATION
    #include PAL_PLATFORM_DEFINED_CONFIGURATION
#elif defined(__LINUX__)
    #include "Linux_default.h"
#elif defined(__FREERTOS__)
    #include "FreeRTOS_default.h"
#elif defined(__MBED__)
    #include "mbedOS_default.h"
#else
    #error "Please specify the platform PAL_PLATFORM_DEFINED_CONFIGURATION"
#endif

/*! \file pal_configuration.h
*  \brief PAL Configuration.
*   This file contains PAL configuration information including the following:
*       1. The flags to enable or disable features.
*       2. The configuration of the number of objects provided by PAL (such as the number of threads supported) or their sizes.
*       3. The configuration of supported cipher suites.
*       4. The configuration for flash memory usage.
*       5. The configuration for the root of trust.
*/


/*
 * Network configuration
 */
//! PAL configuration options
#ifndef PAL_NET_TCP_AND_TLS_SUPPORT
    #define PAL_NET_TCP_AND_TLS_SUPPORT         true/* Add PAL support for TCP. */
#endif

#ifndef PAL_NET_ASYNCHRONOUS_SOCKET_API
    #define PAL_NET_ASYNCHRONOUS_SOCKET_API     true/* Add PAL support for asynchronous sockets. */
#endif

#ifndef PAL_NET_DNS_SUPPORT
    #define PAL_NET_DNS_SUPPORT                 true/* Add PAL support for DNS lookup. */
#endif


#ifndef PAL_SUPPORT_IP_V4
    #define PAL_SUPPORT_IP_V4                 true /* support IPV4 as default*/
#endif
#ifndef PAL_SUPPORT_IP_V6
    #define PAL_SUPPORT_IP_V6                 true /* support IPV6 as default*/
#endif

//values for PAL_NET_DNS_IP_SUPPORT
#define PAL_NET_DNS_ANY          0    /* if PAL_NET_DNS_IP_SUPPORT is set to PAL_NET_DNS_ANY pal_getAddressInfo will return the first available IPV4 or IPV6 address*/
#define PAL_NET_DNS_IPV4_ONLY    2    /* if PAL_NET_DNS_IP_SUPPORT is set to PAL_NET_DNS_IPV4_ONLY pal_getAddressInfo will return the first available IPV4 address*/
#define PAL_NET_DNS_IPV6_ONLY    4    /* if PAL_NET_DNS_IP_SUPPORT is set to PAL_NET_DNS_IPV6_ONLY pal_getAddressInfo will return the first available IPV6 address*/


#ifndef PAL_NET_DNS_IP_SUPPORT
#if PAL_SUPPORT_IP_V6 == true && PAL_SUPPORT_IP_V4 == true
    #define PAL_NET_DNS_IP_SUPPORT  0 /* sets the type of IP addresses returned by  pal_getAddressInfo*/
#elif PAL_SUPPORT_IP_V6 == true
    #define PAL_NET_DNS_IP_SUPPORT  4 /* sets the type of IP addresses returned by  pal_getAddressInfo*/
#else 
    #define PAL_NET_DNS_IP_SUPPORT  2 /* sets the type of IP addresses returned by  pal_getAddressInfo*/
#endif

#endif

//! The maximum number of interfaces that can be supported at a time.
#ifndef PAL_MAX_SUPORTED_NET_INTERFACES
    #define PAL_MAX_SUPORTED_NET_INTERFACES 10
#endif

//!< Stack size for thread created when calling pal_getAddressInfoAsync
#ifndef PAL_NET_ASYNC_DNS_THREAD_STACK_SIZE
    #define PAL_NET_ASYNC_DNS_THREAD_STACK_SIZE (1024 * 2)
#endif


//! If you want PAL Not to perform a rollback/cleanup although main PAL init failed, please set this flag to `false`
#ifndef PAL_CLEANUP_ON_INIT_FAILURE
	#define PAL_CLEANUP_ON_INIT_FAILURE true
#endif

/*
 * RTOS configuration
 */
//! This flag determines if PAL moudles are thread safe. 1 - thread safety is enabled, 0 - thread safety is disabled
#ifndef PAL_THREAD_SAFETY
	#define PAL_THREAD_SAFETY 1
#endif

//! initial time until thread stack cleanup (mbedOs only). This is the amount of time we wait before checking that a thread has completed so we can free it's stack.
#ifndef PAL_RTOS_THREAD_CLEANUP_TIMER_MILISEC
    #define PAL_RTOS_THREAD_CLEANUP_TIMER_MILISEC 200
#endif

//! This define is used to determine the size of the initial random buffer (in bytes) held by PAL for random the algorithm.
#ifndef PAL_INITIAL_RANDOM_SIZE
    #define PAL_INITIAL_RANDOM_SIZE 48
#endif

#ifndef PAL_RTOS_WAIT_FOREVER
    #define PAL_RTOS_WAIT_FOREVER UINT_MAX
#endif

/*
 * TLS configuration
 */

//! The maximum number of supported cipher suites.
#ifndef PAL_MAX_ALLOWED_CIPHER_SUITES
    #define PAL_MAX_ALLOWED_CIPHER_SUITES 1
#endif

//! This value is in milliseconds. 1000 = 1 second.
#ifndef PAL_DTLS_PEER_MIN_TIMEOUT
    #define PAL_DTLS_PEER_MIN_TIMEOUT 1000
#endif

//! The debug threshold for TLS API.
#ifndef PAL_TLS_DEBUG_THRESHOLD
    #define PAL_TLS_DEBUG_THRESHOLD 5
#endif

//! 32 or 48 (depends on the curve) bytes for the X,Y coordinates and 1 for the normalized/non-normalized
#ifndef PAL_CERT_ID_SIZE
    #define PAL_CERT_ID_SIZE 33 
#endif


#ifndef PAL_ENABLE_PSK
	#define PAL_ENABLE_PSK 0
#endif

#ifndef PAL_ENABLE_X509
	#define PAL_ENABLE_X509 1
#endif 

//! Define the cipher suites for TLS (only one cipher suite per device available).
#define PAL_TLS_PSK_WITH_AES_128_CBC_SHA256_SUITE           0x01
#define PAL_TLS_PSK_WITH_AES_128_CCM_8_SUITE                0x02
#define PAL_TLS_PSK_WITH_AES_256_CCM_8_SUITE                0x04
#define PAL_TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8_SUITE        0x08
#define PAL_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256_SUITE   0x10
#define PAL_TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384_SUITE   0x20


//! Use the default cipher suite for TLS/DTLS operations
#if (PAL_ENABLE_X509 == 1)
    #ifndef PAL_TLS_CIPHER_SUITE
        #define PAL_TLS_CIPHER_SUITE PAL_TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8_SUITE
    #endif
#elif (PAL_ENABLE_PSK == 1)
    #ifndef PAL_TLS_CIPHER_SUITE
        #define PAL_TLS_CIPHER_SUITE PAL_TLS_PSK_WITH_AES_128_CCM_8_SUITE
    #endif
#endif

#ifndef PAL_CMAC_SUPPORT
	#define PAL_CMAC_SUPPORT true
#endif

//! Enable the CMAC functionality (This flag was targeted to let the bootloader to be compiled without CMAC)
#ifndef PAL_CMAC_SUPPORT
        #define PAL_CMAC_SUPPORT 1
#endif //PAL_CMAC_SUPPORT

/*
 * UPDATE configuration
 */

#define PAL_UPDATE_USE_FLASH 1
#define PAL_UPDATE_USE_FS    2

#ifndef PAL_UPDATE_IMAGE_LOCATION
	#define PAL_UPDATE_IMAGE_LOCATION PAL_UPDATE_USE_FS     //!< Choose the storage correct Storage option, File System or Flash
#endif

//! Certificate date validation in Unix time format.
#ifndef PAL_CRYPTO_CERT_DATE_LENGTH
    #define PAL_CRYPTO_CERT_DATE_LENGTH sizeof(uint64_t)
#endif

/*
 * FS configuration
 */

/* !\brief file system configurations
 * PAL_NUMBER_OF_PARTITIONS
 * 0 - Default behavior for the platform (Described by either 1 or 2 below).
 * 1 - There is a single partition in which the ARM client applications create and remove files (but do not format it).
 * 2 - There are two partitions in which ARM client applications may format or create and remove files,
 *     depending on PAL_PRIMARY_PARTITION_PRIVATE and PAL_SECONDARY_PARTITION_PRIVATE
 */
#ifndef PAL_NUMBER_OF_PARTITIONS
    #define PAL_NUMBER_OF_PARTITIONS 1 // Default partitions
#endif

#if (PAL_NUMBER_OF_PARTITIONS > 2)
#error "PAL_NUMBER_OF_PARTITIONS cannot be more then 2"
#endif

// PAL_PRIMARY_PARTITION_PRIVATE
// 1 if the primary partition is exclusively dedicated to the ARM client applications.
// 0 if the primary partition is used for storing other files as well.
#ifndef PAL_PRIMARY_PARTITION_PRIVATE
    #define PAL_PRIMARY_PARTITION_PRIVATE 0
#endif

//! PAL_SECONDARY_PARTITION_PRIVATE
//! 1 if the secondary partition is exclusively dedicated to the ARM client applications.
//! 0 if the secondary partition is used for storing other files as well.
#ifndef PAL_SECONDARY_PARTITION_PRIVATE
    #define PAL_SECONDARY_PARTITION_PRIVATE 0
#endif

//! This define is the location of the primary mount point for the file system
#ifndef PAL_FS_MOUNT_POINT_PRIMARY
    #define PAL_FS_MOUNT_POINT_PRIMARY  ""
#endif

//! This define is the location of the secondary mount point for the file system
#ifndef PAL_FS_MOUNT_POINT_SECONDARY
    #define PAL_FS_MOUNT_POINT_SECONDARY ""
#endif

// Update

#ifndef PAL_UPDATE_FIRMWARE_MOUNT_POINT
    #define PAL_UPDATE_FIRMWARE_MOUNT_POINT PAL_FS_MOUNT_POINT_PRIMARY
#endif
//! The location of the firmware update folder
#ifndef PAL_UPDATE_FIRMWARE_DIR
    #define PAL_UPDATE_FIRMWARE_DIR PAL_UPDATE_FIRMWARE_MOUNT_POINT "/firmware"
#endif

/*\brief If flash existed set to 1 else 0, the flash is used for none volatile backup*/
#ifndef PAL_USE_INTERNAL_FLASH 
    #define PAL_USE_INTERNAL_FLASH  0
#endif

#ifndef PAL_INT_FLASH_NUM_SECTIONS
    #define PAL_INT_FLASH_NUM_SECTIONS 0
#endif

#ifndef PAL_USE_HW_ROT 
    #define PAL_USE_HW_ROT     1
#endif

#ifndef PAL_USE_HW_RTC
    #define PAL_USE_HW_RTC    1
#endif

#ifndef PAL_USE_HW_TRNG
    #define PAL_USE_HW_TRNG    1
#endif

//! The number of valid priorities limits the number of concurrent running threads.
#ifndef PAL_MAX_NUMBER_OF_THREADS
    #if PAL_USE_HW_TRNG
        #define PAL_MAX_NUMBER_OF_THREADS 9    
    #else
        #define PAL_MAX_NUMBER_OF_THREADS 8
    #endif
#endif

#if PAL_USE_HW_TRNG
    //! Delay for TRNG noise collecting thread used between calls to TRNG
    #ifndef PAL_NOISE_TRNG_THREAD_DELAY_MILLI_SEC
        #define PAL_NOISE_TRNG_THREAD_DELAY_MILLI_SEC (1000 * 60) // one minute
    #endif
    //! Stack size for TRNG noise collecting thread
    #ifndef PAL_NOISE_TRNG_THREAD_STACK_SIZE
        #define PAL_NOISE_TRNG_THREAD_STACK_SIZE 1536 // 1.5K
    #endif
#endif

#ifndef PAL_USE_SECURE_TIME
    #define PAL_USE_SECURE_TIME 1
#endif

#ifndef PAL_DEVICE_KEY_DERIVATION_BACKWARD_COMPATIBILITY_CALC
    #define PAL_DEVICE_KEY_DERIVATION_BACKWARD_COMPATIBILITY_CALC 0
#endif    

/*\brief  Starting Address for  section 1 Minimum requirement size is 1KB and section must be consecutive sectors*/
#ifndef PAL_INTERNAL_FLASH_SECTION_1_ADDRESS
    #define PAL_INTERNAL_FLASH_SECTION_1_ADDRESS    0
#endif
/*\brief  Starting Address for  section 2 Minimum requirement size is 1KB and section must be consecutive sectors*/
#ifndef PAL_INTERNAL_FLASH_SECTION_2_ADDRESS
    #define PAL_INTERNAL_FLASH_SECTION_2_ADDRESS    0
#endif
/*\brief  Size for  section 1*/
#ifndef PAL_INTERNAL_FLASH_SECTION_1_SIZE
    #define PAL_INTERNAL_FLASH_SECTION_1_SIZE       0
#endif
/*\brief  Size for  section 2*/
#ifndef PAL_INTERNAL_FLASH_SECTION_2_SIZE
    #define PAL_INTERNAL_FLASH_SECTION_2_SIZE       0
#endif

#ifndef PAL_SIMULATOR_TEST_ENABLE
    #define PAL_SIMULATOR_TEST_ENABLE    0
#endif



#if (PAL_SIMULATOR_TEST_ENABLE == 1) 

    #undef PAL_SIMULATE_RTOS_REBOOT
    #define PAL_SIMULATE_RTOS_REBOOT 1

    #undef PAL_SIMULATOR_FLASH_OVER_FILE_SYSTEM
    #define PAL_SIMULATOR_FLASH_OVER_FILE_SYSTEM    1

/*\brief overwrite format command with remove all file and directory only for Linux*/
    #undef PAL_SIMULATOR_FS_RM_INSTEAD_OF_FORMAT
    #define PAL_SIMULATOR_FS_RM_INSTEAD_OF_FORMAT 1
#endif //PAL_SIMULATOR_TEST_ENABLE

#ifndef PAL_SIMULATOR_FLASH_OVER_FILE_SYSTEM
    #define PAL_SIMULATOR_FLASH_OVER_FILE_SYSTEM    0
#endif



#if PAL_SIMULATOR_FLASH_OVER_FILE_SYSTEM


    #undef PAL_USE_INTERNAL_FLASH 
    #define PAL_USE_INTERNAL_FLASH  1

    #undef PAL_INT_FLASH_NUM_SECTIONS
    #define PAL_INT_FLASH_NUM_SECTIONS 2

    #ifndef PAL_SIMULATOR_SOTP_AREA_SIZE
        #define PAL_SIMULATOR_SOTP_AREA_SIZE    4096 /*\brief must be power of two the can be divded to page size without reminder and must be a multiple of sector size*/
    #endif

    #ifndef SIMULATE_FLASH_SECTOR_SIZE
        #define SIMULATE_FLASH_SECTOR_SIZE	    4096 /*\brief  Flash Sector size*/
    #endif

    #ifndef SIMULATE_FLASH_DIR
        #define SIMULATE_FLASH_DIR			    "" /*\brief Directory that holds the flash simulator file*/
    #endif

    #ifndef SIMULATE_FLASH_FILE_NAME
        #define SIMULATE_FLASH_FILE_NAME	    SIMULATE_FLASH_DIR"/flashSim" /*\brief File name and path to the flash simulator file*/
    #endif

    #ifndef SIMULATE_FLASH_PAGE_SIZE
        #define SIMULATE_FLASH_PAGE_SIZE	    8 /*\brief Minumum writing uint to flash (2, 4, 8, 16)*/
    #endif

    #if PAL_SIMULATOR_SOTP_AREA_SIZE < 4096
        #error Minimum Size of 4K
    #endif

    /*\brief  Note - In simulator mode all flash areas are overriden with the simulation sizes and address*/

    #undef PAL_INTERNAL_FLASH_SECTION_1_SIZE
    /*\brief  Size for section 1*/
    #define PAL_INTERNAL_FLASH_SECTION_1_SIZE       PAL_SIMULATOR_SOTP_AREA_SIZE

    #undef PAL_INTERNAL_FLASH_SECTION_2_SIZE
    /*\brief  Size for section 2*/
    #define PAL_INTERNAL_FLASH_SECTION_2_SIZE       PAL_SIMULATOR_SOTP_AREA_SIZE

    #undef PAL_INTERNAL_FLASH_SECTION_1_ADDRESS
    /*\brief  Starting Address for section 1 Minimum requirement size is 1KB and section must be consecutive sectors*/
    #define PAL_INTERNAL_FLASH_SECTION_1_ADDRESS    0

    #undef PAL_INTERNAL_FLASH_SECTION_2_ADDRESS
    /*\brief  Starting Address for section 2 Minimum requirement size is 1KB and section must be consecutive sectors*/
    #define PAL_INTERNAL_FLASH_SECTION_2_ADDRESS    PAL_INTERNAL_FLASH_SECTION_1_SIZE

#endif //PAL_SIMULATOR_FLASH_OVER_FILE_SYSTEM


#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var " = "  VALUE(var)

#if ((!PAL_USE_INTERNAL_FLASH && (!PAL_USE_HW_ROT || !PAL_USE_HW_RTC || !PAL_USE_HW_TRNG))  \
        || ((PAL_INT_FLASH_NUM_SECTIONS == 1) && PAL_USE_INTERNAL_FLASH && (!PAL_USE_HW_RTC || !PAL_USE_HW_TRNG)) \
        || ((PAL_INT_FLASH_NUM_SECTIONS == 2) && PAL_USE_INTERNAL_FLASH && !PAL_USE_HW_TRNG)) 
        #pragma message(VAR_NAME_VALUE(PAL_USE_INTERNAL_FLASH))
        #pragma message(VAR_NAME_VALUE(PAL_USE_HW_ROT))
        #pragma message(VAR_NAME_VALUE(PAL_USE_HW_RTC))
        #pragma message(VAR_NAME_VALUE(PAL_USE_HW_TRNG))
        #pragma message(VAR_NAME_VALUE(PAL_INT_FLASH_NUM_SECTIONS))
    #error Minimum configuration setting does not meet the requirements     
#endif

#if (((PAL_ENABLE_PSK == 1) && (PAL_ENABLE_X509 == 1)) && !(defined(__LINUX__)))
    #error "Please select only one option PSK/X509"
#endif

#if ((PAL_ENABLE_PSK == 0) && (PAL_ENABLE_X509 == 0))
    #error "Please select one option PSK/X509"
#endif



#if ((PAL_ENABLE_PSK == 1) && (PAL_USE_SECURE_TIME == 1))
    #error "PSK feature cannot be configured along with secure time"
#endif



//! Delay (in milliseconds) between calls to TRNG random buffer in case only partial data (PAL_ERR_RTOS_TRNG_PARTIAL_DATA) was generated for the function call
#ifndef PAL_TRNG_COLLECT_DELAY_MILLI_SEC
    #define PAL_TRNG_COLLECT_DELAY_MILLI_SEC 1000
#endif // !PAL_TRNG_COLLECT_DELAY_MILLI_SEC

//! define the number of images
#ifndef IMAGE_COUNT_MAX
	#define IMAGE_COUNT_MAX             1
#endif

#define PAL_NOISE_SIZE_BYTES 48 // max number of bytes for noise
#define PAL_NOISE_SIZE_BITS (PAL_NOISE_SIZE_BYTES * CHAR_BIT) // max number of bits for noise
#define PAL_NOISE_BUFFER_LEN (PAL_NOISE_SIZE_BYTES / sizeof(int32_t)) // length of the noise buffer

#endif //_PAL_COFIGURATION_H
