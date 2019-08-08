// ----------------------------------------------------------------------------
// Copyright 2016-2017 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "update_client_hub_state_machine.h"
#include "update_client_hub_error_handler.h"
#include "update-client-hub/update_client_hub.h"

#include "update-client-common/arm_uc_common.h"
#include "update-client-firmware-manager/arm_uc_firmware_manager.h"
#include "update-client-manifest-manager/update-client-manifest-manager.h"
#include "update-client-source-manager/arm_uc_source_manager.h"
#include "update-client-control-center/arm_uc_control_center.h"
#include "update-client-control-center/arm_uc_pre_shared_key.h"

#include "mbedtls/aes.h"

#include "pal.h"
#include "sotp.h"

#include <inttypes.h>

/*****************************************************************************/
/* Global variables                                                          */
/*****************************************************************************/

#ifndef MAX_FIRMWARE_LOCATIONS
#define MAX_FIRMWARE_LOCATIONS 1
#endif

// state of the hub state machine
static arm_uc_hub_state_t arm_uc_hub_state = ARM_UC_HUB_STATE_UNINITIALIZED;

// the call back function registered by the user to signal end of intilisation
static void (*arm_uc_hub_init_cb)(int32_t) = NULL;

// The hub uses a double buffer system to speed up firmware download and storage
#define BUFFER_SIZE_MAX (ARM_UC_BUFFER_SIZE / 2) //  define size of the double buffers
static uint8_t message[BUFFER_SIZE_MAX];
static arm_uc_buffer_t front_buffer = {
    .size_max = BUFFER_SIZE_MAX,
    .size = 0,
    .ptr = message
};

static uint8_t message2[BUFFER_SIZE_MAX];
static arm_uc_buffer_t back_buffer = {
    .size_max = BUFFER_SIZE_MAX,
    .size = 0,
    .ptr = message2
};

// version (timestamp) of the current running application
static arm_uc_firmware_details_t arm_uc_active_details = { 0 };
static bool arm_uc_active_details_available = false;

// bootloader information
static arm_uc_installer_details_t arm_uc_installer_details = { 0 };

// variable to keep track of the offset into the firmware image during download
static uint32_t firmware_offset = 0;

// variable to store the firmware config during firmware manager setup
static ARM_UCFM_Setup_t arm_uc_hub_firmware_config = { 0 };

// the structure used to count the number of stored update images
typedef struct {
    uint32_t current_id;
    uint32_t num_images;
    uint64_t image_version;
    arm_uc_firmware_details_t details;
} arm_uc_stored_images_t;

// variable to store the stored images structure above
static arm_uc_stored_images_t stored_images = { 0 };

// buffer to store the decoded firmware key
#define PLAIN_FIRMWARE_KEY_SIZE 16
static uint8_t plainFirmwareKey[PLAIN_FIRMWARE_KEY_SIZE];
static arm_uc_buffer_t arm_uc_hub_plain_key = {
    .size_max = PLAIN_FIRMWARE_KEY_SIZE,
    .size     = PLAIN_FIRMWARE_KEY_SIZE,
    .ptr      = plainFirmwareKey
};

static arm_uc_mmContext_t manifestManagerContext = { 0 };
static arm_uc_mmContext_t* pManifestManagerContext = &manifestManagerContext;
static manifest_firmware_info_t fwinfo = { 0 };

// buffer to store a uri struct
#define URI_STRING_LEN 256
uint8_t uri_buffer[URI_STRING_LEN] = {0};

static arm_uc_uri_t uri = {
    .size_max = sizeof(uri_buffer),
    .size     = 0,
    .ptr      = uri_buffer,
    .port     = 0,
    .scheme   = URI_SCHEME_NONE,
    .host     = NULL,
    .path     = NULL,
};

/*****************************************************************************/
/* Debug                                                                     */
/*****************************************************************************/

#if ARM_UC_HUB_TRACE_ENABLE
static void arm_uc_hub_debug_output()
{
    printf("Manifest timestamp: %" PRIu64 "\r\n", fwinfo.timestamp);

    if (uri.scheme == URI_SCHEME_HTTP)
    {
        printf("Firmware URL http://%s:%" PRIu16 "%s\r\n",
            uri.host, uri.port, uri.path);
    }

    printf("Firmware size: %" PRIu32 "\r\n",fwinfo.size);

    printf("Firmware hash (%" PRIu32 "): ", fwinfo.hash.size);
    for (unsigned i = 0; i < fwinfo.hash.size; i++)
    {
        printf("%02" PRIx8, fwinfo.hash.ptr[i]);
    }
    printf("\r\n");

    if (fwinfo.cipherMode == ARM_UC_MM_CIPHERMODE_PSK)
    {
        printf("PSK ID: ");
        for (unsigned i = 0; i < fwinfo.psk.keyID.size; i++)
        {
            printf("%02" PRIx8, *(fwinfo.psk.keyID.ptr+i));
        }
        printf("\r\n");

        printf("cipherKey(16): ");
        for (unsigned i = 0; i < 16; i++)
        {
            printf("%02" PRIx8, *(fwinfo.psk.cipherKey.ptr+i));
        }
        printf("\r\n");

        printf("Decrypted Firmware Symmetric Key(16): ");
        for (unsigned i = 0; i < 16; i++)
        {
            printf("%02" PRIx8, arm_uc_hub_plain_key.ptr[i]);
        }
        printf("\r\n");

        printf("fwinfo.initVector\r\n");
        for (unsigned i = 0; i < 16; i++)
        {
            printf("%02" PRIx8, *(fwinfo.initVector.ptr+i));
        }
        printf("\r\n");
    }

    printf("Storage location: %" PRIu32 "\r\n",
           arm_uc_hub_firmware_config.package_id);
}
#endif

/*****************************************************************************/
/* State machine                                                             */
/*****************************************************************************/

/* Short hand for simple error handling code */
#define HANDLE_ERROR(retval, msg, ...)                  \
    if (retval.error != ERR_NONE)                       \
    {                                                   \
        UC_HUB_ERR_MSG(msg " error code %s",            \
                       ##__VA_ARGS__,                   \
                       ARM_UC_err2Str(retval));         \
        new_state = ARM_UC_HUB_STATE_IDLE;              \
        break;                                          \
    }

arm_uc_hub_state_t ARM_UC_HUB_getState()
{
    return arm_uc_hub_state;
}

void ARM_UC_HUB_setInitializationCallback(void (*callback)(int32_t))
{
    arm_uc_hub_init_cb = callback;
}

/**
 * @brief Return the active firmware details or NULL if they're not yet available.
 */
arm_uc_firmware_details_t* ARM_UC_HUB_getActiveFirmwareDetails(void)
{
    return arm_uc_active_details_available ? &arm_uc_active_details : NULL;
}

void ARM_UC_HUB_setState(arm_uc_hub_state_t new_state)
{
    arm_uc_error_t retval;

    /* Loop until state is unchanged.
       First loop is mandatory regardless of current state.
    */
    do
    {
        /* store new stage */
        arm_uc_hub_state = new_state;

        switch (arm_uc_hub_state)
        {
            /*****************************************************************/
            /* Initialization                                                */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_INITIALIZED:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_INITIALIZED");

                /* signal that the Hub is initialized */
                if (arm_uc_hub_init_cb)
                {
                    arm_uc_hub_init_cb(ARM_UC_INIT_DONE);
                }

                /* report the active firmware hash to the Cloud in parallel
                   with the main user application.
                */
                arm_uc_active_details_available = false;
                new_state = ARM_UC_HUB_STATE_GET_ACTIVE_FIRMWARE_DETAILS;
                break;

            /*****************************************************************/
            /* Report current firmware hash                                  */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_GET_ACTIVE_FIRMWARE_DETAILS:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_GET_ACTIVE_FIRMWARE_DETAILS");

                retval = ARM_UC_FirmwareManager.GetActiveFirmwareDetails(&arm_uc_active_details);
                HANDLE_ERROR(retval, "Firmware manager GetActiveFirmwareDetails failed");
                break;

            case ARM_UC_HUB_STATE_REPORT_ACTIVE_HASH:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_REPORT_ACTIVE_HASH");

                /* copy hash to buffer */
                memcpy(front_buffer.ptr,
                       arm_uc_active_details.hash,
                       ARM_UC_SHA256_SIZE);

                front_buffer.size = ARM_UC_SHA256_SIZE;

                /* send hash to update service */
                ARM_UC_ControlCenter_ReportName(&front_buffer);

                /* signal to the API that the firmware details are now available */
                arm_uc_active_details_available = true;

                new_state = ARM_UC_HUB_STATE_REPORT_ACTIVE_VERSION;
                break;

            /*****************************************************************/
            /* Report current firmware version                               */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_REPORT_ACTIVE_VERSION:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_REPORT_ACTIVE_VERSION");

                UC_HUB_TRACE("Active version: %" PRIu64,
                             arm_uc_active_details.version);

                /* send timestamp to update service */
                ARM_UC_ControlCenter_ReportVersion(arm_uc_active_details.version);

                new_state = ARM_UC_HUB_STATE_GET_INSTALLER_DETAILS;
                break;

            /*****************************************************************/
            /* Report bootloader information                                 */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_GET_INSTALLER_DETAILS:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_GET_INSTALLER_DETAILS");

                retval = ARM_UC_FirmwareManager.GetInstallerDetails(&arm_uc_installer_details);
                if (retval.error != ERR_NONE)
                {
                    UC_HUB_ERR_MSG("Firmware manager GetInstallerDetails failed error code %s",
                                   ARM_UC_err2Str(retval));
                    new_state = ARM_UC_HUB_STATE_CHECK_OEM_MODE_RESET;
                }
                break;

            case ARM_UC_HUB_STATE_REPORT_INSTALLER_DETAILS:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_REPORT_INSTALLER_DETAILS");

#if 0
                printf("bootloader: ");
                for (uint32_t index = 0; index < 20; index++)
                {
                    printf("%02X", arm_uc_installer_details.arm_hash[index]);
                }
                printf("\r\n");

                printf("layout: %" PRIu32 "\r\n", arm_uc_installer_details.layout);
#endif

                /* report installer details to mbed cloud */
                arm_uc_buffer_t bootloader_hash = {
                    .size_max = ARM_UC_SHA256_SIZE,
                    .size = ARM_UC_SHA256_SIZE,
                    .ptr = (arm_uc_installer_details.arm_hash)
                };
                ARM_UC_ControlCenter_ReportBootloaderHash(&bootloader_hash);

                bootloader_hash.ptr = (arm_uc_installer_details.oem_hash);
                ARM_UC_ControlCenter_ReportOEMBootloaderHash(&bootloader_hash);

                /* set new state */
                new_state = ARM_UC_HUB_STATE_CHECK_OEM_MODE_RESET;
                break;

            /*****************************************************************/
            /* Check if the OEM transfer mode needs to be reset              */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_CHECK_OEM_MODE_RESET:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_CHECK_OEM_MODE_RESET");
                {
                    uint32_t temp32 = 0;
                    /* read the "OEM transfer mode" flag and check if it is valid */
                    arm_uc_error_t sotp_status = arm_uc_read_sotp_uint(SOTP_TYPE_OEM_TRANSFER_MODE_ENABLED,
                                                                           sizeof(uint32_t), &temp32);
                    /* if the "OEM transfer mode" is enabled, check if it needs to be reset */
                    if (sotp_status.code == ERR_NONE && temp32 > 0)
                    {
                        UC_HUB_TRACE("Checking if the OEM transfer mode flag needs to be cleared");
                        /* "OEM transfer mode" needs to be reset if there is exactly one
                           candidate image and that it has the same timestamp as the active
                           image. So start looking at the available update images by changing
                           the state to ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS.
                        */
                        new_state = ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS;
                    }
                    else
                    {
                        new_state = ARM_UC_HUB_STATE_IDLE;
                    }
                }
                break;

            case ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS");
                /* Stop looking for firmware images either when the current image ID
                   is too large or when we found more than one valid stored image.
                */
                while (new_state == ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS)
                {
                    if (stored_images.current_id == MAX_FIRMWARE_LOCATIONS || stored_images.num_images > 1)
                    {
                        if (stored_images.num_images == 1 &&
                            stored_images.image_version == arm_uc_active_details.version)
                        {
                            /* set "OEM tranfer mode" to 0 */
                            uint32_t temp32 = 0;
                            arm_uc_error_t sotp_status = arm_uc_write_sotp_uint(SOTP_TYPE_OEM_TRANSFER_MODE_ENABLED,
                                                                                    sizeof(uint32_t), &temp32);
                            if (sotp_status.code == ERR_NONE)
                            {
                                UC_HUB_TRACE("OEM transfer mode flag cleared");
                            }
                            else
                            {
                                UC_HUB_ERR_MSG("Unable to clear the OEM tranfer mode flag");
                            }
                        }
                        new_state = ARM_UC_HUB_STATE_IDLE;
                    }
                    else
                    {
                        UC_HUB_TRACE("GetFirmwareDetails for image %" PRIu32, stored_images.current_id);
                        arm_uc_error_t ucp_status = ARM_UCP_GetFirmwareDetails(stored_images.current_id,
                                                                            &stored_images.details);
                        if (ucp_status.error != ERR_NONE)
                        {
                            UC_HUB_TRACE("GetFirmwareDetails for image %" PRIu32 " failed",
                                        stored_images.current_id);
                            /* error reading this image, try to read the next one */
                            stored_images.current_id ++;
                        }
                        else
                        {
                            /* GetFirmwareDetails will raise an event when it's done, which will change
                               the state to either ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_OK or
                               ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_ERROR below */
                            break;
                        }
                    }
                }
                break;

            case ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_OK:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_OK");
                stored_images.num_images ++;
                if (stored_images.num_images == 1)
                {
                    /* Store the image version that will be checked only for the first image.
                       If there are more images, the version is not relevant anymore, since
                       the "OEM transfer mode" won't be reset anyway, because it requires
                       exacly one valid stored image to be present.
                    */
                    stored_images.image_version = stored_images.details.version;
                }
                stored_images.current_id ++;
                new_state = ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS;
                break;

            case ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_ERROR:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_ERROR");
                stored_images.current_id ++;
                new_state = ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS;
                break;

            /*****************************************************************/
            /* Idle                                                          */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_IDLE:
                UC_HUB_TRACE("ARM_UC_MONITOR_STATE_IDLE");

                /* signal monitor that device has entered IDLE state */
                ARM_UC_ControlCenter_ReportState(ARM_UC_MONITOR_STATE_IDLE);
                break;

            /*****************************************************************/
            /* Download manifest                                             */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_NOTIFIED:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_NOTIFIED");

                /* notification received of a new manifest, hence go get said manifest */
                retval = ARM_UC_SourceManager.GetManifest(&front_buffer, 0);
                HANDLE_ERROR(retval, "Source manager GetManifest failed");
                break;

            case ARM_UC_HUB_STATE_MANIFEST_FETCHED:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_MANIFEST_FETCHED");

                /* Save the manifest for later */
                memcpy(&fwinfo.manifestBuffer, front_buffer.ptr,
                    ARM_UC_util_min(sizeof(fwinfo.manifestBuffer), front_buffer.size));
                /* Save the manifest size for later */
                fwinfo.manifestSize = front_buffer.size;
                /* insert the manifest we just fetched into manifest manager */
                retval = ARM_UC_mmInsert(&pManifestManagerContext, &front_buffer, &back_buffer,  NULL);
                if (retval.code != MFST_ERR_PENDING)
                {
                    HANDLE_ERROR(retval, "Manifest manager Insert failed")
                }
                break;

            /*****************************************************************/
            /* Rollback protection                                           */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_MANIFEST_COMPLETE:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_MANIFEST_COMPLETE");

                /* get the firmware info out of the manifest we just inserted
                   into the manifest manager
                */
                retval = ARM_UC_mmFetchFirmwareInfo(&pManifestManagerContext,
                                                    &fwinfo,
                                                    NULL);
                if (retval.code != MFST_ERR_PENDING)
                {
                    HANDLE_ERROR(retval, "Manifest manager fetch info failed")
                }
                break;

            case ARM_UC_HUB_STATE_CHECK_VERSION:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_CHECK_VERSION");
                {
                    /* check for OEM transfer mode and the minimal firmware version */
                    uint64_t min_fw_version = arm_uc_active_details.version;
                    uint32_t temp32 = 0;
                    /* read the "OEM transfer mode" flag and check if it is valid */
                    arm_uc_error_t sotp_status = arm_uc_read_sotp_uint(SOTP_TYPE_OEM_TRANSFER_MODE_ENABLED,
                                                                           sizeof(uint32_t), &temp32);
                    /* if the "OEM transfer mode" is enabled, read the minimum firmware version */
                    if (sotp_status.code == ERR_NONE && temp32 > 0)
                    {
                        uint64_t temp64 = 0;
                        sotp_status = arm_uc_read_sotp_uint(SOTP_TYPE_MIN_FW_VERSION, sizeof(uint64_t),
                                                                &temp64);
                        if (sotp_status.code == ERR_NONE)
                        {
                            min_fw_version = temp64;
                            UC_HUB_TRACE("OEM transfer mode enabled, minimum FW version is %" PRIu64,
                                        min_fw_version);
                        }
                        else
                        {
                            UC_HUB_TRACE("Can't read minimum FW version, OEM transfer mode disabled");
                        }
                    }

                    /* only continue if timestamp is newer than active version */
                    if (fwinfo.timestamp > min_fw_version)
                    {
                        /* set new state */
                        new_state = ARM_UC_HUB_STATE_PREPARE_FIRMWARE_SETUP;
                    }
                    else
                    {
                        UC_HUB_ERR_MSG("version: %" PRIu64 " <= %" PRIu64,
                                    fwinfo.timestamp,
                                    min_fw_version);

                        /* signal warning through external handler */
                        ARM_UC_HUB_ErrorHandler(HUB_ERR_ROLLBACK_PROTECTION,
                                                ARM_UC_HUB_STATE_CHECK_VERSION);

                        /* set new state */
                        new_state = ARM_UC_HUB_STATE_IDLE;
                    }
                }
                break;

            /*****************************************************************/
            /* Parse manifest                                                */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_PREPARE_FIRMWARE_SETUP:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_PREPARE_FIRMWARE_SETUP");

                /* store pointer to hash */
                arm_uc_hub_firmware_config.hash = &fwinfo.hash;

                /* parse the url string into a arm_uc_uri_t struct */
                retval = arm_uc_str2uri(fwinfo.uri.ptr,
                                        fwinfo.uri.size,
                                        &uri);

                /* URI-based errors are propagated to monitor */
                if (retval.error != ERR_NONE)
                {
                    /* make sure that the URI string is always 0-terminated */
                    fwinfo.uri.ptr[fwinfo.uri.size_max - 1] = '\0';
                    UC_HUB_ERR_MSG("Unable to parse URI string %s", fwinfo.uri.ptr);

                    /* signal warning through external handler */
                    ARM_UC_HUB_ErrorHandler(SOMA_ERR_INVALID_PARAMETER,
                                            ARM_UC_HUB_STATE_PREPARE_FIRMWARE_SETUP);

                    /* set new state */
                    new_state = ARM_UC_HUB_STATE_IDLE;
                    break;
                }

                /* store firmware size */
                arm_uc_hub_firmware_config.package_size = fwinfo.size;

                /* read cryptography mode to determine if firmware is encrypted */
                switch(fwinfo.cipherMode)
                {
                    case ARM_UC_MM_CIPHERMODE_NONE:
                        arm_uc_hub_firmware_config.mode = UCFM_MODE_NONE_SHA_256;
                        break;

                    case ARM_UC_MM_CIPHERMODE_PSK:
                        {
                            /* Get pre-shared-key from the Control Center */
                            /* TODO: this call should be asynchronous */
                            const uint8_t* arm_uc_pre_shared_key = NULL;
                            retval = ARM_UC_PreSharedKey_GetKey(&arm_uc_pre_shared_key, 128);
                            HANDLE_ERROR(retval, "Unable to get PSK");

                            /* Decode the firmware key to be used to decode the firmware */
                            UC_HUB_TRACE("Decoding firmware AES key...");
                            mbedtls_aes_context ctx;
                            mbedtls_aes_init(&ctx);
                            mbedtls_aes_setkey_dec(&ctx, arm_uc_pre_shared_key, 128);
                            mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_DECRYPT, fwinfo.psk.cipherKey.ptr, arm_uc_hub_plain_key.ptr);

                            arm_uc_hub_firmware_config.mode = UCFM_MODE_AES_CTR_128_SHA_256;
                            arm_uc_hub_firmware_config.key  = &arm_uc_hub_plain_key;
                            arm_uc_hub_firmware_config.iv   = &fwinfo.initVector;
                        }
                        break;

                    case ARM_UC_MM_CIPHERMODE_CERT_CIPHERKEY:
                    case ARM_UC_MM_CIPHERMODE_CERT_KEYTABLE:
                    default:
                        retval.code = MFST_ERR_CRYPTO_MODE;
                        HANDLE_ERROR(retval, "Unsupported AES Key distribution mode...");
                        break;
                }

                /* check if storage ID has been set */
                if (fwinfo.strgId.size == 0 || fwinfo.strgId.ptr == NULL)
                {
                    /* no storage ID set, use default value 0 */
                    arm_uc_hub_firmware_config.package_id = 0;
                }
                else
                {
                    /* check if storage ID is "default" */
                    uint32_t location = arm_uc_strnstrn(fwinfo.strgId.ptr,
                                                        fwinfo.strgId.size,
                                                        (const uint8_t*) "default",
                                                        7);

                    if (location != UINT32_MAX)
                    {
                        arm_uc_hub_firmware_config.package_id = 0;
                    }
                    else
                    {
                        /* parse storage ID */
                        bool success = false;
                        arm_uc_hub_firmware_config.package_id =
                            arm_uc_str2uint32(fwinfo.strgId.ptr,
                                              fwinfo.strgId.size,
                                              &success);
                    }
                }

#if ARM_UC_HUB_TRACE_ENABLE
                arm_uc_hub_debug_output();
#endif

                /* Set new state */
                new_state = ARM_UC_HUB_STATE_REQUEST_DOWNLOAD_AUTHORIZATION;
                break;

            /*****************************************************************/
            /* Download authorization                                        */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_REQUEST_DOWNLOAD_AUTHORIZATION:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_REQUEST_DOWNLOAD_AUTHORIZATION");

                /* Signal control center */
                ARM_UC_ControlCenter_GetAuthorization(ARM_UCCC_REQUEST_DOWNLOAD);

                /* Set new state */
                new_state = ARM_UC_HUB_STATE_WAIT_FOR_DOWNLOAD_AUTHORIZATION;
                break;

            case ARM_UC_HUB_STATE_WAIT_FOR_DOWNLOAD_AUTHORIZATION:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_WAIT_FOR_DOWNLOAD_AUTHORIZATION");
                break;

            case ARM_UC_HUB_STATE_DOWNLOAD_AUTHORIZED:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_DOWNLOAD_AUTHORIZED");

                /* Set new state */
                new_state = ARM_UC_HUB_STATE_SETUP_FIRMWARE;
                break;

            /*****************************************************************/
            /* Download firmware                                             */
            /*****************************************************************/

            /* The firmware is downloaded in fragments. While one fragment is
               written to storage, the next fragment is being downloaded.

               In the ARM_UC_HUB_STATE_FETCH_FIRST_FRAGMENT state, the first
               fragment is being downloaded. Once completed, the first fragment
               will be in the front_buffer and both the network stack and
               storage stack will be idle.

               In the ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD state, the front and
               back buffers are swapped. The front buffer is being used for
               downloading the next fragment while the back buffer is being
               written to storage.

               ARM_UC_FirmwareManager.Write and
               ARM_UC_SourceManager.GetFirmwareFragment will both finish
               asynchronously generating two events:
               ARM_UC_SM_EVENT_FIRMWARE and UCFM_EVENT_UPDATE_DONE.

               If the ARM_UC_SM_EVENT_FIRMWARE event is generated first, the
               system enters the ARM_UC_HUB_STATE_WAIT_FOR_STORAGE state.
               If the UCFM_EVENT_UPDATE_DONE event is generated first, the
               system enters the ARM_UC_HUB_STATE_WAIT_FOR_NETWORK state.
               The second generated event will move the system back to the
               ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD state.

               The download will stop once the fragment offset is larger than
               the firmware size written in the manifest. This moves the system
               to the ARM_UC_HUB_STATE_STORE_LAST_FRAGMENT state.

               Once the last fragment is written, the newly written firmware
               committed in the ARM_UC_HUB_STATE_FINALIZE_STORAGE state.
            */
            case ARM_UC_HUB_STATE_SETUP_FIRMWARE:
                {
                    UC_HUB_TRACE("ARM_UC_HUB_STATE_SETUP_FIRMWARE");

                    /* store the firmware info in the manifest_firmware_info_t struct */
                    arm_uc_firmware_details_t arm_uc_hub_firmware_details = { 0 };

                    /* use manifest timestamp as firmware header version */
                    arm_uc_hub_firmware_details.version = fwinfo.timestamp;
                    arm_uc_hub_firmware_details.size    = fwinfo.size;

                    /* copy hash */
                    memcpy(arm_uc_hub_firmware_details.hash,
                        fwinfo.hash.ptr,
                        ARM_UC_SHA256_SIZE);

    #if 0
                    memcpy(arm_uc_hub_firmware_details.campaign,
                        configuration.campaign,
                        ARM_UC_GUID_SIZE);
    #endif

                    /* setup the firmware manager to get ready for firmware storage */
                    retval = ARM_UC_FirmwareManager.Prepare(&arm_uc_hub_firmware_config,
                                                            &arm_uc_hub_firmware_details,
                                                            &front_buffer);
                    HANDLE_ERROR(retval, "ARM_UC_FirmwareManager Setup failed")
                }
                break;

            case ARM_UC_HUB_STATE_FETCH_FIRST_FRAGMENT:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_FETCH_FIRST_FRAGMENT");

                /* set state to downloading when trying to fetch the first chunk of firmware */
                UC_HUB_TRACE("Setting Monitor State: ARM_UC_MONITOR_STATE_DOWNLOADING");
                ARM_UC_ControlCenter_ReportState(ARM_UC_MONITOR_STATE_DOWNLOADING);

                /* reset download values */
                front_buffer.size = 0;
                back_buffer.size = 0;
                firmware_offset = 0;

                /* Check firmware size before entering the download state machine.
                   An empty firmware is used for erasing a slot.
                */
                if (firmware_offset < fwinfo.size)
                {
                    /* get first firmware fragment */
                    UC_HUB_TRACE("Getting next chunk at offset %" PRIu32,
                                 firmware_offset);
                    retval = ARM_UC_SourceManager.GetFirmwareFragment(&uri,
                                                                      &front_buffer,
                                                                      firmware_offset);
                    HANDLE_ERROR(retval, "GetFirmwareFragment failed")
                }
                else
                {
                    /* No download is necessary, send next state to monitor service
                       and close storage slot.
                    */
                    UC_HUB_TRACE("Firmware empty, skip download phase and finalize");
                    UC_HUB_TRACE("Setting Monitor State: ARM_UC_MONITOR_STATE_DOWNLOADED");
                    ARM_UC_ControlCenter_ReportState(ARM_UC_MONITOR_STATE_DOWNLOADED);
                    new_state = ARM_UC_HUB_STATE_FINALIZE_STORAGE;
                }
                break;

            case ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD");

                /* swap the front and back buffers
                   the back buffer contained just downloaded firmware chunk
                   the front buffer can now be cleared and used to download new chunk
                */
                {
                    arm_uc_buffer_t temp_buf_ptr = front_buffer;
                    front_buffer = back_buffer;
                    back_buffer = temp_buf_ptr;
                }

                UC_HUB_TRACE("Fragment: %" PRIu32 " %" PRIu32, firmware_offset, back_buffer.size);

                /* increase offset by the amount that we just downloaded */
                firmware_offset += back_buffer.size;

                /* store the downloaded chunk in the back buffer */
                if (back_buffer.size > 0)
                {
                    retval = ARM_UC_FirmwareManager.Write(&back_buffer);
                    HANDLE_ERROR(retval, "ARM_UC_FirmwareManager Update failed")
                }

                /* go fetch a new chunk using the front buffer if more are expected */
                if (firmware_offset < fwinfo.size)
                {
                    front_buffer.size = 0;
                    UC_HUB_TRACE("Getting next chunk at offset: %" PRIu32, firmware_offset);
                    retval = ARM_UC_SourceManager.GetFirmwareFragment(&uri, &front_buffer, firmware_offset);
                    HANDLE_ERROR(retval, "GetFirmwareFragment failed")
                }
                else
                {
                    UC_HUB_TRACE("Store last fragment");
                    new_state = ARM_UC_HUB_STATE_STORE_LAST_FRAGMENT;
                }

                /* report progress */
                ARM_UC_ControlCenter_ReportProgress(firmware_offset, fwinfo.size);
                break;

            case ARM_UC_HUB_STATE_WAIT_FOR_STORAGE:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_WAIT_FOR_STORAGE");
                break;

            case ARM_UC_HUB_STATE_WAIT_FOR_NETWORK:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_WAIT_FOR_NETWORK");
                break;

            case ARM_UC_HUB_STATE_STORE_LAST_FRAGMENT:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_STORE_LAST_FRAGMENT");

                /* set state to downloaded when the full size of the firmware
                   have been fetched.
                */
                UC_HUB_TRACE("Setting Monitor State: ARM_UC_MONITOR_STATE_DOWNLOADED");
                ARM_UC_ControlCenter_ReportState(ARM_UC_MONITOR_STATE_DOWNLOADED);
                break;

            case ARM_UC_HUB_STATE_FINALIZE_STORAGE:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_FINALIZE_STORAGE");

                retval = ARM_UC_FirmwareManager.Finalize(&front_buffer, &back_buffer);
                HANDLE_ERROR(retval, "ARM_UC_FirmwareManager Finalize failed")
                break;

            /*****************************************************************/
            /* Install authorization                                         */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_STORAGE_FINALIZED:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_STORAGE_FINALIZED");

                /* Signal control center */
                ARM_UC_ControlCenter_GetAuthorization(ARM_UCCC_REQUEST_INSTALL);

                /* Set new state */
                new_state = ARM_UC_HUB_STATE_WAIT_FOR_INSTALL_AUTHORIZATION;
                break;

            case ARM_UC_HUB_STATE_WAIT_FOR_INSTALL_AUTHORIZATION:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_WAIT_FOR_INSTALL_AUTHORIZATION");
                break;

            case ARM_UC_HUB_STATE_INSTALL_AUTHORIZED:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_INSTALL_AUTHORIZED");

                UC_HUB_TRACE("Setting Monitor State: ARM_UC_MONITOR_STATE_UPDATING");
                ARM_UC_ControlCenter_ReportState(ARM_UC_MONITOR_STATE_UPDATING);

/* TODO: set timeout on ReportState before relying on callback to progress state machine */
                new_state = ARM_UC_HUB_STATE_ACTIVATE_FIRMWARE;
                break;

            case ARM_UC_HUB_STATE_ACTIVATE_FIRMWARE:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_ACTIVATE_FIRMWARE");

                /* Firmware verification passes, activate firmware image.
                */
                ARM_UC_FirmwareManager.Activate(arm_uc_hub_firmware_config.package_id);
                break;

            case ARM_UC_HUB_STATE_REBOOT:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_REBOOT");

                /* Firmware activated, now reboot the system to apply
                   the new image.
                */
                pal_osReboot();

                /* Reboot not implemented on this platform.
                   Report new firmware hash and continue operation.
                */
                new_state = ARM_UC_HUB_STATE_GET_ACTIVE_FIRMWARE_DETAILS;
                break;

            /*****************************************************************/
            /* Error                                                         */
            /*****************************************************************/
            case ARM_UC_HUB_STATE_ERROR_FIRMWARE_MANAGER:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_ERROR_FIRMWARE_MANAGER");
                new_state = ARM_UC_HUB_STATE_IDLE;
                break;

            case ARM_UC_HUB_STATE_ERROR_MANIFEST_MANAGER:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_ERROR_MANIFEST_MANAGER");
                new_state = ARM_UC_HUB_STATE_IDLE;
                break;

            case ARM_UC_HUB_STATE_ERROR_SOURCE_MANAGER:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_ERROR_SOURCE_MANAGER");
                new_state = ARM_UC_HUB_STATE_IDLE;
                break;

            case ARM_UC_HUB_STATE_ERROR_CONTROL_CENTER:
                UC_HUB_TRACE("ARM_UC_HUB_STATE_ERROR_CONTROL_CENTER");
                new_state = ARM_UC_HUB_STATE_IDLE;
                break;

            default:
                new_state = ARM_UC_HUB_STATE_IDLE;
                break;
        }
    }
    while (arm_uc_hub_state != new_state);
}

