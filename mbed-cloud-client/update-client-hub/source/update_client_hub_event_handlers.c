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

#include "update_client_hub_event_handlers.h"
#include "update_client_hub_error_handler.h"

#include "update_client_hub_state_machine.h"

#include "update-client-common/arm_uc_common.h"
#include "update-client-firmware-manager/arm_uc_firmware_manager.h"
#include "update-client-manifest-manager/update-client-manifest-manager.h"
#include "update-client-source-manager/arm_uc_source_manager.h"
#include "update-client-control-center/arm_uc_control_center.h"

/**
 * @brief event handler registered with the firmware manager
 *        events drive state changes of the state machine
 */
void ARM_UC_HUB_FirmwareManagerEventHandler(uint32_t event)
{
    arm_uc_hub_state_t arm_uc_hub_state = ARM_UC_HUB_getState();

    switch(event)
    {
        /* Firmware writing setup complete */
        case UCFM_EVENT_PREPARE_DONE:
            UC_HUB_TRACE("UCFM_EVENT_PREPARE_DONE");

            /* Storage prepared for firmware. Download first fragment. */
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_SETUP_FIRMWARE)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_FETCH_FIRST_FRAGMENT);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            }
            break;

        /* Firmware fragment written */
        case UCFM_EVENT_WRITE_DONE:
            UC_HUB_TRACE("UCFM_EVENT_WRITE_DONE");

            /* Firmware fragment stored */

            /* Fragment stored before network could finish,
               i.e., network is the bottleneck:
               Action:
                - wait for network to complete
            */
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_WAIT_FOR_NETWORK);
            }
            /* Fragment stored after network finished,
               i.e, storage is the bottleneck:
               Action:
                - store fragment
                - download next fragment
            */
            else if (arm_uc_hub_state == ARM_UC_HUB_STATE_WAIT_FOR_STORAGE)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD);
            }
            /* Last fragment stored.
               Action:
                - finalize storage
            */
            else if (arm_uc_hub_state == ARM_UC_HUB_STATE_STORE_LAST_FRAGMENT)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_FINALIZE_STORAGE);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            }
            break;

        /* Final firmware fragment written and commited */
        case UCFM_EVENT_FINALIZE_DONE:
            UC_HUB_TRACE("UCFM_EVENT_FINALIZE_DONE");

            /* Firmware stored and verified. */
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_FINALIZE_STORAGE)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_STORAGE_FINALIZED);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            }
            break;

        /* Firmware image marked as active or installed */
        case UCFM_EVENT_ACTIVATE_DONE:
            UC_HUB_TRACE("UCFM_EVENT_ACTIVATE_DONE");

            /* Firmware activated. Reboot system. */
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_ACTIVATE_FIRMWARE)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_REBOOT);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            }
            break;

        case UCFM_EVENT_GET_ACTIVE_FIRMWARE_DETAILS_DONE:
            UC_HUB_TRACE("UCFM_EVENT_GET_ACTIVE_FIRMWARE_DETAILS_DONE");

            ARM_UC_HUB_setState(ARM_UC_HUB_STATE_REPORT_ACTIVE_HASH);
            break;

        case UCFM_EVENT_GET_INSTALLER_DETAILS_DONE:
            UC_HUB_TRACE("UCFM_EVENT_GET_INSTALLER_DETAILS_DONE");

            ARM_UC_HUB_setState(ARM_UC_HUB_STATE_REPORT_INSTALLER_DETAILS);
            break;

        /* Encountered error while writing firmware */
        case UCFM_EVENT_WRITE_ERROR:
            UC_HUB_TRACE("UCFM_EVENT_WRITE_ERROR");

            if ((arm_uc_hub_state == ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD) ||
                (arm_uc_hub_state == ARM_UC_HUB_STATE_WAIT_FOR_STORAGE) ||
                (arm_uc_hub_state == ARM_UC_HUB_STATE_STORE_LAST_FRAGMENT))
            {
                /* write error. */
                ARM_UC_HUB_ErrorHandler(FIRM_ERR_WRITE, arm_uc_hub_state);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            }
            break;

        /* Encountered error while committing final firmware fragment */
        case UCFM_EVENT_FINALIZE_ERROR:
            UC_HUB_TRACE("UCFM_EVENT_FINALIZE_ERROR");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            break;


        /* Got an unexpected hash of the firmare image */
        case UCFM_EVENT_FINALIZE_INVALID_HASH_ERROR:
            UC_HUB_TRACE("UCFM_EVENT_FINALIZE_INVALID_HASH_ERROR");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_HASH, arm_uc_hub_state);
            break;

        /* Encountered error while trying to activate or install firmware image */
        case UCFM_EVENT_ACTIVATE_ERROR:
            UC_HUB_TRACE("UCFM_EVENT_ACTIVATE_ERROR");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(FIRM_ERR_ACTIVATE, arm_uc_hub_state);
            break;

        case UCFM_EVENT_GET_ACTIVE_FIRMWARE_DETAILS_ERROR:
            /* client should be able to proceed as normal */
            UC_HUB_TRACE("UCFM_EVENT_GET_ACTIVE_FIRMWARE_DETAILS_ERROR");

            ARM_UC_HUB_setState(ARM_UC_HUB_STATE_GET_INSTALLER_DETAILS);
            break;

        case UCFM_EVENT_GET_INSTALLER_DETAILS_ERROR:
            /* client should be able to proceed as normal */
            UC_HUB_TRACE("UCFM_EVENT_GET_INSTALLER_DETAILS_ERROR");

            ARM_UC_HUB_setState(ARM_UC_HUB_STATE_IDLE);
            break;

        case UCFM_EVENT_INITIALIZE_DONE:
            /* TODO Fix whole call chain to support async init */
            /* swallow async init done event here */
            UC_HUB_TRACE("UCFM_EVENT_INITIALIZE_DONE");
            break;

        case UCFM_EVENT_GET_FIRMWARE_DETAILS_DONE:
            UC_HUB_TRACE("UCFM_EVENT_GET_FIRMWARE_DETAILS_DONE");
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_OK);
            }
            break;

        case UCFM_EVENT_INITIALIZE_ERROR:
            UC_HUB_TRACE("UCFM_EVENT_INITIALIZE_ERROR");
            break;

        case UCFM_EVENT_PREPARE_ERROR:
            UC_HUB_TRACE("UCFM_EVENT_PREPARE_ERROR");
            break;

        case UCFM_EVENT_GET_FIRMWARE_DETAILS_ERROR:
            UC_HUB_TRACE("UCFM_EVENT_GET_FIRMWARE_DETAILS_ERROR");
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_GET_STORED_FIRMWARE_DETAILS)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_STORED_FIRMWARE_DETAILS_ERROR);
            }
            break;

        default:
            UC_HUB_TRACE("Firmware Manager: Invalid Event: %" PRIu32, event);

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(FIRM_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            break;
    }
}

/**
 * @brief event handler registered with the manifest manager
 *        events drive state changes of the state machine
 */
void ARM_UC_HUB_ManifestManagerEventHandler(uint32_t event)
{
    arm_uc_hub_state_t arm_uc_hub_state = ARM_UC_HUB_getState();

    if (event == ARM_UC_MM_RC_ERROR && arm_uc_hub_state == ARM_UC_HUB_STATE_UNINITIALIZED)
    {
        // TODO: An empty config store may be an error in the future.
        if (ARM_UC_mmGetError().code == MFST_ERR_NO_MANIFEST)
        {
            event = ARM_UC_MM_RC_DONE;
        }
    }
    switch(event)
    {
        /* Status: The manifest manager failed during the previous operation.
           Extract further error information using ARM_UC_mmGetError.
        */
        case ARM_UC_MM_RC_ERROR:
            UC_HUB_TRACE("ARM_UC_MM_RC_ERROR");

            /* Report error. */
            ARM_UC_HUB_ErrorHandler(ARM_UC_mmGetError().code, arm_uc_hub_state);
            break;

        /* Unused */
        case ARM_UC_MM_RC_NONE:
            UC_HUB_TRACE("ARM_UC_MM_RC_NONE");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(MFST_ERR_INVALID_STATE, arm_uc_hub_state);
            break;

        /* Action: The firmware manager needs the manifest specified by
           ARM_UC_mmGetCurrentManifestDependency in order to keep processing
           the firmware update.
        */
        case ARM_UC_MM_RC_NEED_DEP:
            UC_HUB_TRACE("ARM_UC_MM_RC_NEED_DEP");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(MFST_ERR_NO_MANIFEST, arm_uc_hub_state);
            break;

        /* Action: The firmware manager needs the firmware specified by
           ARM_UC_mmGetFirmwareInfo in order to keep processing the firmware
           update.
        */
        case ARM_UC_MM_RC_NEED_FW:
            UC_HUB_TRACE("ARM_UC_MM_RC_NEED_FW");

            /* Download firmware by first reading information from manifest */
            if(arm_uc_hub_state == ARM_UC_HUB_STATE_MANIFEST_COMPLETE)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_CHECK_VERSION);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(MFST_ERR_INVALID_STATE, arm_uc_hub_state);
            }
            break;

        /* Status: The last operation completed successfully */
        case ARM_UC_MM_RC_DONE:
            UC_HUB_TRACE("ARM_UC_MM_RC_DONE");

            /* Update Hub has been initialized. */
            if(arm_uc_hub_state == ARM_UC_HUB_STATE_UNINITIALIZED)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_INITIALIZED);
            }
            /* Manifest processed */
            else if(arm_uc_hub_state == ARM_UC_HUB_STATE_MANIFEST_FETCHED)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_MANIFEST_COMPLETE);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(MFST_ERR_INVALID_STATE, arm_uc_hub_state);
            }
            break;

        default:
            UC_HUB_TRACE("Manifest Manager: Invalid Event");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(MFST_ERR_BAD_EVENT, arm_uc_hub_state);
            break;
    }
}

/**
 * @brief event handler registered with the source manager
 *        events drive state changes of the state machine
 */
void ARM_UC_HUB_SourceManagerEventHandler(uint32_t event)
{
    arm_uc_hub_state_t arm_uc_hub_state = ARM_UC_HUB_getState();

    switch(event)
    {
        /* Received notification */
        case ARM_UC_SM_EVENT_NOTIFICATION:
            UC_HUB_TRACE("ARM_UC_SM_EVENT_NOTIFICATION");

            /* Fetch manifest */
            if(arm_uc_hub_state == ARM_UC_HUB_STATE_IDLE)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_NOTIFIED);
            }
            /* No else. All notifications are ignored during an ongoing update. */
            break;

        /* Downloaded manifest */
        case ARM_UC_SM_EVENT_MANIFEST:
            UC_HUB_TRACE("ARM_UC_SM_EVENT_MANIFEST");

            /* Process the newly downloaded manifest */
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_NOTIFIED)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_MANIFEST_FETCHED);
            }
            else
            {
                /* Invalid state, abort and report error. */
                ARM_UC_HUB_ErrorHandler(SOMA_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            }
            break;

        /* Downloaded firmware fragment */
        case ARM_UC_SM_EVENT_FIRMWARE:
            UC_HUB_TRACE("ARM_UC_SM_EVENT_FIRMWARE");

            /* Received firmware fragment */

            /* 1. First fragment received, source and storage idle, or,
               2. N fragment received after storage went idle.
               Action:
                - store fragment
                - download next fragment
            */
            if ((arm_uc_hub_state == ARM_UC_HUB_STATE_FETCH_FIRST_FRAGMENT) ||
                (arm_uc_hub_state == ARM_UC_HUB_STATE_WAIT_FOR_NETWORK))
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD);
            }
            /* 1. N fragment received, storing still in progress,
               i.e., the storage is the bottleneck:
               - All buffers are in use, wait for storage
            */
            else if (arm_uc_hub_state == ARM_UC_HUB_STATE_STORE_AND_DOWNLOAD)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_WAIT_FOR_STORAGE);
            }
            /* Invalid state, abort and report error. */
            else
            {
                ARM_UC_HUB_ErrorHandler(SOMA_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            }
            break;

        /* Downloaded keytable */
        case ARM_UC_SM_EVENT_KEYTABLE:
            UC_HUB_TRACE("ARM_UC_SM_EVENT_KEYTABLE");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(SOMA_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            break;

        /* Source Manager encountered an error */
        case ARM_UC_SM_EVENT_ERROR:
            UC_HUB_TRACE("ARM_UC_SM_EVENT_ERROR");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(SOMA_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            break;

        default:
            UC_HUB_TRACE("Source Manager: Invalid Event");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(SOMA_ERR_INVALID_PARAMETER, arm_uc_hub_state);
            break;
    }
}

void ARM_UC_HUB_ControlCenterEventHandler(uint32_t event)
{
    arm_uc_hub_state_t arm_uc_hub_state = ARM_UC_HUB_getState();

    switch(event)
    {
        case ARM_UCCC_EVENT_AUTHORIZE_DOWNLOAD:
            UC_HUB_TRACE("ARM_UCCC_EVENT_AUTHORIZE_DOWNLOAD");

            if (arm_uc_hub_state == ARM_UC_HUB_STATE_WAIT_FOR_DOWNLOAD_AUTHORIZATION)
            {
                /* Download approved. Download firmware. */
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_DOWNLOAD_AUTHORIZED);
            }
            break;

        case ARM_UCCC_EVENT_AUTHORIZE_INSTALL:
            UC_HUB_TRACE("ARM_UCCC_EVENT_AUTHORIZE_INSTALL");

            if (arm_uc_hub_state == ARM_UC_HUB_STATE_WAIT_FOR_INSTALL_AUTHORIZATION)
            {
                /* Installation approved. Set firmware as active image and reboot. */
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_INSTALL_AUTHORIZED);
            }
            break;

        case ARM_UCCC_EVENT_MONITOR_SEND_DONE:
            UC_HUB_TRACE("ARM_UCCC_EVENT_MONITOR_SEND_DONE");

/* TODO: use timeout to ensure callback doesn't stall reboot */
#if 0
            if (arm_uc_hub_state == ARM_UC_HUB_STATE_INSTALL_AUTHORIZED)
            {
                ARM_UC_HUB_setState(ARM_UC_HUB_STATE_REBOOT);
            }
#endif
            break;

        default:
            UC_HUB_TRACE("Control Center: Invalid Event");

            /* Invalid state, abort and report error. */
            ARM_UC_HUB_ErrorHandler(ERR_INVALID_PARAMETER, arm_uc_hub_state);
            break;
    }
}
