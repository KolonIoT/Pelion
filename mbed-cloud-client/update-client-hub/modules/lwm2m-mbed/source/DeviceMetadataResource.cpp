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

#include "update-client-lwm2m/DeviceMetadataResource.h"

#include "update-client-common/arm_uc_common.h"
#include "pal4life-device-identity/pal_device_identity.h"

#include <stdio.h>

#define ARM_UCS_LWM2M_INTERNAL_ERROR (-1)
#define ARM_UCS_LWM2M_INTERNAL_SUCCESS (0)

namespace DeviceMetadataResource {
    /* LWM2M Firmware Update Object */
    static M2MObject* deviceMetadataObject;

    /* LWM2M Firmware Update Object resources */
    static M2MResource* protocolSupportedResource = NULL; // /10255/0/0
    static M2MResource* bootloaderHashResource    = NULL; // /10255/0/1
    static M2MResource* OEMBootloaderHashResource = NULL; // /10255/0/2
    static M2MResource* vendorIdResource          = NULL; // /10255/0/3
    static M2MResource* classIdResource           = NULL; // /10255/0/4
    static M2MResource* deviceIdResource          = NULL; // /10255/0/5
}


/**
 * @brief Initialize LWM2M Device Metadata Object
 * @details Sets up LWM2M object with accompanying resources.
 */
void DeviceMetadataResource::Initialize(void)
{
    static bool initialized = false;

    if (!initialized)
    {
        initialized = true;

        /* The LWM2M Firmware Update Object is at /10255 */
        deviceMetadataObject = M2MInterfaceFactory::create_object("10255");

        if (deviceMetadataObject)
        {
            /* Set object operating mode to GET_ALLOWED */
            deviceMetadataObject->set_operation(M2MBase::GET_ALLOWED);
            /* Create first (and only) instance /10255/0 */
            M2MObjectInstance* deviceMetadataInstance = deviceMetadataObject->create_object_instance();

            if (deviceMetadataInstance)
            {
                /* Default values are non-standard, but the standard has no
                   values for indicating that the device is initializing.
                */
                int64_t defaultVersion   = 1;
                const uint8_t invalid_value[]    = "INVALID";
                const uint8_t invalid_value_size = sizeof(invalid_value) - 1;

                arm_uc_error_t err    = { .code = ERR_INVALID_PARAMETER };
                arm_uc_guid_t guid    = { 0 };
                uint8_t* value        = NULL;
                uint32_t value_length = 0;

                /* Set instance operating mode to GET_ALLOWED */
                deviceMetadataInstance->set_operation(M2MBase::GET_ALLOWED);

                /* Create Update resource /10255/0/0 */
                protocolSupportedResource = deviceMetadataInstance->create_dynamic_resource(
                                                    "0",
                                                    "ProtocolSupported",
                                                    M2MResourceInstance::INTEGER,
                                                    true);
                if (protocolSupportedResource)
                {
                    protocolSupportedResource->set_operation(M2MBase::GET_ALLOWED);
                    protocolSupportedResource->set_value(defaultVersion);
                }

                /* Create Update resource /10255/0/1 */
                bootloaderHashResource = deviceMetadataInstance->create_static_resource(
                                                    "1",
                                                    "BootloaderHash",
                                                    M2MResourceInstance::OPAQUE,
                                                    (uint8_t *) invalid_value,
                                                    invalid_value_size);
                if (bootloaderHashResource)
                {
                    bootloaderHashResource->set_operation(M2MBase::GET_ALLOWED);
                }

                /* Create Update resource /10255/0/2 */
                OEMBootloaderHashResource = deviceMetadataInstance->create_static_resource(
                                                    "2",
                                                    "OEMBootloaderHash",
                                                    M2MResourceInstance::OPAQUE,
                                                    (uint8_t *) invalid_value,
                                                    invalid_value_size);
                if (OEMBootloaderHashResource)
                {
                    OEMBootloaderHashResource->set_operation(M2MBase::GET_ALLOWED);
                }

                /* get vendor ID */
                err = pal_getVendorGuid(&guid);
                if (err.error == ERR_NONE)
                {
                    value = (uint8_t *) &guid;
                    value_length = sizeof(arm_uc_guid_t);
                }
                else
                {
                    value = (uint8_t *) invalid_value;
                    value_length = invalid_value_size;
                }

                /* Create Update resource /10255/0/3 */
                vendorIdResource = deviceMetadataInstance->create_dynamic_resource(
                                                    "3",
                                                    "Vendor",
                                                    M2MResourceInstance::OPAQUE,
                                                    true);

                if (vendorIdResource)
                {
                    vendorIdResource->set_operation(M2MBase::GET_ALLOWED);
                    vendorIdResource->set_value(value, value_length);
                }

                /* get class ID */
                err = pal_getClassGuid(&guid);
                if (err.error == ERR_NONE)
                {
                    value = (uint8_t *) &guid;
                    value_length = sizeof(arm_uc_guid_t);
                }
                else
                {
                    value = (uint8_t *) invalid_value;
                    value_length = invalid_value_size;
                }

                /* Create Update resource /10255/0/4 */
                classIdResource = deviceMetadataInstance->create_dynamic_resource(
                                                    "4",
                                                    "Class",
                                                    M2MResourceInstance::OPAQUE,
                                                    true);

                if (classIdResource)
                {
                    classIdResource->set_operation(M2MBase::GET_ALLOWED);
                    classIdResource->set_value(value, value_length);
                }

                /* get device ID */
                err = pal_getDeviceGuid(&guid);
                if (err.error == ERR_NONE)
                {
                    value = (uint8_t *) &guid;
                    value_length = sizeof(arm_uc_guid_t);
                }
                else
                {
                    value = (uint8_t *) invalid_value;
                    value_length = invalid_value_size;
                }

                /* Create Update resource /10255/0/5 */
                deviceIdResource = deviceMetadataInstance->create_static_resource(
                                                    "5",
                                                    "DeviceId",
                                                    M2MResourceInstance::OPAQUE,
                                                    value,
                                                    value_length);
                if (deviceIdResource)
                {
                    deviceIdResource->set_operation(M2MBase::GET_ALLOWED);
                }
            }
        }
    }
}

int32_t DeviceMetadataResource::setBootloaderHash(arm_uc_buffer_t* hash)
{
    UC_SRCE_TRACE("DeviceMetadataResource::setBootloaderHash ptr %p size %u", hash, hash->size);

    int32_t result = ARM_UCS_LWM2M_INTERNAL_ERROR;

    if (bootloaderHashResource && hash && hash->size > 0)
    {
        bool rt = bootloaderHashResource->set_value(hash->ptr, hash->size);
        if (rt == true)
        {
            result = ARM_UCS_LWM2M_INTERNAL_SUCCESS;
        }
    }

    return result;
}

int32_t DeviceMetadataResource::setOEMBootloaderHash(arm_uc_buffer_t* hash)
{
    UC_SRCE_TRACE("DeviceMetadataResource::setOEMBootloaderHash ptr %p size %u", hash, hash->size);

    int32_t result = ARM_UCS_LWM2M_INTERNAL_ERROR;

    if (OEMBootloaderHashResource && hash && hash->size > 0)
    {
        bool rt = OEMBootloaderHashResource->set_value(hash->ptr, hash->size);
        if (rt == true)
        {
            result = ARM_UCS_LWM2M_INTERNAL_SUCCESS;
        }
    }

    return result;
}

M2MObject* DeviceMetadataResource::getObject()
{
    Initialize();

    return deviceMetadataObject;
}

void DeviceMetadataResource::Uninitialize()
{
    UC_SRCE_TRACE("DeviceMetadataResource::Uninitialize");
    delete deviceMetadataObject;
    deviceMetadataObject = NULL;
}