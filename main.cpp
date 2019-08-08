// ----------------------------------------------------------------------------
// Copyright 2016-2018 ARM Ltd.
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

#include "simplem2mclient.h"
#ifdef TARGET_LIKE_MBED
#include "mbed.h"
#endif
#include "application_init.h"
#include "common_button_and_led.h"
#include "blinky.h"

#if 1		/* Step 1 */
// Sensors related includes and initialization
// Temperature reading from microcontroller
#define Kolon_VERSION                  "20190808VT00-Test"
AnalogIn adc_temp(ADC_TEMP);
// Voltage reference reading from microcontroller
AnalogIn adc_vref(ADC_VREF);
#endif 

// event based LED blinker, controlled via pattern_resource
static Blinky blinky;

Thread data_thread;
EventQueue  queue;

// https://os.mbed.com/users/ejteb/code/HC_SR04_Ultrasonic_Library/
//void dist(int distance);
//ultrasonic mu(D7, D8, .1, 1, &dist);
//ultrasonic mu(D7, D8, .1, 1);	// library bug --> _t.start (); xxx

static void main_application(void);

int main(void)
{
    mcc_platform_run_program(main_application);
}

// Pointers to the resources that will be created in main_application().
static M2MResource* button_res;
static M2MResource* pattern_res;
static M2MResource* blink_res;

#if 1 /* Step 3*/
// Additional resources for sensor readings
static M2MResource* res_temperature;
static M2MResource* res_voltage;
#endif 

// Pointer to mbedClient, used for calling close function.
static SimpleM2MClient *client;

void pattern_updated(const char *)
{
    printf("PUT received, new value: %s\n", pattern_res->get_value_string().c_str());
}

void blink_callback(void *)
{
    String pattern_string = pattern_res->get_value_string();
    const char *pattern = pattern_string.c_str();
    printf("LED pattern = %s\n", pattern);

    // The pattern is something like 500:200:500, so parse that.
    // LED blinking is done while parsing.
    const bool restart_pattern = false;
    if (blinky.start((char*)pattern_res->value(), pattern_res->value_length(), restart_pattern) == false) {
        printf("out of memory error\n");
    }
}

void button_notification_status_callback(const M2MBase& object, const NoticationDeliveryStatus status)
{
    switch(status) {
        case NOTIFICATION_STATUS_BUILD_ERROR:
            printf("Notification callback: (%s) error when building CoAP message\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_RESEND_QUEUE_FULL:
            printf("Notification callback: (%s) CoAP resend queue full\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SENT:
            printf("Notification callback: (%s) Notification sent to server\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_DELIVERED:
            printf("Notification callback: (%s) Notification delivered\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SEND_FAILED:
            printf("Notification callback: (%s) Notification sending failed\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_SUBSCRIBED:
            printf("Notification callback: (%s) subscribed\n", object.uri_path());
            break;
        case NOTIFICATION_STATUS_UNSUBSCRIBED:
            printf("Notification callback: (%s) subscription removed\n", object.uri_path());
            break;
        default:
            break;
    }
}

// This function is called when a POST request is received for resource 5000/0/1.
void unregister(void *)
{
    printf("Unregister resource executed\n");
    client->close();
}

// This function is called when a POST request is received for resource 5000/0/2.
void factory_reset(void *)
{
    printf("Factory reset resource executed\n");
    client->close();
    kcm_status_e kcm_status = kcm_factory_reset();

    if (kcm_status != KCM_STATUS_SUCCESS) {
        printf("Failed to do factory reset - %d\n", kcm_status);
    } 
    else {
        printf("Factory reset completed. Now restart the device\n");
    }
}

#if 1		/* Step 2 */
/**
 * Update sensors and report their values.
 * This function is called periodically.
 */
void sensors_update() 
{
    float temp = adc_temp.read()*100;
    float vref = adc_vref.read();
	
    printf("ADC temp:  %6.4f C,  vref: %6.4f %%\r\n", temp, vref);
	
    res_temperature->set_value(temp);
    res_voltage->set_value(vref);
}
#endif

/* */
void data_thread_fun()
{
    uint8_t id;

    printf("thread start\r\n");

    queue.call_every(3000, sensors_update);	/* quene every 3s */

    queue.dispatch();
}

void main_application(void)
{
    mcc_platform_sw_build_info();
    // run_application() will first initialize the program and then call main_application()

    // application_init() runs the following initializations:
    //  1. trace initialization
    //  2. platform initialization
    //  3. print memory statistics if MBED_HEAP_STATS_ENABLED is defined
    //  4. FCC initialization.
    if (!application_init()) {
        printf("Initialization failed, exiting application!\n");
        return;
    }
    else{
        printf("Initialization succeed! \n\r");
        printf("Initialization succeed! \n\r");

	printf(Kolon_VERSION);
	printf("\n\r");		
    }

    // SimpleClient is used for registering and unregistering resources to a server.
    SimpleM2MClient mbedClient;

    // Save pointer to mbedClient so that other functions can access it.
    client = &mbedClient;

#ifdef MBED_HEAP_STATS_ENABLED
    printf("Client initialized\r\n");
    print_heap_stats();
#endif

    //humidity_res = mbedClient.add_cloud_resource(3304, 0, 5700, "humidity_resource", M2MResourceInstance::FLOAT, 
	//			M2MBase::GET_ALLOWED, 0, true, NULL, NULL);
    //motion_res = mbedClient.add_cloud_resource(3300, 0, 5700, "motion_resource", M2MResourceInstance::FLOAT, 
	//			M2MBase::GET_ALLOWED, 0, true, NULL, NULL);

#if 1
    // Create resource for button count. Path of this resource will be: 3303/0/5700.
    res_temperature = mbedClient.add_cloud_resource(3303, 0, 5700, "temperature_resource", M2MResourceInstance::FLOAT, 
				M2MBase::GET_ALLOWED, 0, true, NULL, NULL);

    // Create resource for led blinking pattern. Path of this resource will be: 3316/0/5700.
    res_voltage = mbedClient.add_cloud_resource(3316, 0, 5700, "Voltage", M2MResourceInstance::FLOAT, 
				M2MBase::GET_ALLOWED, 0, true, NULL, NULL);
#endif 

    // Create resource for button count. Path of this resource will be: 3200/0/5501.
    button_res = mbedClient.add_cloud_resource(3200, 0, 5501, "button_resource", M2MResourceInstance::INTEGER,
                              M2MBase::GET_ALLOWED, 0, true, NULL, (void*)button_notification_status_callback);

    // Create resource for led blinking pattern. Path of this resource will be: 3201/0/5853.
    pattern_res = mbedClient.add_cloud_resource(3201, 0, 5853, "pattern_resource", M2MResourceInstance::STRING,
                               M2MBase::GET_PUT_ALLOWED, "500:500:500:500", false, (void*)pattern_updated, NULL);

    // Create resource for starting the led blinking. Path of this resource will be: 3201/0/5850.
    mbedClient.add_cloud_resource(3201, 0, 5850, "blink_resource", M2MResourceInstance::STRING,
                             M2MBase::POST_ALLOWED, "", false, (void*)blink_callback, NULL);

    // Create resource for unregistering the device. Path of this resource will be: 5000/0/1.
    mbedClient.add_cloud_resource(5000, 0, 1, "unregister", M2MResourceInstance::STRING,
                 M2MBase::POST_ALLOWED, NULL, false, (void*)unregister, NULL);

    // Create resource for running factory reset for the device. Path of this resource will be: 5000/0/2.
    mbedClient.add_cloud_resource(5000, 0, 2, "factory_reset", M2MResourceInstance::STRING,
                 M2MBase::POST_ALLOWED, NULL, false, (void*)factory_reset, NULL);

    mbedClient.register_and_connect();

	/* Step 5 : this thead be start */
    data_thread.start(data_thread_fun);

    // Check if client is registering or registered, if true sleep and repeat.
    while (mbedClient.is_register_called()) {
        static int button_count = 0;
        mcc_platform_do_wait(100);
        if (mcc_platform_button_clicked()) {
            button_res->set_value(++button_count);
        }
    }

    // Client unregistered, exit program.
}
