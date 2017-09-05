/*****************************************************
 *      THIS IS A GENERATED FILE. DO NOT EDIT.
 *           Header for Thing LinuxGateway
 *  Generated from ThingML (http://www.thingml.org)
 *****************************************************/

#ifndef LinuxGateway_H_
#define LinuxGateway_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "thingml_typedefs.h"

/*****************************************************************************
 * Headers for type : LinuxGateway
 *****************************************************************************/


// BEGIN: Code from the c_header annotation LinuxGateway
#include <stdlib.h>
// END: Code from the c_header annotation LinuxGateway

// Definition of the instance struct:
struct LinuxGateway_Instance {

// Instances of different sessions
bool active;
// Variables for the ID of the ports of the instance
uint16_t id_sensor;
uint16_t id_clock;
// Variables for the current instance state
int LinuxGateway_Main_State;
// Variables for the properties of the instance
int8_t LinuxGateway_Main_RUNNING_id_var;

};
// Declaration of prototypes outgoing messages :
void LinuxGateway_Main_OnEntry(int state, struct LinuxGateway_Instance *_instance);
void LinuxGateway_handle_clock_timer_timeout(struct LinuxGateway_Instance *_instance, uint8_t id);
void LinuxGateway_handle_sensor_observation(struct LinuxGateway_Instance *_instance, uint8_t id, int16_t current, int16_t voltage, int16_t brightness);
// Declaration of callbacks for incoming messages:
void register_LinuxGateway_send_sensor_requestMeasurement_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t));
void register_external_LinuxGateway_send_sensor_requestMeasurement_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t));
void register_LinuxGateway_send_clock_timer_start_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t, uint32_t));
void register_external_LinuxGateway_send_clock_timer_start_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t, uint32_t));
void register_LinuxGateway_send_clock_timer_cancel_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t));
void register_external_LinuxGateway_send_clock_timer_cancel_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t));

// Definition of the states:
#define LINUXGATEWAY_MAIN_STATE 0
#define LINUXGATEWAY_MAIN_RUNNING_STATE 1



#ifdef __cplusplus
}
#endif

#endif //LinuxGateway_H_
