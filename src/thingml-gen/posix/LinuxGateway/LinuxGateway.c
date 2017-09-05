/*****************************************************
 *      THIS IS A GENERATED FILE. DO NOT EDIT.
 *        Implementation for Thing LinuxGateway
 *  Generated from ThingML (http://www.thingml.org)
 *****************************************************/

#include "LinuxGateway.h"

/*****************************************************************************
 * Implementation for type : LinuxGateway
 *****************************************************************************/

// Declaration of prototypes:
//Prototypes: State Machine
void LinuxGateway_Main_OnExit(int state, struct LinuxGateway_Instance *_instance);
//Prototypes: Message Sending
void LinuxGateway_send_sensor_requestMeasurement(struct LinuxGateway_Instance *_instance, uint8_t id);
void LinuxGateway_send_clock_timer_start(struct LinuxGateway_Instance *_instance, uint8_t id, uint32_t time);
void LinuxGateway_send_clock_timer_cancel(struct LinuxGateway_Instance *_instance, uint8_t id);
//Prototypes: Function
// Declaration of functions:

// Sessions functionss:


// On Entry Actions:
void LinuxGateway_Main_OnEntry(int state, struct LinuxGateway_Instance *_instance) {
switch(state) {
case LINUXGATEWAY_MAIN_STATE:{
_instance->LinuxGateway_Main_State = LINUXGATEWAY_MAIN_RUNNING_STATE;
fprintf(stdout, "Gateway has started\n");
LinuxGateway_Main_OnEntry(_instance->LinuxGateway_Main_State, _instance);
break;
}
case LINUXGATEWAY_MAIN_RUNNING_STATE:{
LinuxGateway_send_clock_timer_start(_instance, 0, 2000);
break;
}
default: break;
}
}

// On Exit Actions:
void LinuxGateway_Main_OnExit(int state, struct LinuxGateway_Instance *_instance) {
switch(state) {
case LINUXGATEWAY_MAIN_STATE:{
LinuxGateway_Main_OnExit(_instance->LinuxGateway_Main_State, _instance);
break;}
case LINUXGATEWAY_MAIN_RUNNING_STATE:{
break;}
default: break;
}
}

// Event Handlers for incoming messages:
void LinuxGateway_handle_clock_timer_timeout(struct LinuxGateway_Instance *_instance, uint8_t id) {
if(!(_instance->active)) return;
//Region Main
uint8_t LinuxGateway_Main_State_event_consumed = 0;
if (_instance->LinuxGateway_Main_State == LINUXGATEWAY_MAIN_RUNNING_STATE) {
if (LinuxGateway_Main_State_event_consumed == 0 && id == 0) {
LinuxGateway_Main_OnExit(LINUXGATEWAY_MAIN_RUNNING_STATE, _instance);
_instance->LinuxGateway_Main_State = LINUXGATEWAY_MAIN_RUNNING_STATE;
LinuxGateway_send_sensor_requestMeasurement(_instance, _instance->LinuxGateway_Main_RUNNING_id_var);
_instance->LinuxGateway_Main_RUNNING_id_var = _instance->LinuxGateway_Main_RUNNING_id_var + 1;
if(_instance->LinuxGateway_Main_RUNNING_id_var > 5) {
_instance->LinuxGateway_Main_RUNNING_id_var = 0;

}
LinuxGateway_Main_OnEntry(LINUXGATEWAY_MAIN_RUNNING_STATE, _instance);
LinuxGateway_Main_State_event_consumed = 1;
}
}
//End Region Main
//End dsregion Main
//Session list: 
}
void LinuxGateway_handle_sensor_observation(struct LinuxGateway_Instance *_instance, uint8_t id, int16_t current, int16_t voltage, int16_t brightness) {
if(!(_instance->active)) return;
//Region Main
uint8_t LinuxGateway_Main_State_event_consumed = 0;
if (_instance->LinuxGateway_Main_State == LINUXGATEWAY_MAIN_RUNNING_STATE) {
if (LinuxGateway_Main_State_event_consumed == 0 && 1) {
LinuxGateway_Main_State_event_consumed = 1;
}
}
//End Region Main
//End dsregion Main
//Session list: 
}

// Observers for outgoing messages:
void (*external_LinuxGateway_send_sensor_requestMeasurement_listener)(struct LinuxGateway_Instance *, uint8_t)= 0x0;
void (*LinuxGateway_send_sensor_requestMeasurement_listener)(struct LinuxGateway_Instance *, uint8_t)= 0x0;
void register_external_LinuxGateway_send_sensor_requestMeasurement_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t)){
external_LinuxGateway_send_sensor_requestMeasurement_listener = _listener;
}
void register_LinuxGateway_send_sensor_requestMeasurement_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t)){
LinuxGateway_send_sensor_requestMeasurement_listener = _listener;
}
void LinuxGateway_send_sensor_requestMeasurement(struct LinuxGateway_Instance *_instance, uint8_t id){
if (LinuxGateway_send_sensor_requestMeasurement_listener != 0x0) LinuxGateway_send_sensor_requestMeasurement_listener(_instance, id);
if (external_LinuxGateway_send_sensor_requestMeasurement_listener != 0x0) external_LinuxGateway_send_sensor_requestMeasurement_listener(_instance, id);
;
}
void (*external_LinuxGateway_send_clock_timer_start_listener)(struct LinuxGateway_Instance *, uint8_t, uint32_t)= 0x0;
void (*LinuxGateway_send_clock_timer_start_listener)(struct LinuxGateway_Instance *, uint8_t, uint32_t)= 0x0;
void register_external_LinuxGateway_send_clock_timer_start_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t, uint32_t)){
external_LinuxGateway_send_clock_timer_start_listener = _listener;
}
void register_LinuxGateway_send_clock_timer_start_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t, uint32_t)){
LinuxGateway_send_clock_timer_start_listener = _listener;
}
void LinuxGateway_send_clock_timer_start(struct LinuxGateway_Instance *_instance, uint8_t id, uint32_t time){
if (LinuxGateway_send_clock_timer_start_listener != 0x0) LinuxGateway_send_clock_timer_start_listener(_instance, id, time);
if (external_LinuxGateway_send_clock_timer_start_listener != 0x0) external_LinuxGateway_send_clock_timer_start_listener(_instance, id, time);
;
}
void (*external_LinuxGateway_send_clock_timer_cancel_listener)(struct LinuxGateway_Instance *, uint8_t)= 0x0;
void (*LinuxGateway_send_clock_timer_cancel_listener)(struct LinuxGateway_Instance *, uint8_t)= 0x0;
void register_external_LinuxGateway_send_clock_timer_cancel_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t)){
external_LinuxGateway_send_clock_timer_cancel_listener = _listener;
}
void register_LinuxGateway_send_clock_timer_cancel_listener(void (*_listener)(struct LinuxGateway_Instance *, uint8_t)){
LinuxGateway_send_clock_timer_cancel_listener = _listener;
}
void LinuxGateway_send_clock_timer_cancel(struct LinuxGateway_Instance *_instance, uint8_t id){
if (LinuxGateway_send_clock_timer_cancel_listener != 0x0) LinuxGateway_send_clock_timer_cancel_listener(_instance, id);
if (external_LinuxGateway_send_clock_timer_cancel_listener != 0x0) external_LinuxGateway_send_clock_timer_cancel_listener(_instance, id);
;
}



