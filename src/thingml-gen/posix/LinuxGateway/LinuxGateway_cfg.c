/*****************************************************
 *      THIS IS A GENERATED FILE. DO NOT EDIT.
 *      Implementation for Application LinuxGateway
 *  Generated from ThingML (http://www.thingml.org)
 *****************************************************/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <pthread.h>
#include "thingml_typedefs.h"
#include "runtime.h"
#include "LinuxGateway.h"
#include "StatsdLogger.h"

#include "Serial.h"
#include "Timer.h"




/*****************************************************************************
 * Definitions for configuration : LinuxGateway
 *****************************************************************************/

//Declaration of instance variables
//Instance gw
// Variables for the properties of the instance
struct LinuxGateway_Instance gw_var;
// Variables for the sessions of the instance
//Instance log
// Variables for the properties of the instance
struct StatsdLogger_Instance log_var;
// Variables for the sessions of the instance

//Instance Timer for network plugin PosixTimerPlugin
struct Timer_Instance Timer_instance;



//New dispatcher for messages
void dispatch_observation(uint16_t sender, uint8_t param_id, int16_t param_current, int16_t param_voltage, int16_t param_brightness) {
if (sender == Serial_instance.listener_id) {
LinuxGateway_handle_sensor_observation(&gw_var, param_id, param_current, param_voltage, param_brightness);

}
if (sender == Serial_instance.listener_id) {
StatsdLogger_handle_statsd_observation(&log_var, param_id, param_current, param_voltage, param_brightness);

}

}


//New dispatcher for messages
void dispatch_timer_timeout(uint16_t sender, uint8_t param_id) {
if (sender == Timer_instance.listener_id) {
LinuxGateway_handle_clock_timer_timeout(&gw_var, param_id);

}

}


int processMessageQueue() {
fifo_lock();
while (fifo_empty()) fifo_wait();
uint8_t mbufi = 0;

// Read the code of the next port/message in the queue
uint16_t code = fifo_dequeue() << 8;

code += fifo_dequeue();

// Switch to call the appropriate handler
switch(code) {
case 20:{
byte mbuf[11 - 2];
while (mbufi < (11 - 2)) mbuf[mbufi++] = fifo_dequeue();
fifo_unlock();
uint8_t mbufi_observation = 2;
union u_observation_id_t {
uint8_t p;
byte bytebuffer[1];
} u_observation_id;
u_observation_id.bytebuffer[0] = mbuf[mbufi_observation + 0];
mbufi_observation += 1;
union u_observation_current_t {
int16_t p;
byte bytebuffer[2];
} u_observation_current;
u_observation_current.bytebuffer[1] = mbuf[mbufi_observation + 0];
u_observation_current.bytebuffer[0] = mbuf[mbufi_observation + 1];
mbufi_observation += 2;
union u_observation_voltage_t {
int16_t p;
byte bytebuffer[2];
} u_observation_voltage;
u_observation_voltage.bytebuffer[1] = mbuf[mbufi_observation + 0];
u_observation_voltage.bytebuffer[0] = mbuf[mbufi_observation + 1];
mbufi_observation += 2;
union u_observation_brightness_t {
int16_t p;
byte bytebuffer[2];
} u_observation_brightness;
u_observation_brightness.bytebuffer[1] = mbuf[mbufi_observation + 0];
u_observation_brightness.bytebuffer[0] = mbuf[mbufi_observation + 1];
mbufi_observation += 2;
dispatch_observation((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_observation_id.p /* id */ ,
 u_observation_current.p /* current */ ,
 u_observation_voltage.p /* voltage */ ,
 u_observation_brightness.p /* brightness */ );
break;
}
case 1:{
byte mbuf[5 - 2];
while (mbufi < (5 - 2)) mbuf[mbufi++] = fifo_dequeue();
fifo_unlock();
uint8_t mbufi_timer_timeout = 2;
union u_timer_timeout_id_t {
uint8_t p;
byte bytebuffer[1];
} u_timer_timeout_id;
u_timer_timeout_id.bytebuffer[0] = mbuf[mbufi_timer_timeout + 0];
mbufi_timer_timeout += 1;
dispatch_timer_timeout((mbuf[0] << 8) + mbuf[1] /* instance port*/,
 u_timer_timeout_id.p /* id */ );
break;
}
}
return 1;
}

void forward_LinuxGateway_send_clock_timer_start(struct LinuxGateway_Instance *_instance, uint8_t id, uint32_t time){
if(_instance->id_clock == gw_var.id_clock) {
forward_Timer_LinuxGateway_send_clock_timer_start(_instance, id, time);
}
}
void forward_LinuxGateway_send_clock_timer_cancel(struct LinuxGateway_Instance *_instance, uint8_t id){
if(_instance->id_clock == gw_var.id_clock) {
forward_Timer_LinuxGateway_send_clock_timer_cancel(_instance, id);
}
}
void forward_LinuxGateway_send_sensor_requestMeasurement(struct LinuxGateway_Instance *_instance, uint8_t id){
if(_instance->id_sensor == gw_var.id_sensor) {
forward_Serial_LinuxGateway_send_sensor_requestMeasurement(_instance, id);
}
}

//external Message enqueue
void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id) {
if ((msgSize >= 2) && (msg != NULL)) {
uint8_t msgSizeOK = 0;
switch(msg[0] * 256 + msg[1]) {
case 20:
if(msgSize == 9) {
msgSizeOK = 1;}
break;
case 1:
if(msgSize == 3) {
msgSizeOK = 1;}
break;
}

if(msgSizeOK == 1) {
fifo_lock();
if ( fifo_byte_available() > (msgSize + 2) ) {
	uint8_t i;
	for (i = 0; i < 2; i++) {
		_fifo_enqueue(msg[i]);
	}
	_fifo_enqueue((listener_id >> 8) & 0xFF);
	_fifo_enqueue(listener_id & 0xFF);
	for (i = 2; i < msgSize; i++) {
		_fifo_enqueue(msg[i]);
	}
}
fifo_unlock_and_notify();
}
}
}

void initialize_configuration_LinuxGateway() {
// Initialize connectors
register_external_LinuxGateway_send_clock_timer_start_listener(&forward_LinuxGateway_send_clock_timer_start);
register_external_LinuxGateway_send_clock_timer_cancel_listener(&forward_LinuxGateway_send_clock_timer_cancel);
register_external_LinuxGateway_send_sensor_requestMeasurement_listener(&forward_LinuxGateway_send_sensor_requestMeasurement);

// Init the ID, state variables and properties for external connector Timer
// Init the ID, state variables and properties for external connector Serial
// Init the ID, state variables and properties for external connector Serial

// Network Initialization

Serial_instance.listener_id = add_instance(&Serial_instance);

//Serial:
Serial_setup();
pthread_t thread_Serial;
pthread_create( &thread_Serial, NULL, Serial_start_receiver_process, NULL);

// Initialise Timer:
Timer_instance.listener_id = add_instance(&Timer_instance);
Timer_setup(&Timer_instance);

pthread_t thread_Timer;
pthread_create( &thread_Timer, NULL, Timer_loop, &Timer_instance);

// End Network Initialization

// Init the ID, state variables and properties for instance gw
gw_var.active = true;
gw_var.id_sensor = add_instance( (void*) &gw_var);
gw_var.id_clock = add_instance( (void*) &gw_var);
gw_var.LinuxGateway_Main_State = LINUXGATEWAY_MAIN_RUNNING_STATE;
gw_var.LinuxGateway_Main_RUNNING_id_var = 0;

LinuxGateway_Main_OnEntry(LINUXGATEWAY_MAIN_STATE, &gw_var);
// Init the ID, state variables and properties for instance log
log_var.active = true;
log_var.id_statsd = add_instance( (void*) &log_var);
log_var.StatsdLogger_SC_State = STATSDLOGGER_SC_READY_STATE;

StatsdLogger_SC_OnEntry(STATSDLOGGER_SC_STATE, &log_var);
}




void term(int signum)
{
    

    fflush(stdout);
    fflush(stderr);
    exit(signum);
}


int main(int argc, char *argv[]) {
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = term;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    init_runtime();
    
    initialize_configuration_LinuxGateway();

    while (1) {
        
// Network Listener// End Network Listener

int emptyEventConsumed = 1;
while (emptyEventConsumed != 0) {
emptyEventConsumed = 0;
}

        processMessageQueue();
  }
}