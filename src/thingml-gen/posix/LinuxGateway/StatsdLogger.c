/*****************************************************
 *      THIS IS A GENERATED FILE. DO NOT EDIT.
 *        Implementation for Thing StatsdLogger
 *  Generated from ThingML (http://www.thingml.org)
 *****************************************************/

#include "StatsdLogger.h"

/*****************************************************************************
 * Implementation for type : StatsdLogger
 *****************************************************************************/


// BEGIN: Code from the c_global annotation StatsdLogger

void diep(char *s) {
  perror(s);
  exit(1);
}
// END: Code from the c_global annotation StatsdLogger

// Declaration of prototypes:
//Prototypes: State Machine
void StatsdLogger_SC_OnExit(int state, struct StatsdLogger_Instance *_instance);
//Prototypes: Message Sending
//Prototypes: Function
void f_StatsdLogger_sendData(struct StatsdLogger_Instance *_instance, uint8_t id, int16_t current, int16_t voltage, int16_t brightness);
// Declaration of functions:
// Definition of function sendData
void f_StatsdLogger_sendData(struct StatsdLogger_Instance *_instance, uint8_t id, int16_t current, int16_t voltage, int16_t brightness) {
char buf[BUFLEN];
	
	    struct sockaddr_in si_other;
	    int s, i, slen=sizeof(si_other);
	
	    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) diep("socket");
	    memset((char *) &si_other, 0, sizeof(si_other));
		si_other.sin_family = AF_INET;
		si_other.sin_port = htons(PORT);
	
	    if (inet_aton("192.168.1.15", &si_other.sin_addr)==0) {
		  fprintf(stderr, "inet_aton() failed\n");
		  exit(1);
		}
	
	    sprintf(buf, "ledtest.led%d.voltage:%.2f|g\nledtest.led%d.current:%.1f|g\nledtest.led%d.brightness:%d|g",
	                 id,        voltage/1000.0f,   id,        current/10.0f,     id,           brightness);
	       
	    if (sendto(s, buf, strlen(buf), 0, (struct sockaddr *)&si_other, slen)==-1) {
		    diep("sendto()");
		}
	
	    close(s);
	
	    //printf("SENT: \n%s\n\n", buf);
}

// Sessions functionss:


// On Entry Actions:
void StatsdLogger_SC_OnEntry(int state, struct StatsdLogger_Instance *_instance) {
switch(state) {
case STATSDLOGGER_SC_STATE:{
_instance->StatsdLogger_SC_State = STATSDLOGGER_SC_READY_STATE;
StatsdLogger_SC_OnEntry(_instance->StatsdLogger_SC_State, _instance);
break;
}
case STATSDLOGGER_SC_READY_STATE:{
break;
}
default: break;
}
}

// On Exit Actions:
void StatsdLogger_SC_OnExit(int state, struct StatsdLogger_Instance *_instance) {
switch(state) {
case STATSDLOGGER_SC_STATE:{
StatsdLogger_SC_OnExit(_instance->StatsdLogger_SC_State, _instance);
break;}
case STATSDLOGGER_SC_READY_STATE:{
break;}
default: break;
}
}

// Event Handlers for incoming messages:
void StatsdLogger_handle_statsd_observation(struct StatsdLogger_Instance *_instance, uint8_t id, int16_t current, int16_t voltage, int16_t brightness) {
if(!(_instance->active)) return;
//Region SC
uint8_t StatsdLogger_SC_State_event_consumed = 0;
if (_instance->StatsdLogger_SC_State == STATSDLOGGER_SC_READY_STATE) {
if (StatsdLogger_SC_State_event_consumed == 0 && 1) {
f_StatsdLogger_sendData(_instance, id, current, voltage, brightness);
StatsdLogger_SC_State_event_consumed = 1;
}
}
//End Region SC
//End dsregion SC
//Session list: 
}

// Observers for outgoing messages:



