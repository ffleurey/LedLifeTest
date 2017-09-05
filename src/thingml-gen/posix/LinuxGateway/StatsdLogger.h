/*****************************************************
 *      THIS IS A GENERATED FILE. DO NOT EDIT.
 *           Header for Thing StatsdLogger
 *  Generated from ThingML (http://www.thingml.org)
 *****************************************************/

#ifndef StatsdLogger_H_
#define StatsdLogger_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "thingml_typedefs.h"

/*****************************************************************************
 * Headers for type : StatsdLogger
 *****************************************************************************/


// BEGIN: Code from the c_header annotation StatsdLogger

#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#define BUFLEN 512
#define NPACK 10
#define PORT 8125

// END: Code from the c_header annotation StatsdLogger

// Definition of the instance struct:
struct StatsdLogger_Instance {

// Instances of different sessions
bool active;
// Variables for the ID of the ports of the instance
uint16_t id_statsd;
// Variables for the current instance state
int StatsdLogger_SC_State;
// Variables for the properties of the instance

};
// Declaration of prototypes outgoing messages :
void StatsdLogger_SC_OnEntry(int state, struct StatsdLogger_Instance *_instance);
void StatsdLogger_handle_statsd_observation(struct StatsdLogger_Instance *_instance, uint8_t id, int16_t current, int16_t voltage, int16_t brightness);
// Declaration of callbacks for incoming messages:

// Definition of the states:
#define STATSDLOGGER_SC_STATE 0
#define STATSDLOGGER_SC_READY_STATE 1



#ifdef __cplusplus
}
#endif

#endif //StatsdLogger_H_
