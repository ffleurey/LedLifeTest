/*****************************************************/
//                    POSIX Timer
/*****************************************************/
#include <stdint.h>

#include "LinuxGateway.h"
#include "StatsdLogger.h"


#define NB_SOFT_TIMERS 4

struct Timer_Instance {
    uint16_t listener_id;
    /*INSTANCE_INFORMATION*/
};

void Timer_setup(struct Timer_Instance *_instance);
void Timer_loop(struct Timer_Instance *_instance);

void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id);

// Forwarding of messages Timer::LinuxGateway::clock::timer_cancel
void forward_Timer_LinuxGateway_send_clock_timer_cancel(struct LinuxGateway_Instance *_instance, uint8_t id);
// Forwarding of messages Timer::LinuxGateway::clock::timer_start
void forward_Timer_LinuxGateway_send_clock_timer_start(struct LinuxGateway_Instance *_instance, uint8_t id, uint32_t time);

