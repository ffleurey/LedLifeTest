#ifndef Serial_lib_h

#define Serial_lib_h

#include "Serial.c"

struct Serial_instance_type Serial_instance;

void Serial_set_listener_id(uint16_t id);
int Serial_setup();
void Serial_forwardMessage(byte * msg, uint8_t size);
void Serial_start_receiver_process();

#endif

// Forwarding of messages Serial::LinuxGateway::sensor::requestMeasurement
void forward_Serial_LinuxGateway_send_sensor_requestMeasurement(struct LinuxGateway_Instance *_instance, uint8_t id);
