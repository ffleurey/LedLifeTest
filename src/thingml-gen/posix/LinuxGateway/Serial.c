#include <stdlib.h>
#include <string.h> // string function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h> // time calls

//#define Serial_TIMEOUT 10 // timeout waiting for messages from the serial device
#define Serial_INPUT_BUFFER_SIZE 90 // for possible future optimizations
#define Serial_MAX_MSG_LENGTH 9
#define Serial_START_BYTE 18
#define Serial_STOP_BYTE 19
#define Serial_ESCAPE_BYTE 125

#define Serial_LISTENER_STATE_IDLE 0
#define Serial_LISTENER_STATE_READING 1
#define Serial_LISTENER_STATE_ESCAPE 2
#define Serial_LISTENER_STATE_ERROR 3
int Serial_device_id;


struct Serial_instance_type {
    uint16_t listener_id;
    /*INSTANCE_INFORMATION*/
};

extern struct Serial_instance_type Serial_instance;

void Serial_set_listener_id(uint16_t id) {
	Serial_instance.listener_id = id;
}

void Serial_parser(byte * msg, uint16_t size) {
externalMessageEnqueue((uint8_t *) msg, size, Serial_instance.listener_id);
}


int Serial_setup() {
        char * device = "/dev/ttyLedTest";
        uint32_t baudrate = 115200;
        printf("[Serial] Opening Serial device %s at %ibps...\n", device, baudrate);
	int result;
	struct termios port_settings;
	result = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (result < 0) {
        printf("[Serial] Error opening Serial port\n");
		perror("Error opening Serial port");
        exit(1); // Exit in case of error
	}
	else if (tcgetattr(result, &port_settings) < 0) {// try to get current options
        printf("[Serial] Error opening Serial port: could not get serial port attributes\n");
		perror("Error opening Serial port: could not get serial port attributes");
        exit(1); // Exit in case of error
	}
	else {
		//printf("Configuring port %s...\n", device);
		switch(baudrate) {
			case 115200:
				cfsetispeed(&port_settings, B115200);    // set baud rates to 115200 ---------- Test with 57600
				cfsetospeed(&port_settings, B115200);
			break;
			
			case 57600:
				cfsetispeed(&port_settings, B57600);    // set baud rates to 115200 ---------- Test with 57600
				cfsetospeed(&port_settings, B57600);
			break;
			
			case 38400:
				cfsetispeed(&port_settings, B38400);    // set baud rates to 38400 ---------- Test with 57600
				cfsetospeed(&port_settings, B38400);
			break;
			
			case 19200:
				cfsetispeed(&port_settings, B19200);    // set baud rates to 19200 ---------- Test with 57600
				cfsetospeed(&port_settings, B19200);
			break;
			
			case 9600:
				cfsetispeed(&port_settings, B9600);    // set baud rates to 115200 ---------- Test with 57600
				cfsetospeed(&port_settings, B9600);
			break;
			
			default:
				cfsetispeed(&port_settings, B115200);    // set baud rates to 115200 ---------- Test with 57600
				cfsetospeed(&port_settings, B115200);
			break;
		}
		// 8N1
		port_settings.c_cflag &= ~PARENB;
		port_settings.c_cflag &= ~CSTOPB;
		port_settings.c_cflag &= ~CSIZE;
		port_settings.c_cflag |= CS8;
		// no flow control
		port_settings.c_cflag &= ~CRTSCTS;
		port_settings.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
		port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
		port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
		port_settings.c_oflag &= ~OPOST; // make raw
		// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
		port_settings.c_cc[VMIN] = 0;
		port_settings.c_cc[VTIME] = 20;
		if (tcsetattr(result, TCSANOW, &port_settings) < 0 ) {
                    printf("[Serial] Error opening Serial port: could not set serial port attributes\n");
                    perror("Error opening Serial port: could not set serial port attributes");
                    exit(1); // Exit in case of error
		}
		sleep(1); // wait a bit
	}

	Serial_device_id = result;
        //printf("[Serial] Serial Port %s open\n", device);
}
	
int Serial_send_byte(int device, uint8_t byte) {
	int n;
	unsigned char data[1];
	data[0] = byte;
	n = write(device, data, 1);
	
        //printf("[Serial] forwarding %i with result %i\n", data[0], n);
	if (n < 0) {
            perror("Error writing to Serial device");
            exit(1); // Exit in case of error
            return -1;
	}
	return 0;
}

void Serial_forwardMessage(byte * msg, uint8_t size) {
        //printf("[Serial] forwarding message\n");
	Serial_send_byte(Serial_device_id, Serial_START_BYTE);
	uint8_t i;
	for(i = 0; i < size; i++) {
		if((msg[i] == Serial_START_BYTE) || (msg[i] == Serial_STOP_BYTE) || (msg[i] == Serial_ESCAPE_BYTE)) {
	  		Serial_send_byte(Serial_device_id, Serial_ESCAPE_BYTE);
		}
		Serial_send_byte(Serial_device_id, msg[i]);
	}
	Serial_send_byte(Serial_device_id, Serial_STOP_BYTE);
}
	
void Serial_start_receiver_process()
{
        printf("[Serial] Starting receiver process\n");
	int device = Serial_device_id;
	int serialListenerState = Serial_LISTENER_STATE_IDLE;
	char serialBuffer[Serial_MAX_MSG_LENGTH];
	int serialMsgSize = 0;
	char buffer[Serial_INPUT_BUFFER_SIZE]; // Data read from the ESUSMS device
	int n; // used to store the results of select and read
	int i; // loop index
	while (1) {
		fd_set rdfs; // The file descriptor to wait on
		FD_ZERO( &rdfs );
		FD_SET(device, &rdfs ); // set to the esusms fd
		n = select(device + 1, &rdfs, NULL, NULL, NULL); // NO Timeout here (last parameter)
		if (n < 0) {
                    printf("[Serial] Error waiting for incoming data from Serial device\n");
                    perror("Error waiting for incoming data from Serial device");
                    exit(1); // Exit in case of error
                    break;
		}
		else if (n == 0) { // timeout
                    printf("[Serial] Timeout waiting for incoming data from Serial device\n");
                    break;
		}
		else { // there is something to read
			n = read(device, &buffer, Serial_INPUT_BUFFER_SIZE * sizeof(char));
			//printf(" n=<%i>\n", n);
			if (n<0) {
                            printf("[Serial] Error reading from Serial device\n");
                            perror("Error reading from Serial device");
                            exit(1); // Exit in case of error
                            break;
			}
			else if (n==0) {
                            printf("[Serial] Nothing to read from Serial device\n");
                            break;
			}
			else { // There are n incoming bytes in buffer
				for (i = 0; i<n; i++) {
					
                                        //printf("[Serial] byte received:%i\n", buffer[i]);
					switch(serialListenerState) {
						case Serial_LISTENER_STATE_IDLE:
                                                        //printf("[Serial] State:IDLE\n");
							if(buffer[i] == Serial_START_BYTE) {
							  serialListenerState = Serial_LISTENER_STATE_READING;
							  serialMsgSize = 0;
							}
						break;

						case Serial_LISTENER_STATE_READING:
                                                        //printf("[Serial] State:READING\n");
							if (serialMsgSize > Serial_MAX_MSG_LENGTH) {
							  serialListenerState = Serial_LISTENER_STATE_ERROR;
							} else {
							  if(buffer[i] == Serial_STOP_BYTE) {
								serialListenerState = Serial_LISTENER_STATE_IDLE;

                                                                //printf("[Serial] Message received\n");
                                                                Serial_parser(serialBuffer, serialMsgSize);

							  } else if (buffer[i] == Serial_ESCAPE_BYTE) {
								serialListenerState = Serial_LISTENER_STATE_ESCAPE;
							  } else {
								serialBuffer[serialMsgSize] = buffer[i];
								serialMsgSize++;
							  }
							}
						break;

						case Serial_LISTENER_STATE_ESCAPE:
                                                        //printf("[Serial] State:ESCAPE\n");
							if (serialMsgSize > Serial_MAX_MSG_LENGTH) {
							  serialListenerState = Serial_LISTENER_STATE_ERROR;
							} else {
							  serialBuffer[serialMsgSize] = buffer[i];
							  serialMsgSize++;
							  serialListenerState = Serial_LISTENER_STATE_READING;
							}
						break;

						case Serial_LISTENER_STATE_ERROR:
                                                        printf("[Serial] Message error: Too long\n");
							serialListenerState = Serial_LISTENER_STATE_IDLE;
							serialMsgSize = 0;
						break;
					}
				}
			}
		}
	}
}

// Forwarding of messages Serial::LinuxGateway::sensor::requestMeasurement
void forward_Serial_LinuxGateway_send_sensor_requestMeasurement(struct LinuxGateway_Instance *_instance, uint8_t id){
byte forward_buf[3];
forward_buf[0] = (10 >> 8) & 0xFF;
forward_buf[1] =  10 & 0xFF;


// parameter id
union u_id_t {
uint8_t p;
byte bytebuffer[1];
} u_id;
u_id.p = id;
forward_buf[2] =  (u_id.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 3);
}

