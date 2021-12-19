/*
 * roboclaw library header
 * 
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */

#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> //uint8_t, int16_t, int32_t
#include <unistd.h> //read, close
#include <fcntl.h> //O_RDWR file open flag
#include <termios.h> //struct termios, tcgetattr, tcsetattr, cfsetispeed, tcflush
#include <assert.h> //assert
#include <endian.h> //htobe16, htobe32, be16toh
#include <string.h> //memcpy
#include <sys/select.h> //select
#include <malloc.h> //malloc, free
#include <errno.h> //errno

//default library values
enum {ROBOCLAW_DEFAULT_RETRIES=3, ROBOCLAW_DEFAULT_STRICT_0XFF_ACK=0, ROBOCLAW_B2400_TIMEOUT_MS=100,
 ROBOCLAW_B9600_TIMEOUT_MS=30,ROBOCLAW_B19200_TIMEOUT_MS=20, ROBOCLAW_B38400_TIMEOUT_MS=15,
 ROBOCLAW_B57600_TIMEOUT_MS=13, ROBOCLAW_B115200_ABOVE_TIMEOUT_MS=12};

//ACK bytes, crc sizes, reply sizes
enum { ROBOCLAW_ACK_BYTE=0xff, ROBOCLAW_ACK_BYTES=1, ROBOCLAW_CRC16_BYTES=2,
 ROBOCLAW_READ_MAIN_BATTERY_REPLY_BYTES=4, ROBOCLAW_READ_ENCODERS_REPLY_BYTES=10};

enum {ROBOCLAW_BUFFER_SIZE=128}; //has to be big enough to accomodate both command and its reply

//internal library data
struct roboclaw
{
	int fd;
	int timeout_ms;
	int retries;
	int strict_0xFF_ACK;
	struct termios initial_termios;
	struct termios actual_termios;
	uint8_t buffer[ROBOCLAW_BUFFER_SIZE];
};

//error codes returned by the functions
enum {ROBOCLAW_ERROR=-1, ROBOCLAW_RETRIES_EXCEEDED=-2, ROBOCLAW_OK=0};

// Roboclaw commands
enum {		M1FORWARD = 0,
			M1BACKWARD = 1,
			SETMINMB = 2,
			SETMAXMB = 3,
			M2FORWARD = 4,
			M2BACKWARD = 5,
			M17BIT = 6,
			M27BIT = 7,
			MIXEDFORWARD = 8,
			MIXEDBACKWARD = 9,
			MIXEDRIGHT = 10,
			MIXEDLEFT = 11,
			MIXEDFB = 12,
			MIXEDLR = 13,
			GETM1ENC = 16,
			GETM2ENC = 17,
			GETM1SPEED = 18,
			GETM2SPEED = 19,
			RESETENC = 20,
			GETVERSION = 21,
			SETM1ENCCOUNT = 22,
			SETM2ENCCOUNT = 23,
			GETMBATT = 24,
			GETLBATT = 25,
			SETMINLB = 26,
			SETMAXLB = 27,
			SETM1PID = 28,
			SETM2PID = 29,
			GETM1ISPEED = 30,
			GETM2ISPEED = 31,
			M1DUTY = 32,
			M2DUTY = 33,
			MIXEDDUTY = 34,
			M1SPEED = 35,
			M2SPEED = 36,
			MIXEDSPEED = 37,
			M1SPEEDACCEL = 38,
			M2SPEEDACCEL = 39,
			MIXEDSPEEDACCEL = 40,
			M1SPEEDDIST = 41,
			M2SPEEDDIST = 42,
			MIXEDSPEEDDIST = 43,
			M1SPEEDACCELDIST = 44,
			M2SPEEDACCELDIST = 45,
			MIXEDSPEEDACCELDIST = 46,
			GETBUFFERS = 47,
			GETPWMS = 48,
			GETCURRENTS = 49,
			MIXEDSPEED2ACCEL = 50,
			MIXEDSPEED2ACCELDIST = 51,
			M1DUTYACCEL = 52,
			M2DUTYACCEL = 53,
			MIXEDDUTYACCEL = 54,
			READM1PID = 55,
			READM2PID = 56,
			SETMAINVOLTAGES = 57,
			SETLOGICVOLTAGES = 58,
			GETMINMAXMAINVOLTAGES = 59,
			GETMINMAXLOGICVOLTAGES = 60,
			SETM1POSPID = 61,
			SETM2POSPID = 62,
			READM1POSPID = 63,
			READM2POSPID = 64,
			M1SPEEDACCELDECCELPOS = 65,
			M2SPEEDACCELDECCELPOS = 66,
			MIXEDSPEEDACCELDECCELPOS = 67,
			SETM1DEFAULTACCEL = 68,
			SETM2DEFAULTACCEL = 69,
			SETPINFUNCTIONS = 74,
			GETPINFUNCTIONS = 75,
			SETDEADBAND	= 76,
			GETDEADBAND	= 77,
			GETENCODERS = 78,
			GETISPEEDS = 79,
			RESTOREDEFAULTS = 80,
			GETTEMP = 82,
			GETTEMP2 = 83,	//Only valid on some models
			GETERROR = 90,
			GETENCODERMODE = 91,
			SETM1ENCODERMODE = 92,
			SETM2ENCODERMODE = 93,
			WRITENVM = 94,
			READNVM = 95,	//Reloads values from Flash into Ram
			SETCONFIG = 98,
			GETCONFIG = 99,
			SETM1MAXCURRENT = 133,
			SETM2MAXCURRENT = 134,
			GETM1MAXCURRENT = 135,
			GETM2MAXCURRENT = 136,
			SETPWMMODE = 148,
			GETPWMMODE = 149,
			FLAGBOOTLOADER = 255};	//Only available via USB communications

/* Initialize roboclaw communication
 * 
 * NULL is returned most likely on incorrect tty device or permission error (are you in dialout group?)
 * 
 * parameters:
 * - tty - the device (e.g. /dev/ttyAMA0, /dev/ttyUSB0, /dev/ttyO1)
 * - baudrate - has to match the one set on roboclaw (2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800)
 * - retries - the number of retries the library should make before reporting failure (both timeout and incorrect crc)
 * - timeout_ms - timeout in ms for the roundtrip sending command and waiting for ACK or response
 * - strict_0xFF_ACK - require strict 0xFF ACK byte matching (treat non 0xFF as crc failure
 * 
 * returns:
 * NULL on failure, and pointer to library data on success, on failure you can query errno for details about error
 *  
*/
struct roboclaw *roboclaw_init_ext(const char* tty, int baudrate, int timeout_ms, int retries, int strict_0xFF_ACK)
{		
	struct roboclaw *rc;
	speed_t speed=0;

	if(baudrate == 2400) speed=B2400;
	else if(baudrate == 9600) speed=B9600;
	else if(baudrate == 19200) speed=B19200;
	else if(baudrate == 38400) speed=B38400;
	else if(baudrate == 57600) speed=B57600;
	else if(baudrate == 115200) speed=B115200;
	else if(baudrate == 230400) speed=B230400;

//B460800 is non-standard but is defined on modern systems
#ifdef B460800
	if(baudrate == 460800) speed=B460800;
#endif

	if(speed == 0)
	{
		errno=EINVAL;
		return NULL;
	}

	rc=(struct roboclaw *)malloc(sizeof(struct roboclaw));

	if( rc == NULL )
		return NULL;

	rc->timeout_ms=timeout_ms;
	rc->retries=retries;
	rc->strict_0xFF_ACK=strict_0xFF_ACK;
	
	if ( (rc->fd=open(tty, O_RDWR)) ==-1 )
	{
		free(rc);
		return NULL;
	}
	
	if(tcgetattr(rc->fd, &rc->initial_termios) < 0)
	{
		close(rc->fd);
		free(rc);
		return NULL;
	}	
	
	rc->actual_termios.c_iflag=rc->actual_termios.c_oflag=rc->actual_termios.c_lflag=0;
	rc->actual_termios.c_cflag=CS8|CREAD|CLOCAL; //8 bit characters	
	rc->actual_termios.c_cc[VMIN]=1;
	rc->actual_termios.c_cc[VTIME]=0;

	if(cfsetispeed(&rc->actual_termios, speed) < 0 || cfsetospeed(&rc->actual_termios, speed) < 0)
	{
		close(rc->fd);
		free(rc);
		return NULL;
	}	

	if(tcsetattr(rc->fd, TCSAFLUSH, &rc->actual_termios) < 0)
	{
		close(rc->fd);
		free(rc);
		return NULL;
	}
	
	// from man (TO DO)
	// Note that tcsetattr() returns success if any of the  requested  changes
    // could  be  successfully  carried  out.  Therefore, when making multiple
    // changes it may be necessary to follow this call with a further call  to
    // tcgetattr() to check that all changes have been performed successfully.
	//
    // this is still edge case to consider, some settings may have not been made
	// solution tcgetattr and check settings we made for equality
	
	return rc;
}

/* Initialize roboclaw communication with default values
 * 
 * NULL is returned most likely on incorrect tty device or permission error (are you in dialout group?)
 * 
 * parameters:
 * - tty - the device (e.g. /dev/ttyAMA0, /dev/ttyUSB0, /dev/ttyO1)
 * - baudrate - has to match the one set on roboclaw (2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800)
 * 
 * returns:
 * NULL on failure, and pointer to library data on success, on failure you can query errno for details about error
 *  
*/
struct roboclaw *roboclaw_init(const char* tty, int baudrate)
{
	int timeout_ms;
	if(baudrate == 2400) timeout_ms=ROBOCLAW_B2400_TIMEOUT_MS;
	else if(baudrate == 9600) timeout_ms=ROBOCLAW_B9600_TIMEOUT_MS;
	else if(baudrate == 19200) timeout_ms=ROBOCLAW_B19200_TIMEOUT_MS;
	else if(baudrate == 38400) timeout_ms=ROBOCLAW_B38400_TIMEOUT_MS;
	else if(baudrate == 57600) timeout_ms=ROBOCLAW_B57600_TIMEOUT_MS;
	else timeout_ms=ROBOCLAW_B115200_ABOVE_TIMEOUT_MS;
	
	return roboclaw_init_ext(tty, baudrate, timeout_ms, ROBOCLAW_DEFAULT_RETRIES, ROBOCLAW_DEFAULT_STRICT_0XFF_ACK);
}



/*
 * Cleans after library 
 *
 * Closes file descriptor, resets terminal to initial settings and frees the memory.
 * Calling this function with NULL argument is legal and will result in ROBOCLAW_OK
 * 
 * parameters:
 * rc - pointer to library internal data
 * 
 * returns:
 * - ROBOCLAW_OK on success, ROBOCLAW_FAILURE on failure and errno is set
 */
int roboclaw_close(struct roboclaw *rc)
{
	int error=0;

	if(rc == NULL)
		return ROBOCLAW_OK;

	// Note that tcsetattr() returns success if any of the  requested  changes
	// could  be  successfully  carried  out.  Therefore, when making multiple
	// changes it may be necessary to follow this call with a further call  to
	// tcgetattr() to check that all changes have been performed successfully.
	error |= tcsetattr(rc->fd, TCSANOW, &rc->initial_termios) < 0;
	error |= close(rc->fd) < 0;

	free(rc);

	if(error)
		return ROBOCLAW_ERROR;
	
	return ROBOCLAW_OK;
}

//
// lowest level interface functions
//
static int encode_uint16(uint8_t *buffer, int bytes, uint16_t value)
{
	value=htobe16(value);
	memcpy(buffer + bytes, &value, sizeof(value));
	return sizeof(value);	
}

static int encode_uint32(uint8_t *buffer, int bytes, uint32_t value)
{
	value=htobe32(value);
	memcpy(buffer + bytes, &value, sizeof(value));
	return sizeof(value);	
}

static uint16_t update_crc16(uint8_t *packet, const int bytes, uint16_t crc)
{
	int byte;
	uint8_t bit;
	
	for (byte = 0; byte < bytes; byte++)
	{
		crc = crc ^ ((unsigned int)packet[byte] << 8);
		for (bit = 0; bit < 8; ++bit)		
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;		
	}
	return crc;	
}

static uint16_t calculate_crc16(uint8_t *packet,const int bytes)
{
	return update_crc16(packet, bytes, 0);
}

static int encode_checksum(uint8_t *buffer, int bytes)
{
	uint16_t checksum=calculate_crc16(buffer, bytes);
	return encode_uint16(buffer, bytes, checksum);	
}

static int write_all(int fd, uint8_t *data, int data_size)
{
	int result;
	int written=0;
	
	while(written<data_size)
	{
		if ((result = write(fd, data+written, data_size-written)) == -1 )
			return -1;
		written += result;		
	}
	return 0;
}

// -1 on error (can be signal like EINTR), 0 on timetout, 1 on  input
// Tell through termios that we want to be woken up only if bytes_read bytes are waiting
// Timewout if ROBOCLAW_PACKET_TIMEOUT_MS is exceeded
static int wait_input(int fd, struct termios *io, int bytes_read,int timeout_ms)
{
	struct timeval tv;
	fd_set rfds;
	int ret;

	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);
	
	tv.tv_sec=0;
	tv.tv_usec = timeout_ms*1000;  

	//tell the system to only wake us up if enough data is waiting
	//
	if(io->c_cc[VMIN] != bytes_read)
	{
		io->c_cc[VMIN]=bytes_read;
		if(tcsetattr(fd, TCSANOW, io) < 0)
			return -1; //different error?
	}

	if( (ret = select(fd+1, &rfds, NULL, NULL, &tv)) < 0 )
		return ret; //EINTR or some other error, errno set

	if(ret == 0)
		return 0;

	return 1;
}

static int flush_io(int fd)
{
	return tcflush(fd, TCIOFLUSH);
}

static uint16_t decode_uint16(uint8_t *buffer)
{
	uint16_t value;
	memcpy(&value, buffer, sizeof(value));
	return be16toh(value);
}

static uint16_t decode_crc16(uint8_t *buffer)
{
	return decode_uint16(buffer);
}

static uint32_t decode_uint32_t(uint8_t *buffer)
{
	uint32_t value;
	memcpy(&value, buffer, sizeof(value));
	return be32toh(value);
}

//
// lower level functions used by top level functions
//

// ROBOCLAW_ERROR (-1) on IO error (errno set), ROBOCLAW_RETRIES_EXCEEDED(-2) on exceeded retries, ROBOCLAW_OK (0) on success
// send the `bytes_write` long command and wait until timeout for `bytes read` bytes reply
// Retry at most `rc->retries` times both if timeout on send or received incorrect checksum
// If expected `bytes read` is single byte, expect the 0xFF ACK byte, otherwise expect reply followed by CRC
static int send_cmd_wait_answer(struct roboclaw *rc, int bytes_write, int bytes_read, uint16_t cmd_crc16)
{
	int r, status;
	
	uint16_t calculated_crc16, received_crc16;

	//block with send/receive/crc check retries (this means roboclaw received correctly but crc failure happened on response)
	for(r=0; r<rc->retries; ++r)
	{		
		//block with send/wait for answer retries on timeout (this means that roboclaw didn't receive correctly)
		for(; r<rc->retries; ++r)
		{
			if( write_all(rc->fd, rc->buffer, bytes_write) == -1)
				return ROBOCLAW_ERROR; //error, errno set

			if( (status=wait_input(rc->fd, &rc->actual_termios, bytes_read, rc->timeout_ms)) == 1 )
				break; //got answer
			
			if(status==-1) 
				return ROBOCLAW_ERROR; //error, errno set
			
			//otherwise ret == 0, timetout, retry if requested
			//fprintf(stderr, "command timeout, retrying %d\n", r);
			if(flush_io(rc->fd) == -1)
				return ROBOCLAW_ERROR;
		}

		if(r >= rc->retries)
		  return ROBOCLAW_RETRIES_EXCEEDED;
			
		//input is waiting
		if( (status=read(rc->fd, rc->buffer+bytes_write, bytes_read)) == -1 )
			return ROBOCLAW_ERROR; //errno set

		//if no error we may expect to be woken when enough bytes are waiting, this is what we told the OS through VMIN
		assert(status == bytes_read);
			
		//special case, command with ACK reply only has no crc only ACK byte
		if(bytes_read==ROBOCLAW_ACK_BYTES)
		{
			if(!rc->strict_0xFF_ACK || rc->buffer[bytes_write] == ROBOCLAW_ACK_BYTE )
				return ROBOCLAW_OK;
			if(flush_io(rc->fd) == -1)
				return ROBOCLAW_ERROR;
			continue;
		}
			
		//now calculate and check crc16		
		calculated_crc16=update_crc16(rc->buffer+bytes_write, bytes_read - ROBOCLAW_CRC16_BYTES, cmd_crc16);
		received_crc16=decode_crc16(rc->buffer+bytes_write+bytes_read - ROBOCLAW_CRC16_BYTES);

		if(calculated_crc16 == received_crc16)
			return ROBOCLAW_OK;

		//if we got this far we received answer with CRC faiulure
		if(flush_io(rc->fd) == -1)
			return ROBOCLAW_ERROR;
	}

	return ROBOCLAW_RETRIES_EXCEEDED;
}

static int encode_duty_m1m2(uint8_t *buffer, uint8_t address, int16_t duty1, int16_t duty2)
{
	uint8_t bytes=0;
	buffer[bytes++]=address;
	buffer[bytes++]=MIXEDDUTY;
	bytes += encode_uint16(buffer, bytes, duty1);
	bytes += encode_uint16(buffer, bytes, duty2);
	bytes += encode_checksum(buffer, bytes);
	
	return bytes;
}

static int encode_speed_m1m2(uint8_t* buffer, uint8_t address, int32_t speed1, int32_t speed2)
{
	uint8_t bytes=0;
	buffer[bytes++]=address;
	buffer[bytes++]=MIXEDSPEED;
	bytes += encode_uint32(buffer, bytes, speed1);
	bytes += encode_uint32(buffer, bytes, speed2);
	bytes += encode_checksum(buffer, bytes);
	
	return bytes;
}


static int encode_speed_accel_m1m2(uint8_t* buffer, uint8_t address, int32_t speed1, int32_t speed2, uint32_t accel)
{
	uint8_t bytes=0;
	buffer[bytes++]=address;
	buffer[bytes++]=MIXEDSPEEDACCEL;
	bytes += encode_uint32(buffer, bytes, accel);
	bytes += encode_uint32(buffer, bytes, speed1);
	bytes += encode_uint32(buffer, bytes, speed2);
	bytes += encode_checksum(buffer, bytes);
	
	return bytes;
}

static int encode_read_encoder_1(uint8_t *buffer, uint8_t address, uint16_t *cmd_crc16)
{
	uint8_t bytes=0;
	buffer[bytes++]=address;
	buffer[bytes++]=GETM1ENC;
	*cmd_crc16 = calculate_crc16(buffer, bytes);
	return bytes;	
}

static int encode_read_encoders(uint8_t *buffer, uint8_t address, uint16_t *cmd_crc16)
{
	uint8_t bytes=0;
	buffer[bytes++]=address;
	buffer[bytes++]=GETENCODERS;
	*cmd_crc16 = calculate_crc16(buffer, bytes);
	return bytes;	
}

static void decode_read_encoder_1(uint8_t *buffer, int32_t *enc1)
{
	//*enc1=decode_uint32_t(buffer);
	*enc1=decode_uint32_t(buffer + sizeof(*enc1));
}


static void decode_read_encoders(uint8_t *buffer, int32_t *enc1, int32_t *enc2)
{
	*enc1=decode_uint32_t(buffer);
	*enc2=decode_uint32_t(buffer + sizeof(*enc1));
}

/*
 * All the commands return:
 * - ROBOCLAW_OK on success
 * - ROBOCLAW_ERROR on IO error with errno set
 * - ROBOCLAW_RETRIES_EXCEEDED if sending didn't result in reply or the checksum didn't match `replies` times (configurable)
 * 
 * The failure codes are guarateed to have negative values
 * so in simplest scenario it is enough to test for ROBOCLAW_OK
 * 
 * All the commands take parametrs:
 * - rc - pointer to library internal data
 * - address - the address of roboclaw that should receive command (from 0x80 to 0x87)
 * - ...
 * - and command specific parameters
 * 
 */

//int roboclaw_duty_m1m2(struct roboclaw *rc, uint8_t address, int16_t duty_m1, int16_t duty_m2);
int roboclaw_duty_m1m2(struct roboclaw *rc, uint8_t address, int16_t duty_m1, int16_t duty_m2)
{
	int bytes=encode_duty_m1m2(rc->buffer, address, duty_m1, duty_m2);
	return send_cmd_wait_answer(rc, bytes, ROBOCLAW_ACK_BYTES, 0);
}


int roboclaw_speed_m1m2(struct roboclaw *rc, uint8_t address, int speed_m1, int speed_m2)
{	
	int bytes=encode_speed_m1m2(rc->buffer, address, speed_m1, speed_m2);				
	return send_cmd_wait_answer(rc, bytes, ROBOCLAW_ACK_BYTES, 0);
}


int roboclaw_speed_accel_m1m2(struct roboclaw *rc, uint8_t address, int speed_m1, int speed_m2, int accel)
{	
	int bytes=encode_speed_accel_m1m2(rc->buffer, address, speed_m1, speed_m2, accel);
	return send_cmd_wait_answer(rc, bytes, ROBOCLAW_ACK_BYTES, 0);
}


//int roboclaw_main_battery_voltage(struct roboclaw *rc,uint8_t address, int16_t *voltage);


int roboclaw_encoders(struct roboclaw *rc, uint8_t address, int32_t *enc_m1, int32_t *enc_m2)
{
	int bytes, ret;
	uint16_t sent_crc16; 
	
	bytes=encode_read_encoders(rc->buffer, address, &sent_crc16);

	if( (ret=send_cmd_wait_answer(rc, bytes, ROBOCLAW_READ_ENCODERS_REPLY_BYTES, sent_crc16)) < 0 ) 
		return ret; //IO error or retries exceeded
		
	decode_read_encoders(rc->buffer+bytes, enc_m1, enc_m2);

	return ROBOCLAW_OK;
}

int32_t ReadEncM1(struct roboclaw *rc, uint8_t address)
{
	int bytes, ret;
	uint16_t sent_crc16;
	int32_t enc_m1;
	
	bytes=encode_read_encoder_1(rc->buffer, address, &sent_crc16);

	if( (ret=send_cmd_wait_answer(rc, bytes, ROBOCLAW_READ_ENCODERS_REPLY_BYTES, sent_crc16)) < 0 ) 
		return ret; //IO error or retries exceeded
		
	decode_read_encoder_1(rc->buffer+bytes, &enc_m1);

	return enc_m1;
}

#ifdef __cplusplus
}
#endif

#endif // ROBOCLAW_H
