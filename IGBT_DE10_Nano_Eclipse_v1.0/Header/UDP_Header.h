/*
 * UDP_Hedaer.h
 *
 *  Created on: Feb 24, 2020
 *      Author: chenc
 */

#ifndef HEADER_UDP_HEADER_H_
#define HEADER_UDP_HEADER_H_

// ======================================================
	// UDP stuff from
	// http://www.linuxhowtos.org/C_C++/socket.htm
	// source code: client_udp.c
int udp_sock, n;
unsigned int udp_length;
struct sockaddr_in udp_server;
struct hostent *udp_hp;
char udp_buffer[10];

char UDP_START[] = "TransmitionStart";
char UDP_END[] = "TransmitionEnd" ;

int UDP_TRANSMISSION_STATUS;
// enable ethernet UDP transmission
#define UDP_TRANSMISSION_ENABLE 	0x01
#define UDP_TRANSMISSION_DISABLE 	0x00

// unsigned int rddata [128000];
unsigned int *rddata;
// unsigned int rddata_16[32000000];
//unsigned int *rddata_16;

#endif /* HEADER_UDP_HEADER_H_ */
