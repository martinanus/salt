/*
 * gps.h
 *
 *  Created on: May 14, 2024
 *      Author: manus
 */

#ifndef SRC_GPS_H_
#define SRC_GPS_H_

// #include "stm32f4xx_hal.h"

#define MAX_STRING_LENGTH 100
#define MAX_FIELD_LENGTH 20
#define LOG_HEADER_LENGTH 5
#define CHECKSUM_LENGTH 2

#define LATITUDE_DEGREES_LENGTH 2
#define LATITUDE_MINUTES_LENGTH 2
#define LATITUDE_SECONDS_LENGTH 11
#define LONGITUDE_DEGREES_LENGTH 2
#define LONGITUDE_MINUTES_LENGTH 2
#define LONGITUDE_SECONDS_LENGTH 11


#define DAYS_LENGTH 2
#define MONTHS_LENGTH 2
#define YEARS_LENGTH 2
#define UTC_HOURS_LENGTH 2
#define UTC_MINUTES_LENGTH 2
#define UTC_SECOND_LENGTH 5


typedef struct {
	int degrees;
	int minutes;
	int seconds;
	char direction;
} coordinates_t;

typedef struct {
	int year;
	int month;
	int day;
	int hours;
	int minutes;
	float seconds;
} datetime_t;

struct GPRMC {
	char log_header[LOG_HEADER_LENGTH + 1];  //  1- $ + GPRMC
	datetime_t datetime;                     //  2- hhmmss.ss | 10- ddmmmyy
	char status;                             //  3- A=valid | V=invalid
	coordinates_t latitude;   //  4- latitude DDmm.mm | 5- N=north | S=south
	coordinates_t longitude;  //  6- latitude DDdmm.mm | 7- E=east | W=west
	float speed;              //  8- SoG, knots
	// track make good                          9
	// mag var                                 11
	// mag var dir                             12
	char checksum[CHECKSUM_LENGTH + 1];  // 13- Mode + *xx + CR-LF
};

void parse_GPRMC(const char *line, struct GPRMC *data);
void print_GPRMC(struct GPRMC *data);
// void transmit_GPRMC(UART_HandleTypeDef *huart, struct GPRMC *data);


#endif /* SRC_GPS_H_ */
