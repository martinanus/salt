/*
 * gps.c
 *
 *  Created on: May 14, 2024
 *      Author: manus
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gps.h"

void
parse_GPRMC(const char *line, struct GPRMC *data)
{
	int fieldStart = 0;
	int count = 1;
	char temp_buff[MAX_FIELD_LENGTH + 1];

	// fieldize the input line
	for (int i = 0; line[i] != '\0'; i++) {
		if (line[i] == ',' || line[i + 1] == '\0') {
			int fieldLength = i - fieldStart;
			char field[MAX_FIELD_LENGTH + 1];

			strncpy(field, line + fieldStart, fieldLength);
			field[fieldLength] = '\0';
			printf("count %i fiel %s\r\n", count, field);

			switch (count) {
			case 1:
				strncpy(data->log_header,
				        field + 1,
				        LOG_HEADER_LENGTH);
				data->log_header[LOG_HEADER_LENGTH] = '\0';
				break;
			case 2:
				strncpy(temp_buff, field, UTC_HOURS_LENGTH);
				temp_buff[UTC_HOURS_LENGTH] = '\0';
				data->datetime.hours = atoi(temp_buff);

				strncpy(temp_buff,
				        field + UTC_HOURS_LENGTH,
				        UTC_MINUTES_LENGTH);
				temp_buff[2] = '\0';
				data->datetime.minutes = atoi(temp_buff);

				strncpy(temp_buff,
				        field + UTC_HOURS_LENGTH +
				                UTC_MINUTES_LENGTH,
				        UTC_SECOND_LENGTH);
				temp_buff[UTC_SECOND_LENGTH] = '\0';
				data->datetime.seconds = atof(temp_buff);
				break;
			case 3:
				data->status = field[0];
				break;
			case 4:
				if (data->status == 'A') {
					strncpy(temp_buff,
					        field,
					        LATITUDE_DEGREES_LENGTH);
					temp_buff[LATITUDE_DEGREES_LENGTH] = '\0';
					data->latitude.degrees = atoi(temp_buff);

					strncpy(temp_buff,
					        field + LATITUDE_DEGREES_LENGTH,
					        LATITUDE_MINUTES_LENGTH);
					temp_buff[LATITUDE_MINUTES_LENGTH] = '\0';
					data->latitude.minutes = atoi(temp_buff);

					strncpy(temp_buff,
					        field + LATITUDE_DEGREES_LENGTH +
					                LATITUDE_MINUTES_LENGTH + 1,
					        LATITUDE_SECONDS_LENGTH);
					temp_buff[LATITUDE_SECONDS_LENGTH] = '\0';
					data->latitude.seconds = atoi(temp_buff);
				}

				break;
			case 5:
				if (data->status == 'A') {
					data->latitude.direction = field[0];
				}
				break;
			case 6:
				if (data->status == 'A') {
					strncpy(temp_buff,
					        field,
					        LONGITUDE_DEGREES_LENGTH);
					temp_buff[LONGITUDE_DEGREES_LENGTH] = '\0';
					data->longitude.degrees = atoi(temp_buff);

					strncpy(temp_buff,
					        field + LONGITUDE_DEGREES_LENGTH,
					        LONGITUDE_MINUTES_LENGTH);
					temp_buff[2] = '\0';
					data->longitude.minutes = atoi(temp_buff);

					strncpy(temp_buff,
					        field + LONGITUDE_DEGREES_LENGTH +
					                LONGITUDE_MINUTES_LENGTH +
					                1,
					        LONGITUDE_SECONDS_LENGTH);
					temp_buff[LONGITUDE_SECONDS_LENGTH] = '\0';
					data->longitude.seconds = atoi(temp_buff);
				}
				break;
			case 7:
				if (data->status == 'A') {
					data->longitude.direction = field[0];
				}
				break;
			case 8:
				data->speed = atof(field);
				break;
			case 10:
				strncpy(temp_buff, field, DAYS_LENGTH);
				temp_buff[DAYS_LENGTH] = '\0';
				data->datetime.day = atoi(temp_buff);

				strncpy(temp_buff,
				        field + DAYS_LENGTH,
				        MONTHS_LENGTH);
				temp_buff[MONTHS_LENGTH] = '\0';
				data->datetime.month = atoi(temp_buff);

				strncpy(temp_buff,
				        field + DAYS_LENGTH + MONTHS_LENGTH,
				        YEARS_LENGTH);
				temp_buff[YEARS_LENGTH] = '\0';
				data->datetime.year = atof(temp_buff);
				break;
			case 13:
				strncpy(data->checksum, field + 2, CHECKSUM_LENGTH);
				data->checksum[CHECKSUM_LENGTH] = '\0';
				break;
			}

			count++;
			fieldStart = i + 1;
		}
	}
}

void
print_GPRMC(struct GPRMC *data)
{
	// Access the struct members and do something with them
	printf("log_header: %s\r\n", data->log_header);
	printf("status:     %c\r\n", data->status);

	printf("LATITUDE\r\n");
	printf("   deg:     %i\r\n", data->latitude.degrees);
	printf("   min:     %i\r\n", data->latitude.minutes);
	printf("   sec:     %i\r\n", data->latitude.seconds);
	printf("   dir:     %c\r\n", data->latitude.direction);

	printf("LATITUDE\r\n");
	printf("   deg:     %i\r\n", data->longitude.degrees);
	printf("   min:     %i\r\n", data->longitude.minutes);
	printf("   sec:     %i\r\n", data->longitude.seconds);
	printf("   dir:     %c\r\n", data->longitude.direction);

	printf("DATETIME\r\n");
	printf("   year:    %i\r\n", data->datetime.year);
	printf("   month:   %i\r\n", data->datetime.month);
	printf("   day:     %i\r\n", data->datetime.day);
	printf("   hrs:     %i\r\n", data->datetime.hours);
	printf("   min:     %i\r\n", data->datetime.minutes);
	printf("   sec:     %f\r\n", data->datetime.seconds);


	printf("speed:      %f\r\n", data->speed);
	printf("checksum:   %s\r\n", data->checksum);
}

/*
void transmit_GPRMC(UART_HandleTypeDef *huart, struct GPRMC *data)
{
        char txBuffer[MAX_STRING_LENGTH];
        // Access the struct members and do something with them
        sprintf(txBuffer, "Log header: %s\r\n", data->log_header);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "status:     %c\r\n", data->status);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "LATITUDE\r\n");
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   deg:     %i\r\n", data->latitude.degrees);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   min:     %i\r\n", data->latitude.minutes);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   sec:     %i\r\n", data->latitude.seconds);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   dir:     %c\r\n", data->latitude.direction);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "LONGITUDE\r\n");
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   deg:     %i\r\n", data->longitude.degrees);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   min:     %i\r\n", data->longitude.minutes);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   sec:     %i\r\n", data->longitude.seconds);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   dir:     %c\r\n", data->longitude.direction);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "DATETIME\r\n");
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   year:    %i\r\n", data->datetime.year);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   month:   %i\r\n", data->datetime.month);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   day:     %i\r\n", data->datetime.day);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   hrs:     %i\r\n", data->datetime.hours);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   min:     %i\r\n", data->datetime.minutes);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "   sec:     %f\r\n", data->datetime.seconds);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "speed:      %f\r\n", data->speed);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

        sprintf(txBuffer, "checksum:   %s\r\n", data->checksum);
        HAL_UART_Transmit(huart, (uint8_t *)txBuffer, strlen(txBuffer), 1000);

}
*/
