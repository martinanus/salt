/*
 * sd.h
 *
 *  Created on: May 21, 2024
 *      Author: manus
 */

#ifndef SRC_SD_H_
#define SRC_SD_H_

#define MAX_BUFF_SIZE 256

void mount_filesystem(FATFS *fs);
void list_root_files(void);
void write_in_file(const char *filename, const char *buffer);
void read_file_line_by_line(const char *filename, UART_HandleTypeDef * uart_handle);

#endif /* SRC_SD_H_ */
