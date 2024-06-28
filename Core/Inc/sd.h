/*
 * sd.h
 *
 *  Created on: May 21, 2024
 *      Author: manus
 */

#ifndef SRC_SD_H_
#define SRC_SD_H_

void mount_filesystem(FATFS *fs);
void list_root_files(void);
void log_event(const char *filename, const char *time, const char *event);

#endif /* SRC_SD_H_ */
