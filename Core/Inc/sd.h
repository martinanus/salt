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
void write_in_file(const char *filename, const char *buffer);

#endif /* SRC_SD_H_ */
