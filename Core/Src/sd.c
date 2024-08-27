/*
 * sd.c
 *
 *  Created on: May 21, 2024
 *      Author: manus
 */
#include <string.h>
#include <stdio.h>
#include "ff.h"

void mount_filesystem(FATFS *fs)
{
    if (f_mount(fs, "", 0) != FR_OK)
    {
        printf("Failed to mount filesystem\n");
    }
}
void list_root_files(void)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    char *path = ""; // Root directory
    char buffer[256];

    // Open the root directory
    res = f_opendir(&dir, path);
    if (res == FR_OK)
    {
        while (1)
        {
            // Read a directory item
            res = f_readdir(&dir, &fno);
            // Break on error or end of directory
            if (res != FR_OK || fno.fname[0] == 0)
                break;
            // Skip hidden files
            if (fno.fattrib & AM_HID)
                continue;

            // Check if it is a directory or file
            if (fno.fattrib & AM_DIR)
            {
                sprintf(buffer, "DIR:  %s\r\n", fno.fname);
            }
            else
            {
                sprintf(buffer, "FILE: %s\r\n", fno.fname);
            }
            // Print the file or directory name
            printf("%s", buffer);
        }
        f_closedir(&dir);
    }
    else
    {
        printf("Failed to open directory, error code: %d\r\n", res);
    }
}

void write_in_file(const char *filename, const char *buffer)
{
    FIL file;    // File object
    FRESULT res; // FatFS result

    res = f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND);
    if (res == FR_OK)
    {
        // Create log entry with a timestamp

        unsigned int bytes_written;
        if (f_write(&file, buffer, strlen(buffer), &bytes_written) == FR_OK)
        {
            printf("%s wrote in file: %s\r\n", buffer, filename);
        }
        else
        {
            printf("Error writing the file in the SD CARD\n");
        }
        if (f_close(&file) != FR_OK)
        {
            printf("Error closing the file in the SD CARD\n");
        }
    }
    else
    {
        printf("Error opening the file in the SD CARD\n");
    }
}
