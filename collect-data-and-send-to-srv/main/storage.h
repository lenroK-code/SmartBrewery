#pragma once
#include <stdio.h>

void storage_init(void);
void storage_close_log(FILE *f);
void storage_dump_log_to_uart(const char* path_to_file);
void storage_delete_log(const char *path_to_file);
FILE *storage_open_log_for_read(const char* path_to_file);
void storage_append(const char* path_to_file, const char *line);
// void storage_delete_log(void);
// FILE *storage_open_log_for_read(void);
// void storage_append(const char *line);
// void storage_dump_log_to_uart(void);