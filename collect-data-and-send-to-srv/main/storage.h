#pragma once
#include <stdio.h>

void storage_init(void);
void storage_append(const char *line);
FILE *storage_open_log_for_read(void);
void storage_close_log(FILE *f);
void storage_delete_log(void);
void storage_dump_log_to_uart(void);