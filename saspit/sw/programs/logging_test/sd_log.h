#include <ff.h>

typedef struct {
  FATFS fatfs;
  FIL logfile;
} sd_log_t;

void sd_log_init();
void sd_log_write(const char msg[], const uint32_t msg_len, const uint8_t do_sync);