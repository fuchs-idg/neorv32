// Mounts sd card
// Creates new file with date as file name

#include <neorv32.h>
#include <string.h>
#include "sd_log.h"

static sd_log_t sd_log;

// init logging to file "0:/logs/<BUILD_DATE>"
void sd_log_init() {

  FIL *fp = &sd_log.logfile;
  FATFS *fatfs = &sd_log.fatfs;
  // Set file name
  char file_name[9] = {};
  strncpy(file_name, __DATE__, 6);
  file_name[3] = '-';
  file_name[6] = '\0';

  // Mount sd card
  FRESULT fresult;
  const char logs_path[] = "0:/logs/";
  char file_path[9 + sizeof(logs_path) + 1] = {};
  strncpy(file_path, logs_path, sizeof(file_path));
  strncat(file_path, file_name, sizeof(file_name));

  fresult = f_mount(fatfs,"0:",0);
  
  // fresult == FR_OK == 0 if successful
  if (!fresult) {
    fresult = f_mkdir("logs");
  }
  else {
    neorv32_uart0_printf("Could not mount sd card!\r");
  }

  if (fresult == FR_OK || fresult == FR_EXIST) {
    fresult = f_open(fp, file_path, FA_WRITE | FA_OPEN_APPEND);
  }
  else {
    //neorv32_uart0_printf("Could not mount sd card!\r");
    neorv32_uart0_printf("Error when creating logs subdirectory!\r");
  }


  if (!fresult) {
    const char msg[] =  "\r\r[sd_log_init] Initialized logging on sd card, Build Time : ";
    neorv32_uart0_printf("Logging to file %s\r", file_path);
    fresult = f_write(fp, msg, sizeof(msg) - 1, NULL);
    fresult |= f_write(fp, __TIME__, sizeof(__TIME__) - 1, NULL);
    fresult |= f_write(fp, "\r", 1, NULL);
    fresult |= f_sync(fp);
  }
  else {
    neorv32_uart0_printf("Could not open file %s!\r", file_path);
    neorv32_uart0_printf("Error Type : %i\r", fresult);
  }

  if (fresult) {
    neorv32_uart0_printf("Could not write to file %s!\r", file_path);
    neorv32_uart0_printf("Last errors : %i\r", fresult);
  }
}

/*
Input: 
  const char msg[] : The log message
  const uint32_t msg_len : The length of the log message (sans final '\0' character)
  uint8_t sync : Sync directly after writing log if true (aka if not zero)
*/
void sd_log_write(const char msg[], const uint32_t msg_len, const uint8_t do_sync) {
  f_write(&sd_log.logfile, msg, msg_len, NULL);
  if (do_sync) {
    f_sync(&sd_log.logfile);
  }
}