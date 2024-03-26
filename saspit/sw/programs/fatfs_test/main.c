// #################################################################################################
// # << NEORV32 - SPI Bus Explorer Demo Program >>                                                 #
// # ********************************************************************************************* #
// # BSD 3-Clause License                                                                          #
// #                                                                                               #
// # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
// #                                                                                               #
// # Redistribution and use in source and binary forms, with or without modification, are          #
// # permitted provided that the following conditions are met:                                     #
// #                                                                                               #
// # 1. Redistributions of source code must retain the above copyright notice, this list of        #
// #    conditions and the following disclaimer.                                                   #
// #                                                                                               #
// # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
// #    conditions and the following disclaimer in the documentation and/or other materials        #
// #    provided with the distribution.                                                            #
// #                                                                                               #
// # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
// #    endorse or promote products derived from this software without specific prior written      #
// #    permission.                                                                                #
// #                                                                                               #
// # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
// # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
// # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
// # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
// # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
// # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
// # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
// # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
// # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
// # ********************************************************************************************* #
// # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
// #################################################################################################


/**********************************************************************//**
 * @file saspit/sw/programs/fatfs_test/main.c
 * @author Philipp Fuchs
 * @brief Testing sdspi and fatfs.
 **************************************************************************/

#include <neorv32.h>
#include <string.h>

#include "ff.h"


/**********************************************************************//**
 * @name User configuration
 **************************************************************************/
/**@{*/
/** UART BAUD rate */
#define BAUD_RATE 19200
/**@}*/

// Global variables
uint32_t spi_configured;

// Prototypes
uint32_t hexstr_to_uint(char *buffer, uint8_t length);
void aux_print_hex_byte(uint8_t byte);

void setup_als(void);
uint32_t read_als(void);

void gptmr_firq_handler(void);

void sd_write(uint32_t sector, uint32_t *buf);

FRESULT hlp_open_file_write_append(FIL *fp, const TCHAR *path);
FRESULT hlp_open_file_write(FIL *fp, const TCHAR *path);
FRESULT hlp_open_file_read(FIL *fp, const TCHAR *path);
FRESULT hlp_read_file(FIL *fp, void *buff, UINT btr, UINT *br);
FRESULT hlp_write_file(FIL *fp, const void *buff, UINT btw, UINT *bw);
FRESULT hlp_close_file(FIL *fp);

/**********************************************************************//**
 * This program provides an interactive console to communicate with SPI devices.
 *
 * @note This program requires the UART and the SPI to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // check if UART0 unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  // install irq handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 1Hz ticks with clock divisor = 8 and enable timer-match interrupt
  neorv32_gptmr_setup(CLK_PRSC_2, NEORV32_SYSINFO->CLK / (2 * 1000), 1);

  // check if SPI unit is implemented at all
  if (neorv32_spi_available() == 0) {
    neorv32_uart0_printf("ERROR! No SPI unit implemented.");
    return 1;
  }

  // disable and reset SPI module
  neorv32_spi_disable();
  setup_als();

  neorv32_mtime_set_time(0);

  // enable interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // FATFS Test Begin
  neorv32_uart0_printf("\r\nFATFS TEST\r\n");

  // 1. Create New FATFS object, new FRESULT object, new FIL object
  FRESULT myFRESULT;
  volatile FATFS myFATFS;
  FIL myFIL, myFILtoWrite;
  char buffer[512*200];
  for (int i=0;i<sizeof(buffer);i++) {
    buffer[i]=0;
  }
  char fileName[64];
  char line[255];
  uint32_t read_count, write_count;
  const char lineOfText[] = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r";
  const char longLineOfText[] = "1123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r"
                                "2123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r"
                                "3123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r"
                                "4123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r"
                                "5123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r"
                                "6123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r";
  // 2. Mount disk
  neorv32_uart0_printf("Mounting drive.\r");
  myFRESULT = f_mount(&myFATFS,"0:",0);
  if (myFRESULT == FR_OK) {
    neorv32_uart0_printf("Success!\r");
  } else {
    neorv32_uart0_printf("Error!\rErrorcode: %i\r", (uint32_t) myFRESULT);
    goto end;
  }

  // 3. Info functions //f_getfree()
  //f_getfree()
  
  // 4. Open file, read file and close file
  //f_open();
  //f_read();
  //f_close();

  // neorv32_uart0_printf("Give file name:\r");
  // neorv32_uart0_scan(&fileName, 64, 1);
  const char constName[] = "0:TEST12.TXT";
  strcpy(fileName,constName);

  uint32_t nRounds = 2;
  for(int i=1;i<=nRounds;i++) {
    if (i == 1) {
      myFRESULT = hlp_open_file_write(&myFIL, fileName);
      if (myFRESULT)
        break;
      myFRESULT = hlp_write_file(&myFIL, longLineOfText, sizeof(longLineOfText) - 1, &write_count);
      if (myFRESULT)
        break;
      myFRESULT = hlp_close_file(&myFIL);
      if (myFRESULT)
        break;
    }

    myFRESULT = hlp_open_file_read(&myFIL, fileName);
    if (myFRESULT)
      break;
    myFRESULT = hlp_read_file(&myFIL, buffer, sizeof(buffer), &read_count);
    if (myFRESULT)
      break;
    myFRESULT = hlp_close_file(&myFIL);
    if (myFRESULT)
      break;
    
    for (int i=0;i<sizeof(buffer);i++) {
      buffer[i]=0;
    }

    if (i % 2 == 0) {
      myFRESULT = hlp_open_file_write_append(&myFIL, fileName);
      if (myFRESULT)
        break;
      myFRESULT = hlp_write_file(&myFIL, lineOfText, sizeof(lineOfText) - 1, &write_count);
      if (myFRESULT)
        break;
      myFRESULT = hlp_close_file(&myFIL);
      if (myFRESULT)
        break;
    }
    
    myFRESULT = hlp_open_file_read(&myFIL, fileName);
    if (myFRESULT)
      break;
    myFRESULT = hlp_read_file(&myFIL, buffer, sizeof(buffer), &read_count);
    if (myFRESULT)
      break;
    myFRESULT = hlp_close_file(&myFIL);
    if (myFRESULT)
      break;
  }

  if (!myFRESULT) {
    neorv32_uart0_printf("\r\r\rFATFS Test successful!\r\r");
  }
  f_unmount("0:");

end:
  while(1) {
    neorv32_cpu_sleep();
  }
  return 0;
}


void gptmr_firq_handler(void) {
  neorv32_cpu_csr_write(CSR_MIP, ~(1<<GPTMR_FIRQ_PENDING)); // clear/ack pending FIRQ
  uint32_t als_result;
  uint8_t leds;
  als_result = read_als();
  leds = 0;
  int i=0;
  for(i=0;i<8;i++) {
    if (als_result >= 1<<i) {
      leds |= 1<<i;
    }
  }
  neorv32_gpio_port_set(leds & 0xFF);
}

/**********************************************************************//**
 * Helper function to convert N hex chars string into uint32_T
 *
 * @param[in,out] buffer Pointer to array of chars to convert into number.
 * @param[in,out] length Length of the conversion string.
 * @return Converted number.
 **************************************************************************/
uint32_t hexstr_to_uint(char *buffer, uint8_t length) {

  uint32_t res = 0, d = 0;
  char c = 0;

  while (length--) {
    c = *buffer++;

    if ((c >= '0') && (c <= '9'))
      d = (uint32_t)(c - '0');
    else if ((c >= 'a') && (c <= 'f'))
      d = (uint32_t)((c - 'a') + 10);
    else if ((c >= 'A') && (c <= 'F'))
      d = (uint32_t)((c - 'A') + 10);
    else
      d = 0;

    res = res + (d << (length*4));
  }

  return res;
}


/**********************************************************************//**
 * Print HEX byte.
 *
 * @param[in] byte Byte to be printed as 2-cahr hex value.
 **************************************************************************/
void aux_print_hex_byte(uint8_t byte) {

  static const char symbols[] = "0123456789abcdef";

  neorv32_uart0_putc(symbols[(byte >> 4) & 0x0f]);
  neorv32_uart0_putc(symbols[(byte >> 0) & 0x0f]);
}

void setup_als(void) {
  neorv32_spi_setup(4, 8, 1, 1, 0);
}

uint32_t read_als(void) {
  uint32_t result = 0;
  uint8_t buf[2] = {0};
  
  neorv32_spi_cs_en(1);

  buf[0] = neorv32_spi_trans(0);
  buf[1] = neorv32_spi_trans(0);

  neorv32_spi_cs_dis();

  result = buf[0] << 3;
  result |= buf[1] >> 5;

  return result;
}

// helper functions
FRESULT hlp_open_file_write_append(FIL *fp, const TCHAR *path) {
  FRESULT result;
  neorv32_uart0_printf("Opening File '%s' to write\r", path);
  result = f_open(fp, path, FA_WRITE | FA_OPEN_APPEND);
  if (result == FR_OK) {
    neorv32_uart0_printf("Success!\r");
  } else {
  neorv32_uart0_printf("Error!\rErrorcode: %i\r", (uint32_t) result);
  }
  return result;
}

FRESULT hlp_open_file_write(FIL *fp, const TCHAR *path) {
  FRESULT result;
  neorv32_uart0_printf("Opening File '%s' to write\r", path);
  result = f_open(fp, path, FA_WRITE | FA_CREATE_ALWAYS );
  if (result == FR_OK) {
    neorv32_uart0_printf("Success!\r");
  } else {
  neorv32_uart0_printf("Error!\rErrorcode: %i\r", (uint32_t) result);
  }
  return result;
}

FRESULT hlp_open_file_read(FIL *fp, const TCHAR *path) {
  FRESULT result;
  neorv32_uart0_printf("Opening File '%s' to read\r", path);
  result = f_open(fp, path, FA_READ);
  if (result == FR_OK) {
    neorv32_uart0_printf("Success!\r");
  } else {
    neorv32_uart0_printf("Error!\rErrorcode: %i\r", (uint32_t) result);
  }
  return result;
}

FRESULT hlp_read_file(FIL *fp, void *buff, UINT btr, UINT *br) {
  FRESULT result;
  uint32_t read_count;
  neorv32_uart0_printf("Reading File 'test.txt'\r");
  result = f_read(fp, buff, btr, &read_count);
  if (result == FR_OK) {
    neorv32_uart0_printf("Success!\r");
  } else {
    neorv32_uart0_printf("Error!\rErrorcode: %i\r", (uint32_t) result);
  }
  neorv32_uart0_printf("Contents of file:\r\"\r%s\r\"\r", buff);
  neorv32_uart0_printf("Read %d characters\r", read_count);

  // for (int i=0;i<read_count;i++) {
  //   neorv32_uart0_putc(*((char *) buff + i));
  //   if (*((char *) buff + i) == 0)
  //     neorv32_uart0_puts("[NULL]");
  // }
  *br = read_count;
  return result;
}

FRESULT hlp_write_file(FIL *fp, const void *buff, UINT btw, UINT *bw) {
  FRESULT result;
  neorv32_uart0_printf("Writing to the file\r");
  result = f_write(fp, buff, btw, bw);
  if (result == FR_OK) {
    neorv32_uart0_printf("Success!\r");
  } else {
  neorv32_uart0_printf("Error!\rErrorcode: %i\r", (uint32_t) result);
  }
  neorv32_uart0_printf("Written %d characters\r", *bw);
  return result;
}

FRESULT hlp_close_file(FIL *fp) {
  FRESULT result;
  neorv32_uart0_printf("Closing File 'test.txt'\r");
  result = f_close(fp);
  if (result == FR_OK) {
    neorv32_uart0_printf("Success!\r");
  } else {
    neorv32_uart0_printf("Error!\rErrorcode: %i\r", (uint32_t) result);
  }
  return result;
}

