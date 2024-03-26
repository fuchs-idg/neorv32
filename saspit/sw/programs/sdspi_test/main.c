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
 * @file saspit/sw/programs/main.c
 * @author Philipp Fuchs
 * @brief Test the sdspi module.
 **************************************************************************/

#include <neorv32.h>
#include "neorv32_uart.h"
#include <string.h>


// sdspi includes, defines and definitions
typedef	struct SDSPI_S {
	uint32_t	sd_ctrl, sd_data, sd_fifo[2];
} SDSPI;
static volatile SDSPI *const _sdcard = ((SDSPI *)0xC0000000);
#define CMD _sdcard->sd_ctrl
#define DATA _sdcard->sd_data
#define FIFO _sdcard->sd_fifo[0]

#define SD_SETAUX 0x0ff // contents of data register is sent to sdspi-internal config register
#define SD_READAUX 0x0bf // internal config register is copied to sd_data

#define SD_CMD 0x040
#define SD_READREG 0x0200 // OR into command when reading a sd-card register

#define SD_FIFO_OP 0x800
#define SD_WRITEOP 0xc00
#define SD_ALTFIFO 0x1000

#define SD_BUSY 0x04000
#define SD_ERROR 0x08000
#define SD_CLEARERR 0x08000

#define SD_REMOVED 0x40000
#define SD_PRESENTN 0x80000

#define SD_GO_IDLE ((SD_REMOVED|SD_CLEARERR|SD_CMD))

#define SD_READ_SECTOR ((SD_CMD|SD_CLEARERR|SD_FIFO_OP)+17)
#define SD_WRITE_SECTOR ((SD_CMD|SD_CLEARERR|SD_WRITEOP)+24)

#define SD_WAIT_WHILE_BUSY while(_sdcard->sd_ctrl & SD_BUSY)



/// not yet working:
// #define	_BOARD_HAS_SDSPI
// #include "diskio.h"
// #include "sdspidrv.h"

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

uint32_t hlp_force_erase_pwd();
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

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  // install irq handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 1Hz ticks with clock divisor = 8 and enable timer-match interrupt
  neorv32_gptmr_setup(CLK_PRSC_8, NEORV32_SYSINFO->CLK / (8 * 10), 1);

  // // enable interrupt
  // neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  // neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  // neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // check if UART0 unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // intro
  //neorv32_uart0_printf("\n<<< SPI Bus Explorer >>>\n\n");

  // check if SPI unit is implemented at all
  if (neorv32_spi_available() == 0) {
    neorv32_uart0_printf("ERROR! No SPI unit implemented.");
    return 1;
  }

  // info
  // neorv32_uart0_printf("This program allows to create SPI transfers by hand.\n"
  //                      "Type 'help' to see the help menu.\n\n");

  // disable and reset SPI module
  neorv32_spi_disable();

  setup_als();

  uint32_t buf[512 / 4];
  for (int i = 0; i<128; i++) {
    buf[i] = 0;
  }
  uint8_t dev_busy = 0;
  uint32_t OCR;
  uint32_t card_status;
  //uint32_t sd_read_addr =  0x1da00U;
  uint32_t sd_read_addr = 0;
  //uint32_t sd_write_addr = 0xabababU;
  uint32_t sd_write_addr = 0x100U;
  uint8_t old_sd = 0;

  uint8_t done_writing = 0;

  //neorv32_uart0_printf("Test Vector : \r");
  uint32_t test_vector[128];
  for (int i = 0; i < 128; i++) {
    //test_vector[i] = (0xC0FFE0U << 8) + i;
    test_vector[i] = 0;
    //neorv32_uart0_printf("%x\r", test_vector[i]);
  }

  // sdspi test begin
  // Using page 7 of http://www.dejazzer.com/ee379/lecture_notes/lec12_sd_card.pdf for card initialization
  

    // CLEAR ANY ERROR FLAGS
    SD_WAIT_WHILE_BUSY;
    _sdcard->sd_ctrl = SD_CLEARERR;

    // SET SPI CLOCK FREQUENCY TO 400 kHz
    _sdcard->sd_data = 0x7c;
    _sdcard->sd_ctrl = SD_SETAUX;

    // Check if SD-Card has been removed
    _sdcard->sd_data = 0;
	  _sdcard->sd_ctrl = SD_CLEARERR|SD_READAUX;
    if (_sdcard->sd_ctrl & SD_PRESENTN) {
      neorv32_uart0_printf("SDCARD: No card present\n");
      goto exit;
	  } else {
      neorv32_uart0_printf("SDCARD: Card is present and fully operational!\n");
    }

retry:
    // GO INTO RESET STATE IF IN SD MODE
    do {
      neorv32_uart0_printf("Sending CMD 0: Move to Reset State if in SD mode\r");
      _sdcard->sd_data = 0;
      _sdcard->sd_ctrl = SD_GO_IDLE;
      SD_WAIT_WHILE_BUSY;
      neorv32_uart0_printf("Response: 0x%x\n", CMD);
    } while ((CMD & 0xff) != 0x01);


    // GO INTO IDLE STATE
    neorv32_uart0_printf("Sending CMD 0: Move to Init state\r");
    _sdcard->sd_data = 0;
    _sdcard->sd_ctrl = SD_GO_IDLE;
    SD_WAIT_WHILE_BUSY;
    neorv32_uart0_printf("Response: 0x%x\n", CMD);

    neorv32_uart0_printf("Sending CMD 8\n");
    DATA = 0x1a5;
    CMD = (SD_CMD|SD_READREG)+8;
    SD_WAIT_WHILE_BUSY;
    neorv32_uart0_printf("Response: 0x%x\n", CMD);
    neorv32_uart0_printf("DATA Reg after CMD8 (should be 0x000001a5): 0x%x\n", _sdcard->sd_data);

    if (DATA != 0x1a5) {
      old_sd = 1;
      SD_WAIT_WHILE_BUSY;
      _sdcard->sd_ctrl = SD_CLEARERR;
    }
    
    // SEND CMD 1 INSTEAD OF ACMD41 IN CASE OF OLD CARD
    // neorv32_uart0_printf("Sending CMD 1 to tell the SD Card whos boss: \n");
    // _sdcard->sd_data = 0x40000000;
    // _sdcard->sd_ctrl = SD_CMD + 1;
    // SD_WAIT_WHILE_BUSY;
    // neorv32_uart0_printf("Response: 0x%x\n", CMD);
    // dev_busy = CMD&1;



    // neorv32_uart0_printf("Sending CMD 58: Read OCR Register\r");
    // CMD = (SD_READREG|SD_CMD)+58;
    // SD_WAIT_WHILE_BUSY;
    // OCR = DATA;
    // neorv32_uart0_printf("Response: 0x%x\r", CMD & 0xff);
    // neorv32_uart0_printf("OCR: 0x%x\n", OCR);

    do {
      // CMD55 gives us access to SD specific commands
      neorv32_uart0_printf("Sending CMD 55\r");
      DATA = 0;
      CMD = SD_CMD+55;
      SD_WAIT_WHILE_BUSY;
      neorv32_cpu_delay_ms(10);
      neorv32_uart0_printf("Response: 0x%x\n", CMD);
      // Now we can issue the ACMD41, to get the idle
      // status
      neorv32_uart0_printf("Sending CMD 41\r");
      if (old_sd) {
        DATA = 0;
      } else {
        DATA = 0x40000000;
      }

      CMD = SD_CMD+41;
      SD_WAIT_WHILE_BUSY;
      neorv32_uart0_printf("Response: 0x%x\n", CMD);
      if (CMD & 0x8000) goto retry;
      // The R1 response can be found in the lower 8 bits
      // of the CMD register after the command is complete.
      // Bit 1 of R1 indicates the card hasn’t finished its
      // startup
      dev_busy = CMD&1;
      SD_WAIT_WHILE_BUSY;
      if (dev_busy)
        neorv32_uart0_printf("Device busy. Retrying...\n");
    } while(dev_busy);

    neorv32_uart0_printf("Sending CMD 58: Read OCR Register\r");
    CMD = (SD_READREG|SD_CMD)+58;
    SD_WAIT_WHILE_BUSY;
    OCR = DATA;
    neorv32_uart0_printf("Response: 0x%x\r", CMD & 0xff);
    neorv32_uart0_printf("OCR: 0x%x\n", OCR);

    SD_WAIT_WHILE_BUSY;
    _sdcard->sd_ctrl = SD_CLEARERR;

    neorv32_uart0_printf("Setting FIFO length to 2**4\n");
    DATA = 0x040000;
    CMD = SD_SETAUX;

    CMD = SD_READAUX;
    neorv32_uart0_printf("Confirm AUX Register Contents: %x\n", DATA);

    neorv32_uart0_printf("Sending CMD 9: Reading CSD Register\r");
    int CSD[4];
    DATA = 0;
    CMD = (SD_FIFO_OP|SD_CMD)+9;
    SD_WAIT_WHILE_BUSY;
    for(int i=0; i<4; i++)
      CSD[i] = _sdcard->sd_fifo[0];
    neorv32_uart0_printf("Response: 0x%x\r", CMD & 0xff);
    neorv32_uart0_printf("CSD[0]: 0x%x\r", CSD[0]);
    neorv32_uart0_printf("CSD[1]: 0x%x\r", CSD[1]);
    neorv32_uart0_printf("CSD[2]: 0x%x\r", CSD[2]);
    neorv32_uart0_printf("CSD[3]: 0x%x\n", CSD[3]);

    // neorv32_uart0_printf("Sending CMD 13: Reading Card Status\r");
    // DATA = 0;
    // CMD = SD_CMD+55;
    // SD_WAIT_WHILE_BUSY;
    // neorv32_uart0_printf("Response: 0x%x\n", CMD);
    // DATA = 0;
    // CMD = (SD_CMD | SD_READREG)+13;
    // SD_WAIT_WHILE_BUSY;
    // card_status = DATA;
    // neorv32_uart0_printf("Response: 0x%x\r", CMD);
    // neorv32_uart0_printf("card_status = 0x%x\n", card_status);

    // neorv32_uart0_printf("Sending CMD 13: Reading Card Status\r");
    // DATA = 0;
    // CMD = SD_CMD+55;
    // SD_WAIT_WHILE_BUSY;
    // neorv32_uart0_printf("Response: 0x%x\n", CMD);
    // DATA = 0;
    // CMD = (SD_CMD | SD_READREG)+13;
    // SD_WAIT_WHILE_BUSY;
    // card_status = DATA;
    // neorv32_uart0_printf("Response: 0x%x\r", CMD);
    // neorv32_uart0_printf("card_status = 0x%x\n", card_status);

    // neorv32_uart0_printf("Sending CMD 42\r");
    // DATA = 0;
    // CMD = SD_CMD+42;
    // SD_WAIT_WHILE_BUSY;
    // neorv32_uart0_printf("Response: 0x%x\n", CMD);

    // neorv32_uart0_printf("Sending CMD 13: Reading Card Status\r");
    // DATA = 0;
    // CMD = SD_CMD+55;
    // SD_WAIT_WHILE_BUSY;
    // neorv32_uart0_printf("Response: 0x%x\n", CMD);
    // DATA = 0;
    // CMD = (SD_CMD| SD_READREG)+13;
    // SD_WAIT_WHILE_BUSY;
    // card_status = DATA;
    // neorv32_uart0_printf("Response: 0x%x\r", CMD);
    // neorv32_uart0_printf("card_status = 0x%x\n", card_status);

    DATA = 0x09007cU;
    CMD = SD_SETAUX;

    CMD = SD_READAUX;
    neorv32_uart0_printf("Confirm AUX Register Contents: %x\n", DATA);

  while(1) {
    uint8_t nz = 0;

    // READ TEST BEGIN
    for (int j=0;j<=0;j++) {
    //if (done_writing) {
      neorv32_uart0_printf("Attempting to read sector 0x%x into buf\r", sd_read_addr);
      DATA = sd_read_addr;
      CMD = SD_READ_SECTOR;
      SD_WAIT_WHILE_BUSY;
      neorv32_uart0_printf("Response: 0x%x\r", CMD);

      if (CMD & 0x8000) {
        neorv32_uart0_printf("Sending CMD 13: Reading Card Status\r");
        DATA = 0;
        CMD = (SD_CLEARERR|SD_CMD| SD_READREG)+13;
        SD_WAIT_WHILE_BUSY;
        card_status = DATA;
        neorv32_uart0_printf("Response: 0x%x\r", CMD);
        neorv32_uart0_printf("card_status = 0x%x\n", card_status);
        SD_WAIT_WHILE_BUSY;
        _sdcard->sd_ctrl = SD_CLEARERR;
      }
      
      for(int i=0; i<512/4; i++) {
        buf[i] = FIFO;
        SD_WAIT_WHILE_BUSY; 
        if (buf[i])
          nz += 1;
      }
      if (nz) {
        neorv32_uart0_printf("Non zero contents in buffer!\r");
        neorv32_uart0_printf("Current address: 0x%x\rContents:\r", sd_read_addr);
        for (int i=0;i<512/4;i++) {
          neorv32_uart0_printf("buf[%d]: \t0x%x\r", i, buf[i]);
        }
        nz = 0;
      }
      else {
        neorv32_uart0_printf("Sector 0x%x is empty!\r", sd_read_addr);
      }
      sd_read_addr += 0x200U;
    }
    // READ TEST END
    //hlp_force_erase_pwd();
    //goto exit;

    // WRITE TEST BEGIN
  //   else {
  //     neorv32_uart0_printf("WRITE TEST\n");
  //     done_writing = 1;

  //     neorv32_uart0_printf("Attempting write test vector into sector 0x%x\r", sd_write_addr);
  //     neorv32_uart0_printf("Print any key to proceed.");
  //     char dummybuf;
  //     (void) neorv32_uart0_scan(&dummybuf, 1, 0);

  //     DATA = sd_write_addr;
  //     CMD = SD_READ_SECTOR;
  //     SD_WAIT_WHILE_BUSY;
  //     neorv32_uart0_printf("Response: 0x%x\r", CMD & 0xffU);
  //     for(int i=0; i<512/4; i++) {
  //       buf[i] = FIFO;
  //       SD_WAIT_WHILE_BUSY; 
  //       if (buf[i])
  //         nz += 1;
  //     }
  //     if (nz) {
  //       neorv32_uart0_printf("Non zero contents in buffer!\r");
  //       neorv32_uart0_printf("Current address: 0x%x\rContents:\r", sd_write_addr);
  //       for (int i=0;i<512/4;i++) {
  //         neorv32_uart0_printf("buf[%d]: \t0x%x\r", i, buf[i]);
  //       }
  //       neorv32_uart0_printf("Overwriting Sector %x\r Press any key to proceed...\r", sd_write_addr);
  //       (void) neorv32_uart0_scan(&dummybuf, 1, 0);
  //       sd_write(sd_write_addr, test_vector);
  //     } else {
  //       neorv32_uart0_printf("Sector 0x%x is empty. Writing test vector!\r", sd_write_addr);
  //       sd_write(sd_write_addr, test_vector);
  //       neorv32_uart0_printf("Response: 0x%x\r", CMD);
  //     }


  //   }
  //   // WRITE TEST END
    char scan_buf[8];
    uint8_t scan_buf_ln = 0;
    neorv32_uart0_printf("Enter sector address to read from: 0x");
    scan_buf_ln = neorv32_uart0_scan(scan_buf, 8, 1);
    sd_read_addr = hexstr_to_uint(scan_buf, scan_buf_ln);
  }


  // sdspi test end
exit:
  while(1) {
    neorv32_cpu_sleep();
  }
  return 0;
}

uint32_t sd_read(uint32_t sector, uint32_t *buf) {
  uint32_t nz=0;

  DATA = 0x090000U;
  CMD = SD_SETAUX;

  DATA = sector;
  CMD = SD_READ_SECTOR;
  SD_WAIT_WHILE_BUSY;

  for(int i=0; i<512/4 ; i++) {
    buf[i] = FIFO;
    if (buf[i])
      nz++;
  }
  return nz;
}

void sd_write(uint32_t sector, uint32_t *buf) {
  DATA = 0x090000;
  CMD = SD_SETAUX;

  for(int i=0; i<512/4; i++) {
    FIFO = buf[i];
  }

  DATA = sector;
  CMD = SD_WRITE_SECTOR;
  SD_WAIT_WHILE_BUSY;
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

/*
Force erase the SD card password.
This also deletes all data contents on the card.
*/
uint32_t hlp_force_erase_pwd() {
  // First: send CMD16 to set pw length in CMD42 transmission
  neorv32_uart0_printf("Sending CMD 16.\r");
  DATA = 0x02;
  CMD = SD_CMD + 16;
  SD_WAIT_WHILE_BUSY;
  neorv32_uart0_printf("Response: %d\r", CMD);
  if (CMD != 0x0) {
    return 1;
  }

  // Then: send CMD42 with fifo length set to pw length
    // First byte in fifo must be 0x04

  // uint32_t payload[2] = {0x04000000, 0x0};
  // DATA = 0x010000;
  // CMD = SD_SETAUX;

  // for(int i=0; i<2; i++) {
  //   FIFO = payload[i];
  // }
  // neorv32_uart0_printf("Sending CMD 42.\r");
  // DATA = 0;
  // CMD = (SD_CMD|SD_CLEARERR|SD_WRITEOP)+42;
  // SD_WAIT_WHILE_BUSY;
  // neorv32_uart0_printf("Response: %d\r", CMD);
  // if (CMD != 0x0) {
  //   return 1;
  // }
  return 0;
}