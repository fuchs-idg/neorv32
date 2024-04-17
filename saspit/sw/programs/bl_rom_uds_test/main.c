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
 * @file saspit/sw/programs/logging_test/main.c
 * @author Philipp Fuchs
 * @brief Using sdspi and fatfs to implement a rudimentary logging functionality on an sd card for the NeoRV32
 **************************************************************************/

#include <neorv32.h>

#include <string.h>
//#include "sd_log.h"


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


/**********************************************************************//**
 * This program tests sd card logging
 *
 * @note This program requires the UART and the SPI to be synthesized.
 *
 * @return Irrelevant.
 **************************************************************************/
int main() {

  // check if UART0 unit is implemented at all
  if (neorv32_uart0_available() == 0) {
    return 1;
  }

  // setup UART at default baud rate, no interrupts
  neorv32_uart0_setup(BAUD_RATE, 0);

  // clear GPIO output (set all bits to 0)
  neorv32_gpio_port_set(0);

  // capture all exceptions and give debug info via UART
  // this is not required, but keeps us safe
  neorv32_rte_setup();

  // install irq handler
  neorv32_rte_handler_install(GPTMR_RTE_ID, gptmr_firq_handler);

  // configure timer for 100 Hz ticks with clock divisor = 8 and enable timer-match interrupt
  neorv32_gptmr_setup(CLK_PRSC_8, NEORV32_SYSINFO->CLK / (8 * 1000), 1);

  // check if SPI unit is implemented at all
  if (neorv32_spi_available() == 0) {
    neorv32_uart0_printf("ERROR! No SPI unit implemented.");
    return 1;
  }

  // disable and reset SPI module
  neorv32_spi_disable();
  setup_als();

  // enable interrupt
  neorv32_cpu_csr_clr(CSR_MIP, 1 << GPTMR_FIRQ_PENDING);  // make sure there is no GPTMR IRQ pending already
  neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);   // enable GPTMR FIRQ channel
  neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE); // enable machine-mode interrupts

  // Logging Test Begin
  neorv32_uart0_printf("\r\nBootloader-ROM Read TEST\r\n");

  char buffer[512*5];
  for (int i=0;i<sizeof(buffer);i++) {
    buffer[i]=i;
  }

  uint8_t * test_ptr_auto;
  uint8_t * test_ptr_manual_charp;
  uint32_t * test_ptr_manual_uintp;
  typedef uint32_t (*myfunc)();

  myfunc testfunc = (myfunc) 0xffffdf00;
  test_ptr_manual_charp = (uint8_t *) 0xffffde20;
  test_ptr_auto = (uint8_t *)   0xffffcf90;
  test_ptr_manual_uintp = (uint32_t *) 0xffffde00;

  if (*test_ptr_manual_charp == 'M') {
    neorv32_uart0_printf("Successfully read from bootloader rom!\r\n");
  }

  neorv32_uart0_printf("Automatic placement (string) : %s\r", (char *) test_ptr_auto);
  neorv32_uart0_printf("Manual placement (string) : %s\r", (char *) test_ptr_manual_charp);
  neorv32_uart0_printf("Manual placement (uint32_t) : 0x%x\r", *test_ptr_manual_uintp);
  neorv32_uart0_printf("Manual placement of function (uint32_t) : 0x%x\r\n", (*testfunc)());

  neorv32_uart0_printf("Contents of Bootloader ROM starting from %x:\r\n", test_ptr_manual_uintp - 16);



  neorv32_uart0_printf("Before lock:\r\n");

  for (int *p = test_ptr_manual_uintp - 16;p<0xffffe000;p++) {
    neorv32_uart0_printf("(ADDR)0x%x\t(CONTENT)0x%x\r", (uint32_t) (p),  *(p));
  }

  neorv32_cpu_csr_write(CSR_PMPADDR0, (0xffffde0c >> 2));
  neorv32_uart0_printf("New Contents of CSR_PMPADDR0: 0x%x\r", neorv32_cpu_csr_read(CSR_PMPADDR0) << 2);

  neorv32_cpu_csr_write(CSR_PMPCFG0, (1 << 7 | 3 << 3));
  neorv32_uart0_printf("New Contents of CSR_PMPCFG0: 0x%x\r", neorv32_cpu_csr_read(CSR_PMPCFG0));

  neorv32_uart0_printf("After lock:\r\n");

  for (int *p = test_ptr_manual_uintp - 16;p<0xffffe000;p++) {
    neorv32_uart0_printf("(ADDR)0x%x\t(CONTENT)0x%x\r", (uint32_t) (p),  *(p));
  }

  /* This block crashes the neorv32 since the RTE treats instruction access faults as fatal errors. */
  // neorv32_uart0_printf("Function in BL ROM before lock : result = 0x%x\r\n", (*testfunc)());

  // neorv32_cpu_csr_write(CSR_PMPADDR1, (0xffffdf0c >> 2));
  // neorv32_uart0_printf("Contents of CSR_PMPADDR1: 0x%x\r", neorv32_cpu_csr_read(CSR_PMPADDR1) << 2);

  // neorv32_cpu_csr_write(CSR_PMPCFG0, (1 << 7 | 3 << 3) << 8);
  // neorv32_uart0_printf("Contents of CSR_PMPCFG1: 0x%x\r", neorv32_cpu_csr_read(CSR_PMPCFG0));

  // neorv32_uart0_printf("Function in BL ROM after lock : result = 0x%x\r\n", (*testfunc)());

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