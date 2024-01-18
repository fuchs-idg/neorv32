----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 06.11.2023 08:29:11
-- Design Name: 
-- Module Name: top - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
library UNISIM;
use UNISIM.VComponents.all;

library neorv32;
use neorv32.neorv32_package.all;

entity top is
  generic (
      -- adapt these for your setup --
      CLOCK_FREQUENCY   : natural := 100000000; -- clock frequency of clk_i in Hz
      MEM_INT_IMEM_SIZE : natural := 512*1024;   -- size of processor-internal instruction memory in bytes
      MEM_INT_DMEM_SIZE : natural := 256*1024     -- size of processor-internal data memory in bytes
    );
  Port ( clk : in std_logic;
         rstn_i : in std_logic;
         led_o : out std_logic_vector(7 downto 0);
         twi_sda : inout std_logic;
         twi_scl : inout std_logic;
         spi_sck  : out std_logic;
         spi_mosi : out std_logic;
         spi_miso : in  std_logic;
         spi_ss   : out std_logic;
         uart_tx : out std_logic;
         uart_rx : in std_logic );
end top;

architecture Behavioral of top is
        
    signal con_gpio_o : std_ulogic_vector(63 downto 0);
    signal gpio_o : std_logic_vector (7 downto 0);
    signal twi_sda_i, twi_sda_o, twi_scl_i, twi_scl_o : std_ulogic;
    signal spi_clk_o, spi_dat_o, spi_dat_i : std_ulogic;
    signal spi_csn_o : std_ulogic_vector(07 downto 0);
    
begin
    
    -- Signal Assignments for Processor interfaces
    -- -------------------------------------------
    -- SPI
    --spi_mosi <= std_logic(spi_dat_o);
    spi_sck <= std_logic(spi_clk_o);
    spi_ss <= std_logic(spi_csn_o(0));
    spi_dat_i <= std_ulogic(spi_miso);
    
    -- TWI/I2C
    twi_sda       <= '0' when (twi_sda_o = '0') else 'Z'; -- drive
    twi_scl       <= '0' when (twi_scl_o = '0') else 'Z'; -- drive
    twi_sda_i <= std_ulogic(twi_sda); -- sense
    twi_scl_i <= std_ulogic(twi_scl); -- sense
    
    -- GPIO output to LD0-LD7
    led_o <= std_logic_vector(con_gpio_o(7 downto 0));
    
    -- The Core Of The Problem ----------------------------------------------------------------
    -- ----------------------------------------------------------------------------------------
    neorv32_top_inst: neorv32_top
    generic map (
      -- General --
      CLOCK_FREQUENCY            => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
      INT_BOOTLOADER_EN          => true,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
      -- On-Chip Debugger (OCD) --
      ON_CHIP_DEBUGGER_EN        => true,              -- implement on-chip debugger
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A      => true,              -- implement atomic memory operations extension?
      CPU_EXTENSION_RISCV_B      => true,              -- implement bit-manipulation extension?                
      CPU_EXTENSION_RISCV_C      => true,              -- implement compressed extension?    
      CPU_EXTENSION_RISCV_M      => true,              -- implement mul/div extension? 
      CPU_EXTENSION_RISCV_U      => true,              -- implement user mode extension?    
      CPU_EXTENSION_RISCV_Zfinx  => true,              -- implement 32-bit floating-point extension (using INT regs!)
      CPU_EXTENSION_RISCV_Zicntr => true,              -- implement base counters?
      -- Tuning Options --                                                                                                
      FAST_MUL_EN                => true,              -- use DSPs for M extension's multiplier
      FAST_SHIFT_EN              => true,              -- use barrel shifter for shift operations
      -- Internal Instruction memory --
      MEM_INT_IMEM_EN            => true,              -- implement processor-internal instruction memory
      MEM_INT_IMEM_SIZE          => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
      -- Internal Data memory --
      MEM_INT_DMEM_EN            => true,              -- implement processor-internal data memory
      MEM_INT_DMEM_SIZE          => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes
      -- Processor peripherals --
      IO_GPIO_NUM                => 8,                 -- number of GPIO input/output pairs (0..64)
      IO_MTIME_EN                => true,              -- implement machine system timer (MTIME)?
      IO_UART0_EN                => true,              -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART0_RX_FIFO           => 1,                 -- RX fifo depth, has to be a power of two, min 1
      IO_UART0_TX_FIFO           => 1,                 -- TX fifo depth, has to be a power of two, min 1
      IO_UART1_EN                => false,             -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_UART1_RX_FIFO           => 1,                 -- RX fifo depth, has to be a power of two, min 1
      IO_UART1_TX_FIFO           => 1,                 -- TX fifo depth, has to be a power of two, min 1
      IO_SPI_EN                  => true,             -- implement serial peripheral interface (SPI)?
      IO_SPI_FIFO                => 2,                 -- RTX fifo depth, has to be a power of two, min 1
      IO_SDI_EN                  => false,             -- implement serial data interface (SDI)?
      IO_SDI_FIFO                => 1,                 -- RTX fifo depth, has to be zero or a power of two, min 1
      IO_TWI_EN                  => true,              -- implement two-wire interface (TWI)?
      IO_PWM_NUM_CH              => 0,                 -- number of PWM channels to implement (0..12); 0 = disabled
      IO_WDT_EN                  => false,             -- implement watch dog timer (WDT)?
      IO_TRNG_EN                 => true,              -- implement true random number generator (TRNG)?
      IO_TRNG_FIFO               => 1                  -- data fifo depth, has to be a power of two, min 1
    )
    port map (
      -- Global control --
      clk_i       => clk,       -- global clock, rising edge
      rstn_i      => rstn_i,      -- global reset, low-active, async
      -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
      jtag_trst_i => '1', -- low-active TAP reset (optional)
      jtag_tck_i  => '1',  -- serial clock
      jtag_tdi_i  => '1',  -- serial data input
      jtag_tdo_o  => open,  -- serial data output
      jtag_tms_i  => '1',  -- mode select
      -- GPIO (available if IO_GPIO_NUM > 0) --
      gpio_o      => con_gpio_o,  -- parallel output
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o => uart_tx, -- UART0 send data
      uart0_rxd_i => uart_rx,  -- UART0 receive data
      -- SPI
      spi_clk_o   => spi_clk_o,
      spi_dat_o   => spi_dat_o,
      spi_dat_i   => spi_dat_i,
      spi_csn_o   => spi_csn_o,
      -- TWI (available if IO_TWI_EN = true) --
      twi_sda_i => twi_sda_i,
      twi_sda_o => twi_sda_o,
      twi_scl_i => twi_scl_i,
      twi_scl_o => twi_scl_o
    );


end Behavioral;
