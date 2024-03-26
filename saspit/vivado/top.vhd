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
         
         spi_ss   : out std_logic_vector (1 downto 0);
         -- spi0_sck  : out std_logic;  -- spi_sck is connected to the USRCCLKO port of the STARTUPE2 primitive
         spi0_mosi : out std_logic;
         spi0_miso : in  std_logic;
         spi1_sck  : out std_logic;
         spi1_mosi : out std_logic;
         spi1_miso : in  std_logic;
         
         sd_sck : out std_logic;
         sd_cdn : in std_logic;
         sd_sdi : out std_logic;
         sd_sdo : in std_logic;
         sd_csn : out std_logic;
         sd_reset : out std_logic;
         
         uart_tx : out std_logic;
         uart_rx : in std_logic );
end top;

architecture Behavioral of top is
    
    -- active high reset signal
    signal rst : std_logic;
    
    -- gpio
    signal con_gpio_o : std_ulogic_vector(63 downto 0);
    signal gpio_o : std_logic_vector (7 downto 0);
    
    -- i2c
    signal twi_sda_i, twi_sda_o, twi_scl_i, twi_scl_o : std_ulogic;
    signal spi_clk_o, spi_dat_o, spi_dat_i : std_ulogic;
    signal spi_csn_o : std_ulogic_vector(07 downto 0);
    signal spi0_sck : std_logic;
    
    -- sd card detect
    signal sd_cd : std_logic;
    
    -- wb4 --------------------------------------------------------------
    -- wb4 std_logic
    signal wbm_addr : std_logic_vector (1 downto 0);
    signal wbs_data, wbm_data : std_logic_vector (31 downto 0);
    signal wbm_we, wbm_stb, wbm_cyc, wbs_ack : std_logic;
    signal wbm_sel : std_logic_vector (3 downto 0);
    
    -- wb4 std_ulogic
    signal wb_adr_o, wb_dat_o, wb_dat_i : std_ulogic_vector (31 downto 0);
    signal wb_we_o, wb_stb_o, wb_cyc_o, wb_ack_i : std_ulogic; 
    signal wb_sel_o : std_ulogic_vector (3 downto 0);
    
    -- wb4 constants 
    -- The gateway redirects any bus access with address outside of the ranges for IMEM, DMEM, XIP, BOOT, IO to the external Wishbone bus
    -- One option for a device connected is therefore the address = DMEM_BASE + 0x4000_000 = 0xC000_0000. This leaves 1024 kByte room for DMEM.
    constant c_sdspi_addr_base : std_logic_vector(31 downto 0) := x"C000_0000";
    
    component sdspi is
    generic (
      OPT_CARD_DETECT : std_logic := '1';
      OPT_LITTLE_ENDIAN : std_logic := '1';
      LGFIFOLN : integer := 7;
      POWERUP_IDLE : integer := 1000;
      STARTUP_CLOCKS : integer := 75;
      CKDIV_BITS : integer := 8;
      INITIAL_CLKDIV : std_logic_vector(7 downto 0);
      OPT_SPI_ARBITRATION : std_logic := '0'
      );
    port(
      i_clk : in std_logic;
      i_sd_reset : in std_logic;
      
      i_wb_cyc : in std_logic;
      i_wb_stb : in std_logic;
      i_wb_we  : in std_logic;
      i_wb_addr : in std_logic_vector(1 downto 0);
      i_wb_data : in std_logic_vector(31 downto 0);
      i_wb_sel : in std_logic_vector(3 downto 0);
      o_wb_stall : out std_logic;
      o_wb_ack : out std_logic;
      o_wb_data : out std_logic_vector(31 downto 0);
      
      o_cs_n : out std_logic;
      o_sck : out std_logic;
      o_mosi : out std_logic;
      i_miso : in std_logic;
      i_card_detect : in std_logic;
      
      o_int : out std_logic;
      i_bus_grant : in std_logic;
      o_debug : out std_logic_vector(31 downto 0)
      );
    end component;

begin
    
    -- Signal Assignments for Processor interfaces
    -- -------------------------------------------
    -- SPI
    spi_ss <= std_logic(spi_csn_o(1)) & std_logic(spi_csn_o(0));
    spi0_sck <= std_logic(spi_clk_o);
    spi1_sck <= std_logic(spi_clk_o);
    spi0_mosi <= std_logic(spi_dat_o);
    spi1_mosi <= std_logic(spi_dat_o);
    spi_dat_i <= std_ulogic(spi0_miso) when spi_csn_o(0) = '0' else 
                 std_ulogic(spi1_miso) when spi_csn_o(1) = '0' else '0';
    
    -- TWI/I2C
    twi_sda       <= '0' when (twi_sda_o = '0') else 'Z'; -- drive
    twi_scl       <= '0' when (twi_scl_o = '0') else 'Z'; -- drive
    twi_sda_i <= std_ulogic(twi_sda); -- sense
    twi_scl_i <= std_ulogic(twi_scl); -- sense
    
    -- GPIO output to LD0-LD7
    led_o <= std_logic_vector(con_gpio_o(7 downto 0));
    
    -- sd card detect
    sd_cd <= not sd_cdn;
    -- Wishbone interface to sdspi module
    --  (for now: address translation and conversion between std_logic, std_ulogic
    --  later: wishbone interconnect module to attach several slaves to wb bus at different addresses)
    wbm_data <= std_logic_vector(wb_dat_o);
    wbm_we <= std_logic(wb_we_o);
    wbm_stb <= std_logic(wb_stb_o);
    wbm_cyc <= std_logic(wb_cyc_o);
    wbm_sel <= std_logic_vector(wb_sel_o); -- BYTE ENABLE ALL ONES!
    wb_dat_i <= std_ulogic_vector(wbs_data);
    wb_ack_i <= std_ulogic(wbs_ack);
    wbm_addr <= std_logic_vector(wb_adr_o(3 downto 2)); -- no other device connected to wb as yet

    
    
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
      -- External memory interface (WISHBONE) --
      MEM_EXT_EN                 => true,              -- implement external memory bus interface?
      MEM_EXT_TIMEOUT            => 41,               -- cycles after a pending bus access auto-terminates (0 = disabled)
      MEM_EXT_PIPE_MODE          => true,             -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
      MEM_EXT_BIG_ENDIAN         => false,             -- byte order: true=big-endian, false=little-endian
      MEM_EXT_ASYNC_RX           => false,             -- use register buffer for RX data when false
      MEM_EXT_ASYNC_TX           => false,             -- use register buffer for TX data when false      
      -- Processor peripherals --
      IO_GPIO_NUM                => 8,                 -- number of GPIO input/output pairs (0..64)
      IO_MTIME_EN                => true,              -- implement machine system timer (MTIME)?
      IO_UART0_EN                => true,              -- implement primary universal asynchronous receiver/transmitter (UART0)?
      IO_UART0_RX_FIFO           => 1,                 -- RX fifo depth, has to be a power of two, min 1
      IO_UART0_TX_FIFO           => 1,                 -- TX fifo depth, has to be a power of two, min 1
      IO_UART1_EN                => false,             -- implement secondary universal asynchronous receiver/transmitter (UART1)?
      IO_UART1_RX_FIFO           => 1,                 -- RX fifo depth, has to be a power of two, min 1
      IO_UART1_TX_FIFO           => 1,                 -- TX fifo depth, has to be a power of two, min 1
      IO_SPI_EN                  => true,              -- implement serial peripheral interface (SPI)?
      IO_SPI_FIFO                => 2,                 -- RTX fifo depth, has to be a power of two, min 1
      IO_SDI_EN                  => false,             -- implement serial data interface (SDI)?
      IO_SDI_FIFO                => 1,                 -- RTX fifo depth, has to be zero or a power of two, min 1
      IO_TWI_EN                  => true,              -- implement two-wire interface (TWI)?
      IO_PWM_NUM_CH              => 0,                 -- number of PWM channels to implement (0..12); 0 = disabled
      IO_WDT_EN                  => false,             -- implement watch dog timer (WDT)?
      IO_TRNG_EN                 => true,              -- implement true random number generator (TRNG)?
      IO_TRNG_FIFO               => 1,                 -- data fifo depth, has to be a power of two, min 1
      IO_GPTMR_EN                => true              -- implement general purpose timer (GPTMR)?
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
      -- Wishbone bus interface (available if MEM_EXT_EN = true) --
      wb_tag_o    => open,
      wb_adr_o    => wb_adr_o, -- has to be resolved downto 2 bits
      wb_dat_i    => wb_dat_i,
      wb_dat_o    => wb_dat_o,
      wb_we_o     => wb_we_o,
      wb_sel_o    => wb_sel_o,
      wb_stb_o    => wb_stb_o,
      wb_cyc_o    => wb_cyc_o,
      wb_ack_i    => wb_ack_i,
      wb_err_i    => '0',
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
    
    rst <= not rstn_i;
    sd_reset <= rst;
    sdspi_inst : sdspi
    generic map (
      OPT_CARD_DETECT => '1',
      OPT_LITTLE_ENDIAN => '1',
      LGFIFOLN => 7,
      POWERUP_IDLE => 1000,
      STARTUP_CLOCKS => 75,
      CKDIV_BITS => 8,
      INITIAL_CLKDIV => x"7C",
      OPT_SPI_ARBITRATION => '0')
    port map (
      i_clk => clk,
      i_sd_reset => rst, -- todo: is this ok?
      
      i_wb_cyc => wbm_cyc,
      i_wb_stb => wbm_stb,
      i_wb_we  => wbm_we,
      i_wb_addr => wbm_addr,
      i_wb_data => wbm_data,
      i_wb_sel => "1111",
      o_wb_stall => open,
      o_wb_ack => wbs_ack,
      o_wb_data => wbs_data,
      
      o_cs_n => sd_csn,
      o_sck => sd_sck,
      o_mosi => sd_sdi,
      i_miso => sd_sdo,
      i_card_detect => sd_cd,
      
      o_int => open,
      i_bus_grant => '1');
    
       STARTUPE2_inst : STARTUPE2
       generic map (
          PROG_USR => "FALSE",  -- Activate program event security feature. Requires encrypted bitstreams.
          SIM_CCLK_FREQ => 0.0  -- Set the Configuration Clock Frequency(ns) for simulation.
       )
       port map (
          CFGCLK => open,       -- 1-bit output: Configuration main clock output
          CFGMCLK => open,     -- 1-bit output: Configuration internal oscillator clock output
          EOS => open,             -- 1-bit output: Active high output signal indicating the End Of Startup.
          PREQ => open,           -- 1-bit output: PROGRAM request to fabric output
          CLK => '0',             -- 1-bit input: User start-up clock input -- TODO: IS CONNECTING TO LOW OK?
          GSR => '0',             -- 1-bit input: Global Set/Reset input (GSR cannot be used for the port name)
          GTS => '0',             -- 1-bit input: Global 3-state input (GTS cannot be used for the port name)
          KEYCLEARB => '0', -- 1-bit input: Clear AES Decrypter Key input from Battery-Backed RAM (BBRAM)
          PACK => '0',           -- 1-bit input: PROGRAM acknowledge input
          USRCCLKO => spi0_sck,   -- 1-bit input: User CCLK input
                                  -- For Zynq-7000 devices, this input must be tied to GND
          USRCCLKTS => '0', -- 1-bit input: User CCLK 3-state enable input
                                  -- For Zynq-7000 devices, this input must be tied to VCC
          USRDONEO => '1',   -- 1-bit input: User DONE pin output control  -- TODO: FIND OUT BEST VALUE HERE
          USRDONETS => '1'  -- 1-bit input: User DONE 3-state enable output -- TODO: FIND OUT BEST VALUE HERE
       );
       
end Behavioral;
