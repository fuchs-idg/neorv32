-- #################################################################################################
-- # << NEORV32 - RISC-V Debug Transport Module (DTM) >>                                           #
-- # ********************************************************************************************* #
-- # Provides a JTAG-compatible TAP to access the DMI register interface.                          #
-- # Compatible to the RISC-V debug specification version 1.0.                                     #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # https://github.com/stnolting/riscv-debug-dtm                              (c) Stephan Nolting #
-- #################################################################################################
----------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------
-- This is a quick-and-dirty adaptation of the NeoRV32 Debug Transfer Module to employ Xilinx 
-- 7-Series BSCANE2 Primitives for access to the data registers necessary for RISC-V debugging.
-- This enables neorv32 OpenOCD/GDB debugging through the same USB port used for the FPGA 
-- bitstream transfer instead of having to connect a JTAG adapter cable to the FPGA board.
--
-- A drawback of this method is that simultaneous debugging of both the PL and the RISC-V core is 
-- not possible.
----------------------------------------------------------------------------------------------------
-- ## OpenOCD configuration for use with the Nexys Video Board (via FT2232H, Channel 1) ------------
-- adapter driver ftdi
-- ftdi vid_pid 0x0403 0x6010
-- ftdi channel 1
-- ftdi layout_init 0x0088 0x008b
-- reset_config none
--
-- adapter speed 10000
-- transport select jtag
--
-- set _CHIPNAME neorv32
-- jtag newtap $_CHIPNAME cpu -irlen 6
--
-- set _TARGETNAME $_CHIPNAME.cpu
-- target create $_TARGETNAME.0 riscv -chain-position $_TARGETNAME
--
-- ## IDCODE reg in fabric:
-- # riscv set_ir idcode 0x02
-- ## native FPGA IDCODE reg: 
-- riscv set_ir idcode 0x09
-- riscv set_ir dtmcs 0x22
-- riscv set_ir dmi 0x23
--
-- riscv set_mem_access progbuf
--
-- riscv expose_csrs 2048=cfusel
-- riscv expose_csrs 2049=cfureg
-- riscv expose_csrs 4032=mxisa
--
-- gdb_report_data_abort enable
--
-- init
-- halt
----------------------------------------------------------------------------------------------------
-- If you have an FPGA board that uses another FTDI chip, you can use this OpenOCD configuration
-- but you must adapt the ftdi adapter configuration.
-- In case of an FT2232H, you only have to take note which channel is connected to the FPGA.
-- Using a VHDL generic, you can choose between the neorv32's or the FPGA's IDCODE register.
--
-- Adaptation by Philipp Fuchs (Ingenics Digital GmbH).
-- With inspiration and help from:
--  https://jsteward.moe/risc-v-hardware-design-part-b-edgeboard-series.html
--  https://github.com/pulp-platform/riscv-dbg/blob/master/src/dmi_bscane_tap.sv
--  https://github.com/riscv/riscv-openocd/blob/riscv/src/target/riscv/riscv.c
--  https://docs.xilinx.com/r/en-US/ug953-vivado-7series-libraries/BSCANE2
--  https://docs.xilinx.com/v/u/en-US/ug470_7Series_Config
--  https://github.com/stnolting/neorv32/issues/567
----------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library UNISIM;
use UNISIM.VComponents.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_debug_dtm_bscane2 is
    generic (
    IDCODE_VERSION : std_ulogic_vector(03 downto 0); -- version
    IDCODE_PARTID  : std_ulogic_vector(15 downto 0); -- part number
    IDCODE_MANID   : std_ulogic_vector(10 downto 0);  -- manufacturer id
    RISCV_IDCODE_REGISTER : boolean := false -- false = use FPGAs own idcode dr (at 0x09 on 7-series devices)
  );
  port (
    -- global control --
    clk_i       : in  std_ulogic; -- global clock line
    rstn_i      : in  std_ulogic; -- global reset line, low-active
    -- NOT USED:
    jtag_trst_i : in  std_ulogic;
    jtag_tck_i  : in  std_ulogic;
    jtag_tdi_i  : in  std_ulogic;
    jtag_tdo_o  : out std_ulogic;
    jtag_tms_i  : in  std_ulogic;
    -- debug module interface (DMI) --
    dmi_req_o   : out dmi_req_t; -- request
    dmi_rsp_i   : in  dmi_rsp_t  -- response
  );
end neorv32_debug_dtm_bscane2;

architecture Behavioral of neorv32_debug_dtm_bscane2 is

    -- DMI Configuration (fixed!) --
    constant dmi_idle_c    : std_ulogic_vector(02 downto 0) := "000";    -- no idle cycles required
    constant dmi_version_c : std_ulogic_vector(03 downto 0) := "0001";   -- debug spec. version (0.13 & 1.0)
    constant dmi_abits_c   : std_ulogic_vector(05 downto 0) := "000111"; -- number of DMI address bits (7)

    signal jtag_tck, jtag_tms, jtag_tdi : std_logic;
    signal tap_capture,  tap_runtest,  tap_shift, tap_update : std_logic;
    signal dmi_drck, dtmcs_drck, idcode_drck, dtmcs_sel, idcode_sel, dmi_sel : std_logic;
    
    -- synchronizer signals for general JTAG and TAG signals taken from any one BSCANE2 primitve
    type tap_sync_t is record
        -- internal --
        -- jtag signals --
        --trst_ff     : std_ulogic_vector(2 downto 0);
        tck_ff      : std_ulogic_vector(2 downto 0);
        tdi_ff      : std_ulogic_vector(2 downto 0);
        tms_ff      : std_ulogic_vector(2 downto 0);
        -- tap status signals --
        capture_ff      : std_ulogic_vector(2 downto 0);
        runtest_ff      : std_ulogic_vector(2 downto 0);
        shift_ff        : std_ulogic_vector(2 downto 0);
        update_ff       : std_ulogic_vector(2 downto 0);
        -- external --
        -- jtag signals --
        --trst        : std_ulogic;
        tck_rising  : std_ulogic;
        tck_falling : std_ulogic;
        tdi         : std_ulogic;
        tms         : std_ulogic;
        -- tap status signals --
        capture         : std_ulogic;
        runtest         : std_ulogic;
        shift_datareg   : std_ulogic;
        update          : std_ulogic;
        update_rising   : std_ulogic;
    end record;
    signal tap_sync : tap_sync_t;
    
    -- synchronizer signals signals specific to each instantiated BSCANE2 primitive
    type bscane2_sync_t is record
        -- internal --
        drck_ff         : std_ulogic_vector(2 downto 0);
        sel_ff          : std_ulogic_vector(2 downto 0);
        -- external --
        drck_rising     : std_ulogic;
        drck_falling    : std_ulogic;
        sel             : std_ulogic;
    end record;
    signal idcode_sync, dtmcs_sync, dmi_sync : bscane2_sync_t;
    signal idcode_tdo, dtmcs_tdo, dmi_tdo : std_logic;
    
      -- dtm data registers --
    type tap_reg_t is record
        -- ireg             : std_ulogic_vector(04 downto 0);
        -- bypass           : std_ulogic;
        idcode           : std_ulogic_vector(31 downto 0);
        dtmcs, dtmcs_nxt : std_ulogic_vector(31 downto 0);
        dmi,   dmi_nxt   : std_ulogic_vector((7+32+2)-1 downto 0); -- 7-bit address + 32-bit data + 2-bit operation
    end record;
    signal tap_reg : tap_reg_t;
    
      -- debug module interface controller --
    type dmi_ctrl_t is record
        busy         : std_ulogic;
        op           : std_ulogic_vector(01 downto 0);
        dmihardreset : std_ulogic;
        dmireset     : std_ulogic;
        err          : std_ulogic;
        rdata        : std_ulogic_vector(31 downto 0);
        wdata        : std_ulogic_vector(31 downto 0);
        addr         : std_ulogic_vector(06 downto 0);
    end record;
    signal dmi_ctrl : dmi_ctrl_t;
    
begin

    -- Synchronization logic for general JTAG Signals from the FPGA's tap
    -- ------------------------------------------------------------------
    tap_synchronizer: process(rstn_i, clk_i)
    begin
        if (rstn_i = '0') then
          tap_sync.tck_ff  <= (others => '0');
          tap_sync.tdi_ff  <= (others => '0');
          tap_sync.tms_ff  <= (others => '0');
          tap_sync.runtest_ff  <= (others => '0');
          tap_sync.capture_ff  <= (others => '0');
          tap_sync.shift_ff  <= (others => '0');
          tap_sync.update_ff  <= (others => '0');
        elsif rising_edge(clk_i) then
          tap_sync.tck_ff  <= tap_sync.tck_ff( 1 downto 0) & jtag_tck;
          tap_sync.tdi_ff  <= tap_sync.tdi_ff( 1 downto 0) & jtag_tdi;
          tap_sync.tms_ff  <= tap_sync.tms_ff( 1 downto 0) & jtag_tms;
          tap_sync.runtest_ff  <= tap_sync.runtest_ff( 1 downto 0) & tap_runtest;
          tap_sync.capture_ff  <= tap_sync.capture_ff( 1 downto 0) & tap_capture;
          tap_sync.update_ff  <= tap_sync.update_ff( 1 downto 0) & tap_update;
          tap_sync.shift_ff  <= tap_sync.shift_ff( 1 downto 0) & tap_shift;      
        end if;
    end process tap_synchronizer;
    
    -- JTAG clock edge --
    tap_sync.tck_rising  <= '1' when (tap_sync.tck_ff(2 downto 1) = "01") else '0';
    tap_sync.tck_falling <= '1' when (tap_sync.tck_ff(2 downto 1) = "10") else '0';
    
    -- JTAG test mode select --
    tap_sync.tms <= tap_sync.tms_ff(2);
    
    -- JTAG serial data input --
    tap_sync.tdi <= tap_sync.tdi_ff(2);
    
    -- FPGA TAP Status signals --
    tap_sync.capture <= tap_sync.capture_ff(2);
    tap_sync.shift_datareg <= tap_sync.shift_ff(2);
    tap_sync.update <= tap_sync.update_ff(2);
    
    -- replacement for the update trigger logic
    tap_sync.update_rising <= '1' when (tap_sync.update_ff(2 downto 1) = "01") else '0';
    
    -- Tap Register Access
    -- -------------------
    reg_access: process(rstn_i, clk_i)
    begin
    if (rstn_i = '0') then
      tap_reg.idcode <= (others => '0');
      tap_reg.dtmcs  <= (others => '0');
      tap_reg.dmi    <= (others => '0');
    elsif rising_edge(clk_i) then
    
      -- serial data input: data register selection via BSCANE2 'sel' signals
      if idcode_sync.sel = '1' then
        if tap_sync.capture = '1' then
            tap_reg.idcode <= IDCODE_VERSION & IDCODE_PARTID & IDCODE_MANID & '1';
        elsif tap_sync.shift_datareg = '1' then
          if tap_sync.tck_rising = '1' then
            tap_reg.idcode <= tap_sync.tdi & tap_reg.idcode(tap_reg.idcode'left downto 1);
          end if;
        end if;
      end if;
      
      if dtmcs_sync.sel = '1' then
        if tap_sync.capture = '1' then
            tap_reg.dtmcs  <= tap_reg.dtmcs_nxt; -- status register
        elsif tap_sync.shift_datareg = '1' then
            if tap_sync.tck_rising = '1' then
                tap_reg.dtmcs  <= tap_sync.tdi & tap_reg.dtmcs(tap_reg.dtmcs'left downto 1);
            end if;
        end if;
      end if;  
      
      if dmi_sync.sel = '1' then
        if tap_sync.capture = '1' then
            tap_reg.dmi    <= tap_reg.dmi_nxt; -- register interface
        elsif tap_sync.shift_datareg = '1' then
            if (tap_sync.tck_rising = '1') then
                tap_reg.dmi    <= tap_sync.tdi & tap_reg.dmi(tap_reg.dmi'left downto 1);
            end if;
        end if;
      end if; 
    
    end if;
    end process reg_access;
    
    -- DTM Control and Status Register (dtmcs) --
    tap_reg.dtmcs_nxt(31 downto 18) <= (others => '0'); -- reserved
    tap_reg.dtmcs_nxt(17)           <= dmi_ctrl.dmihardreset; -- dmihardreset
    tap_reg.dtmcs_nxt(16)           <= dmi_ctrl.dmireset; -- dmireset
    tap_reg.dtmcs_nxt(15)           <= '0'; -- reserved
    tap_reg.dtmcs_nxt(14 downto 12) <= dmi_idle_c; -- minimum number of idle cycles
    tap_reg.dtmcs_nxt(11 downto 10) <= tap_reg.dmi_nxt(1 downto 0); -- dmistat
    tap_reg.dtmcs_nxt(09 downto 04) <= dmi_abits_c; -- number of DMI address bits
    tap_reg.dtmcs_nxt(03 downto 00) <= dmi_version_c; -- version

    -- DMI register read access --
    tap_reg.dmi_nxt(40 downto 34) <= dmi_ctrl.addr; -- address
    tap_reg.dmi_nxt(33 downto 02) <= dmi_ctrl.rdata; -- read data
    tap_reg.dmi_nxt(01 downto 00) <= (others => dmi_ctrl.err); -- status
    
    -- Debug Module Interface
    -- ----------------------
    dmi_controller: process(rstn_i, clk_i)
    begin
    if (rstn_i = '0') then
      dmi_ctrl.busy         <= '0';
      dmi_ctrl.op           <= "00";
      dmi_ctrl.dmihardreset <= '1';
      dmi_ctrl.dmireset     <= '0';
      dmi_ctrl.err          <= '0';
      dmi_ctrl.rdata        <= (others => '0');
      dmi_ctrl.wdata        <= (others => '0');
      dmi_ctrl.addr         <= (others => '0');
    elsif rising_edge(clk_i) then
    
      -- DMI reset control --
      if (tap_sync.update_rising = '1') and (dtmcs_sync.sel = '1') then -- should this be dtmcs.sel or dmi_sync.sel?
        dmi_ctrl.dmireset     <= tap_reg.dtmcs(16);
        dmi_ctrl.dmihardreset <= tap_reg.dtmcs(17);
      elsif (dmi_ctrl.busy = '0') then
        dmi_ctrl.dmihardreset <= '0';
        dmi_ctrl.dmireset     <= '0';
      end if;
    
      -- sticky error --
      if (dmi_ctrl.dmireset = '1') or (dmi_ctrl.dmihardreset = '1') then
        dmi_ctrl.err <= '0';
      elsif (dmi_ctrl.busy = '1') and (tap_sync.update_rising = '1') and (dmi_sync.sel = '1') then -- access attempt while DMI is busy
        dmi_ctrl.err <= '1';
      end if;
    
      -- DMI interface arbiter --
      dmi_ctrl.op <= dmi_req_nop_c; -- default
      if (dmi_ctrl.busy = '0') then -- idle: waiting for new request
    
        if (dmi_ctrl.dmihardreset = '0') then -- no DMI hard reset
          if (tap_sync.update_rising = '1') and (dmi_sync.sel = '1') then
            dmi_ctrl.addr  <= tap_reg.dmi(40 downto 34);
            dmi_ctrl.wdata <= tap_reg.dmi(33 downto 02);
            if (tap_reg.dmi(1 downto 0) = dmi_req_rd_c) or (tap_reg.dmi(1 downto 0) = dmi_req_wr_c) then
              dmi_ctrl.op   <= tap_reg.dmi(1 downto 0);
              dmi_ctrl.busy <= '1';
            end if;
          end if;
        end if;
    
        else -- busy: read/write access in progress
    
          dmi_ctrl.rdata <= dmi_rsp_i.data;
          if (dmi_rsp_i.ack = '1') then
            dmi_ctrl.busy <= '0';
          end if;
    
      end if;
    end if;
    end process dmi_controller;
    
    -- direct DMI output --
    dmi_req_o.op   <= dmi_ctrl.op;
    dmi_req_o.data <= dmi_ctrl.wdata;
    dmi_req_o.addr <= dmi_ctrl.addr;
   

    -- Synchronization Logic for BSCANE2 Signals
    -- -----------------------------------------
    -- USER1 : idcode
    g_idcode : if RISCV_IDCODE_REGISTER = true generate
        idcode_synchronizer: process(rstn_i, clk_i)
        begin
            if (rstn_i = '0') then
              idcode_sync.drck_ff  <= (others => '0');
              idcode_sync.sel_ff  <= (others => '0');
            elsif rising_edge(clk_i) then
              idcode_sync.drck_ff  <= idcode_sync.drck_ff( 1 downto 0) & idcode_drck;
              idcode_sync.sel_ff  <= idcode_sync.sel_ff( 1 downto 0) & idcode_sel;
            end if;
        end process idcode_synchronizer;
        
        idcode_sync.drck_rising  <= '1' when (idcode_sync.drck_ff(2 downto 1) = "01") else '0';
        idcode_sync.drck_falling <= '1' when (idcode_sync.drck_ff(2 downto 1) = "10") else '0';
        idcode_sync.sel <= idcode_sync.sel_ff(2);
    end generate;
    
    -- USER3 : dtmcs
    dtmcs_synchronizer: process(rstn_i, clk_i)
    begin
        if (rstn_i = '0') then
          dtmcs_sync.drck_ff  <= (others => '0');
          dtmcs_sync.sel_ff  <= (others => '0');
        elsif rising_edge(clk_i) then    
          dtmcs_sync.drck_ff  <= dtmcs_sync.drck_ff( 1 downto 0) & dtmcs_drck;
          dtmcs_sync.sel_ff  <= dtmcs_sync.sel_ff( 1 downto 0) & dtmcs_sel;
        end if;
    end process dtmcs_synchronizer;
    
    dtmcs_sync.drck_rising  <= '1' when (dtmcs_sync.drck_ff(2 downto 1) = "01") else '0';
    dtmcs_sync.drck_falling <= '1' when (dtmcs_sync.drck_ff(2 downto 1) = "10") else '0';
    dtmcs_sync.sel <= dtmcs_sync.sel_ff(2);
    
    -- USER4 : dmi
    dmi_synchronizer: process(rstn_i, clk_i)
    begin
        if (rstn_i = '0') then
          dmi_sync.drck_ff  <= (others => '0');
          dmi_sync.sel_ff  <= (others => '0');
        elsif rising_edge(clk_i) then
          dmi_sync.drck_ff  <= dmi_sync.drck_ff( 1 downto 0) & dmi_drck;
          dmi_sync.sel_ff  <= dmi_sync.sel_ff( 1 downto 0) & dmi_sel;
        end if;
    end process dmi_synchronizer;
    
    dmi_sync.drck_rising  <= '1' when (dmi_sync.drck_ff(2 downto 1) = "01") else '0';
    dmi_sync.drck_falling <= '1' when (dmi_sync.drck_ff(2 downto 1) = "10") else '0';
    dmi_sync.sel <= dmi_sync.sel_ff(2);
    
    -- BSCANE2 Instantiations
    -- ----------------------
    g_idcode_bscane2 : if RISCV_IDCODE_REGISTER = true generate
        idcode_data_register_access : BSCANE2
        generic map (
            JTAG_CHAIN => 1  -- Value for USER command.
        )
        port map (
            CAPTURE => open, -- 1-bit output: CAPTURE output from TAP controller.
            DRCK => idcode_drck,       -- 1-bit output: Gated TCK output. When SEL is asserted, DRCK toggles when CAPTURE or
                              -- SHIFT are asserted.
            RESET => open,     -- 1-bit output: Reset output for TAP controller.
            RUNTEST => open, -- 1-bit output: Output asserted when TAP controller is in Run Test/Idle state.
            SEL => idcode_sel,         -- 1-bit output: USER instruction active output.
            SHIFT => open,     -- 1-bit output: SHIFT output from TAP controller.
            TCK => open,         -- 1-bit output: Test Clock output. Fabric connection to TAP Clock pin.
            TDI => open,         -- 1-bit output: Test Data Input (TDI) output from TAP controller.
            TMS => open,         -- 1-bit output: Test Mode Select output. Fabric connection to TAP.
            UPDATE => open,   -- 1-bit output: UPDATE output from TAP controller
            TDO => tap_reg.idcode(0)          -- 1-bit input: Test Data Output (TDO) input for USER function.
        );
    end generate;
    g_no_idcode_bscane2 : if RISCV_IDCODE_REGISTER = false generate
        idcode_sel <= '0';
        idcode_drck <= '0';
    end generate;

    dtmcs_data_register_access : BSCANE2
    generic map (
        JTAG_CHAIN => 3  -- Value for USER command.
    )
    port map (
        CAPTURE => tap_capture, -- 1-bit output: CAPTURE output from TAP controller.
        DRCK => dtmcs_drck,       -- 1-bit output: Gated TCK output. When SEL is asserted, DRCK toggles when CAPTURE or
                          -- SHIFT are asserted.
        RESET => open,     -- 1-bit output: Reset output for TAP controller.
        RUNTEST => tap_runtest, -- 1-bit output: Output asserted when TAP controller is in Run Test/Idle state.
        SEL => dtmcs_sel,         -- 1-bit output: USER instruction active output.
        SHIFT => tap_shift,     -- 1-bit output: SHIFT output from TAP controller.
        TCK => jtag_tck,         -- 1-bit output: Test Clock output. Fabric connection to TAP Clock pin.
        TDI => jtag_tdi,         -- 1-bit output: Test Data Input (TDI) output from TAP controller.
        TMS => jtag_tms,         -- 1-bit output: Test Mode Select output. Fabric connection to TAP.
        UPDATE => tap_update,   -- 1-bit output: UPDATE output from TAP controller
        TDO => tap_reg.dtmcs(0)         -- 1-bit input: Test Data Output (TDO) input for USER function.
    );

    dmi_data_register_access : BSCANE2
    generic map (
        JTAG_CHAIN => 4  -- Value for USER command.
    )
    port map (
        CAPTURE => open, -- 1-bit output: CAPTURE output from TAP controller.
        DRCK => dmi_drck,       -- 1-bit output: Gated TCK output. When SEL is asserted, DRCK toggles when CAPTURE or
                          -- SHIFT are asserted.
        RESET => open,     -- 1-bit output: Reset output for TAP controller.
        RUNTEST => open, -- 1-bit output: Output asserted when TAP controller is in Run Test/Idle state.
        SEL => dmi_sel,         -- 1-bit output: USER instruction active output.
        SHIFT => open,     -- 1-bit output: SHIFT output from TAP controller.
        TCK => open,         -- 1-bit output: Test Clock output. Fabric connection to TAP Clock pin.
        TDI => open,         -- 1-bit output: Test Data Input (TDI) output from TAP controller.
        TMS => open,         -- 1-bit output: Test Mode Select output. Fabric connection to TAP.
        UPDATE => open,   -- 1-bit output: UPDATE output from TAP controller
        TDO => tap_reg.dmi(0)          -- 1-bit input: Test Data Output (TDO) input for USER function.
    );
    
end Behavioral;
