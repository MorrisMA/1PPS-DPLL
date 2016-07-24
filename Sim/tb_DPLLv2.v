////////////////////////////////////////////////////////////////////////////////
//
//  1PPS DPLL Project
//
//  Copyright (C) 2008-2016  Michael A. Morris
//
//  All rights reserved. The source code contained herein is publicly released
//  under the terms and conditions of the GNU General Public License as conveyed
//  in the license provided below.
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along with
//  this program.  If not, see <http://www.gnu.org/licenses/>, or write to
//
//  Free Software Foundation, Inc.
//  51 Franklin Street, Fifth Floor
//  Boston, MA  02110-1301 USA
//
//  Further, no use of this source code is permitted in any form or means
//  without inclusion of this banner prominently in any derived works.
//
//  Michael A. Morris <morrisma_at_mchsi_dot_com>
//  164 Raleigh Way
//  Huntsville, AL 35811
//  USA
//
////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company:         M. A. Morris & Associates
// Engineer:        Michael A. Morris
// 
// Create Date:     06:48:17 06/29/2008
// Design Name:     DPLL.v
// Module Name:     C:/XProjects/ISE10.1i/0403DPLL/tb_DPLLv2.v
// Project Name:    0403DPLL
// Target Devices:  XC2S30-5VQ100I
// Tool versions:   ISE 10.1i SP3
//
// Description: This module provides a testbench for the DPLL - Digital 
//              Phase-Locked Loop module. Two clocks sources are provided in
//              module. One for the system clock, Clk, arbitrarily set to 
//              100 MHz, and the second set as a 1000 PPS external reference.
//              The actual period of the xPPS reference is intended to cause
//              default frequency of the DDS to advance over a number of 
//              periods.
//
// Verilog Test Fixture created by ISE for module: DPLL
//
// Dependencies:    redet.v
// 
// Revision History: 
//
//  0.00    08F29   MAM     File Created
//
// Additional Comments: 
//
////////////////////////////////////////////////////////////////////////////////

module tb_DPLLv2;

parameter pAlpha      =  1;      // NCO Loop Filter Time Constant
parameter pErrCntrLen = 16;      // Phase Error Counter Length

	// Inputs
	reg Rst;
	reg Clk;
	reg xPPS_In;

	// Outputs
	wire DPLL_En;
	wire xPPS_Out;
	wire iPPS_Out;
	wire CE_NCO;
	wire NCO_Out;
    wire xPPS_TFF;
    wire iPPS_TFF;
    wire PPSGate;
    wire Lock;
    
    wire [23:0] MissingPlsCntr;
    wire xPPS_Missing;
    
    wire [23:0] iPPS_Cntr;
    
    wire Up;
    wire Dn;
    wire [31:0] PhiErr;
    wire [31:0] Err;
    
    wire [(pErrCntrLen - 1):0] ErrCntr;
    wire ErrLim;
    wire [ 3:0] LockCntr;

    wire [31:0] Kphi;
    wire [31:0] NCODrv;
    wire [31:0] NCO;

	// Instantiate the Unit Under Test (UUT)
	DPLLv2  #(
//                .pPhRefCnt(24'd10_000),             // 1 kHz PPS rate
//                .pMissingPPS_Cnt(24'd15_000),       // missing pulse
//                .pPhRefCnt(24'd16_777),             // 1 kHz PPS rate
//                .pMissingPPS_Cnt(24'd25_165),       // missing pulse
//                .pPhRefCnt(24'd100_000),            // 100 Hz PPS rate
//                .pMissingPPS_Cnt(24'd150_000),      // missing pulse
                .pPhRefCnt(24'd10_000),            // 100 Hz PPS rate
                .pMissingPPS_Cnt(24'd15_000),      // missing pulse
//                .pPhRefCnt(24'd41_943),             // 100 Hz PPS rate
//                .pMissingPPS_Cnt(24'd62_915),       // missing pulse
//                .pPhRefCnt(24'd49_152),             // 100 Hz PPS rate
//                .pMissingPPS_Cnt(24'd82_084),       // missing pulse
//
//                .pBasePhaseIncrement(32'h1999_9999),// 10MHz@100MHz
//                .pPosFreqPhiBase(32'h0000_A7C6),    // 1 cycle, 1@100 MHz, 1kHz
//                .pNegFreqPhiBase(32'hFFFF_583A),    //-1 cycle, 1@100 MHz, 1kHz
//
//                .pBasePhaseIncrement(32'h2AF3_1DC4),// 16.777216MHz@100MHz
//                .pPosFreqPhiBase(32'h0000_A7C6),    // 1 cycle, 1@100 MHz, 1kHz
//                .pNegFreqPhiBase(32'hFFFF_583A),    //-1 cycle, 1@100 MHz, 1kHz
//
//                .pBasePhaseIncrement(32'h1999_9999),// 10MHz@100MHz
//                .pPosFreqPhiBase(32'h0000_1C06),    // 1 cycle, 1@100 MHz, 100 Hz
//                .pNegFreqPhiBase(32'hFFFF_F3FA),    //-1 cycle, 1@100 MHz, 100 Hz
//
                .pBasePhaseIncrement(32'h0555_5555),// 1MHz@48MHz
                .pPosFreqPhiBase(32'h0000_22F3),    // 1 cycle, 1@48 MHz, 100 Hz
                .pNegFreqPhiBase(32'hFFFF_DD0D),    //-1 cycle, 1@48 MHz, 100 Hz
//
//                .pBasePhaseIncrement(32'h165E_9F80),// 4.194304MHz@48MHz
//                .pPosFreqPhiBase(32'h0000_22F3),    // 1 cycle, 1@48 MHz, 100 Hz
//                .pNegFreqPhiBase(32'hFFFF_DD0D),    //-1 cycle, 1@48 MHz, 100 Hz
//                
                .pAlpha(pAlpha),                    // Loop Filter Configuration
                .pErrCntrLen(pErrCntrLen)           // Phase Error Counter Len
            ) uut (
                .Rst(Rst), 
                .Clk(Clk), 
                .xPPS_In(xPPS_In), 
                .DPLL_En(DPLL_En), 
                .xPPS_Out(xPPS_Out), 
                .iPPS_Out(iPPS_Out), 
                .CE_NCO(CE_NCO), 
                .NCO_Out(NCO_Out), 
                .xPPS_TFF(xPPS_TFF),
                .iPPS_TFF(iPPS_TFF),
                .PPSGate(PPSGate), 
                .Lock(Lock),

                .MissingPlsCntr(MissingPlsCntr),
                .xPPS_Missing(xPPS_Missing),
                
                .iPPS_Cntr(iPPS_Cntr),

                .Up(Up),
                .Dn(Dn),
                .Err(Err),
                .PhiErr(PhiErr),

                .ErrCntr(ErrCntr),
                .ErrLim(ErrLim),
                .LockCntr(LockCntr),

                .Kphi(Kphi),
                .NCODrv(NCODrv),
                .NCO(NCO)
            );

	initial begin
		// Initialize Inputs
		Rst = 1;
		Clk = 0;
		xPPS_In = 0;

		// Wait 100 ns for global reset to finish
		#101 Rst = 0;
        
		// Add stimulus here
        
	end
    
//    always #5 Clk = ~Clk;         // 100 MHz
    always #10.416 Clk = ~Clk;      //  48 MHz
    
    always begin
        #200    xPPS_In = 1;
        #100000 xPPS_In = 0;
//        #699800 xPPS_In = 0;
//        #738660.8 xPPS_In = 0;
//        #7899800 xPPS_In = 0;
        #9899800 xPPS_In = 0;
    end

endmodule

