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
// Create Date:     19:41:24 06/25/2008
// Design Name:     1PPS Digital Phase-Locked Loop Direct Digital Synthesiser
// Module Name:     DPLLv2.v
// Project Name:    0403DPLL
// Target Devices:  XC2S30-5VQ100I
// Tool versions:   ISE 10.1i SP3
//
// Description:
//
//  This module implements a Digital Phase-Locked Loop (DPLL) based on the con-
//  cept that the output frequency can be generated as a Direct Digital Synthe-
//  sized (DDS) signal based on a phase comparator which is a counter that mea-
//  sures the frequency of the DDS output over a period of time controlled by an
//  external pulse.
//
//  In the case of application to which this module is targeted, the external
//  pulse is provided by a GPS 1PPS input signal. This module synchronizes that
//  external pulse to the DDS reference clock, and then computes an adjustment
//  to the DDS phase increment value. Unlike the previous version, this module
//  uses a Phase-Frequency Detector (PFD) to implement the phase comparator and
//  error signal generator. Parameterization is included to limit the error sig-
//  nal passed to the loop filter.
//
//  A Lock output is provided. The module considers Lock to have been achieved
//  when the magnitude of the error is less than ±15.
//
// Dependencies:    None
//
// Revision History:
//
//  1.00    14I21   MAM     Initial development of this version of the DPLL.
//
//  1.10    16G24   MAM     Modified some comments. Partially corrected logic to
//                          coast and re-lock following loss of xPPS_In signal.
//
//  2.00    16G24   MAM     Completed correction of logic to coast and re-lock 
//                          following loss of xPPS_In signal. Recovery logic
//                          different for DPLL using Phase-Frequency Detector.
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module DPLLv2 #(
    parameter pPhRefCnt           = 24'd10_000_000, // #cycles in interval
    parameter pMissingPPS_Cnt     = 24'd16_000_000, // #cyles for missing PPS
    //parameter pBasePhaseIncrement = 32'h5000_0000,  // 10MHz@32MHz
    parameter pBasePhaseIncrement = 32'h3555_5555,  // 10MHz@48MHz
    //parameter pBasePhaseIncrement = 32'h1999_9999,  // 10MHz@100MHz

    parameter pRefScaleFactor = 1,                  // Frequency Scale Factor

    //parameter pPosFreqPhiBase = 32'h0000_0086,      // 1 cycle, 1@32 MHz
    //parameter pNegFreqPhiBase = 32'hFFFF_FF7A,      //-1 cycle, 1@32 MHz
    parameter pPosFreqPhiBase = 32'h0000_0059,      // 1 cycle, 1@48 MHz
    parameter pNegFreqPhiBase = 32'hFFFF_FFA7,      //-1 cycle, 1@48 MHz
    //parameter pPosFreqPhiBase = 32'h0000_002B,     // 1 cycle, 1@100 MHz
    //parameter pNegFreqPhiBase = 32'hFFFF_FFD5,     //-1 cycle, 1@100 MHz

    parameter pAlpha       = 1,                     // NCO Filter Time Constant
    parameter pErrCntrLen  = 6                      // Phase Error Counter Len
)(
    input   Rst,                // System Reset
    input   Clk,                // System Clock, DDS Reference
    output  reg DPLL_En,        // DPLL Enable - enables xPPS_Input processing
    input   xPPS_In,            // External Unsynchronized 1PPS Input
    output  xPPS_Out,           // Synchronized External 1PPS Input
    output  iPPS_Out,           // Internal 1PPS Output
    output  reg CE_NCO,         // NCO Output Clock Enable Out
    output  NCO_Out,            // NCO Output Frequency
    output  reg xPPS_TFF,       // xPPS Toggle FF output
    output  reg iPPS_TFF,       // Internal PPS Toggle FF output
    output  reg PPSGate,        // PPS Select Gate
    output  reg Lock,           // DPLL Lock Indicator

    output  reg [23:0] MissingPlsCntr,  // Missing Pulse Counter
    output  reg xPPS_Missing,           // xPPS Missing Pulse FF
    
    output  reg [23:0] iPPS_Cntr,       // Internal PPS Counter

    output  reg Up,             // Phase/Frequency Detector (PFD) Up FF
    output  reg Dn,             // PFD Dn FF
    output  reg [31:0] Err,     // PFD Phase Error Correction Value
    output  reg [31:0] PhiErr,  // PFD Phase Adjustment Value

    output  reg [(pErrCntrLen - 1):0] ErrCntr,  // Phase Error Counter
    output  reg ErrLim,         // Allowable Phase Error Limit FF
    output  reg [3:0] LockCntr, // DPLL Lock Delay Cntr
    
    output  [31:0] Kphi,        // NCO Frequency Control Word (FCW)
    output  [31:0] NCODrv,      // NCO Loop Filter Output
    output  reg [31:0] NCO      // NCO Phase Register
);

////////////////////////////////////////////////////////////////////////////////
//
//  Module Local Signal Declarations
//

    reg     [1:0] dxPPS_In;             // xPPS_In sampled with CE10M_DDS
    reg     xPPS;                       // External synchronized 1PPS pulse

    wire    CE_DPLL_En;                 // DPLL Enable - Clock Enable

    wire    Rst_MissingPlsCntr;         // Missing Pulse Counter Control Signals
    wire    CE_MissingPlsCntr;

    wire    Rst_iPPS_Cntr;              // Reset iPPS Counter
    wire    TC_iPPS_Cntr;               // Terminal Count - iPPS Counter

    reg     iPPS;                       // iPPS pulse FF

    wire    [31:0] Sum;                 // DDS External Summer
    wire    iCE_NCO;                    // CE_NCO FF input signal
    
    wire    Rst_LockCntr;               // DPLL Lock Delay Cntr Control Signals
    wire    CE_LockCntr;
    wire    Dec_LockCntr;
    wire    TC_LockCntr;                // DPLL Lock Delay Cntr Terminal Count

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//
//  Synchronize External 1PPS pulse to CE_NCO

always @(posedge Clk)
begin
    if(Rst)
        dxPPS_In <= #1 0;
    else if(CE_NCO)
        dxPPS_In <= #1 {dxPPS_In[0], xPPS_In};
end

//  Generate Rising Edge Detection Pulse synchronized to CE_NCO

always @(posedge Clk)
begin
    if(Rst)
        xPPS <= #1 0;
    else
        xPPS <= #1 iCE_NCO & (dxPPS_In[0] & ~dxPPS_In[1]);
end

assign xPPS_Out = xPPS; // Instrument external 1PPS synchcronized pulse

//  Set Enable FF when first xPPS detected, Clr Enable FF when xPPS missing

assign CE_DPLL_En = (xPPS | xPPS_Missing | (Lock & Rst_LockCntr));

always @(posedge Clk)
begin
    if(Rst)
        DPLL_En <= #1 0;
    else if(CE_DPLL_En)
        DPLL_En <= #1 xPPS;
end

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Detect Missing xPPS pulses
//

assign Rst_MissingPlsCntr = Rst | xPPS | xPPS_Missing;
assign CE_MissingPlsCntr  = CE_NCO & DPLL_En;

always  @(posedge Clk)
begin
    if(Rst_MissingPlsCntr)
        MissingPlsCntr <= #1 (pMissingPPS_Cnt - 1);
    else if(CE_MissingPlsCntr)
        MissingPlsCntr <= #1 MissingPlsCntr - 1;
end

//  Set xPPS Missing FF

always @(posedge Clk)
begin
    if(Rst)
        xPPS_Missing <= #1 0;
    else if(CE_NCO)
        xPPS_Missing <= #1 ~|MissingPlsCntr;
end

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Generate a 1PPS signal based on the internal CE_NCO
//

assign Rst_iPPS_Cntr = (Rst | ((DPLL_En & ~PPSGate) ? CE_LockCntr : iPPS));

always @(posedge Clk)
begin
    if(Rst_iPPS_Cntr)
        iPPS_Cntr <= #1 (pPhRefCnt - 1);
    else if(CE_NCO)
        iPPS_Cntr <= #1 (iPPS_Cntr - 1);
end

assign TC_iPPS_Cntr = ~|iPPS_Cntr;

always @(posedge Clk)
begin
    if(Rst)
        iPPS <= #1 0;
    else
        iPPS <= #1 iCE_NCO & TC_iPPS_Cntr;
end

assign iPPS_Out = iPPS;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Implement Phase/Frequency Detector
//      if xPPS leads iPPS, NCO frequency too slow, speed DPLL up
//      if xPPS lags  iPPS, NCO frequency too fast, slow DPLL down

always @(posedge Clk)
begin
    if(Rst | ~DPLL_En)
//        {Up, Dn} <= #1 {(xPPS & PPSGate), 1'b0};
        {Up, Dn} <= #1 0;
    else if(DPLL_En)
        case({Up, Dn, xPPS, iPPS})
            4'b0000 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b0001 : {Up, Dn} <= #1 {1'b0, 1'b1};
            4'b0010 : {Up, Dn} <= #1 {1'b1, 1'b0};
            4'b0011 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b0100 : {Up, Dn} <= #1 {1'b0, 1'b1};
            4'b0101 : {Up, Dn} <= #1 {1'b0, 1'b1};
            4'b0110 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b0111 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b1000 : {Up, Dn} <= #1 {1'b1, 1'b0};
            4'b1001 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b1010 : {Up, Dn} <= #1 {1'b1, 1'b0};
            4'b1011 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b1100 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b1101 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b1110 : {Up, Dn} <= #1 {1'b0, 1'b0};
            4'b1111 : {Up, Dn} <= #1 {1'b0, 1'b0};
        endcase
end

//  Set the error value based on the value of the {Up, Dn} FFs

assign Rst_Err = (Rst | ~DPLL_En | (xPPS & Dn) | (iPPS & Up));
assign CE_Err  = (CE_NCO & ~ErrLim & ((xPPS | Up) | (iPPS | Dn)));

always @(posedge Clk)
begin
    if(Rst_Err)
        Err <= #1 0;
    else if(CE_Err)
        Err <= #1 Err + ((xPPS | Up) ? pPosFreqPhiBase
                                     : (iPPS | Dn) ? pNegFreqPhiBase : 0);
end

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Integrate/Accumulate phase error for NCO Frequecy Control Word
//

assign CE_PhiErr = (CE_NCO & ~ErrLim & ((xPPS & Dn) | (iPPS & Up)));

always @(posedge Clk)
begin
    if(Rst)
        PhiErr <= #1 0;
    else if(CE_PhiErr)
        PhiErr <= #1 PhiErr + Err;
end

//  Offset nominal NCO FCW by the accumulated/integrated Phase Error

assign Kphi = pBasePhaseIncrement + PhiErr;

//  Filter NCO FCW before applying to NCO (DDS)

assign CE_NCODrv = CE_NCO;

VCODriveFilter  #(
                    .pDefaultValue(pBasePhaseIncrement),
                    .pAlpha(pAlpha)
                ) NCODrv1 (
                    .Rst(Rst),
                    .Clk(Clk),
                    .CE(CE_NCODrv),
                    .In(Kphi),
                    .Out(NCODrv)
                );

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Numerically-Controlled Oscillator (DDS)
//      CE_NCO  <= Falling edge of the MSB
//      NCO_Out <= Complement of MSB of the NCO
//

assign Sum = NCO + NCODrv;

always @(posedge Clk)
begin
    if(Rst)
        NCO <= #1 0;
    else
        NCO <= #1 Sum;
end

assign iCE_NCO = (NCO[31] & ~Sum[31]);

always @(posedge Clk)
begin
    if(Rst)
        CE_NCO <= #1 0;
    else
        CE_NCO <= #1 iCE_NCO;
end

assign NCO_Out = ~NCO[31];   // Frequency equal to MSB of NCO Accumulator

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Develop a DPLL Lock Signal to indicate whether the DPLL is locked to the
//      external 1PPS input signal. Also generate a gate signal to be used to
//      select the external or internal 1PPS signal as the synchronization
//      signal for the time generator/accumulator

//  Measure the magnitude of the error between xPPS and iPPS

assign Rst_ErrCntr = (Rst | ~DPLL_En | (xPPS & iPPS));
assign Ld_ErrCntr  = (CE_NCO & (xPPS ^ iPPS) & (~Up & ~Dn));
assign CE_ErrCntr  = (CE_NCO & ~ErrLim & ((~xPPS & ~iPPS) & (Dn | Up)));

always @(posedge Clk)
begin
    if(Rst_ErrCntr)
        ErrCntr <= #1 0;
    else if(Ld_ErrCntr)
        ErrCntr <= #1 1;
    else if(CE_ErrCntr)
        ErrCntr <= #1 (ErrCntr + 1);
end

assign Rst_ErrLim = Rst | Ld_ErrCntr;
assign CE_ErrLim  = (Up | Dn);

always @(posedge Clk)
begin
    if(Rst_ErrLim)
        ErrLim <= #1 0;
    else if(CE_ErrLim)
        ErrLim <= #1 &ErrCntr;
end

//  Lock Counter
//      When minimum count reached, the DPLL is assumed to be locked
//      Lock deasserted when external reset applied, or when the DPLL is not
//      enabled, or when the Error Limit exceeds the lock threshold, +/- 15.

assign Rst_LockCntr = Rst | ~DPLL_En | (|ErrCntr[(pErrCntrLen - 1):4]);
assign CE_LockCntr  = ((xPPS & Dn) | (iPPS & Up) | (xPPS & iPPS));
assign Dec_LockCntr = ~ErrLim & ~TC_LockCntr & (~|ErrCntr[(pErrCntrLen - 1):4]);

always @(posedge Clk)
begin
    if(Rst_LockCntr)
        LockCntr <= #1 ~0;
    else if(CE_LockCntr)
        LockCntr <= #1 ((Dec_LockCntr) ? (LockCntr - 1) : LockCntr);
end

assign TC_LockCntr = ~|LockCntr;

//  Indicate DPLL Lock when LockCntr reaches minimum count

always @(posedge Clk)
begin
    if(Rst_LockCntr)
        Lock <= #1 0;
    else if(CE_LockCntr)
        Lock <= #1 TC_LockCntr;
end

//  Set PPSGate: 0 - select external 1PPS; 1 - select internal 1PPS

assign Rst_PPSGate = Rst | (xPPS & ~DPLL_En);
assign CE_PPSGate  = CE_LockCntr & TC_LockCntr;

always @(posedge Clk)
begin
    if(Rst_PPSGate)
        PPSGate <= #1 0;
    else if(CE_PPSGate)
        PPSGate <= #1 1;
end

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//  Implement TFF for both external and internal 1PPS signals
//

always @(posedge Clk)
begin
    if(Rst | (xPPS & ~DPLL_En))
        xPPS_TFF <= #1 1;
    else if(xPPS & DPLL_En)
        xPPS_TFF <= #1 ~xPPS_TFF;
end

always @(posedge Clk)
begin
    if(Rst)
        iPPS_TFF <= #1 1;
    else if(iPPS_Out)
        iPPS_TFF <= #1 ~iPPS_TFF;
end

endmodule

