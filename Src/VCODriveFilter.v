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
// Create Date:     10:29:30 07/06/2008
// Design Name:     1PPS Digital Phase-Locked Loop Direct Digital Synthesizer
// Module Name:     VCODriveFilter.v
// Project Name:    0403DPLL
// Target Devices:  XC2S30-5VQ100I
// Tool versions:   ISE 10.1i
//
// Description: This module implements a digital low-pass filter in the form
//              of an alpha filter. Parameterization allows for the alpha
//              coefficient to programmable as a power of two, and the input
//              signal width can also be set as a parameter. Finally, the
//              initial, default value of the filter can be set using a third
//              parameter.
//
//              Saturation logic is implemented for the filter output register.
//              Saturation logic is used to limit the effect of numerical over-
//              flow and underflow. For this implementation, the output is con-
//              sidered to be an unsigned long value. The input is a 2's com-
//              plement signed 32-bit value.
//
//              A output hold function is implemented to prevent the filter
//              output from drooping at a rate of (1-alpha) in the event the
//              input signal is 0. This will allow the filter output to drive
//              the DDS (and when there is no error or external 1PPS reference)
//              without decaying when there is no signal input, i.e. no phase
//              error.
//
// Dependencies:    None
//
// Revision History:
//
//  0.00    08G06   MAM     File Created
//
// Additional Comments:
//
////////////////////////////////////////////////////////////////////////////////

module VCODriveFilter(Rst, Clk, CE, In, Out);

////////////////////////////////////////////////////////////////////////////////
//
//  Module Parameters
//

parameter pWidth = 32;
parameter pAlpha = 4;
parameter pDefaultValue = 32'h5000_0000;  // 10MHz@32MHz

////////////////////////////////////////////////////////////////////////////////
//
//  Port Declarations
//

    input  Rst;
    input  Clk;
    input  CE;
    input  [(pWidth - 1):0] In;

    output [(pWidth - 1):0] Out;

////////////////////////////////////////////////////////////////////////////////
//
//  Signal Declarations
//

    wire    Hold;

    wire    [(pWidth + pAlpha):0] A, B, C;
    wire    [(pWidth + pAlpha):0] Sum;
    wire    CY, OV, UV;
    wire    [(pWidth + pAlpha - 1):0] rYIn;
    reg     [(pWidth + pAlpha - 1):0] rY;

////////////////////////////////////////////////////////////////////////////////
//
//  Implementation
//

assign Hold = ~|In;    //  Memory Enable, if (In == 0), hold rY value

//  (A   : s_ssss_snnn_nnnn_nnnn_nnnn_nnnn_nnnn_nnnn.ffff)
//  (B   : 0_0nnn_nnnn_nnnn_nnnn_nnnn_nnnn_nnnn_nnnn.ffff)
//  (C   : 0_0000_0nnn_nnnn_nnnn_nnnn_nnnn_nnnn_nnnn.ffff)
//  (Sum : 0_0nnn_nnnn_nnnn_nnnn_nnnn_nnnn_nnnn_nnnn.ffff)

assign A = {{(pAlpha+1){In[(pWidth-1)]}}, In};          // A = In / 16
assign B = {rY[(pWidth+pAlpha-1)], rY};                 // B = rY
assign C = {{(pAlpha+1){rY[(pWidth+pAlpha-1)]}}, rY[(pWidth+pAlpha-1):pAlpha]};

//  Compute the Error Sum with negative feedback term (lossy integrator)
//      Sum = (1/(2**pAlpha))Xn + (((2**pAlpha) - 1)/(2**pAlpha))Yn

assign Sum = A + B - C;

//  Compute Overflow/Underflow for unsigned rY

assign CY = Sum[(pWidth + pAlpha)];
assign OV = ~CY & Sum[(pWidth + pAlpha - 1)]; // unsigned
assign UV =  CY & Sum[(pWidth + pAlpha - 1)]; // unsigned

//  Limit rY to positive values: 0..32'h7FFF_FFFF

assign rYIn = ((UV) ? (0) : ((OV) ? ((1 << (pWidth + pAlpha - 1)) - 1)
                                  : Sum));

//  Register limited sum into filter memory

always @(posedge Clk)
begin
    if(Rst)
        #1 rY <= #1 {pDefaultValue, {(pAlpha){1'b0}}};
    else if(CE & ~Hold)
        #1 rY <= #1 rYIn[(pWidth + pAlpha - 1):0];
end

//  Output pWidth MSBs of filter memory register, rY

assign Out = rY[(pWidth + pAlpha - 1):pAlpha];

endmodule

