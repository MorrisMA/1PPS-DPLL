1PPS Digital Phase-Locked Loop
=======================

Copyright (C) 2008-2016, Michael A. Morris <morrisma@mchsi.com>.
All Rights Reserved.

Released under GPL v3.

General Description
-------------------

This project provides a synthesizable DPLL that locks a DDS-based digital 
oscillator to an external 1PPS timing source.

Implementation
--------------

The implementation of the 1PPS DPLL core provided consists of the following 
Verilog source files:

    DPLLv2.v                - DPLL using Phase-Frequency Detector
        VCODrvFilter.v      - Multiplier-less Loop Filter

    tb_DPLLv2.v             - Completed 1PPS DPLL testbench
