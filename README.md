# FPGA-Verilog
Projects implemented in Spartan3 and Spartan3e for the Digital Systems class which include:

- LED Driver for Spartan3 FPGA - Lab01
- UART implementation - Lab02
- VGA Driver - Lab03
- LCD driver for Spartan3e FPGA - Lab04

The following tools and manuals were used: 
- Xillinx ISE Design Suite 14.7
- Manual of Spartan3 FPGA (https://www.xilinx.com/support/documentation/boards_and_kits/ug130.pdf)
- Manual of Spartan3e FPGA (https://www.xilinx.com/support/documentation/boards_and_kits/ug230.pdf)

Each small project has the source code, the UCF file required for the connectivity and the final bitfile of the
each project.

Projects are split into parts basen on each assignment's requirement of each part and in each lab
assignment we have a report of the whole project.

Generally the last part will have the full project.

## LED Driver
The LED Driver was implemented in the Spartan3 FPGA, on the onboard LED Display.

The project was to implement 2 things:
- Display a static sequence and then rotate it at the press of a button (Part3 folder)
- Display a static sequence which rotates at a set frequency (Part4 folder)

The seqeunce was a simple "0123456789ABCDEF" message.

## UART Implementation
The UART implementation was only tested in the RTL department without any practical 
testing on any FPGA.

It implements the UART transfer hardware-protocol with both a receiver and a trasmitter
which have variable BaudRates which we can set ourselves.

We implement also system flags in case of framing errors, parity errors or some other case.

## VGA Driver
The VGA driver was implemented in Spartan3.

It basicaly is an VGA driver for 640x480 resolution. But in our case due to memory limitations
we implement 640x480 driver which displays an 128x96 image in RGB222 (Basically each color channel is on or off).

For now a static image is displayed and no video is produced but this could be easily changed.

The data to be diplayed is stored in a chunk of BRAM in Spartan3.

## LCD Driver
The LCD Driver was implemented in Spartan3e's onboard LCD screen.

We initialize, configure and display a message with a cursor on the LCD screen.

We display yet again, a message of "0123456789abcde" and a cursor in the last position.

The cursor blinks every 1s
