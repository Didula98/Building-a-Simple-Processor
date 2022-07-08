# Building-a-Simple-Processor

## Pre requests
verilog compiler(Icarus Verilog) with GTKWave.

## Compile & run the verilog file

`iverilog -o fileName fileName.v`

Then it will create a binary file

`vvp fileName`

This will run the binary file and give outputs

## Run the GTKwaveData (if avalable)

`gtkwave GTKwaveFileName.vcd`

The GTKWave will open automatically
Then you can select components that want to show from the left tab

Instruction file "instr_mem.mem" will be needed to run the CPU properly.
It should be in the same directory with verilog file. If not, change the path in verilog file.

## Overview of CPU

![Capture](https://user-images.githubusercontent.com/84913495/177954119-0fb61cb7-6c44-48ab-9df5-beb8ae2cdfb1.PNG)
