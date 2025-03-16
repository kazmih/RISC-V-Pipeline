# RISC-V 5-Stage Pipelined CPU Implementation

This repository contains a complete Verilog implementation of a RISC-V processor with a 5-stage pipeline. The pipelined architecture significantly improves performance by allowing multiple instructions to be processed simultaneously, with each instruction going through fetch, decode, execute, memory, and writeback stages.

## Pipeline Architecture

The processor implements the classic 5-stage RISC-V pipeline:

1. **Fetch (F)** - Retrieves instructions from instruction memory
2. **Decode (D)** - Decodes instructions and reads register values
3. **Execute (E)** - Performs ALU operations and branch/jump calculations
4. **Memory (M)** - Performs memory access operations
5. **Writeback (W)** - Writes results back to the register file

## Pipeline Registers

Inter-stage registers are implemented to hold data between pipeline stages:

- **Pipe_D**: Between Fetch and Decode stages
- **Pipe_E**: Between Decode and Execute stages
- **Pipe_M**: Between Execute and Memory stages
- **Pipe_W**: Between Memory and Writeback stages

## Key Components

### Control Unit
The control unit consists of two primary components:
- **Control_Unit**: Decodes opcodes to generate control signals
- **ALU_Control**: Determines specific ALU operations based on function codes

### Datapath Components
- **Program Counter**: Keeps track of the instruction address
- **Instruction Memory**: Stores program instructions
- **Register File**: 32×32-bit register file with synchronous write (on negative clock edge)
- **ALU**: Performs arithmetic and logical operations
- **Data Memory**: Stores data for load and store operations
- **Immediate Generator**: Creates immediate values from instruction fields

### Multiplexers
Several multiplexers control data flow through the pipeline:
- **PCSrcMUX**: Selects between PC+4 and branch/jump target
- **ALUSrcMUX**: Selects between register data and immediate values
- **ResultSrcMUX**: Selects between ALU result, memory data, and PC+4

## Instruction Support & Instructions Used

The processor supports the following RISC-V instruction types:
- **R-type**: Register-register operations (ADD, SUB, AND, OR, SLT)
- **I-type**: Immediate and load operations (ADDI, LW)
- **S-type**: Store operations (SW)
- **B-type**: Branch operations (BEQ)

```
addi x22,x0,0 # i - loop variable
addi x23,x0,0 # j - loop variable
addi x10,x0,10 # Maximum Loop count

# To load the array
Loop1:
    slli x24, x22, 2
    sw x22,0x200(x24)
    addi x22,x22,1
    bne x22,x10,Loop1

addi x22,x0,0 # Initializing loop variable i for Loop2
Loop2:
    slli x24, x22, 2 # i*4 for offset of array
    add x23,x22,x0   # j=i, first value of j initialized for inner loop
Loop3:
    slli x25, x23, 2 # j*4 for offset of array
    lw x1,0x200(x24) # a[i]
    lw x2,0x200(x25) # a[j]
    bge x1,x2,EndIf  # if a[i] >= a[j], then end the loop

    # If a[i] < a[j], then swap
    add x5,x1,x0       # temp <- a[i]
    sw x2,0x200(x24)   # a[i] <- a[j]
    sw x5,0x200(x25)   # a[j] <- temp
EndIf:
    addi x23,x23,1
    bne x23,x10,Loop3 # Repeat Loop3 with next value of j

    addi x22,x22,1    # Else end Loop3 and
    bne x22,x10,Loop2 # Go to Loop2 with the next value of i
```

## Control Signals

Key control signals include:
- **RegWrite**: Controls register write operations
- **MemWrite**: Controls memory write operations
- **ALUSrc**: Selects ALU source (register or immediate)
- **ResultSrc**: Selects result source for register write
- **ImmSrc**: Selects immediate format
- **Branch/Jump**: Controls program flow changes
- **Operation**: Selects specific ALU operation

## Features

1. **Performance**: The 5-stage pipeline allows for instruction-level parallelism
2. **Modularity**: Clean separation of control and datapath components
3. **Completeness**: Full implementation of core RISC-V instructions
4. **Verification**: Comprehensive testbench for simulation and debugging

## Usage

To use this processor in your own designs:
1. Instantiate the top module `RISC_V_Processor_Pipelined`
2. Connect clock and reset signals
3. Load your program into the instruction memory

## Acknowledgements

This implementation follows the standard RISC-V ISA specification and demonstrates the fundamental concepts of pipelined processor design.

# Simulation Results
![Pipeline](https://github.com/user-attachments/assets/e56f8bff-b9b0-4b04-a042-606891447cd9)
