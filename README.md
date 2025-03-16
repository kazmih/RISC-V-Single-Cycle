# RISC-V Single Cycle CPU Implementation

This repository contains a complete Verilog implementation of a RISC-V single-cycle CPU. The implementation follows the classic RISC-V architecture and executes each instruction in a single clock cycle.

## Architecture Overview

The CPU is designed with the following key components:

1. **Program Counter (PC)** - Holds the address of the current instruction
2. **Instruction Memory** - Stores the program instructions
3. **Register File** - Contains 32 general-purpose registers (x0-x31)
4. **ALU (Arithmetic Logic Unit)** - Performs arithmetic and logical operations
5. **Data Memory** - Stores data that can be loaded and stored
6. **Control Unit** - Generates control signals based on instruction opcode
7. **ALU Control** - Determines ALU operation based on function codes and ALU operation code
8. **Immediate Generator** - Extracts and extends immediate values from instructions

## Instruction Support

The current implementation supports the following RISC-V instruction types:

- **R-type**: Register-Register operations (ADD, SUB, AND, OR, SLT)
- **I-type**: Immediate and Load operations (ADDI, LW)
- **S-type**: Store operations (SW)
- **B-type**: Branch operations (BEQ)
- **J-type**: Jump operations (JAL)

## Module Descriptions

### Control Unit
Decodes the instruction opcode and generates appropriate control signals:
- `ALUSrc`: Selects between register or immediate value for ALU
- `ResultSrc`: Selects the source of result to write back to register
- `ImmSrc`: Determines the immediate format for the immediate generator
- `RegWrite`: Enables writing to the register file
- `MemWrite`: Enables writing to data memory
- `Branch`: Indicates a branch instruction
- `ALUOp`: Indicates the type of ALU operation
- `Jump`: Indicates a jump instruction

### ALU Control
Generates specific ALU operation codes based on:
- `ALUOp` from the Control Unit
- `Funct3` and `Funct7` fields from the instruction

### Register File
A 32×32 register file with:
- Two read ports for source operands (RS1, RS2)
- One write port for result (RD)
- Synchronous write and asynchronous read operations

### ALU
Performs operations such as:
- Addition
- Subtraction
- Set Less Than (SLT)
- Logical OR
- Logical AND
- Generates Zero flag for branch decisions

### Immediate Generator
Extracts immediate values from instructions and sign-extends them according to instruction type.

### Data Memory
A byte-addressable memory for load and store instructions.

### Program Counter
Holds the address of the current instruction and updates it on every clock cycle.

### Instruction Memory
Contains the program to be executed.

## Datapath

The datapath connects all components:
1. The PC provides an address to the instruction memory
2. The instruction is decoded by the control unit
3. Register values are read from the register file
4. The ALU performs the operation
5. Results are written back to registers or memory
6. The PC is updated for the next instruction

## Acknowledgements

This implementation follows the standard RISC-V ISA specification and is designed for educational purposes to demonstrate the fundamental concepts of a single-cycle CPU architecture.

# Simulation Results 

![lab 11 sim](https://github.com/user-attachments/assets/6eb9462f-7f03-471a-803f-447b4199350a)


