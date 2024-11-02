# Yatch

## Description

A machine code interpreter for RISC-V, in C++. This attempts at simulating the execution of a RISC-V machine code program. The program reads the machine code from a file, and executes it. The program is capable of executing the following instructions:

## Features

- Supports R-type instructions: `add`, `sub`, `and`, `or`, `xor`, `sll`, `srl`, `sra`, `slt`, `sltu`
- Supports I-type instructions: `addi`, `andi`, `ori`, `xori`, `slti`, `sltiu`
- Supports load and store instructions: `lb`, `lh`, `lw`, `lbu`, `lhu`, `sb`, `sh`, `sw`
- Supports branch instructions: `beq`, `bne`, `blt`, `bge`, `bltu`, `bgeu`
- Supports jump instructions: `jal`, `jalr`
- Supports upper immediate instructions: `lui`, `auipc`
- Single-stage core simulation
- Five-stage core simulation (WIP)

## Setup

### Prerequisites

- C++ compiler (e.g., g++)
- Make (optional, for convenience)

### Building the Project

1. Clone the repository:
    ```sh
    git clone https://github.com/fuzzymfx/yatch.git
    ```

2. Compile the project:
    ```sh
    g++ -std=c++20 -o .dist/yatch_simulator src/main.cpp 
    ```

3. Alternatively, you can use a Makefile (if provided):
    ```sh
    make
    ```

## Usage

1. Prepare the memory files:
    - `imem.txt`: Contains the instruction memory.
    - `dmem.txt`: Contains the data memory.

2. Run the interpreter:
    ```sh
		./.dist/yatch_simulator src/input
    ```

3. Follow the prompts to provide the path to the memory files if not specified as a command-line argument.

## Memory Files Format

- `imem.txt`: Each line represents an 8-bit instruction memory value in binary format.
- `dmem.txt`: Each line represents an 8-bit data memory value in binary format.

## Output

- The state of the register file after each cycle is written to `RFResult.txt`.
- The state of the data memory after execution is written to `DmemSS_DMEMResult.txt`.

## Example

### imem.txt

00000000
00000000
00000000
10000011
00000000
01000000
00000001
00000011
00000000
00100000
10000001
10110011
00000000
00110000
00100100
00100011
11111111
11111111
11111111
11111111

### dmem.txt

01010101
01010101
01010101
01010101
00110011
00110011
00110011
00110011

## Liscense

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## References

- [RISC-V ISA Specification](https://github.com/riscv/riscv-isa-manual/releases/): The official RISC-V ISA manual.