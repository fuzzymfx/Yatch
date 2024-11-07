#ifndef TYPES_H
#define TYPES_H

#include <iostream>
#include <string>
#include <vector>
#include <bitset>
#include <fstream>
#include <stdexcept>

using namespace std;

#define MemSize 1000 // memory size, in reality, the memory size should be 2^32, but for this lab, for the space resaon, we keep it as this large number, but the memory is still 32-bit addressable.

struct IFStruct
{
	bitset<32> PC;
	bool nop;
};

struct IDStruct
{
	bitset<32> Instr;
	bool nop;
	bitset<32> PC;
};

enum ALUOp
{
	NOP,	// No operation
	ADDU, // Addition (Unsigned)
	SUBU, // Subtraction (Unsigned)
	AND,	// Bitwise AND
	OR,		// Bitwise OR
	XOR,	// Bitwise XOR
	SLL,	// Shift Left Logical
	SRL,	// Shift Right Logical (Logical shift)
	SRA,	// Shift Right Arithmetic
	SLT,	// Set Less Than (Signed comparison)
	SLTU	// Set Less Than Unsigned (Unsigned comparison)
};

enum MemOp
{
	MemNone, // No memory operation
	MemLB,	 // Load Byte
	MemLH,	 // Load Half Word
	MemLW,	 // Load Word
	MemLBU,	 // Load Byte Unsigned
	MemLHU,	 // Load Half Word Unsigned
	MemSB,	 // Store Byte
	MemSH,	 // Store Half Word
	MemSW		 // Store Word
};
struct EXStruct
{
	bitset<32> Read_data1;
	bitset<32> Read_data2;
	bitset<32> Imm;
	bitset<5> Rs;
	bitset<5> Rt;
	bitset<5> Wrt_reg_addr;
	bool is_I_type;
	bool rd_mem;
	bool wrt_mem;
	ALUOp alu_op;
	MemOp mem_op;
	bool wrt_enable;
	bool nop;

	uint32_t opcode;
	bitset<32> PC;
	bool is_jal;
	bool is_jalr;
	bool is_auipc;
};

struct MEMStruct
{
	bitset<32> ALUresult;
	bitset<32> Store_data;
	bitset<5> Rs;
	bitset<5> Rt;
	bitset<5> Wrt_reg_addr;
	MemOp mem_op;
	bool rd_mem;
	bool wrt_mem;
	bool wrt_enable;
	bool nop;
};

struct WBStruct
{
	bitset<32> Wrt_data;
	bitset<5> Rs;
	bitset<5> Rt;
	bitset<5> Wrt_reg_addr;
	bool wrt_enable;
	bool nop;
};

class MemType
{
public:
	enum Type
	{
		Imem,
		DmemSS,
		DmemFS
	};
	static string toString(Type type);
};

struct SingleStage
{
	bitset<32> Instruction; // Current Instruction
	bitset<32> PC;					// Program Counter
	bitset<32> result;			// Result of the current operation, if needed
	bool nop;
};

struct stateStruct
{
	IFStruct IF;
	IDStruct ID;
	EXStruct EX;
	MEMStruct MEM;
	WBStruct WB;
	SingleStage SS;
};

class InsMem
{
public:
	MemType::Type id;
	string ioDir;
	InsMem(MemType::Type type, string ioDir);
	bitset<32> readInstr(bitset<32> ReadAddress);
	int totalInstructions();

private:
	int size;
	vector<bitset<8>> IMem;
};

class DataMem
{
public:
	string opFilePath, ioDir;
	DataMem(MemType::Type type, string ioDir);
	bitset<8> readByte(uint32_t address);
	bitset<16> readHalfWord(uint32_t address);
	void writeByte(uint32_t address, bitset<8> data);
	void writeHalfWord(uint32_t address, bitset<16> data);
	bitset<32> readDataMem(bitset<32> Address);
	void writeDataMem(bitset<32> Address, bitset<32> WriteData);
	void outputDataMem();

private:
	MemType::Type id;
	vector<bitset<8>> DMem;
};

class RegisterFile
{
public:
	string outputFile;
	RegisterFile(string ioDir);
	bitset<32> readRF(bitset<5> Reg_addr);
	void writeRF(bitset<5> Reg_addr, bitset<32> Wrt_reg_data);
	void outputRF(int cycle);

private:
	vector<bitset<32>> Registers;
};

class Core
{
public:
	RegisterFile myRF;
	uint32_t cycle = 0;
	bool halted = false;
	string ioDir;
	struct stateStruct state, nextState;
	InsMem ext_imem;
	DataMem ext_dmem;
	Core(string ioDir, InsMem &imem, DataMem &dmem);
	virtual void step() {}
	virtual void printState() {}
};

#endif // TYPES_H
