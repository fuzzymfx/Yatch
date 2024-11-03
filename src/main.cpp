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

// The possible types of memory
class MemType
{
public:
	enum Type
	{
		Imem,
		DmemSS,
		DmemFS
	};

	// Convert the memory type to a string to be used in the file path- to read/write the memory
	static string toString(Type type)
	{
		switch (type)
		{
		case Imem:
			return "Imem";
		case DmemSS:
			return "DmemSS";
		case DmemFS:
			return "DmemFS";
		default:
			return "Unknown";
		}
	}
};

class InsMem
{
private:
	vector<bitset<8>> IMem;

public:
	MemType::Type id;
	string ioDir;
	InsMem(MemType::Type type, string ioDir) : id(type), ioDir(ioDir)
	{
		IMem.resize(MemSize);
		ifstream imem;
		string line;
		int i = 0;
		try
		{
			imem.open(ioDir + "/imem.txt");
			if (!imem.is_open())
			{
				throw runtime_error("Unable to open IMEM input file.");
			}
			while (getline(imem, line))
			{
				// Technically, the IMem should be 32-bit addressable, we keep it as 8-bit addressable here because the imem.txt file is in byte format. Each line is a byte(8 bits)
				IMem[i] = bitset<8>(line);
				i++;
			}
			imem.close();
		}
		catch (const exception &e)
		{
			throw runtime_error("An error occurred: " + string(e.what()));
		}
	}

	bitset<32> readInstr(bitset<32> ReadAddress)
	{
		uint32_t address = (uint32_t)ReadAddress.to_ulong();

		if (address + 3 >= MemSize)
		{
			throw runtime_error("Read address is out of bounds.");
		}

		bitset<32> instruction;
		for (int i = 0; i < 4; ++i)
		{
			instruction <<= 8;																			 // Shift left by 8 bits to make room for the next byte
			instruction |= bitset<32>(IMem[address + i].to_ulong()); // Combine the next byte
		}

		return instruction;
	}
};

class DataMem
{
public:
	string opFilePath, ioDir;
	DataMem(MemType::Type type, string ioDir) : id{type}, ioDir{ioDir}
	{
		DMem.resize(MemSize);
		string name = MemType::toString(id);

		opFilePath = ioDir + "/" + name + "_DMEMResult.txt";
		ifstream dmem;
		string line;
		int i = 0;
		try
		{
			dmem.open(ioDir + "/dmem.txt");
			if (!dmem.is_open())
			{
				throw runtime_error("Unable to open DMEM input file.");
			}
			while (getline(dmem, line))
			{
				DMem[i] = bitset<8>(line);
				i++;
			}
			dmem.close();
		}
		catch (const exception &e)
		{
			throw runtime_error("An error occurred: " + string(e.what()));
		}
	}

	bitset<8> readByte(uint32_t address)
	{
		if (address < DMem.size())
		{
			return DMem[address];
		}
		else
		{
			// Handle out-of-bounds access
			throw runtime_error("Read Byte Error: Address out of bounds.");
			return bitset<8>(0);
		}
	}

	// Method to read a half-word (2 bytes) from data memory at a given address
	bitset<16> readHalfWord(uint32_t address)
	{
		if (address + 1 < DMem.size())
		{
			// Big-Endian: lower address holds the most significant byte
			uint16_t msb = DMem[address].to_ulong();		 // Most significant byte
			uint16_t lsb = DMem[address + 1].to_ulong(); // Least significant byte
			uint16_t data = (msb << 8) | lsb;
			return bitset<16>(data);
		}
		else
		{
			// Handle out-of-bounds access
			throw("Read Half-Word Error: Address out of bounds");
			return bitset<16>(0);
		}
	}
	// Method to write a byte to data memory at a given address
	void writeByte(uint32_t address, bitset<8> data)
	{
		if (address < DMem.size())
		{
			DMem[address] = data;
		}
		else
		{
			throw runtime_error("Write Byte Error: Address out of bounds");
		}
	}

	void writeHalfWord(uint32_t address, bitset<16> data)
	{
		if (address + 1 < DMem.size())
		{
			uint16_t value = data.to_ulong();
			// Big-Endian: lower address holds the most significant byte
			DMem[address] = bitset<8>((value >> 8) & 0xFF); // MSB
			DMem[address + 1] = bitset<8>(value & 0xFF);		// LSB
		}
		else
		{
			throw runtime_error("Write Half-Word Error: Address out of bounds");
		}
	}

	bitset<32> readDataMem(bitset<32> Address)
	{
		uint32_t address = (uint32_t)Address.to_ulong();

		if (address + 3 < DMem.size())
		{
			uint32_t data = (DMem[address].to_ulong() << 24) |
											(DMem[address + 1].to_ulong() << 16) |
											(DMem[address + 2].to_ulong() << 8) |
											DMem[address + 3].to_ulong();

			return bitset<32>(data);
		}
		else
		{
			throw runtime_error("Read address is out of bounds.");
		}
	}

	void writeDataMem(bitset<32> Address, bitset<32> WriteData)
	{
		// Convert Address to an integer for indexing
		uint32_t address = Address.to_ulong();

		// Bounds check to ensure we're within memory limits
		if (address + 3 >= MemSize)
		{
			throw runtime_error("Write address is out of bounds.");
		}

		// Write each byte of WriteData to memory (Big-Endian order)
		for (int i = 3; i >= 0; --i)
		{ // Start from the most significant byte
			DMem[address + i] = bitset<8>((WriteData >> (8 * (3 - i))).to_ulong() & 0xFF);
		}
	}

	void outputDataMem()
	{
		ofstream dmemout;
		dmemout.open(opFilePath, std::ios_base::trunc);
		if (dmemout.is_open())
		{
			for (int j = 0; j < 1000; j++)
			{
				dmemout << DMem[j] << endl;
			}
		}
		else
			throw runtime_error("Unable to open " + MemType::toString(id) + " DMEM result file.");
		dmemout.close();
	}

private:
	MemType::Type id;
	vector<bitset<8>> DMem;
};

class RegisterFile
{
private:
	vector<bitset<32>> Registers; // A vector of 32-bit registers

public:
	// Output file path for the register file
	string outputFile;
	// Init
	RegisterFile(string ioDir) : outputFile{ioDir + "RFResult.txt"}
	{
		Registers.resize(32);					// 32 registers in total, each 32 bits wide
		Registers[0] = bitset<32>(0); // Register x0 is always 0 in RISC-V
	}

	bitset<32> readRF(bitset<5> Reg_addr)
	{
		// Converts Reg_addr to integer for indexing
		uint32_t reg_index = Reg_addr.to_ulong();

		// Returns the register value at the given index
		return Registers[reg_index];
	}

	void writeRF(bitset<5> Reg_addr, bitset<32> Wrt_reg_data)
	{
		// Debug statement
		cout << "[DEBUG] Writing to register " << Reg_addr.to_ulong() << ": " << Wrt_reg_data << endl;
		// Converts Reg_addr to integer for indexing
		uint32_t reg_index = Reg_addr.to_ulong();

		// x0 is hardwired to 0, cannot be overwritten
		if (reg_index != 0)
		{
			Registers[reg_index] = Wrt_reg_data; // Write the data to the register
		}
		else
		{
			throw runtime_error("Cannot write to register x0.");
		}
	}

	// Writes the current state of the register file to the output file
	void outputRF(int cycle)
	{
		try
		{
			ofstream rfout;
			// If it's the first cycle, truncate the file, otherwise append
			if (cycle == 0)
				rfout.open(outputFile, std::ios_base::trunc);
			else
				rfout.open(outputFile, std::ios_base::app);
			if (rfout.is_open())
			{
				rfout << "State of RF after executing cycle:\t" << cycle << endl;
				for (int j = 0; j < 32; j++)
				{
					// Write each register value to the file
					rfout << Registers[j] << endl;
				}
			}
			else
				throw runtime_error("Unable to open RF output file.");
			rfout.close();
		}
		catch (const exception &e)
		{
			throw runtime_error("An error occurred in RegisterFile::outputRF: " + string(e.what()));
		}
	}
};

class Core
{
public:
	RegisterFile myRF;
	uint32_t cycle = 0;
	bool halted = false;
	string ioDir;
	struct stateStruct state, nextState;
	InsMem &ext_imem;
	DataMem &ext_dmem;

	Core(string ioDir, InsMem &imem, DataMem &dmem) : myRF(ioDir), ioDir{ioDir}, ext_imem{imem}, ext_dmem{dmem} {}

	virtual void step() {}

	virtual void printState(stateStruct state, int cycle) {}
};

class SingleStageCore : public Core
{

private:
	string opFilePath;

public:
	SingleStageCore(string ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir + "/SS_", imem, dmem), opFilePath(ioDir + "/StateResult_SS.txt")
	{
		// Initialize PC to 0
		state.SS.PC = bitset<32>(0);
	}

	void step()
	{
		try
		{
			// Fetches the instruction from the instruction memory
			bitset<32> instruction = ext_imem.readInstr(state.SS.PC);
			uint32_t instr = instruction.to_ulong();

			// Debug statement
			cout << "[DEBUG] Cycle: " << cycle + 1 << ", PC: " << state.SS.PC.to_ulong() << ", Instruction: " << instruction << endl;

			// Checks if it's a HALT instruction
			if (instr == 0xFFFFFFFF)
			// *This is not a valid RISC-V instruction*. It's a placeholder for HALT
			// Halt instructions are not present in RISC-V ISA. This is a placeholder for the simulator
			// RISC V uses ECALL and EBREAK to halt the program and trigger an exception to be handled by the OS
			{
				halted = true;
				state.SS.nop = true;
				// Outputs register file and state for this cycle
				myRF.outputRF(cycle);
				printState(state, cycle);
				return;
			}

			// Decodes the instruction
			uint32_t opcode = instr & 0x7F;
			uint32_t rd = (instr >> 7) & 0x1F;
			uint32_t funct3 = (instr >> 12) & 0x7;
			uint32_t rs1 = (instr >> 15) & 0x1F;
			uint32_t rs2 = (instr >> 20) & 0x1F;
			uint32_t funct7 = (instr >> 25) & 0x7F;

			bitset<32> result;
			uint32_t address;
			bool pc_updated = false;

			// Execute the instruction based on the opcode
			switch (opcode)
			{
			case 0x33: // R-type instructions
			{
				switch (funct3)
				{
				case 0x0:
					if (funct7 == 0x00)
					{
						// ADD
						result = bitset<32>(myRF.readRF(rs1).to_ulong() + myRF.readRF(rs2).to_ulong());
					}
					else if (funct7 == 0x20)
					{
						// SUB
						result = bitset<32>(myRF.readRF(rs1).to_ulong() - myRF.readRF(rs2).to_ulong());
					}
					else
					{
						throw runtime_error("Unknown funct7 in ADD/SUB: " + to_string(funct7));
					}
					break;
				case 0x1:
					// SLL
					result = bitset<32>(myRF.readRF(rs1).to_ulong() << (myRF.readRF(rs2).to_ulong() & 0x1F));
					break;
				case 0x2:
					// SLT
					result = bitset<32>((int32_t)myRF.readRF(rs1).to_ulong() < (int32_t)myRF.readRF(rs2).to_ulong());
					break;
				case 0x3:
					// SLTU
					result = bitset<32>(myRF.readRF(rs1).to_ulong() < myRF.readRF(rs2).to_ulong());
					break;
				case 0x4:
					// XOR
					result = bitset<32>(myRF.readRF(rs1).to_ulong() ^ myRF.readRF(rs2).to_ulong());
					break;
				case 0x5:
					if (funct7 == 0x00)
					{
						// SRL
						result = bitset<32>(myRF.readRF(rs1).to_ulong() >> (myRF.readRF(rs2).to_ulong() & 0x1F));
					}
					else if (funct7 == 0x20)
					{
						// SRA
						result = bitset<32>((int32_t)myRF.readRF(rs1).to_ulong() >> (myRF.readRF(rs2).to_ulong() & 0x1F));
					}
					else
					{
						throw runtime_error("Unknown funct7 in SRL/SRA: " + to_string(funct7));
					}
					break;
				case 0x6:
					// OR
					result = bitset<32>(myRF.readRF(rs1).to_ulong() | myRF.readRF(rs2).to_ulong());
					break;
				case 0x7:
					// AND
					result = bitset<32>(myRF.readRF(rs1).to_ulong() & myRF.readRF(rs2).to_ulong());
					break;
				default:
					throw runtime_error("Unknown funct3 in R-type instruction: " + to_string(funct3));
					break;
				}
				if (rd != 0) // x0 is hardwired to 0, cannot be overwritten
					myRF.writeRF(rd, result);
				break;
			}
			case 0x03: // Load instructions
			{
				int32_t imm = get_imm_i(instr);
				address = myRF.readRF(rs1).to_ulong() + imm;

				bitset<32> result;

				switch (funct3)
				{
				case 0x0: // LB
				{
					bitset<8> data = ext_dmem.readByte(address);
					int8_t signed_data = (int8_t)data.to_ulong();
					result = bitset<32>(signed_data);
					break;
				}
				case 0x1: // LH
				{
					bitset<16> data = ext_dmem.readHalfWord(address);
					int16_t signed_data = (int16_t)data.to_ulong();
					result = bitset<32>(signed_data);
					break;
				}
				case 0x2: // LW
				{
					result = ext_dmem.readDataMem(address);
					break;
				}
				case 0x4: // LBU
				{
					bitset<8> data = ext_dmem.readByte(address);
					result = bitset<32>(data.to_ulong());
					break;
				}
				case 0x5: // LHU
				{
					bitset<16> data = ext_dmem.readHalfWord(address);
					result = bitset<32>(data.to_ulong());
					break;
				}
				default:
					throw runtime_error("Unknown funct3 in Load instruction: " + to_string(funct3));
					break;
				}
				if (rd != 0)
					myRF.writeRF(rd, result);
				break;
			}
			case 0x23: // Store instructions
			{
				int32_t imm = get_imm_s(instr);
				address = myRF.readRF(rs1).to_ulong() + imm;

				switch (funct3)
				{
				case 0x0: // SB
				{
					bitset<8> data = bitset<8>(myRF.readRF(rs2).to_ulong() & 0xFF);
					ext_dmem.writeByte(address, data);
					break;
				}
				case 0x1: // SH
				{
					bitset<16> data = bitset<16>(myRF.readRF(rs2).to_ulong() & 0xFFFF);
					ext_dmem.writeHalfWord(address, data);
					break;
				}
				case 0x2: // SW
				{
					bitset<32> data = myRF.readRF(rs2);
					ext_dmem.writeDataMem(address, data);
					break;
				}
				default:
					throw runtime_error("Unknown funct3 in Store instruction: " + to_string(funct3));
					break;
				}
				break;
			}
			case 0x13: // I-type arithmetic instructions
			{
				bool isLogical = (funct3 == 0x4) || (funct3 == 0x6) || (funct3 == 0x7); // XORI, ORI, ANDI
				int32_t imm = get_imm_i(instr, isLogical);

				switch (funct3)
				{
				case 0x0: // ADDI
					result = bitset<32>((int32_t)myRF.readRF(rs1).to_ulong() + imm);
					break;
				case 0x2: // SLTI
					result = bitset<32>((int32_t)myRF.readRF(rs1).to_ulong() < imm ? 1 : 0);
					break;
				case 0x3: // SLTIU
					result = bitset<32>(myRF.readRF(rs1).to_ulong() < (uint32_t)imm ? 1 : 0);
					break;
				case 0x4: // XORI
					result = bitset<32>(myRF.readRF(rs1).to_ulong() ^ (uint32_t)imm);
					break;
				case 0x6: // ORI
					result = bitset<32>(myRF.readRF(rs1).to_ulong() | (uint32_t)imm);
					break;
				case 0x7: // ANDI
					result = bitset<32>(myRF.readRF(rs1).to_ulong() & (uint32_t)imm);
					break;
				case 0x1: // SLLI
					result = bitset<32>(myRF.readRF(rs1).to_ulong() << (imm & 0x1F));
					break;
				case 0x5:
					if ((imm >> 10) == 0x00)
					{
						// SRLI
						result = bitset<32>(myRF.readRF(rs1).to_ulong() >> (imm & 0x1F));
					}
					else if ((imm >> 10) == 0x20)
					{
						// SRAI
						result = bitset<32>((int32_t)myRF.readRF(rs1).to_ulong() >> (imm & 0x1F));
					}
					else // Should not reach here
					{
						throw runtime_error("Unknown funct7 in SRLI/SRAI instruction: " + to_string(imm >> 10));
					}
					break;
				default: // Should not reach here
					throw runtime_error("Unknown funct3 in I-type arithmetic instruction: " + to_string(funct3));
					break;
				}
				if (rd != 0)
					myRF.writeRF(rd, result);
				break;
			}
			case 0x63: // Branch instructions
			{
				int32_t imm = get_imm_b(instr);

				switch (funct3)
				{
				case 0x0: // BEQ
					if (myRF.readRF(rs1).to_ulong() == myRF.readRF(rs2).to_ulong())
					{
						state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + imm);
						pc_updated = true;
					}
					break;
				case 0x1: // BNE
					if (myRF.readRF(rs1).to_ulong() != myRF.readRF(rs2).to_ulong())
					{
						state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + imm);
						pc_updated = true;
					}
					break;
				case 0x4: // BLT
					if ((int32_t)myRF.readRF(rs1).to_ulong() < (int32_t)myRF.readRF(rs2).to_ulong())
					{
						state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + imm);
						pc_updated = true;
					}
					break;
				case 0x5: // BGE
					if ((int32_t)myRF.readRF(rs1).to_ulong() >= (int32_t)myRF.readRF(rs2).to_ulong())
					{
						state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + imm);
						pc_updated = true;
					}
					break;
				case 0x6: // BLTU
					if (myRF.readRF(rs1).to_ulong() < myRF.readRF(rs2).to_ulong())
					{
						state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + imm);
						pc_updated = true;
					}
					break;
				case 0x7: // BGEU
					if (myRF.readRF(rs1).to_ulong() >= myRF.readRF(rs2).to_ulong())
					{
						state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + imm);
						pc_updated = true;
					}
					break;
				default: // Should not reach here
					throw runtime_error("Unknown funct3 in Branch instruction: " + to_string(funct3));
					break;
				}
				break;
			}
			case 0x6F: // JAL
			{
				int32_t imm = get_imm_j(instr);
				if (rd != 0)
					myRF.writeRF(rd, bitset<32>(state.SS.PC.to_ulong() + 4));
				state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + imm);
				pc_updated = true;
				break;
			}
			case 0x67: // JALR
			{
				int32_t imm = get_imm_i(instr);
				uint32_t target = (myRF.readRF(rs1).to_ulong() + imm) & ~1;
				if (rd != 0)
					myRF.writeRF(rd, bitset<32>(state.SS.PC.to_ulong() + 4));
				state.SS.PC = bitset<32>(target);
				pc_updated = true;
				break;
			}
			case 0x37: // LUI
			{
				int32_t imm = get_imm_u(instr);
				if (rd != 0)
					myRF.writeRF(rd, bitset<32>(imm));
				break;
			}
			case 0x17: // AUIPC
			{
				int32_t imm = get_imm_u(instr);
				if (rd != 0)
					myRF.writeRF(rd, bitset<32>(state.SS.PC.to_ulong() + imm));
				break;
			}
			case 0x73: // System instructions
			{
				switch (funct3)
				{
				case 0x0:
					if (funct7 == 0x0)
					{
						// ECALL
						cout << "ECALL executed." << endl;
					}
					else if (funct7 == 0x1)
					{
						// EBREAK
						cout << "EBREAK executed." << endl;
						halted = true;
					}
					else
					{
						throw runtime_error("Unknown funct7 in ECALL/EBREAK: " + to_string(funct7));
					}
					break;
				case 0x1:
				{
					// CSRRW
					uint32_t csr = (instr >> 20) & 0xFFF;
					if (rd != 0)
					{
						bitset<32> csr_val = bitset<32>(0); // Placeholder for CSR value
						myRF.writeRF(rd, csr_val);
					}
				}
				break;
				default:
					break;
				}
				break;
			}
			default: // Should not reach here
				throw runtime_error("Unknown opcode encountered: " + to_string(opcode));
				break;
			}

			// Update the PC if it wasn't updated by a branch or jump instruction
			if (!pc_updated)
			{
				state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + 4);
				// Increment PC by 4 bytes. This would point to a new instruction. An instruction is 4 bytes wide
			}

			// Output register file and state for this cycle
			myRF.outputRF(cycle);			// dump RF
			printState(state, cycle); // print states after executing this cycle

			cycle++;
		}
		catch (const exception &e)
		{
			throw runtime_error("An error occurred in SingleStageCore::step: " + string(e.what()));
		}
	}

	void printState(stateStruct state, int cycle) // A Debug function to print the state of the core- used for verifying the cycle-by-cycle operation
	{
		try
		{
			ofstream printstate;
			// The first cycle indicates a new run and truncates the file
			if (cycle == 0)
			{
				printstate.open(opFilePath, std::ios_base::trunc);
				printstate << "----------------------------------------------------------------------" << endl;
			}

			else // Append to the file for subsequent cycles
				printstate.open(opFilePath, std::ios_base::app);
			if (printstate.is_open())
			{
				printstate << "State after executing cycle:\t" << cycle << endl;
				printstate << "IF.PC:\t" << state.SS.PC.to_ulong() << endl;
				printstate << "IF.nop:\t" << (state.SS.nop ? "True" : "False") << endl;
				printstate << "----------------------------------------------------------------------" << endl;
			}
			else
			{
				throw runtime_error("Unable to open SS StateResult output file.");
			}
			printstate.close();
		}
		catch (const exception &e)
		{
			throw runtime_error("An error occurred in SingleStageCore::printState: " + string(e.what()));
		}
	}

private:
	// Helper function to sign-extend I-type immediate
	int32_t get_imm_i(uint32_t instr, bool isLogical = false)
	{
		int32_t imm = (instr >> 20) & 0xFFF; // Extract bits [31:20]
		if (!isLogical && (imm & 0x800))		 // Check if sign bit (bit 11) is set and not logical
			imm |= 0xFFFFF000;								 // Sign-extend to 32 bits
		return imm;
	}

	// Helper function to sign-extend S-type immediate
	int32_t get_imm_s(uint32_t instr)
	{
		int32_t imm = ((instr >> 25) << 5) | ((instr >> 7) & 0x1F); // Extract bits [31:25] and [11:7]
		if (imm & 0x800)																						// Check if sign bit (bit 11) is set
			imm |= 0xFFFFF000;																				// Sign-extend to 32 bits
		return imm;
	}

	// Helper function to sign-extend B-type immediate
	int32_t get_imm_b(uint32_t instr)
	{
		int32_t imm = ((instr >> 31) << 12) |					// Extract bit 31 and shift to bit 12
									(((instr >> 7) & 0x1) << 11) |	// Extract bit 7 and shift to bit 11
									(((instr >> 25) & 0x3F) << 5) | // Extract bits [31:25] and shift to bits [10:5]
									(((instr >> 8) & 0xF) << 1);		// Extract bits [11:8] and shift to bits [4:1]
		if (imm & 0x1000)
			imm |= 0xFFFFE000; // Sign-extend to 32 bits
		return imm;
	}

	// Helper function to get U-type immediate
	int32_t get_imm_u(uint32_t instr)
	{
		int32_t imm = instr & 0xFFFFF000; // Extract bits [31:12]
		return imm;
	}

	// Helper function to sign-extend J-type immediate
	int32_t get_imm_j(uint32_t instr)
	{
		int32_t imm = ((instr >> 31) << 20) |					 // Extract bit 31 and shift to bit 20
									(((instr >> 12) & 0xFF) << 12) | // Extract bits [19:12] and shift to bits [19:12]
									(((instr >> 20) & 0x1) << 11) |	 // Extract bit 20 and shift to bit 11
									(((instr >> 21) & 0x3FF) << 1);	 // Extract bits [10:1] and shift to bits [10:1]

		if (imm & 0x100000)
			imm |= 0xFFE00000;
		return imm;
	}
};

class FiveStageCore : public Core
{
public:
	FiveStageCore(string ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir + "/FS_", imem, dmem), opFilePath(ioDir + "/StateResult_FS.txt")
	{
		// Initialize pipeline stages to nop
		state.IF.nop = false;
		state.ID.nop = true;
		state.EX.nop = true;
		state.MEM.nop = true;
		state.WB.nop = true;

		// Initialize PC to 0
		state.IF.PC = bitset<32>(0);
	}

	void step()
	{
		// Initialize nextState to default values
		nextState = state;

		/* --------------------- WB stage --------------------- */
		if (!state.WB.nop) // checks if writing is allowed for the current stage
		{
			if (state.WB.wrt_enable)
			{
				// Write back to register file
				if (state.WB.Wrt_reg_addr.to_ulong() != 0)													 // x0 is always zero. Cannot ovveride
					myRF.writeRF(state.WB.Wrt_reg_addr.to_ulong(), state.WB.Wrt_data); // Write data to register file
			}
		}

		/* --------------------- MEM stage -------------------- */
		/*RISC-V requires that word and half-word accesses are aligned; misaligned accesses should raise an exception or be handled appropriately.*/
		// memory alignment checks are missing
		if (!state.MEM.nop)
		{
			// Prepare data for WB stage
			nextState.WB.nop = false;
			// pass the values from MEM to WB
			nextState.WB.Rs = state.MEM.Rs;
			nextState.WB.Rt = state.MEM.Rt;
			nextState.WB.Wrt_reg_addr = state.MEM.Wrt_reg_addr;
			nextState.WB.wrt_enable = state.MEM.wrt_enable;

			if (state.MEM.rd_mem)
				if (state.MEM.rd_mem)
				{
					// Load from memory based on MemOp
					uint32_t address = state.MEM.ALUresult.to_ulong();
					bitset<32> memData;

					switch (state.MEM.mem_op)
					{
					case MemLB: // Load Byte
					{
						bitset<8> data = ext_dmem.readByte(address);
						int8_t signed_data = (int8_t)data.to_ulong();
						memData = bitset<32>(signed_data);
						break;
					}
					case MemLH: // Load Half Word
					{
						bitset<16> data = ext_dmem.readHalfWord(address);
						int16_t signed_data = (int16_t)data.to_ulong();
						memData = bitset<32>(signed_data);
						break;
					}
					case MemLW: // Load Word
					{
						memData = ext_dmem.readDataMem(address);
						break;
					}
					case MemLBU: // Load Byte Unsigned
					{
						bitset<8> data = ext_dmem.readByte(address);
						memData = bitset<32>(data.to_ulong());
						break;
					}
					case MemLHU: // Load Half Word Unsigned
					{
						bitset<16> data = ext_dmem.readHalfWord(address);
						memData = bitset<32>(data.to_ulong());
						break;
					}
					default:
						throw runtime_error("Unknown MemOp in MEM stage: " + to_string(state.MEM.mem_op));
					}

					nextState.WB.Wrt_data = memData;
				}
				else if (state.MEM.wrt_mem)
				{
					// Store to memory based on MemOp
					uint32_t address = state.MEM.ALUresult.to_ulong();
					bitset<32> storeData = state.MEM.Store_data;

					switch (state.MEM.mem_op)
					{
					case MemSB: // Store Byte
					{
						bitset<8> data = bitset<8>(storeData.to_ulong() & 0xFF);
						ext_dmem.writeByte(address, data);
						break;
					}
					case MemSH: // Store Half Word
					{
						bitset<16> data = bitset<16>(storeData.to_ulong() & 0xFFFF);
						ext_dmem.writeHalfWord(address, data);
						break;
					}
					case MemSW: // Store Word
					{
						ext_dmem.writeDataMem(address, storeData);
						break;
					}
					default:
						throw runtime_error("Unknown MemOp in MEM stage: " + to_string(state.MEM.mem_op));
					}

					nextState.WB.Wrt_data = 0; // Not used
				}
				else
				{
					// No memory operation
					nextState.WB.Wrt_data = state.MEM.ALUresult;
				}
		}
		else
			nextState.WB.nop = true; // MEM stage is nop, so WB stage is also nop

		/* --------------------- EX stage --------------------- */
		if (!state.EX.nop)
		{
			// Perform ALU operations
			bitset<32> operand1 = state.EX.Read_data1;
			bitset<32> operand2 = state.EX.is_I_type ? state.EX.Imm : state.EX.Read_data2;

			// Forwarding from MEM stage
			if (state.MEM.wrt_enable && state.MEM.Wrt_reg_addr.to_ulong() != 0)
			{
				if (state.MEM.Wrt_reg_addr == state.EX.Rs)
				{
					operand1 = state.MEM.ALUresult;
				}
				if (!state.EX.is_I_type && state.MEM.Wrt_reg_addr == state.EX.Rt)
				{
					operand2 = state.MEM.ALUresult;
				}
			}

			// Forwarding from WB stage
			if (state.WB.wrt_enable && state.WB.Wrt_reg_addr.to_ulong() != 0)
			{
				if (state.WB.Wrt_reg_addr == state.EX.Rs)
				{
					operand1 = state.WB.Wrt_data;
				}
				if (!state.EX.is_I_type && state.WB.Wrt_reg_addr == state.EX.Rt)
				{
					operand2 = state.WB.Wrt_data;
				}
			}

			bitset<32> ALUresult;

			// ALU operation based on alu_op
			switch (state.EX.alu_op)
			{
			case ADDU:
				ALUresult = bitset<32>(operand1.to_ulong() + operand2.to_ulong());
				break;
			case SUBU:
				ALUresult = bitset<32>(operand1.to_ulong() - operand2.to_ulong());
				break;
			case AND:
				ALUresult = bitset<32>(operand1.to_ulong() & operand2.to_ulong());
				break;
			case OR:
				ALUresult = bitset<32>(operand1.to_ulong() | operand2.to_ulong());
				break;
			case XOR:
				ALUresult = bitset<32>(operand1.to_ulong() ^ operand2.to_ulong());
				break;
			case SLL:
				ALUresult = bitset<32>(operand1.to_ulong() << (operand2.to_ulong() & 0x1F));
				break;
			case SRL:
				ALUresult = bitset<32>(operand1.to_ulong() >> (operand2.to_ulong() & 0x1F));
				break;
			case SRA:
				ALUresult = bitset<32>((int32_t)operand1.to_ulong() >> (operand2.to_ulong() & 0x1F));
				break;
			case SLT:
				ALUresult = bitset<32>((int32_t)operand1.to_ulong() < (int32_t)operand2.to_ulong());
				break;
			case SLTU:
				ALUresult = bitset<32>(operand1.to_ulong() < operand2.to_ulong());
				break;
			default:
				throw runtime_error("Unknown ALU operation: " + to_string(state.EX.alu_op));
				break;
			}

			// Prepare data for MEM stage
			nextState.MEM.nop = false;
			nextState.MEM.ALUresult = ALUresult;
			nextState.MEM.Store_data = operand2; // For store instructions
			nextState.MEM.Rs = state.EX.Rs;
			nextState.MEM.Rt = state.EX.Rt;
			nextState.MEM.Wrt_reg_addr = state.EX.Wrt_reg_addr;
			nextState.MEM.rd_mem = state.EX.rd_mem;
			nextState.MEM.wrt_mem = state.EX.wrt_mem;
			nextState.MEM.wrt_enable = state.EX.wrt_enable;
			nextState.MEM.mem_op = state.EX.mem_op;
			nextState.MEM.mem_op = state.EX.mem_op; // Pass mem_op to MEM stage
		}
		else
			nextState.MEM.nop = true; // EX stage is nop, so MEM stage is also nop

		/* --------------------- ID stage --------------------- */
		if (!state.ID.nop)
		{
			bitset<32> instruction = state.ID.Instr;

			// Decode instruction
			uint32_t instr = instruction.to_ulong();
			uint32_t opcode = instr & 0x7F;
			uint32_t rd = (instr >> 7) & 0x1F;
			uint32_t funct3 = (instr >> 12) & 0x7;
			uint32_t rs1 = (instr >> 15) & 0x1F;
			uint32_t rs2 = (instr >> 20) & 0x1F;
			uint32_t funct7 = (instr >> 25) & 0x7F;

			// Read registers
			bitset<32> Read_data1 = myRF.readRF(rs1);
			bitset<32> Read_data2 = myRF.readRF(rs2);

			// Hazard Detection
			bool stall = false;

			// Check for RAW hazards with EX stage
			if (state.EX.rd_mem && state.EX.Wrt_reg_addr.to_ulong() != 0) // checks if the instruction in EX stage is a load instr and that the writeback register is not x0
			{
				if (state.EX.Wrt_reg_addr.to_ulong() == rs1 || state.EX.Wrt_reg_addr.to_ulong() == rs2)
					stall = true; // Stall required
												// checks if the destination register of the instruction in EX stage is the source register of the instruction in ID
												// If the dest. reg = = source reg,
												// then there is a RAW hazard(Read after Write)
												// The instruction tries to read the value before the value is written back
			}

			if (stall)
			{
				// Insert stall
				nextState.ID = state.ID; // Keep the instruction in ID stage
				nextState.EX.nop = true; // Insert nop in EX stage
				// IF stage also needs to stall
				nextState.IF = state.IF;
				// stall helps the EX stage to finish the operation before the ID stage reads the value
			}
			else
			{
				// Prepare data for EX stage
				nextState.EX.nop = false;
				nextState.EX.Read_data1 = Read_data1;
				nextState.EX.Read_data2 = Read_data2;
				nextState.EX.Rs = rs1;
				nextState.EX.Rt = rs2;
				nextState.EX.Wrt_reg_addr = rd;
				nextState.EX.Imm = bitset<32>(signExtendImmediate(instr));

				// Set control signals based on opcode
				setControlSignals(opcode, funct3, funct7, nextState.EX);

				// Branch Handling
				if (opcode == 0x63) // Branch instructions
				{
					// Control Hazard Detection
					bool branchTaken = evaluateBranch(funct3, Read_data1, Read_data2);
					// checks if the branch condition is satisfied
					if (branchTaken)
					{
						// Flush IF stage
						nextState.IF.nop = true;
						// this is done to prevent the instruction in the IF stage from being executed- preventing control hazards
						// control hazards can also be prevented by stalling the pipeline, but branch prediction and flushing is more efficient

						// Update PC
						int32_t imm = getBranchImmediate(instr);
						nextState.IF.PC = bitset<32>(state.ID.PC.to_ulong() + imm);
					}
				}
			}
		}
		else
			nextState.EX.nop = true; // ID stage is nop, so EX stage is also nop

		/* --------------------- IF stage --------------------- */
		if (!state.IF.nop)
		{
			// Fetch instruction from instruction memory
			bitset<32> instruction = ext_imem.readInstr(state.IF.PC);

			// Prepare data for ID stage
			nextState.ID.nop = false;
			nextState.ID.Instr = instruction;
			nextState.ID.PC = state.IF.PC;

			// Update PC
			nextState.IF.PC = bitset<32>(state.IF.PC.to_ulong() + 4); // Fetches the next instruction
		}
		else
			nextState.ID.nop = true; // IF stage is nop, so ID stage is also nop

		/* --------------------- Check for halt & print state --------------------- */

		// Check if pipeline is empty
		if (state.IF.nop && state.ID.nop && state.EX.nop && state.MEM.nop && state.WB.nop)
			halted = true;

		myRF.outputRF(cycle);					// Dump RF
		printState(nextState, cycle); // Print states after executing this cycle

		state = nextState; // Update state
		cycle++;
	}

	void printState(stateStruct state, int cycle)
	{
		ofstream printstate;
		if (cycle == 0)
			printstate.open(opFilePath, std::ios_base::trunc);
		else
			printstate.open(opFilePath, std::ios_base::app);
		if (printstate.is_open())
		{
			printstate << "State after executing cycle:\t" << cycle << endl;

			printstate << "IF.PC:\t" << state.IF.PC.to_ulong() << endl;
			printstate << "IF.nop:\t" << state.IF.nop << endl;

			printstate << "ID.Instr:\t" << state.ID.Instr << endl;
			printstate << "ID.nop:\t" << state.ID.nop << endl;

			printstate << "EX.Read_data1:\t" << state.EX.Read_data1 << endl;
			printstate << "EX.Read_data2:\t" << state.EX.Read_data2 << endl;
			printstate << "EX.Imm:\t" << state.EX.Imm << endl;
			printstate << "EX.Rs:\t" << state.EX.Rs << endl;
			printstate << "EX.Rt:\t" << state.EX.Rt << endl;
			printstate << "EX.Wrt_reg_addr:\t" << state.EX.Wrt_reg_addr << endl;
			printstate << "EX.is_I_type:\t" << state.EX.is_I_type << endl;
			printstate << "EX.rd_mem:\t" << state.EX.rd_mem << endl;
			printstate << "EX.wrt_mem:\t" << state.EX.wrt_mem << endl;
			printstate << "EX.alu_op:\t" << state.EX.alu_op << endl;
			printstate << "EX.wrt_enable:\t" << state.EX.wrt_enable << endl;
			printstate << "EX.nop:\t" << state.EX.nop << endl;

			printstate << "MEM.ALUresult:\t" << state.MEM.ALUresult << endl;
			printstate << "MEM.Store_data:\t" << state.MEM.Store_data << endl;
			printstate << "MEM.Rs:\t" << state.MEM.Rs << endl;
			printstate << "MEM.Rt:\t" << state.MEM.Rt << endl;
			printstate << "MEM.Wrt_reg_addr:\t" << state.MEM.Wrt_reg_addr << endl;
			printstate << "MEM.rd_mem:\t" << state.MEM.rd_mem << endl;
			printstate << "MEM.wrt_mem:\t" << state.MEM.wrt_mem << endl;
			printstate << "MEM.wrt_enable:\t" << state.MEM.wrt_enable << endl;
			printstate << "MEM.nop:\t" << state.MEM.nop << endl;

			printstate << "WB.Wrt_data:\t" << state.WB.Wrt_data << endl;
			printstate << "WB.Rs:\t" << state.WB.Rs << endl;
			printstate << "WB.Rt:\t" << state.WB.Rt << endl;
			printstate << "WB.Wrt_reg_addr:\t" << state.WB.Wrt_reg_addr << endl;
			printstate << "WB.wrt_enable:\t" << state.WB.wrt_enable << endl;
			printstate << "WB.nop:\t" << state.WB.nop << endl;
		}
		else
			cout << "Unable to open FS StateResult output file." << endl;
		throw runtime_error("Unable to open FS StateResult output file.");
		printstate.close();
	}

private:
	string opFilePath;

	int32_t signExtendImmediate(uint32_t instr)
	{
		uint32_t opcode = instr & 0x7F;
		if (opcode == 0x13 || opcode == 0x03 || opcode == 0x67) // I-type
		{
			int32_t imm = (instr >> 20) & 0xFFF;
			if (imm & 0x800) // Sign bit
				imm |= 0xFFFFF000;
			return imm;
		}
		else if (opcode == 0x23) // S-type
		{
			int32_t imm = ((instr >> 25) << 5) | ((instr >> 7) & 0x1F);
			if (imm & 0x800) // Sign bit
				imm |= 0xFFFFF000;
			return imm;
		}
		// Add other types as needed
		return 0;
	}

	int32_t getBranchImmediate(uint32_t instr)
	{
		int32_t imm = ((instr >> 31) << 12) |
									(((instr >> 7) & 0x1) << 11) |
									(((instr >> 25) & 0x3F) << 5) |
									(((instr >> 8) & 0xF) << 1);
		if (imm & 0x1000) // Sign bit
			imm |= 0xFFFFE000;
		return imm;
	}

	void setControlSignals(uint32_t opcode, uint32_t funct3, uint32_t funct7, struct EXStruct &EX)
	{
		// Reset control signals
		EX.is_I_type = false;
		EX.rd_mem = false;
		EX.wrt_mem = false;
		EX.wrt_enable = false;
		EX.alu_op = NOP;		 // Default ALU operation
		EX.mem_op = MemNone; // Default memory operation

		switch (opcode)
		{
		case 0x33: // R-type instructions
			EX.is_I_type = false;
			EX.wrt_enable = true;
			EX.rd_mem = false;
			EX.wrt_mem = false;

			switch (funct3)
			{
			case 0x0:
				if (funct7 == 0x00)
				{
					// ADD
					EX.alu_op = ADDU;
				}
				else if (funct7 == 0x20)
				{
					// SUB
					EX.alu_op = SUBU;
				}
				break;
			case 0x1:
				// SLL
				EX.alu_op = SLL;
				break;
			case 0x2:
				// SLT
				EX.alu_op = SLT;
				break;
			case 0x3:
				// SLTU
				EX.alu_op = SLTU;
				break;
			case 0x4:
				// XOR
				EX.alu_op = XOR;
				break;
			case 0x5:
				if (funct7 == 0x00)
				{
					// SRL
					EX.alu_op = SRL;
				}
				else if (funct7 == 0x20)
				{
					// SRA
					EX.alu_op = SRA;
				}
				break;
			case 0x6:
				// OR
				EX.alu_op = OR;
				break;
			case 0x7:
				// AND
				EX.alu_op = AND;
				break;
			default:
				// Unknown funct3
				throw runtime_error("Unknown funct3 in R-type instruction: " + to_string(funct3));
				break;
			}
			break;

		case 0x03: // Load instructions
			EX.is_I_type = true;
			EX.wrt_enable = true;
			EX.rd_mem = true;
			EX.wrt_mem = false;
			EX.alu_op = ADDU; // Address calculation

			// Determine memory operation based on funct3
			switch (funct3)
			{
			case 0x0:
				EX.mem_op = MemLB; // Load Byte
				break;
			case 0x1:
				EX.mem_op = MemLH; // Load Half Word
				break;
			case 0x2:
				EX.mem_op = MemLW; // Load Word
				break;
			case 0x4:
				EX.mem_op = MemLBU; // Load Byte Unsigned
				break;
			case 0x5:
				EX.mem_op = MemLHU; // Load Half Word Unsigned
				break;
			default:
				throw runtime_error("Unknown funct3 in Load instruction: " + to_string(funct3));
			}
			break;

		case 0x23: // Store instructions
			EX.is_I_type = true;
			EX.wrt_enable = false;
			EX.rd_mem = false;
			EX.wrt_mem = true;
			EX.alu_op = ADDU; // Address calculation

			// Determine memory operation based on funct3
			switch (funct3)
			{
			case 0x0:
				EX.mem_op = MemSB; // Store Byte
				break;
			case 0x1:
				EX.mem_op = MemSH; // Store Half Word
				break;
			case 0x2:
				EX.mem_op = MemSW; // Store Word
				break;
			default:
				throw runtime_error("Unknown funct3 in Store instruction: " + to_string(funct3));
			}
			break;

		case 0x13: // I-type arithmetic instructions
			EX.is_I_type = true;
			EX.wrt_enable = true;
			EX.rd_mem = false;
			EX.wrt_mem = false;

			switch (funct3)
			{
			case 0x0: // ADDI
				EX.alu_op = ADDU;
				break;
			case 0x2: // SLTI
				EX.alu_op = SLT;
				break;
			case 0x3: // SLTIU
				EX.alu_op = SLTU;
				break;
			case 0x4: // XORI
				EX.alu_op = XOR;
				break;
			case 0x6: // ORI
				EX.alu_op = OR;
				break;
			case 0x7: // ANDI
				EX.alu_op = AND;
				break;
			case 0x1: // SLLI
				EX.alu_op = SLL;
				break;
			case 0x5:
				if ((funct7 & 0x20) == 0x00)
				{
					// SRLI
					EX.alu_op = SRL;
				}
				else if ((funct7 & 0x20) == 0x20)
				{
					// SRAI
					EX.alu_op = SRA;
				}
				else
				{
					throw runtime_error("Unknown funct7 in SRLI/SRAI instruction: " + to_string(funct7));
				}
				break;
			default:
				throw runtime_error("Unknown funct3 in I-type arithmetic instruction: " + to_string(funct3));
				break;
			}
			break;

		case 0x63: // Branch instructions
			// Branch instructions are handled in the ID stage
			EX.is_I_type = false;
			EX.wrt_enable = false;
			EX.rd_mem = false;
			EX.wrt_mem = false;
			EX.alu_op = SUBU; // For comparison
			break;

		case 0x6F: // JAL
			// JAL instruction
			EX.is_I_type = false;
			EX.wrt_enable = true;
			EX.rd_mem = false;
			EX.wrt_mem = false;
			// No ALU operation needed
			break;

		case 0x67: // JALR
			// JALR instruction
			EX.is_I_type = true;
			EX.wrt_enable = true;
			EX.rd_mem = false;
			EX.wrt_mem = false;
			// No ALU operation needed
			break;

		case 0x37: // LUI
			EX.is_I_type = true;
			EX.wrt_enable = true;
			EX.rd_mem = false;
			EX.wrt_mem = false;
			EX.alu_op = ADDU; // Load upper immediate
			break;

		case 0x17: // AUIPC
			EX.is_I_type = true;
			EX.wrt_enable = true;
			EX.rd_mem = false;
			EX.wrt_mem = false;
			EX.alu_op = ADDU; // Add upper immediate to PC
			break;

		case 0x73: // System instructions
			// System instructions like ECALL, EBREAK
			EX.is_I_type = false;
			EX.wrt_enable = false;
			EX.rd_mem = false;
			EX.wrt_mem = false;
			break;

		default:
			throw runtime_error("Unknown opcode encountered: " + to_string(opcode));
			break;
		}
	}

	bool evaluateBranch(uint32_t funct3, bitset<32> Read_data1, bitset<32> Read_data2)
	{
		// Convert bitsets to unsigned and signed integers
		uint32_t op1_u = Read_data1.to_ulong();
		uint32_t op2_u = Read_data2.to_ulong();
		int32_t op1_s = static_cast<int32_t>(op1_u);
		int32_t op2_s = static_cast<int32_t>(op2_u);

		switch (funct3)
		{
		case 0x0: // BEQ
			return op1_u == op2_u;
		case 0x1: // BNE
			return op1_u != op2_u;
		case 0x4: // BLT
			return op1_s < op2_s;
		case 0x5: // BGE
			return op1_s >= op2_s;
		case 0x6: // BLTU
			return op1_u < op2_u;
		case 0x7: // BGEU
			return op1_u >= op2_u;
		default:
			// Unknown branch type
			return false;
		}
	}
};

/*
 * The main function will include the following:
 * - Instantiation of the instruction memory, data memory, register file, and the cores.
 * - The main loop that will run the cores until they are halted.
 * - Dumping the data memory and register file at the end of the simulation.
 */
int main(int argc, char *argv[])
{
	try
	{
		// The input directory for the memory files
		string ioDir = "";
		if (argc == 1)
		{
			cout << "Enter path containing the memory files: ";
			cin >> ioDir;
		}
		else if (argc > 2)
		{
			throw runtime_error("Invalid number of arguments. Machine stopped.");
			return -1;
		}
		else
		{
			ioDir = argv[1];
			cout << "IO Directory: " << ioDir << endl;
		}

		InsMem imem = InsMem(MemType::Imem, ioDir);
		DataMem dmem_ss = DataMem(MemType::DmemSS, ioDir);
		DataMem dmem_fs = DataMem(MemType::DmemFS, ioDir);

		SingleStageCore SSCore(ioDir, imem, dmem_ss);
		// FiveStageCore FSCore(ioDir, imem, dmem_fs);

		while (1)
		{
			if (!SSCore.halted)
				SSCore.step();
			else
			{
				// Ensure the nop flag is set correctly after halting
				SSCore.state.SS.nop = true;
				SSCore.printState(SSCore.state, SSCore.cycle); // The final state after halting
				break;
			}

			// if (!FSCore.halted)
			// 	FSCore.step();

			// if (SSCore.halted && FSCore.halted)
			// 	break;
		}

		// dump SS and FS data mem.
		dmem_ss.outputDataMem();
		// dmem_fs.outputDataMem();
	}
	catch (const exception &e)
	{
		throw runtime_error("An error occurred: " + string(e.what()));
		return -1;
	}

	return 0;
}
