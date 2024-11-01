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
};

struct EXStruct
{
	bitset<32> Read_data1;
	bitset<32> Read_data2;
	bitset<16> Imm;
	bitset<5> Rs;
	bitset<5> Rt;
	bitset<5> Wrt_reg_addr;
	bool is_I_type;
	bool rd_mem;
	bool wrt_mem;
	bool alu_op; // 1 for addu, lw, sw, 0 for subu
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
				IMem[i] = bitset<8>(line);
				i++;
			}
			imem.close();
		}
		catch (const exception &e)
		{
			cout << e.what() << endl;
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
			cout << e.what() << endl;
		}
	}

	bitset<32> readDataMem(bitset<32> Address)
	{
		// Convert Address to an integer for indexing
		uint32_t address = (uint32_t)Address.to_ulong();

		// Bounds check to ensure we're within memory limits
		if (address + 3 >= MemSize)
		{
			throw runtime_error("Read address is out of bounds.");
		}

		// Combine 4 bytes to form a 32-bit word (Big-Endian order)
		bitset<32> data;
		for (int i = 0; i < 4; ++i)
		{
			data <<= 8;																				// Shift left by 8 bits to make room for the next byte
			data |= bitset<32>(DMem[address + i].to_ulong()); // Combine the byte
		}

		return data;
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
			cout << "Unable to open " << MemType::toString(id) << " DMEM result file." << endl;
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
				cout << "Unable to open RF output file." << endl;
			rfout.close();
		}
		catch (const exception &e)
		{
			cout << "An error occurred in RegisterFile::outputRF: " << e.what() << endl;
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
	InsMem ext_imem;
	DataMem ext_dmem;

	Core(string ioDir, InsMem &imem, DataMem &dmem) : myRF(ioDir), ioDir{ioDir}, ext_imem{imem}, ext_dmem{dmem} {}

	virtual void step() {}

	virtual void printState() {}
};

class SingleStageCore : public Core
{
public:
	SingleStageCore(string ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir + "/SS_", imem, dmem), opFilePath(ioDir + "/StateResult_SS.txt") {}

	void step()
	{
		try
		{
			// if (cycle > 10)
			// { // Arbitrary cycle limit for debugging
			// 	halted = true;
			// 	// Limiting the number of cycles to prevent infinite loops for simulation, real hardware would not have this limit constraint
			// 	// Comment out this code block to run the simulation indefinitely
			// 	throw runtime_error("Cycle limit reached, halting execution.");
			// }

			// 1. Fetch the instruction at the current PC
			bitset<32> instruction = ext_imem.readInstr(state.SS.PC);

			// Debug statement
			cout << "[DEBUG] Cycle: " << cycle + 1 << ", PC: " << state.SS.PC.to_ulong() << ", Instruction: " << instruction << endl;

			// Check if it's a HALT instruction (all 1s in RISC-V convention for HALT)
			if (instruction.to_ulong() == 0xFFFFFFFF)
			{
				halted = true;
				state.SS.nop = true;
				printState(state, cycle);
				return;
			}

			// 2. Decode the instruction fields
			int opcode = stoi(instruction.to_string().substr(25, 7), nullptr, 2); // opcode is last 7 bits
			int rd = stoi(instruction.to_string().substr(20, 5), nullptr, 2);			// destination register
			int funct3 = stoi(instruction.to_string().substr(17, 3), nullptr, 2); // funct3 field
			int rs1 = stoi(instruction.to_string().substr(12, 5), nullptr, 2);		// source register 1
			int rs2 = stoi(instruction.to_string().substr(7, 5), nullptr, 2);			// source register 2
			int funct7 = stoi(instruction.to_string().substr(0, 7), nullptr, 2);	// funct7 for R-type

			// Prepare variables for result
			bitset<32> result;
			bitset<32> address;

			// 3. Execute based on opcode
			switch (opcode)
			{
			case 0x33: // R-type instructions (e.g., ADD, SUB)
				switch (funct3)
				{
				case 0x0: // ADD or SUB based on funct7
					if (funct7 == 0x00)
					{ // ADD
						result = bitset<32>(myRF.readRF(rs1).to_ulong() + myRF.readRF(rs2).to_ulong());
					}
					else if (funct7 == 0x20)
					{ // SUB
						result = bitset<32>(myRF.readRF(rs1).to_ulong() - myRF.readRF(rs2).to_ulong());
					}
					break;
					// Add more cases here if other R-type instructions are required
				}
				myRF.writeRF(rd, result);
				break;

			case 0x03: // I-type instructions (e.g., LW)
				switch (funct3)
				{
				case 0x2: // LW (Load Word)
					address = bitset<32>(myRF.readRF(rs1).to_ulong() + stoi(instruction.to_string().substr(0, 12), nullptr, 2));
					result = ext_dmem.readDataMem(address);
					myRF.writeRF(rd, result);
					break;
				}
				break;

			case 0x23: // S-type instructions (e.g., SW)
				switch (funct3)
				{
				case 0x2: // SW (Store Word)
					address = bitset<32>(myRF.readRF(rs1).to_ulong() + stoi(instruction.to_string().substr(0, 12), nullptr, 2));
					ext_dmem.writeDataMem(address, myRF.readRF(rs2));
					break;
				}
				break;

			default:
				cout << "Unknown opcode encountered: " << opcode << endl;
				break;
			}

			// 4. Update PC for the next instruction
			state.SS.PC = bitset<32>(state.SS.PC.to_ulong() + 4);
			// Debug statement
			// cout << "PC: " << state.SS.PC.to_ulong() << endl;

			// Output register file and state for this cycle
			myRF.outputRF(cycle);			// dump RF
			printState(state, cycle); // print states after executing this cycle

			// Single stage core only executes one instruction per cycle, there is no pipeline, thus the state is not updated
			// state = nextState; // This statement is commented on purpose
			cycle++;
		}
		catch (const exception &e)
		{
			cout << "An error occurred in SingleStageCore::step: " << e.what() << endl;
		}
	}

	void printState(stateStruct state, int cycle)
	{
		try
		{
			ofstream printstate;
			// If it's the first cycle, truncate the file, otherwise append
			if (cycle == 0)
			{
				printstate.open(opFilePath, std::ios_base::trunc);
				printstate << "----------------------------------------------------------------------" << endl;
			}

			else
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
				cout << "Unable to open SS StateResult output file." << endl;
			}
			printstate.close();
		}
		catch (const exception &e)
		{
			cout << "An error occurred in SingleStageCore::printState: " << e.what() << endl;
		}
	}

private:
	string opFilePath;
};

// class FiveStageCore : public Core
// {
// public:
// 	FiveStageCore(string ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir + "\\FS_", imem, dmem), opFilePath(ioDir + "\\StateResult_FS.txt") {}

// 	void step()
// 	{
// 		/* Your implementation */
// 		/* --------------------- WB stage --------------------- */

// 		/* --------------------- MEM stage -------------------- */

// 		/* --------------------- EX stage --------------------- */

// 		/* --------------------- ID stage --------------------- */

// 		/* --------------------- IF stage --------------------- */

// 		halted = true;
// 		if (state.IF.nop && state.ID.nop && state.EX.nop && state.MEM.nop && state.WB.nop)
// 			halted = true;

// 		myRF.outputRF(cycle);					// dump RF
// 		printState(nextState, cycle); // print states after executing cycle 0, cycle 1, cycle 2 ...

// 		state = nextState; // The end of the cycle and updates the current state with the values calculated in this cycle
// 		cycle++;
// 	}

// 	void printState(stateStruct state, int cycle)
// 	{
// 		ofstream printstate;
// 		if (cycle == 0)
// 			printstate.open(opFilePath, std::ios_base::trunc);
// 		else
// 			printstate.open(opFilePath, std::ios_base::app);
// 		if (printstate.is_open())
// 		{
// 			printstate << "State after executing cycle:\t" << cycle << endl;

// 			printstate << "IF.PC:\t" << state.IF.PC.to_ulong() << endl;
// 			printstate << "IF.nop:\t" << state.IF.nop << endl;

// 			printstate << "ID.Instr:\t" << state.ID.Instr << endl;
// 			printstate << "ID.nop:\t" << state.ID.nop << endl;

// 			printstate << "EX.Read_data1:\t" << state.EX.Read_data1 << endl;
// 			printstate << "EX.Read_data2:\t" << state.EX.Read_data2 << endl;
// 			printstate << "EX.Imm:\t" << state.EX.Imm << endl;
// 			printstate << "EX.Rs:\t" << state.EX.Rs << endl;
// 			printstate << "EX.Rt:\t" << state.EX.Rt << endl;
// 			printstate << "EX.Wrt_reg_addr:\t" << state.EX.Wrt_reg_addr << endl;
// 			printstate << "EX.is_I_type:\t" << state.EX.is_I_type << endl;
// 			printstate << "EX.rd_mem:\t" << state.EX.rd_mem << endl;
// 			printstate << "EX.wrt_mem:\t" << state.EX.wrt_mem << endl;
// 			printstate << "EX.alu_op:\t" << state.EX.alu_op << endl;
// 			printstate << "EX.wrt_enable:\t" << state.EX.wrt_enable << endl;
// 			printstate << "EX.nop:\t" << state.EX.nop << endl;

// 			printstate << "MEM.ALUresult:\t" << state.MEM.ALUresult << endl;
// 			printstate << "MEM.Store_data:\t" << state.MEM.Store_data << endl;
// 			printstate << "MEM.Rs:\t" << state.MEM.Rs << endl;
// 			printstate << "MEM.Rt:\t" << state.MEM.Rt << endl;
// 			printstate << "MEM.Wrt_reg_addr:\t" << state.MEM.Wrt_reg_addr << endl;
// 			printstate << "MEM.rd_mem:\t" << state.MEM.rd_mem << endl;
// 			printstate << "MEM.wrt_mem:\t" << state.MEM.wrt_mem << endl;
// 			printstate << "MEM.wrt_enable:\t" << state.MEM.wrt_enable << endl;
// 			printstate << "MEM.nop:\t" << state.MEM.nop << endl;

// 			printstate << "WB.Wrt_data:\t" << state.WB.Wrt_data << endl;
// 			printstate << "WB.Rs:\t" << state.WB.Rs << endl;
// 			printstate << "WB.Rt:\t" << state.WB.Rt << endl;
// 			printstate << "WB.Wrt_reg_addr:\t" << state.WB.Wrt_reg_addr << endl;
// 			printstate << "WB.wrt_enable:\t" << state.WB.wrt_enable << endl;
// 			printstate << "WB.nop:\t" << state.WB.nop << endl;
// 		}
// 		else
// 			cout << "Unable to open FS StateResult output file." << endl;
// 		printstate.close();
// 	}

// private:
// 	string opFilePath;
// };

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
			cout << "Invalid number of arguments. Machine stopped." << endl;
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

			// if (!FSCore.halted)
			// 	FSCore.step();

			if (SSCore.halted)
				// && FSCore.halted)
				break;
		}

		// dump SS and FS data mem.
		dmem_ss.outputDataMem();
		// dmem_fs.outputDataMem();
	}
	catch (const exception &e)
	{
		cout << "An error occurred: " << e.what() << endl;
		return -1;
	}

	return 0;
}
