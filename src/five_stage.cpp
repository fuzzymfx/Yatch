#include "types.h"
#include <five_stage.h>

using namespace std;
using namespace std::filesystem;

FiveStageCore::FiveStageCore(path ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir, imem, dmem, "FS"), opFilePath(ioDir / "FS_StateResult.txt")
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

void FiveStageCore::step()
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

		// Handle control flow instructions
		if (state.EX.is_jal || state.EX.is_jalr)
		{
			// Compute the return address
			bitset<32> return_address = bitset<32>(state.EX.PC.to_ulong() + 4);

			// Compute the target address
			bitset<32> target_address;
			if (state.EX.is_jal)
			{
				// JAL: PC + immediate
				target_address = bitset<32>(state.EX.PC.to_ulong() + state.EX.Imm.to_ulong());
			}
			else // JALR
			{
				// JALR: (rs1 + immediate) & ~1
				uint32_t temp_address = operand1.to_ulong() + state.EX.Imm.to_ulong();
				target_address = bitset<32>(temp_address & ~1); // Clear the least significant bit
			}

			// Update the PC
			nextState.IF.PC = target_address;

			// Set ALUresult to return address to write back to rd
			ALUresult = return_address;

			// Handle control hazards: Flush ID stage
			nextState.ID.nop = true;
		}
		else if (state.EX.is_auipc)
		{
			// AUIPC: PC + (immediate << 12)
			ALUresult = bitset<32>(state.EX.PC.to_ulong() + (state.EX.Imm.to_ulong() << 12));
		}
		else
		{
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
		// Check for RAW hazards with EX stage
		if (state.EX.wrt_enable && state.EX.Wrt_reg_addr.to_ulong() != 0)
		{
			if (state.EX.Wrt_reg_addr == rs1 || (!state.EX.is_I_type && state.EX.Wrt_reg_addr == rs2))
			{
				stall = true; // Stall required
			}
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
			nextState.EX.PC = state.ID.PC;
			nextState.EX.opcode = opcode;

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

void FiveStageCore::printState(stateStruct state, int cycle)
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

void FiveStageCore::printEvaluation(string ioDir)
{
	try
	{
		ofstream eval;
		string filePath = ioDir + "/FS_PerformanceMetrics.txt";
		eval.open(filePath, std::ios_base::trunc);
		if (eval.is_open())
		{
			eval << "-----------------------------Single Stage Core Performance Metrics-----------------------------" << endl;
			eval << "Number of cycles taken: " << cycle + 2 << endl;
			eval << "Total Number of Instructions: " << ext_imem.totalInstructions() << endl;
			eval << "Cycles per instruction: " << ((double)cycle + 2) / ext_imem.totalInstructions() << endl;
			eval << "Instructions per cycle: " << ext_imem.totalInstructions() / ((double)cycle + 2) << endl;
		}
		else
		{
			throw runtime_error("Unable to open SS StateResult output file.");
		}
	}
	catch (const exception &e)
	{
		throw runtime_error("An error occurred in SingleStageCore::printEvaluation: " + string(e.what()));
	}
}

int32_t FiveStageCore::getJTypeImmediate(uint32_t instr)
{
	int32_t imm = 0;
	imm |= ((instr >> 31) & 0x1) << 20;	 // imm[20]
	imm |= ((instr >> 21) & 0x3FF) << 1; // imm[10:1]
	imm |= ((instr >> 20) & 0x1) << 11;	 // imm[11]
	imm |= ((instr >> 12) & 0xFF) << 12; // imm[19:12]
	// Sign-extend to 21 bits
	if (imm & (1 << 20))
		imm |= 0xFFE00000;
	return imm;
}
int32_t FiveStageCore::getITypeImmediate(uint32_t instr, bool isShift = false)
{
	if (isShift)
	{
		// Extract shamt for shift instructions (bits [24:20])
		return (instr >> 20) & 0x1F;
	}
	else
	{
		int32_t imm = instr >> 20;
		if (imm & 0x800) // Sign-extend
			imm |= 0xFFFFF000;
		return imm;
	}
}
int32_t FiveStageCore::getBTypeImmediate(uint32_t instr)
{
	int32_t imm = 0;
	imm |= ((instr >> 31) & 0x1) << 12; // imm[12]
	imm |= ((instr >> 25) & 0x3F) << 5; // imm[11:5]
	imm |= ((instr >> 8) & 0xF) << 1;		// imm[4:1]
	imm |= ((instr >> 7) & 0x1) << 11;	// imm[10]
	// Sign-extend to 13 bits
	if (imm & (1 << 12))
		imm |= 0xFFFFE000;
	return imm;
}
int32_t FiveStageCore::getSTypeImmediate(uint32_t instr)
{
	int32_t imm = 0;
	imm |= ((instr >> 25) & 0x7F) << 5; // imm[11:5]
	imm |= ((instr >> 7) & 0x1F);				// imm[4:0]
	// Sign-extend to 12 bits
	if (imm & (1 << 11))
		imm |= 0xFFFFF000;
	return imm;
}
int32_t FiveStageCore::getUTypeImmediate(uint32_t instr)
{
	int32_t imm = instr & 0xFFFFF000;
	return imm;
}

int32_t FiveStageCore::signExtendImmediate(uint32_t instr)
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

int32_t FiveStageCore::getBranchImmediate(uint32_t instr)
{
	int32_t imm = ((instr >> 31) << 12) |
								(((instr >> 7) & 0x1) << 11) |
								(((instr >> 25) & 0x3F) << 5) |
								(((instr >> 8) & 0xF) << 1);
	if (imm & 0x1000) // Sign bit
		imm |= 0xFFFFE000;
	return imm;
}

void FiveStageCore::setControlSignals(uint32_t opcode, uint32_t funct3, uint32_t funct7, struct EXStruct &EX)
{
	// Reset control signals
	EX.is_I_type = false;
	EX.rd_mem = false;
	EX.wrt_mem = false;
	EX.wrt_enable = false;
	EX.alu_op = NOP;		 // Default ALU operation
	EX.mem_op = MemNone; // Default memory operation
	EX.is_jal = false;
	EX.is_jalr = false;
	EX.is_auipc = false;

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
		EX.is_jal = true;
		EX.is_I_type = false;
		EX.wrt_enable = true;
		EX.rd_mem = false;
		EX.wrt_mem = false;
		// No ALU operation needed
		break;

	case 0x67: // JALR
		// JALR instruction
		EX.is_jalr = true;
		EX.is_I_type = true;
		EX.wrt_enable = true;
		EX.rd_mem = false;
		EX.wrt_mem = false;
		// No ALU operation needed
		break;

	case 0x37: // LUI
		EX.is_I_type = false;
		EX.wrt_enable = true;
		EX.rd_mem = false;
		EX.wrt_mem = false;
		EX.alu_op = NOP; // No ALU operation needed
		break;

	case 0x17: // AUIPC
		EX.is_auipc = true;
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

bool FiveStageCore::evaluateBranch(uint32_t funct3, bitset<32> Read_data1, bitset<32> Read_data2)
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

void FiveStageCore::flushPipeline(stateStruct &nextState)
{
	nextState.ID.nop = true;
	nextState.EX.nop = true;
	nextState.MEM.nop = true;
	nextState.WB.nop = true;
};
