#include "single_stage.h"

SingleStageCore::SingleStageCore(path ioDir, InsMem &imem, DataMem &dmem) : Core(ioDir, imem, dmem, "SS"), opFilePath(ioDir / "SS_StateResult.txt")
{
	// Initialize PC to 0
	state.SS.PC = bitset<32>(0);
}

void SingleStageCore::step()
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

void SingleStageCore::printState(stateStruct state, int cycle) // A Debug function to print the state of the core- used for verifying the cycle-by-cycle operation
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

void SingleStageCore::printEvaluation(string ioDir) // A function to print the performance of the single-stage pipeline
{
	try
	{
		ofstream eval;
		string filePath = ioDir + "/SS_PerformanceMetrics.txt";
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

int32_t SingleStageCore::get_imm_i(uint32_t instr, bool isLogical)
{
	int32_t imm = (instr >> 20) & 0xFFF; // Extract bits [31:20]
	if (!isLogical && (imm & 0x800))		 // Check if sign bit (bit 11) is set and not logical
		imm |= 0xFFFFF000;								 // Sign-extend to 32 bits
	return imm;
}

// Helper function to sign-extend S-type immediate
int32_t SingleStageCore::get_imm_s(uint32_t instr)
{
	int32_t imm = ((instr >> 25) << 5) | ((instr >> 7) & 0x1F); // Extract bits [31:25] and [11:7]
	if (imm & 0x800)																						// Check if sign bit (bit 11) is set
		imm |= 0xFFFFF000;																				// Sign-extend to 32 bits
	return imm;
}

// Helper function to sign-extend B-type immediate
int32_t SingleStageCore::get_imm_b(uint32_t instr)
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
int32_t SingleStageCore::get_imm_u(uint32_t instr)
{
	int32_t imm = instr & 0xFFFFF000; // Extract bits [31:12]
	return imm;
}

// Helper function to sign-extend J-type immediate
int32_t SingleStageCore::get_imm_j(uint32_t instr)
{
	int32_t imm = ((instr >> 31) << 20) |					 // Extract bit 31 and shift to bit 20
								(((instr >> 12) & 0xFF) << 12) | // Extract bits [19:12] and shift to bits [19:12]
								(((instr >> 20) & 0x1) << 11) |	 // Extract bit 20 and shift to bit 11
								(((instr >> 21) & 0x3FF) << 1);	 // Extract bits [10:1] and shift to bits [10:1]

	if (imm & 0x100000)
		imm |= 0xFFE00000;
	return imm;
}