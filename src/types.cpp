#include "types.h"
#include <string>

// Implementation of MemType::toString
string MemType::toString(Type type)
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

// Implementation of InsMem methods
InsMem::InsMem(MemType::Type type, string ioDir) : id(type), ioDir(ioDir)
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
			size++;
		}
		imem.close();
	}
	catch (const exception &e)
	{
		throw runtime_error("An error occurred: " + string(e.what()));
	}
}

int InsMem::totalInstructions()
{
	return size / 4;
}

bitset<32> InsMem::readInstr(bitset<32> ReadAddress)
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

// Implementation of DataMem methods
DataMem::DataMem(MemType::Type type, string ioDir) : id{type}, ioDir{ioDir}
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

bitset<8> DataMem::readByte(uint32_t address)
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

bitset<16> DataMem::readHalfWord(uint32_t address)
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

void DataMem::writeByte(uint32_t address, bitset<8> data)
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

void DataMem::writeHalfWord(uint32_t address, bitset<16> data)
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

bitset<32> DataMem::readDataMem(bitset<32> Address)
{
	uint32_t address = (uint32_t)Address.to_ulong();

	if (address % 4 != 0)
	{
		throw runtime_error("Misaligned memory access at address: " + to_string(address));
	}

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

void DataMem::writeDataMem(bitset<32> Address, bitset<32> WriteData)
{
	// Convert Address to an integer for indexing
	uint32_t address = Address.to_ulong();

	if (address % 4 != 0)
	{
		throw runtime_error("Misaligned memory access at address: " + to_string(address));
	}

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

void DataMem::outputDataMem()
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

// Implementation of RegisterFile methods
RegisterFile::RegisterFile(string ioDir) : outputFile{ioDir + "RFResult.txt"}
{
	Registers.resize(32);					// 32 registers in total, each 32 bits wide
	Registers[0] = bitset<32>(0); // Register x0 is always 0 in RISC-V
}

bitset<32> RegisterFile::readRF(bitset<5> Reg_addr)
{
	// Converts Reg_addr to integer for indexing
	uint32_t reg_index = Reg_addr.to_ulong();

	// Returns the register value at the given index
	return Registers[reg_index];
}

void RegisterFile::writeRF(bitset<5> Reg_addr, bitset<32> Wrt_reg_data)
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
void RegisterFile::outputRF(int cycle)
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

// Implementation of Core constructor
Core::Core(string ioDir, InsMem &imem, DataMem &dmem) : myRF(ioDir), ioDir{ioDir}, ext_imem{imem}, ext_dmem{dmem} {}
