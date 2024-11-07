#include <iostream>
#include <string>
#include <vector>
#include <bitset>
#include <fstream>
#include <stdexcept>
#include "types.cpp"
#include "single_stage.cpp"

using namespace std;

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
				SSCore.printEvaluation(ioDir);
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
