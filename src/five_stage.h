#include "types.h"

using namespace std;
using namespace std::filesystem;

class FiveStageCore : public Core
{
public:
	FiveStageCore(path ioDir, InsMem &imem, DataMem &dmem);
	void step();
	void printState(stateStruct state, int cycle);
	void printEvaluation(string ioDir);

private:
	path opFilePath;
	int32_t getJTypeImmediate(uint32_t instr);
	int32_t getITypeImmediate(uint32_t instr, bool isLogical = false);
	int32_t getSTypeImmediate(uint32_t instr);
	int32_t getUTypeImmediate(uint32_t instr);
	int32_t getBTypeImmediate(uint32_t instr);
	int32_t signExtendImmediate(uint32_t instr);
	int32_t getBranchImmediate(uint32_t instr);
	void setControlSignals(uint32_t opcode, uint32_t funct3, uint32_t funct7, struct EXStruct &EX);
	bool evaluateBranch(uint32_t funct3, bitset<32> Read_data1, bitset<32> Read_data2);
	void flushPipeline(stateStruct &nextState);
};
