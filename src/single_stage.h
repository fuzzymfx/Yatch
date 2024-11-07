#include "types.h"

class SingleStageCore : public Core
{
public:
	SingleStageCore(string ioDir, InsMem &imem, DataMem &dmem);
	void step();
	void printState(stateStruct state, int cycle);
	void printEvaluation(string ioDir);

private:
	string opFilePath;
	int32_t get_imm_i(uint32_t instr, bool isLogical = false);
	int32_t get_imm_s(uint32_t instr);
	int32_t get_imm_b(uint32_t instr);
	int32_t get_imm_u(uint32_t instr);
	int32_t get_imm_j(uint32_t instr);
};
