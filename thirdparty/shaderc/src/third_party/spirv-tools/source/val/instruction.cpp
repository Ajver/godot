// Copyright (c) 2015-2016 The Khronos Group Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "val/instruction.h"

#include <utility>

using std::make_pair;

namespace spvtools {
namespace val {

#define OPERATOR(OP)                                                 \
  bool operator OP(const Instruction& lhs, const Instruction& rhs) { \
    return lhs.id() OP rhs.id();                                     \
  }                                                                  \
  bool operator OP(const Instruction& lhs, uint32_t rhs) {           \
    return lhs.id() OP rhs;                                          \
  }

OPERATOR(<)
OPERATOR(==)
#undef OPERATOR

Instruction::Instruction(const spv_parsed_instruction_t* inst,
                         Function* defining_function,
                         BasicBlock* defining_block)
    : words_(inst->words, inst->words + inst->num_words),
      operands_(inst->operands, inst->operands + inst->num_operands),
      inst_({words_.data(), inst->num_words, inst->opcode, inst->ext_inst_type,
             inst->type_id, inst->result_id, operands_.data(),
             inst->num_operands}),
      instruction_position_(0),
      function_(defining_function),
      block_(defining_block),
      uses_() {}

void Instruction::RegisterUse(const Instruction* inst, uint32_t index) {
  uses_.push_back(make_pair(inst, index));
}

}  // namespace val
}  // namespace spvtools
