#ifndef KICKPARAMBLOCK_FD54EE88
#define KICKPARAMBLOCK_FD54EE88

#include "MemoryBlock.h"
#include <motion/KickParameters.h>

struct KickParamBlock : public MemoryBlock {
  KickParamBlock():
    send_params_(false),
    params_(),
    params_super_()
  {
    header.version = 3;
    header.size = sizeof(KickParamBlock);
  }
  
  bool send_params_;
  KickParameters params_;
  KickParameters params_super_;
};

#endif /* end of include guard: KICKPARAMBLOCK_FD54EE88 */
