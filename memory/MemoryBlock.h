#ifndef MEMORYBLOCK_4MKDDAED
#define MEMORYBLOCK_4MKDDAED

#include <cstring>
#include <string>
#include <stdlib.h>
#include <memory/StreamBuffer.h>

struct MemoryBlockHeader {
  unsigned int version;
  unsigned int size;
  unsigned int frameid;
};

namespace MemoryOwner {
  enum Owner {
    UNKNOWN, // means it didn't get set = BAD
    MOTION,
    VISION,
    INTERFACE,
    IMAGE_CAPTURE,
    SYNC,
    TOOL_MEM, // no blocks are currently owned by the tool, but for the memory itself?
    SHARED
  };
};

class MemoryBlock {
public:
  MemoryBlock();
  virtual ~MemoryBlock() {}
  
  MemoryBlockHeader header;
  bool log_block;
  MemoryOwner::Owner owner;

  virtual MemoryBlock& operator=(const MemoryBlock &that);

  void serialize(StreamBuffer& buffer);
  bool deserialize(const StreamBuffer& buffer);
  bool validateHeader(const StreamBuffer& buffer);
  bool validateHeader(const MemoryBlockHeader& theader);

  bool checkOwner(const std::string &name, MemoryOwner::Owner expect_owner, bool no_exit = false) const;

  bool buffer_logging_;
};

#endif /* end of include guard: MEMORYBLOCK_4MKDDAED */
