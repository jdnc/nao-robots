#ifndef LOGREADER_ZN55JIC8
#define LOGREADER_ZN55JIC8

#include <fstream>
#include <cstring>
#include <vector>
#include <memory/Log.h>
#include <memory/StreamBuffer.h>
#include <memory/Memory.h>
#include <memory/MemoryBlock.h>

class LogReader {
public:
  LogReader (const char *filename);
  LogReader (const StreamBuffer& buffer);
  bool readMemory(Memory &memory, bool suppress_errors = false);
  virtual ~LogReader ();
  Log* readLog(int start = 0, int finish = -1);

private:

  bool readMemoryHeader(const StreamBuffer& buffer, MemoryHeader &header);
  bool readBlock(const MemoryBlockHeader &header,MemoryBlock &module);
  void readAndIgnoreBlock(const MemoryBlockHeader &header);
  void close();

  std::ifstream log_file_;
  bool using_buffers_;
  std::string filename_;
  std::string data_dir_;

  void read();
  bool good();

  StreamBuffer main_buffer_;

};

#endif /* end of include guard: LOGREADER_ZN55JIC8 */
