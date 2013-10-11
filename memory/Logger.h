#ifndef LOGGER_3AB7OTVI
#define LOGGER_3AB7OTVI

#include <fstream>
#include <memory/Memory.h>
#include <memory/MemoryBlock.h>
#include <common/InterfaceInfo.h>
#include <memory/StreamBuffer.h>
#include <sys/stat.h>

class Logger {
public:
  virtual ~Logger ();

  void writeMemory(Memory &memory);
  void open(const char *filename, bool appendUniqueId = false);
  void close();
  void write();
  void setType(int type);
  void clearBuffer();
  static void mkdir_recursive(const char* dir);
  inline const StreamBuffer& getBuffer() const { return main_buffer_; }

protected:
  Logger(bool useBuffers, const char* filename, bool appendUniqueId, bool useAllBlocks = false);

private:
  void generateUniqueFileName(char *fileName, const char *prefix, const char *ext);
  void writeMemoryHeader(const MemoryHeader &header, StreamBuffer& buffer);

  std::ofstream log_file_;
  std::string data_dir_, log_path_;
  int type_;
  static int frame_id_;
  
  bool using_buffers_, use_all_blocks_;
  StreamBuffer main_buffer_;
};

class StreamLogger : public Logger {
  public:
    StreamLogger() : Logger(true, NULL, false) { }
};

class FileLogger : public Logger {
  public:
    FileLogger() : Logger(false, NULL, true) { }
    FileLogger(const char* filename) : Logger(false, filename, false) { }
};

class EditLogger : public Logger {
  public:
    EditLogger(const char* filename) : Logger(false, filename, false, true) { }
};

#endif /* end of include guard: LOGGER_3AB7OTVI */
