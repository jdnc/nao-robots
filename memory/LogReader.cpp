#include "LogReader.h"
#include <memory/ImageBlock.h>
#include <memory/RobotVisionBlock.h>

#define MAX_EXPECTED_MODULES_PER_MEMORY 40

LogReader::LogReader(const char *filename):
  using_buffers_(false)
{
  filename_ = filename;
  log_file_.open(filename,std::ios::binary);
  data_dir_ = filename_ + "_data";
  if (!good()) {
    std::cout << "problem opening log" << std::endl << std::flush;
  }
}

LogReader::LogReader(const StreamBuffer& buffer) :
  using_buffers_(true), main_buffer_(buffer) {
}

LogReader::~LogReader() {
  close();
}

#include <iostream>

bool LogReader::readMemoryHeader(const StreamBuffer& buffer, MemoryHeader &header) {
  std::vector<StreamBuffer> blockNames;
  StreamBuffer::separate(buffer, blockNames);
  if (blockNames.size() > MAX_EXPECTED_MODULES_PER_MEMORY) {
    std::cout << "LogReader::readMemoryHeader: BAD NUMBER OF BLOCKS " << blockNames.size() << std::endl;
    return false;
  }
  for(unsigned int i = 0; i < blockNames.size(); i++) {
    std::string name = (const char*)blockNames[i].buffer;
    header.block_names.push_back(name);
  }
  StreamBuffer::clear(blockNames);
  return true;
}

bool LogReader::readMemory(Memory &memory, bool /*suppress_errors*/) {
  // get the header and see how many blocks we need to read
  std::vector<StreamBuffer> buffers;
  StreamBuffer::separate(main_buffer_, buffers);

  MemoryHeader header;
  bool res;
  res = readMemoryHeader(buffers[0], header);
  if (!res) {
    StreamBuffer::clear(buffers);
    return false;
  }
  if (header.block_names.size()<1) return false;
  for (unsigned int i = 1; i < buffers.size(); i++) {
    std::string &id = header.block_names[i - 1];
    MemoryBlock *block = memory.getBlockPtrByName(id);
    if (block == NULL) {
      bool res = memory.addBlockByName(id);
      if (!res) {
        std::cout << "Adding block " << id << " failed, just skipping" << std::endl << std::flush;
        continue;
      }
      else {
        block = memory.getBlockPtrByName(id);
        assert(block != NULL); // really shouldn't happen
      }
    }
    block->buffer_logging_ = using_buffers_;
    bool valid = true;
    if(id == "raw_image")
      valid = ((ImageBlock*)block)->deserialize(buffers[i], data_dir_);
    else if (id == "robot_vision")
      valid = ((RobotVisionBlock*)block)->deserialize(buffers[i], data_dir_);
    else
      valid = block->deserialize(buffers[i]);
    if(!valid)
      fprintf(stderr, "Error deserializing %s\n", id.c_str());
  }
  StreamBuffer::clear(buffers);
  return true;
}

void LogReader::close() {
  if (! using_buffers_)
    log_file_.close();
}

bool LogReader::good() {
  if (using_buffers_) {
    return true;
  } else
    return log_file_.good();
}

Log* LogReader::readLog(int start, int finish) {
  Log* frames = new Log();
  bool ok = true;
  int count = 0;
  while (ok) {
    read();
    Memory frame(false,MemoryOwner::TOOL_MEM, 0, 1);
    ok = readMemory(frame, true);
    bool loadFrame = count >= start && (finish == -1 || count < finish);
    if (ok && loadFrame) frames->push_back(frame);
    if(log_file_.peek() == EOF) break;
    count++;
  }
  close();
  frames->name = filename_;
  return frames;
}

void LogReader::read() {
  char sizeBuf[sizeof(unsigned int)];
  log_file_.read(sizeBuf, sizeof(unsigned int));
  int size = *((int*)sizeBuf);
  main_buffer_.write(log_file_, size);
}
