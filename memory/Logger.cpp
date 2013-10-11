#include "Logger.h"
#include <iostream>
#include <ctime>
#include <memory/ImageBlock.h>
#include <memory/RobotVisionBlock.h>

Logger::Logger(bool useBuffers, const char* filename, bool appendUniqueId, bool useAllBlocks):
  using_buffers_(useBuffers), use_all_blocks_(useAllBlocks)
{
  if (filename) {
    open(filename, appendUniqueId);
  }
}

Logger::~Logger() {
  if(using_buffers_) {
    if(main_buffer_.buffer)
      delete [] main_buffer_.buffer;
  }
  close();
}

void Logger::clearBuffer() {
  main_buffer_.reset();
}

void Logger::writeMemoryHeader(const MemoryHeader &header, StreamBuffer& buffer) {
  unsigned int num_blocks = header.block_names.size();
  std::vector<StreamBuffer> buffers;
  for (unsigned int i = 0; i < num_blocks; i++) {
    StreamBuffer bbuffer;
    bbuffer.write(header.block_names[i].c_str(), header.block_names[i].size() + 1);
    buffers.push_back(bbuffer);
  }
  StreamBuffer::combine(buffers, buffer);
}

void Logger::writeMemory(Memory &memory) {
  if (using_buffers_ || log_file_.is_open()) {
    MemoryHeader header;
    MemoryBlock *block;
    // first get a list of the blocks we're loading
    memory.getBlockNames(header.block_names, !use_all_blocks_);
    // write the header
    StreamBuffer hbuffer;
    writeMemoryHeader(header, hbuffer);
    // write each block
    std::vector<StreamBuffer> buffers;
    buffers.push_back(hbuffer);
    for (unsigned int i = 0; i < header.block_names.size(); i++) {
      std::string& id = header.block_names[i];
      block = memory.getBlockPtrByName(id);
      block->buffer_logging_ = using_buffers_;
      block->header.frameid = frame_id_;
      StreamBuffer sb;
      if(id == "raw_image")
        ((ImageBlock*)block)->serialize(sb, data_dir_);
      else if(id == "robot_vision")
        ((RobotVisionBlock*)block)->serialize(sb, data_dir_);
      else
        block->serialize(sb);
      buffers.push_back(sb);
    }
    StreamBuffer::combine(buffers, main_buffer_);
    if(!using_buffers_) write();
    StreamBuffer::clear(buffers);
    frame_id_++;
  }
}

void Logger::open(const char *filename, bool appendUniqueId) {
  close(); // any previously opened logs
  if (appendUniqueId) {
    char buffer[100];
    generateUniqueFileName(buffer, filename, "log");
    log_path_ = buffer;
  } else {
    log_path_ = filename;
  }
  data_dir_ = log_path_ + "_data";
  std::cout << "Logging to file: " << log_path_ << ", data: " << data_dir_ << std::endl;
  log_file_.open(log_path_.c_str(), std::ios::binary);
  mkdir_recursive(data_dir_.c_str());
}

void Logger::mkdir_recursive(const char *dir) {
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp),"%s",dir);
  len = strlen(tmp);
  if(tmp[len - 1] == '/')
    tmp[len - 1] = 0;
  for(p = tmp + 1; *p; p++)
    if(*p == '/') {
      *p = 0;
      mkdir(tmp, S_IRWXU);
      *p = '/';
    }
  mkdir(tmp, S_IRWXU);
}

void Logger::close() {
  if (log_file_.is_open()) {
    std::cout << "Closing log file" << std::endl;
    log_file_.close();
  }
  frame_id_ = 0;
}

void Logger::setType(int t){
  type_ = t;
}

void Logger::generateUniqueFileName(char *fileName, const char *prefix, const char *ext) {
  std::string robotbase = "/home/nao/logs/";
  std::string base = robotbase;
  if (type_ == CORE_SIM) {
    std::string naohome = getenv("NAO_HOME");
    std::string simbase = naohome + "/logs/";
    base = simbase;
  } else if (type_ == CORE_TOOL) {
    std::string naohome = getenv("NAO_HOME");
    std::string toolbase = naohome + "/logs/";
    base = toolbase;
  }

  std::string logFile = base + std::string(prefix) + "_%y_%m_%d-%H_%M_%S." + std::string(ext);
  time_t rawtime;
  struct tm *timeInfo;
  time(&rawtime);
  timeInfo = localtime(&rawtime);
  strftime(fileName, 80, logFile.c_str(), timeInfo);
}

void Logger::write() {
  log_file_.write((const char*)&(main_buffer_.size), sizeof(int));
  log_file_.write((const char*)main_buffer_.buffer, main_buffer_.size);
}

int Logger::frame_id_ = 0;
