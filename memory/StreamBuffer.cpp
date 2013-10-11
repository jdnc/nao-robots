#include <memory/StreamBuffer.h>

#define VERIFY_RESIZE \
  if(n == 0 || n > 10e8) {\
    fprintf(stderr, "INVALID BUFFER REQUEST!! The log file is most likely broken\n");\
    return;\
  }\

StreamBuffer::StreamBuffer() : size(0), capacity(0), buffer(NULL) {
}

StreamBuffer::StreamBuffer(unsigned char* b, unsigned int n) : size(n), capacity(n), buffer(b) {
}

StreamBuffer::StreamBuffer(char* b, unsigned int n) : size(n), capacity(n), buffer((unsigned char*)b) {
}

void StreamBuffer::clear() {
  if(buffer != NULL) delete [] buffer;
  capacity = 0;
  size = 0;
}

void StreamBuffer::resize(unsigned int n) {
  if(capacity < n) {
    if(buffer != NULL)
      delete [] buffer;
    buffer = new unsigned char[n];
    capacity = n;
  }
}

void StreamBuffer::reset() {
  size = 0;
}

void StreamBuffer::write(const unsigned char* data, unsigned int n) {
  VERIFY_RESIZE;
  resize(n);
  memcpy(buffer, data, n);
  size = n;
}

void StreamBuffer::write(const char* data, unsigned int n) {
  VERIFY_RESIZE;
  resize(n);
  memcpy((char*)buffer, data, n);
  size = n;
}

void StreamBuffer::write(std::istream& is, unsigned int n) {
  VERIFY_RESIZE;
  resize(n);
  is.read((char*)buffer, n);
  size = n;
}

void StreamBuffer::combine(const std::vector<StreamBuffer>& buffers, StreamBuffer& combined) {
  unsigned int totalSize = 0;
  for(unsigned int i = 0; i < buffers.size(); i++)
    totalSize += buffers[i].size;

  combined.resize(totalSize + buffers.size() * sizeof(unsigned int) + 1);
  unsigned int offset = 0;
  assert(buffers.size() < 0xFF);
  combined.buffer[offset++] = buffers.size();
  for(unsigned int i = 0; i < buffers.size(); i++) {
    unsigned int *pieceSize = (unsigned int*)(combined.buffer + offset);
    *pieceSize = buffers[i].size;
    offset += sizeof(unsigned int);
    memcpy(combined.buffer + offset, buffers[i].buffer, buffers[i].size);
    offset += buffers[i].size;
  }
  combined.size = offset;
}

void StreamBuffer::separate(const StreamBuffer& buffer, std::vector<StreamBuffer>& separated) {
  unsigned int offset = 0;
  unsigned char bufCount = buffer.buffer[offset++];
  while(separated.size() < bufCount)
    separated.push_back(StreamBuffer());
  for(unsigned int i = 0; i < bufCount; i++) {
    StreamBuffer& piece = separated[i];
    unsigned int *pieceSize = (unsigned int*)(buffer.buffer + offset);
    offset += sizeof(unsigned int);
    piece.write(buffer.buffer + offset, *pieceSize);
    offset += piece.size;
  }
}

void StreamBuffer::clear(std::vector<StreamBuffer>& buffers) {
  for(unsigned int i = 0; i < buffers.size(); i++)
    buffers[i].clear();
}

