#ifndef STREAM_BUFFER_H
#define STREAM_BUFFER_H

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <assert.h>
#include <vector>

struct StreamBuffer {
  unsigned int size;
  unsigned int capacity;
  unsigned char* buffer;

  StreamBuffer();
  StreamBuffer(unsigned char* b, unsigned int n);
  StreamBuffer(char* b, unsigned int n);
  void clear();
  void resize(unsigned int n);
  void reset();
  void write(const unsigned char* data, unsigned int n);
  void write(const char* data, unsigned int n);
  void write(std::istream& is, unsigned int n);
  static void combine(const std::vector<StreamBuffer>& buffers, StreamBuffer& combined);
  static void separate(const StreamBuffer& buffer, std::vector<StreamBuffer>& separated);
  static void clear(std::vector<StreamBuffer>& buffers);
};

#endif
