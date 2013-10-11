#include "PrivateMemory.h"
#include <iostream>
#include <memory/ImageBlock.h>
#include <memory/RobotVisionBlock.h>

PrivateMemory::PrivateMemory() : is_copy_(false) {
  //std::cout << "PRIVATE MEMORY CONSTRUCTOR" << std::endl << std::flush;
}

PrivateMemory::PrivateMemory(const PrivateMemory &mem) : is_copy_(true) {
  //char *temp;
  MemoryBlock *temp;
  for (MemMap::const_iterator it = mem.blocks_.begin(); it != mem.blocks_.end(); it++) {
    if(it->first == "raw_image") {
      temp = new ImageBlock();
      *((ImageBlock*)temp) = *((ImageBlock*)it->second);
    }
    else if (it->first == "robot_vision") {
      temp = new RobotVisionBlock();
      *((RobotVisionBlock*)temp) = *((RobotVisionBlock*)it->second);
    }
    else {
      temp = (MemoryBlock*) malloc(it->second->header.size);
      memcpy(temp,it->second,it->second->header.size);
    }
    blocks_.insert(std::pair<std::string,MemoryBlock*>(it->first,temp));
  }
}

PrivateMemory::~PrivateMemory() {
  if(is_copy_) {
    for (MemMap::iterator it = blocks_.begin(); it != blocks_.end(); it ++) {
      if(it->first == "raw_image") {
        ImageBlock* block = (ImageBlock*)it->second;
        delete block;
      }
      else if (it->first == "robot_vision") {
        RobotVisionBlock* block = (RobotVisionBlock*)it->second;
        delete block;
      }
      else {
        free(it->second);
      }
    }
  }
  blocks_.clear();
}

bool PrivateMemory::addBlock(const std::string &name,MemoryBlock *block) {
  std::pair<MemMap::iterator,bool> res = blocks_.insert(std::pair<std::string,MemoryBlock*>(name,block));
  // check if the block was inserted
  if (!res.second) {
    std::cerr << "PrivateMemory::addBlock - ERROR ADDING BLOCK " << name << " already exists, Deleting given block" << std::endl;
    delete block;
  }
  return res.second;
}

MemoryBlock* PrivateMemory::getBlockPtr(const std::string &name) {
  MemMap::iterator it = blocks_.find(name);
  if (it == blocks_.end()) // the block isn't in our map
    return NULL;
  else
    return (*it).second;
}

const MemoryBlock* PrivateMemory::getBlockPtr(const std::string &name) const {
  MemMap::const_iterator it = blocks_.find(name);
  if (it == blocks_.end()) // the block isn't in our map
    return NULL;
  else
    return (*it).second;
}

void PrivateMemory::getBlockNames(std::vector<std::string> &module_names, bool only_log, MemoryOwner::Owner for_owner) const {
  for (MemMap::const_iterator it = blocks_.begin(); it != blocks_.end(); it++) {
    if (((!only_log) || (*it).second->log_block) && ((*it).second->checkOwner((*it).first,for_owner,true)))
      module_names.push_back((*it).first);
  }
}
