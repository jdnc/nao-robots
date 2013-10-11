#ifndef LOG_H
#define LOG_H

#include "Memory.h"
#include "ImageBlock.h"
#include <common/RobotInfo.h>
#include <vector>

class Log : public std::vector<Memory> {
    public:
        std::string name;
        std::vector<ImageParams> getTopParams() {
          std::vector<ImageParams> params;
          int count = size();
          for(int i = 0; i < count; i++){
              Memory& m = (*this)[i];
              ImageBlock* block = 0;
              m.getBlockByName(block, "raw_image", false);
              if (block != NULL)
                params.push_back(block->top_params_);
          }
          return params;
        }
        
        std::vector<ImageParams> getBottomParams() {
          std::vector<ImageParams> params;
          int count = size();
          for(int i = 0; i < count; i++){
              Memory& m = (*this)[i];
              ImageBlock* block = 0;
              m.getBlockByName(block, "raw_image", false);
              if (block != NULL)
                params.push_back(block->bottom_params_);
          }
          return params;
        }

        std::vector<unsigned char*> getRawTopImages() {
            std::vector<unsigned char*> images;
            int count = size();
            for(int i = 0; i < count; i++){
                Memory& m = (*this)[i];
                ImageBlock* block = 0;
                m.getBlockByName(block, "raw_image", false);
                if (block != NULL)
                  images.push_back(block->img_top_.get());
            }
            return images;
        }
        std::vector<unsigned char*> getRawBottomImages() {
            std::vector<unsigned char*> images;
            int count = size();
            for(int i = 0; i < count; i++){
                Memory& m = (*this)[i];
                ImageBlock* block = 0;
                m.getBlockByName(block, "raw_image", false);
                if (block != NULL)
                  images.push_back(block->img_bottom_.get());
            }
            return images;
        }
};

#endif
