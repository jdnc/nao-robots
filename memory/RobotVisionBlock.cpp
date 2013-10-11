#include <memory/RobotVisionBlock.h>
void RobotVisionBlock::serialize(StreamBuffer& buffer, std::string) {
  std::vector<StreamBuffer> all;

  StreamBuffer main;
  main.write((unsigned char*)&header, sizeof(MemoryBlockHeader));
  all.push_back(main);

  StreamBuffer tparams, bparams;
  tparams.write((unsigned char*)&top_params_, sizeof(ImageParams));
  all.push_back(tparams);
  bparams.write((unsigned char*)&bottom_params_, sizeof(ImageParams));
  all.push_back(bparams);

  StreamBuffer top;
  top.write(getSegImgTop(), top_params_.size);
  all.push_back(top);

  StreamBuffer bottom;
  bottom.write(getSegImgBottom(), bottom_params_.size);
  all.push_back(bottom);

  StreamBuffer hbuff;
  hbuff.write((unsigned char*)&horizon, sizeof(HorizonLine));
  all.push_back(hbuff);

  StreamBuffer::combine(all, buffer);
  StreamBuffer::clear(all);
}

bool RobotVisionBlock::deserialize(const StreamBuffer& buffer, std::string) {
  std::vector<StreamBuffer> parts;
  StreamBuffer::separate(buffer, parts);
  StreamBuffer 
    &main = parts[0], 
    &tparams = parts[1], 
    &bparams = parts[2],
    &timage = parts[3],
    &bimage = parts[4],
    &hbuff = parts[5]; 
  if(!validateHeader(main)) {
    StreamBuffer::clear(parts);
    return false;
  }
  if(top_params_.rawSize < timage.size) {
    delete [] segImgTopLocal;
    segImgTopLocal = new unsigned char[timage.size];
  }
  if(bottom_params_.rawSize < bimage.size) {
    delete [] segImgBottomLocal;
    segImgBottomLocal = new unsigned char[bimage.size];
  }
  memcpy(segImgTopLocal, timage.buffer, timage.size);
  memcpy(segImgBottomLocal, bimage.buffer, bimage.size);
  memcpy((unsigned char*)&header, main.buffer, main.size);
  memcpy((unsigned char*)&top_params_, tparams.buffer, tparams.size);
  memcpy((unsigned char*)&bottom_params_, bparams.buffer, bparams.size);
  memcpy((unsigned char*)&horizon, hbuff.buffer, hbuff.size);
  loaded_ = true;

  segImgTop = segImgTopLocal;
  segImgBottom = segImgBottomLocal;
  StreamBuffer::clear(parts);
  return true;
}
