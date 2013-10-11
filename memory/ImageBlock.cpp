#include <memory/ImageBlock.h>
  
void ImageBlock::serialize(StreamBuffer& buffer, std::string data_dir) {
  std::vector<StreamBuffer> all;

  StreamBuffer main;
  main.write((unsigned char*)&header, sizeof(MemoryBlockHeader));
  all.push_back(main);

  StreamBuffer tparams, bparams;
  tparams.write((unsigned char*)&top_params_, sizeof(ImageParams));
  all.push_back(tparams);
  bparams.write((unsigned char*)&bottom_params_, sizeof(ImageParams));
  all.push_back(bparams);

  StreamBuffer imageLoaded;
  imageLoaded.write((unsigned char*)&loaded_, sizeof(bool));
  all.push_back(imageLoaded);

  if(buffer_logging_) {
    StreamBuffer top;
    top.write(getImgTop(), top_params_.rawSize);
    all.push_back(top);

    StreamBuffer bottom;
    bottom.write(getImgBottom(), bottom_params_.rawSize);
    all.push_back(bottom);
  }
  else {
    std::stringstream ss;
    char buf[10];
    sprintf(buf, "%02d", header.frameid);
    ss << data_dir << "/" << buf << "top.bmp";
    writeImage(getImgTop(), ss.str(), top_params_);
    ss.str("");
    ss << data_dir << "/" << buf << "bottom.bmp";
    writeImage(getImgBottom(), ss.str(), bottom_params_);
  }

  StreamBuffer::combine(all, buffer);
  StreamBuffer::clear(all);
}

bool ImageBlock::deserialize(const StreamBuffer& buffer, std::string data_dir) {
  std::vector<StreamBuffer> parts;
  StreamBuffer::separate(buffer, parts);
  StreamBuffer 
    &main = parts[0], 
    &tparams = parts[1], 
    &bparams = parts[2],
    &imageLoaded = parts[3],
    &timage = parts[4],
    &bimage = parts[5];
  if(!validateHeader(main)) {
    StreamBuffer::clear(parts);
    return false;
  }
  int tsize = top_params_.rawSize, bsize = bottom_params_.rawSize;
  memcpy((unsigned char*)&header, main.buffer, main.size);
  memcpy((unsigned char*)&top_params_, tparams.buffer, tparams.size);
  memcpy((unsigned char*)&bottom_params_, bparams.buffer, bparams.size);
  memcpy((unsigned char*)&loaded_, imageLoaded.buffer, imageLoaded.size);
  if(tsize < top_params_.rawSize) {
    delete [] img_top_local_;
    img_top_local_ = new unsigned char[top_params_.rawSize];
  }
  if(bsize < bottom_params_.rawSize) {
    delete [] img_bottom_local_;
    img_bottom_local_ = new unsigned char[bottom_params_.rawSize];
  }
  if(buffer_logging_) {
    memcpy(img_top_local_, timage.buffer, timage.size);
    memcpy(img_bottom_local_, bimage.buffer, bimage.size);
  }
  else {
    std::stringstream ss;
    char buf[10];
    sprintf(buf, "%02d", header.frameid);
    ss << data_dir << "/" << buf << "top.bmp";
    readImage(img_top_local_, ss.str(), top_params_);
    ss.str("");
    ss << data_dir << "/" << buf << "bottom.bmp";
    readImage(img_bottom_local_, ss.str(), bottom_params_);
  }
  img_top_ = boost::interprocess::offset_ptr<unsigned char>(img_top_local_);
  img_bottom_ = boost::interprocess::offset_ptr<unsigned char>(img_bottom_local_);
  StreamBuffer::clear(parts);
  return true;
}
