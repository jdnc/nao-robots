#include "TextLogger.h"
#include <iostream>
#include <ctime>
#include <stdarg.h>

// Fill in this array with levels that you want to restrict to
// ex: char __TextLoggerLevels[2] { 5, 18 };
char __TextLoggerLevels[0] = { };
#define CHECK_LOGGER_LEVELS \
  static const int levels = sizeof(__TextLoggerLevels) / sizeof(char); \
  if(levels > 0) { \
    for(int i = 0; i < levels; i++) \
      if(__TextLoggerLevels[i] == logLevel) \
        goto next; \
    return; \
  } \
  next:

TextLogger::TextLogger(const char *filename, bool appendUniqueId) {
  enabled = false;
  toolSimMode = false;
  if (filename) {
    open(filename, appendUniqueId);
  }
  fileLogLevel = 100;
  screenLogLevel = 5;
}

TextLogger::~TextLogger() {
  close();
}

void TextLogger::open(const char *filename, bool appendUniqueId) {
  close(); // any previously opened textlog
  if (appendUniqueId) {
    char buffer[100];
    generateUniqueFileName(buffer, filename, "txt");
    std::cout << "Text logging to file " << buffer << std::endl;
    text_file_ = fopen(buffer, "w");
  } else {
    std::cout << "Text logging to file " << filename << std::endl;
    text_file_ = fopen(filename, "w");
  }
  enabled = true;
}

void TextLogger::close() {
  if (enabled) {
    enabled = false;
    std::cout << "Closing text log file" << std::endl;
    fclose(text_file_);
  }
}

void TextLogger::setFrameInfo(FrameInfoBlock* fi){
  frameInfo = fi;
}

void TextLogger::writeDebug(int loglevel, int frame, int moduleType, char *msg ) {

  if ( toolSimMode ){
    char buffer[1024];
    sprintf(buffer, "loglev %d: frame %d: module %d: %s", loglevel, frame, moduleType, msg );
    textEntries.push_back(buffer);
    return;
  }

  // print to file
  if ( enabled ) {
    if (loglevel <= fileLogLevel)
      fprintf( text_file_, "loglev %d: frame %d: module %d: %s\n", loglevel, frame, moduleType, msg );

  }

  // print to screen
  if ( loglevel <= screenLogLevel ){
    printf("frame %d: %s\n",frame, msg);
  }

}

void TextLogger::log(int logLevel, int moduleType, const char* format, ...) {
  CHECK_LOGGER_LEVELS
  va_list args;
  char temp[1024];
  va_start( args, format );
  int l=vsprintf( temp, format, args );

  // kill warnings
  l = l;

  va_end( args );

  writeDebug(logLevel, frameInfo->frame_id, moduleType, temp);
}

void TextLogger::logFromVision(int logLevel, const char* format, ...) {
  CHECK_LOGGER_LEVELS
  va_list args;
  char temp[1024];
  va_start( args, format );
  int l=vsprintf( temp, format, args );

  // kill warnings
  l = l;

  va_end( args );

  writeDebug(logLevel, frameInfo->frame_id, VisionModuleLog, temp);
}


void TextLogger::logFromUKF(int logLevel, const char* format, ...) {
  CHECK_LOGGER_LEVELS
  va_list args;
  char temp[1024];
  va_start( args, format );
  int l=vsprintf( temp, format, args );

  // kill warnings
  l = l;

  va_end( args );

  writeDebug(logLevel, frameInfo->frame_id, LocalizationModuleLog, temp);
}

void TextLogger::logFromOpp(int logLevel, const char* format, ...) {
  CHECK_LOGGER_LEVELS
  va_list args;
  char temp[1024];
  va_start( args, format );
  int l=vsprintf( temp, format, args );

  // kill warnings
  l = l;

  va_end( args );

  writeDebug(logLevel, frameInfo->frame_id, OppModuleLog, temp);
}

void TextLogger::setType(int t){
  type_ = t;
}

void TextLogger::generateUniqueFileName(char *fileName, const char *prefix, const char *ext) {
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
