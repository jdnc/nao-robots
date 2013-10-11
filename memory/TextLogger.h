#ifndef TEXTLOGGER_
#define TEXTLOGGER_

#include <fstream>
#include <string>
#include <vector>

#include <memory/FrameInfoBlock.h>
#include <common/InterfaceInfo.h>

// NOTE: Workaround for problems setting compile definitions in build/core/CMakeLists.txt
#ifndef ALLOW_DEBUG_LOG
#ifdef TOOL
#define ALLOW_DEBUG_LOG
#endif
#endif

// NOTE: if you add another logging type, add it to build/core/CMakeLists.txt to the variable LOG_DEFINES
#ifdef ALLOW_DEBUG_LOG
#define debugLog(arglist) textlogger->log arglist
#define visionLog(arglist) textlogger->logFromVision arglist
#define ukfLog(arglist) textlogger->logFromUKF arglist
#define oppLog(arglist) textlogger->logFromOpp arglist
#else
#define debugLog(arglist)
#define visionLog(arglist)
#define ukfLog(arglist)
#define oppLog(arglist)
#endif

// module types for text log
enum modulesTypes {
  VisionModuleLog,
  BehaviorModuleLog,
  LocalizationModuleLog,
  OppModuleLog,
  KinematicsModuleLog,
  SensorsModuleLog,
  NUM_MODULE_TYPES
};

class TextLogger {
public:
  TextLogger (const char *filename = NULL, bool appendUniqueId = false);
  virtual ~TextLogger ();

  void writeDebug(int loglevel, int frame, int moduleType, char *msg );
  void setFrameInfo(FrameInfoBlock* fi);
  void setType(int t);

  void open(const char *filename = NULL, bool appendUniqueId = false);
  void close();

  // log methods for specific modules
  void log(int logLevel, int moduleType, const char* format, ... );
  void logFromVision(int logLevel, const char* format, ... );
  void logFromUKF(int logLevel, const char* format, ... );
  void logFromOpp(int logLevel, const char* format, ... );

  std::vector<std::string> textEntries;
  bool toolSimMode;

private:
  void generateUniqueFileName(char *fileName, const char *prefix, const char *ext);

  bool enabled;
  FILE *text_file_;
  FrameInfoBlock* frameInfo;
  int screenLogLevel;
  int fileLogLevel;
  int type_;
};

#endif /* end of include guard: LOGGER_3AB7OTVI */
