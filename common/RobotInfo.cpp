#include "RobotInfo.h"

double getSystemTime() {
  struct timezone tz;
  timeval timeT;
  gettimeofday(&timeT, &tz);
  return timeT.tv_sec + (timeT.tv_usec / 1000000.0);
}
