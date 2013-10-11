#include <common/Profiling.h>

static int __id = 0;
static int __lastid = __id;
static std::map<int,timeval> __times;
static std::map<int,TimeList> __timelists;

int tic() {
  timeval t;
  gettimeofday(&t, NULL);
  __times[__id] = t;
  __lastid = __id;
  return __id++;
}

double toc(int id) {
  if(id < 0) id = __lastid;
  timeval tstart = __times[id];
  timeval tend;
  gettimeofday(&tend, NULL);
  double elapsed = 
    (tend.tv_sec - tstart.tv_sec) +
    (tend.tv_usec - tstart.tv_usec) / 1000000.0;
  return elapsed;
}

int clockavg(int maxSize) {
  TimeList tl;
  tl.maxSize = maxSize;
  __timelists[__id] = tl;
  return __id++;
}

void ticavg(int id) {
  timeval t;
  gettimeofday(&t, NULL);
  __times[id] = t;
}

double tocavg(int id) {
  double elapsed = toc(id);
  TimeList& tl = __timelists[id];
  tl.times.push_back(elapsed);
  if(tl.times.size() > tl.maxSize)
    tl.times.pop_front();
  double avg = 0;
  for(std::list<double>::iterator it = tl.times.begin(); it != tl.times.end(); it++) {
    double tim = *it;
    avg += tim;
  }
  avg /= tl.maxSize;
  return avg;
}

void printtime(int id) {
  double seconds = toc(id);
  printf("time: %2.4f\n", seconds);
}

void printtime(const char* name, int id) {
  printf("%s ", name);
  printtime(id);
}
