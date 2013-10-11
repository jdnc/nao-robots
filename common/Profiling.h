#ifndef PROFILING_H
#define PROFILING_H

#include <sys/time.h>
#include <map>
#include <list>
#include <stdio.h>
#include <iostream>

struct TimeList {
  int maxSize;
  std::list<double> times;
};

int tic();
double toc(int = -1);
void printtime(int = -1);
void printtime(const char*, int = -1);

int clockavg(int = 10);
void ticavg(int id);
double tocavg(int id);

#endif
