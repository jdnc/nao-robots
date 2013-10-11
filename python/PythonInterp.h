#ifndef _PYTHON_INTERP_H
#define _PYTHON_INTERP_H

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <python/PythonInterface.h>


void __EXIT_HANDLER(int s);

class PythonInterp {
  private:
    void addPath(std::string pathStr);
    void setExitHandler();
    PythonInterface pyface_;
   public:
    PythonInterp();
    ~PythonInterp();

    void init();
    void finalize();

    bool require(std::string lib);
    bool call(std::string cmd);
};

#endif // _PYTHON_H
