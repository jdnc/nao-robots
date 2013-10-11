#include "PythonInterp.h"

using namespace std;

// While python is running it doesn't allow the executable to be killed,
// so catch the ctrl-c signal and force exit.
static bool __EXIT_HANDLER_ASSIGNED = false;
void __EXIT_HANDLER(int s) {
  if(s == 2)
    exit(1);
}

void PythonInterp::setExitHandler() {
  if(__EXIT_HANDLER_ASSIGNED) return;
  __EXIT_HANDLER_ASSIGNED = true;
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = __EXIT_HANDLER;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
}

PythonInterp::PythonInterp() {
  setExitHandler();
  init();
}

PythonInterp::~PythonInterp() {
  finalize();
}

void PythonInterp::init() {
  pyface_.Init();
}

void PythonInterp::finalize() {
  pyface_.Finalize();
}

bool PythonInterp::require(std::string lib) {
  return false;
}

bool PythonInterp::call(std::string cmd) {
  pyface_.Execute(cmd);
  return true;
}

