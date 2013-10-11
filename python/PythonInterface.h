#ifndef PYTHON_INTERFACE
#define PYTHON_INTERFACE

#include <string>

class VisionCore;

class PythonInterface {
  public:
    void Init();
    void Import(std::string);
    void Execute(std::string);
    void Finalize();
    static VisionCore* CORE_INSTANCE;
};

#endif
