#ifndef CONFIG_B30XZCNH
#define CONFIG_B30XZCNH

#include <fstream>

class Config {

  public:

  bool readFromFile(std::string fileName) {
    std::ifstream fin(fileName.c_str());
    if (fin.is_open()) {
      fin >> robot_id_;
      fin >> team_;
      fin >> role_;
      fin.close();
      return true;
    }
    return false;
  }

  void writeToFile(std::string fileName) {
    std::ofstream fout(fileName.c_str());
    fout << robot_id_ << std::endl;
    fout << team_ << std::endl;
    fout << role_ << std::endl;
    fout.close();  
  }

  public:

  int robot_id_;
  int team_;
  int role_;

};

#endif /* end of include guard: CONFIG_B30XZCNH */
