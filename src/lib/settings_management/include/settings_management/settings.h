#ifndef SETTINGS_MANAGEMENT_SETTINGS_H
#define SETTINGS_MANAGEMENT_SETTINGS_H

#include <any>
#include <map>
#include <iostream>

#include <opencv2/core/persistence.hpp>

class Settings {
public:
  Settings(const std::string& fname);

  template <typename T>
  const T getParameter(const std::string& name, bool& found) {
    if (_params.count(name) == 0) {
      found = false;
      return T();
    }

    std::any a_value = _params[name];
    T t_value;
    try {
      t_value = std::any_cast<T>(a_value);
    } catch(const std::exception& e) {
      std::cerr << e.what() << '\n';
      found = false;
      return T();
    }

    found = true;

    return t_value;   
  }

  template <typename T>
  const void getRequiredParameter(const std::string& name, T& param) {
    bool isFound;
    param = getParameter<T>(name, isFound);
    if (!isFound) {
      std::cerr << "Required parameter " << name << " not set\n";
      exit(1);
    }
  }

  template<typename T>
  void addParameter(const std::string& name, const T value) {
    std::cout << "Adding parameter name " << name << " with value " << value << std::endl;
    _params[name] = value;
  }


  void readAllSettings();
  void readSensorSettings();
  void readROSSettings();
  void readK4ASettings();
  void readORBSettings();
  void readTextRecognitionSettings();
  void readMapMergingSettings();
  void readCameraIntrinsics(const std::string& name);
  void readAprilTagSettings();

private:
  enum PARAM_TYPE{
    STRING,
    INT,
    FLOAT,
    MAP,
    SECQUENCE
  };

  bool verifyParamRead(const std::string& name, PARAM_TYPE type);

  cv::FileStorage _fdata;
  std::map<std::string, std::any> _params;
  const std::string _fname;
};

#endif