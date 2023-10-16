#include "settings.h"

Settings::Settings(const std::string& fname):
    _fname(fname), _fdata(fname, cv::FileStorage::READ) {}

bool Settings::verifyParamRead(const std::string& name, PARAM_TYPE type) {
  cv::FileNode node = _fdata[name];
  if (node.empty()) {
    std::cerr << "Parameter name " << name << " not found in settings file " << _fname << std::endl;
    return false;
  }

  switch (type) {
    case STRING:
      if (!node.isString()) {
        std::cerr << name << " is not of type string.\n";
        return false;
      } else {
        addParameter(name, node.string());
        return true;
      }
      break;
    case INT:
      if (!node.isInt()) {
        std::cerr << name << " is not of type int.\n";
        return false;
      } else {
        addParameter(name, (int)node);
        return true;
      }
      break;
    case FLOAT:
      if (!node.isReal()) {
        std::cerr << name << " is not of type float.\n";
        return false;
      } else {
        addParameter(name, (float)node);
        return true;
      }
      break;
    default:
      std::cerr << "Unimplemented parameter read type.\n";
      return false;
      break;
  }
}

void Settings::readAllSettings() {
  readSensorSettings();
  readORBSettings();
  readTextRecognitionSettings();
  readMapMergingSettings();
}

void Settings::readSensorSettings() {
  verifyParamRead("Save.Path", STRING);

  const std::string name = "Sensor.Type";
  if (!verifyParamRead(name, STRING)) {
    exit(-1);
  }

  bool found;
  const std::string& sensor_type = getParameter<std::string>(name, found);

  if (sensor_type == "ROS") {
    readROSSettings();
  } else if (sensor_type == "K4A") {
    readK4ASettings();
  } else {
    std::cerr << "Cannot parse parameter file " << _fname
              << "\nUnown sensor type " << sensor_type << std::endl;
    exit(-1);
  }
  verifyParamRead("Recording.Start", INT);
  verifyParamRead("Recording.End", INT);

}

void Settings::readROSSettings() {
  verifyParamRead("ROSBag.Path", STRING);
  verifyParamRead("RGB.Camera.Info.Topic", STRING);
  verifyParamRead("RGB.Camera.Image.Topic", STRING);
  verifyParamRead("Depth.Camera.Info.Topic", STRING);
  verifyParamRead("Depth.Camera.Image.Topic", STRING);
  verifyParamRead("Image.Width", INT);
  verifyParamRead("Image.Height", INT);
  verifyParamRead("Depth.Width", INT);
  verifyParamRead("Depth.Height", INT);
  verifyParamRead("Depth.Min", FLOAT);
  verifyParamRead("Depth.Max", FLOAT);
}

void Settings::readK4ASettings() {
  verifyParamRead("Record.Path", STRING);
  verifyParamRead("Depth.Min", FLOAT);
  verifyParamRead("Depth.Max", FLOAT);
}

void Settings::readORBSettings() {
  verifyParamRead("ORB.Vocab", STRING);
  verifyParamRead("ORB.Settings", STRING);
}

void Settings::readTextRecognitionSettings() {
  verifyParamRead("YOLO.Config", STRING);
  verifyParamRead("YOLO.Weights", STRING);
  verifyParamRead("YOLO.Size", INT);
  verifyParamRead("YOLO.Rows", INT);
  verifyParamRead("YOLO.Output", STRING);

  verifyParamRead("EAST.Net", STRING);
  verifyParamRead("Tess.Data", STRING);
  verifyParamRead("Tess.Whitelist", STRING);
  verifyParamRead("Keyframes.Path", STRING);
  verifyParamRead("Regex1", STRING);
  verifyParamRead("Regex2", STRING);

  verifyParamRead("Placard.Height", FLOAT);
  verifyParamRead("Placard.Width", FLOAT);
  verifyParamRead("Placard.Observations", STRING);

  verifyParamRead("Threshold.Confidence", FLOAT);
  verifyParamRead("Threshold.NMS", FLOAT);
}

void Settings::readMapMergingSettings() {
  verifyParamRead("NSamples", INT);
  verifyParamRead("Step.Decay", FLOAT);
  verifyParamRead("Radius.Max", FLOAT);
  verifyParamRead("Threshold.Close", FLOAT);
  verifyParamRead("Linear.Step.Min", FLOAT);
  verifyParamRead("Angular.Step.Min", FLOAT);
  verifyParamRead("Unmerged.Frames.Max", INT);
  verifyParamRead("Linear.Velocity.Max", FLOAT);
  verifyParamRead("Angular.Velocity.Max", FLOAT);
}

void Settings::readAprilTagSettings() {
  verifyParamRead("AprilTag.Width", FLOAT);
  verifyParamRead("AprilTag.Height", FLOAT);
}

void Settings::readCameraIntrinsics(const std::string& name) {
  std::string prefix = name+".Camera.";
  verifyParamRead(prefix + "AlphaX", FLOAT);
  verifyParamRead(prefix + "AlphaY", FLOAT);
  verifyParamRead(prefix + "Gamma", FLOAT);
  verifyParamRead(prefix + "u0", FLOAT);
  verifyParamRead(prefix + "v0", FLOAT);
}