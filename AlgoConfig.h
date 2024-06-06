#ifndef ALGO_CONFIG_H
#define ALGO_CONFIG_H

#include <iostream>
#include <string>
#include <array>
#include "pugixml.hpp"

typedef enum{
    D2D = 0,
    C2C = 1,
    MIXED =2,
    NONE,
} DETECTION_MODE;

class AlgoConfig {
public:

  AlgoConfig(){};
  ~AlgoConfig(){};

  static AlgoConfig* instance() {
    if (!instance_) {
      instance_ = new AlgoConfig();
    }
    return instance_;
  };

  static void release() {
    if (instance_) {
      delete instance_;
      instance_ = NULL;
    }
  };

  int LoadFromXml(std::string file_name);
  DETECTION_MODE GetDetectionMode() { return detection_mode_;};
  void getImgSize(int& width, int& height){width=mImgWidth, height=mImgHeight;};
  void setWorkPath(std::string& workPath){mWorkPath=workPath;};
  void getWorkPath(std::string& workPath){workPath=mWorkPath;};
  pugi::xml_node SelectNodes(std::string xpath);
  void configure();
  static std::array<int,3> job_id_;

private:
  AlgoConfig(const AlgoConfig&) = delete;
  AlgoConfig& operator=(const AlgoConfig&) = delete;

private:
  static AlgoConfig* instance_;
  pugi::xml_document doc_;
  DETECTION_MODE detection_mode_;
  int mImgWidth, mImgHeight;
  std::string mWorkPath;
};

#endif
