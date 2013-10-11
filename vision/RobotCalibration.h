#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#define YAML_EMIT(emitter,x) emitter << YAML::Key << #x << YAML::Value << x
#define YAML_READ(node,x) node[#x] >> x

#include <common/Field.h>
#include <common/RobotInfo.h>
#include <common/RobotDimensions.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <stdlib.h>
#include <constants/ImageConstants.h>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

class RobotCalibration {
  public:
    Camera::Type camera;

    bool useLeft;

    float dimensionValues_[RobotDimensions::NUM_DIMENSIONS];
    float jointValues_[NUM_JOINTS];
    float sensorValues_[NUM_SENSORS];

    float
      poseX,
      poseY,
      poseTheta;

    float // Camera Offsets
      topFOVx,
      topFOVy,
      bottomFOVx,
      bottomFOVy;

    float // Distortion Parameters
      k1, k2, k3,
      p1, p2;

    RobotCalibration(){
      topFOVx = bottomFOVx = 0;
      topFOVy = bottomFOVy = 0;
      memset(jointValues_, 0, NUM_JOINTS * sizeof(float));
      memset(sensorValues_, 0, NUM_SENSORS * sizeof(float));
      memset(dimensionValues_, 0, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
      k1 = -0.0958081087594769;
      k2 =  0.1014458308701546;
      p1 =  0.01421471087609939;
      p2 = -0.004138935948982259;
      k3 = -0.04019608341725905;
      poseX = 0; poseY = -HALF_FIELD_Y; poseTheta = 90 * DEG_T_RAD;
    }

    bool loadFromFile(std::string file){
      std::ifstream input(file.c_str());
      if(!input.good()) return false;
      YAML::Parser parser(input);
      YAML::Node doc;
      parser.GetNextDocument(doc);
      Deserialize(doc);
      return true;
    }

    void Deserialize(const YAML::Node& node) {
      YAML_READ(node, poseX);
      YAML_READ(node, poseY);
      YAML_READ(node, poseTheta);
      YAML_READ(node, topFOVx);
      YAML_READ(node, topFOVy);
      YAML_READ(node, bottomFOVx);
      YAML_READ(node, bottomFOVy);
      YAML_READ(node, k1);
      YAML_READ(node, k2);
      YAML_READ(node, k3);
      YAML_READ(node, p1);
      YAML_READ(node, p2); 
      for(int i = 0; i < NUM_JOINTS; i++)
        node[JointNames[i]] >> jointValues_[i];

      for(int i = 0; i < NUM_SENSORS; i++)
        node[SensorNames[i]] >> sensorValues_[i];

      for(int i = 0; i < RobotDimensions::NUM_DIMENSIONS; i++)
        node[DimensionNames[i]] >> dimensionValues_[i];
    }

    void Serialize(YAML::Emitter& emitter) const {
      emitter << YAML::BeginMap;
      YAML_EMIT(emitter, poseX);
      YAML_EMIT(emitter, poseY);
      YAML_EMIT(emitter, poseTheta);
      YAML_EMIT(emitter, topFOVx);
      YAML_EMIT(emitter, topFOVy);
      YAML_EMIT(emitter, bottomFOVx);
      YAML_EMIT(emitter, bottomFOVy);
      YAML_EMIT(emitter, k1);
      YAML_EMIT(emitter, k2);
      YAML_EMIT(emitter, k3);
      YAML_EMIT(emitter, p1);
      YAML_EMIT(emitter, p2); 
      for(int i = 0; i < NUM_JOINTS; i++)
        emitter << YAML::Key << JointNames[i] << YAML::Value << jointValues_[i];

      for(int i = 0; i < NUM_SENSORS; i++)
        emitter << YAML::Key << SensorNames[i] << YAML::Value << sensorValues_[i];

      for(int i = 0; i < RobotDimensions::NUM_DIMENSIONS; i++)
        emitter << YAML::Key << DimensionNames[i] << YAML::Value << dimensionValues_[i];
      emitter << YAML::EndMap;
    }

    void saveToFile(std::string file) {
      std::ofstream output(file.c_str());
      YAML::Emitter emitter;
      emitter << YAML::BeginDoc;
      Serialize(emitter);
      emitter << YAML::EndDoc;
      output << emitter.c_str();
    }

    Eigen::VectorXf extrinsicVector() const {
      Eigen::VectorXf v;
      return v;
    }

    void fromExtrinsicVector(const Eigen::VectorXf& /*v*/) {
    }

    void applyJoints(float* joints) {
      for(int i = 0; i < NUM_JOINTS; i++)
        joints[i] += jointValues_[i];
    }

    void applySensors(float* sensors) {
      for(int i = 0; i < NUM_SENSORS; i++)
        sensors[i] += sensorValues_[i];
    }

    void applyDimensions(float* dimensions) {
      for(int i = 0; i < RobotDimensions::NUM_DIMENSIONS; i++)
        dimensions[i] += dimensionValues_[i];
    }


  private:
    bool readLine(std::ifstream& datafile, std::string& key, double& value) {
      std::string line;
      if(getline(datafile,line)) {
        int i = line.rfind(":");
        key = line.substr(0, i);
        std::string valueString = line.substr(i + 1, line.length() - i);
        value = atof(valueString.c_str());
        return true;
      }
      return false;
    }
};
#endif
