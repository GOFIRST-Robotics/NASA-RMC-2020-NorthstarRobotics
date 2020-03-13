

#ifndef DECAWAVE_H
#define DECAWAVE_H
// VERSION 1.0.0

#include <string>
#include <vector>
#include <serial/serial.h>
#include <memory>
#include <random>
#include <iostream>
#include <tf/transform_listener.h>
#include <gazebo_msgs/LinkStates.h>

namespace decawave{
struct Anchor{
  int id;
  double distance;
  int distance_quality;
  double position[3];
  int quality_factor;
};

class IDecawave {
  public:
  virtual std::vector<Anchor> updateSamples() = 0;
};

class Decawave : public IDecawave {
public:
  Decawave(std::string port_name);
  ~Decawave();
  std::vector<Anchor> updateSamples() override;
private:
  int coords_updated;
  std::shared_ptr<serial::Serial> my_serial;
};
class DecawaveSim : public IDecawave {
  public:
  DecawaveSim(std::string my_frame, std::string map_frame, int anchor0_id, int anchor1_id, std::string anchor0_frame, std::string anchor1_frame, std::string gazebo_prefix, 
              std::string robot_frame_id, double sim_dist_stdev, tf::TransformListener * tf);
  ~DecawaveSim();
  std::vector<Anchor> updateSamples() override;
  void gazebo_joint_callback(const gazebo_msgs::LinkStates& msg);
  private:
  std::string my_frame;
  std::string map_frame;
  int anchor0_id;
  int anchor1_id;
  std::string anchor0_frame;
  std::string anchor1_frame;
  std::string gazebo_prefix;
  std::string robot_frame_id;
  tf::TransformListener * tf;
  gazebo_msgs::LinkStates lastMsg;
  std::default_random_engine generator;
  std::normal_distribution<double> * gaussian;
  bool has_pos = false;
};
}
#endif
