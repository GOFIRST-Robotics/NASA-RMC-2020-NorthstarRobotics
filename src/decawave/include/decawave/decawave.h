

#ifndef DECAWAVE_H
#define DECAWAVE_H
// VERSION 1.0.0

#include <string>
#include <vector>
#include <serial/serial.h>
#include <memory>

namespace decawave{
struct Anchor{
  int id;
  double distance;
  int distance_quality;
  double position[3];
  int quality_factor;
};

class Decawave{
public:
  Decawave(std::string port_name);
  ~Decawave();
  std::vector<Anchor> updateSamples();
private:
  int coords_updated;
  std::shared_ptr<serial::Serial> my_serial;
};
}
#endif
