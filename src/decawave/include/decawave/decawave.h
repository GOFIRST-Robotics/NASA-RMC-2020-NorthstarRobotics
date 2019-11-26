#ifndef DECAWAVE_H
#define DECAWAVE_H
// VERSION 1.0.0

#include <serial/serial.h>
#include <memory>

struct anchor{
  int id;
  int distance=0;
  int distance_quality;
  int position[3];
  int quality_factor;
};

class Decawave{
public:
  Decawave(std::string port_name);
  ~Decawave();
  std::vector<anchor> updateSamples();
private:
  int coords_updated;
  std::shared_ptr<serial::Serial> my_serial;
};

#endif
