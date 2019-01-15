#ifndef DECAWAVE_H
#define DECAWAVE_H
// VERSION 1.0.0

#include <serial/serial.h>
#include <memory>

struct coordinate{
  double x;
  double y;
};

class Decawave{
public:
  Decawave();
  ~Decawave();
  void updateSamples();
  coordinate getPos();
  double anchor1[8];//was unsigned long int *was a double before...
  double anchor2[8];
private:
  int index;
  std::shared_ptr<serial::Serial> my_serial;

  coordinate anchor1Pos;
  coordinate anchor2Pos;
  coordinate tagPos;
  double anchorSeparation;
};

#endif
