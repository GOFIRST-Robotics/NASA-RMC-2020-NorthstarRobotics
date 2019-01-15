//g++ decawaveDriver.cpp -std=c++11 decawave.cpp -lserial


#include "decawave/decawave.h"

#include <string>
#include <vector>
#include <iostream>
#include <math.h>

#include <serial/serial.h>

int main(){
  Decawave piTag;
  //if error, may need a delay?
  for(int i = 0; i < 10; ++i){
    piTag.updateSamples();
  }
  coordinate tagPos= piTag.getPos();
  std::cout<<"Robot position:";
  std::cout<<std::endl<<" "<<tagPos.x<<" "<<tagPos.y<<" "<<std::endl<<std::endl;
  std::cout<<"Anchor radii"<<std::endl;
  std::cout<<piTag.anchor1[0]<<std::endl<<piTag.anchor1[7]<<std::endl;
  std::cout<<piTag.anchor2[0]<<std::endl<<piTag.anchor2[1]<<std::endl;
  return 0;
}
