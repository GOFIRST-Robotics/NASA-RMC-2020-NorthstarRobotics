/*
 * decawave.cpp
 * Interfaces with Decawave 1001 module over USB
 * VERSION: 0.0
 * Last changed: 2019-04-01
 * Authors: {insert molly} <@umn.edu>
 * Maintainers: {insert molly} <@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

#include <string>
#include <vector>
#include <iostream>
#include <math.h>

#include "decawave/decawave.h"
#include "serial/serial.h"

Decawave::Decawave(){
  int index=0; //from 0 to 7, for donut array

  //anchor positions - these don't change
  coordinate anchor1Pos;
  coordinate anchor2Pos;
  anchor1Pos.x=-0.825;
  anchor1Pos.y=0.0;
  anchor2Pos.x=0.825;
  anchor2Pos.y=0.0;
  anchorSeparation=anchor2Pos.x-anchor1Pos.x; //distance between the 2 anchors

  //connecting via serial
  std::string port = "/dev/serial0"; // could be something else

  // Find serial port with "SEGGER" (aka decawave attached)- currently doesn't work, defaults to ^
  std::vector<serial::PortInfo> devices_found = serial::list_ports();
  std::vector<serial::PortInfo>::iterator iter = devices_found.begin();
  while(iter != devices_found.end()){
    serial::PortInfo device = *iter++;
    if(device.description.find("SEGGER") != std::string::npos){
      port = device.port;
    }
  }

  //Serial connection to decawave (tag)
  my_serial.reset(new serial::Serial(port, 115200, serial::Timeout::simpleTimeout(1000)));//opens the port
}

void Decawave::updateSamples(){
  //sending command: dwm_loc_get (see 4.3.9 in dwm1001 api)
  if(my_serial->isOpen()){
    my_serial->write((std::vector <unsigned char>){0x0c,0x00});
  }

  //array to hold bytes received from decawave
  unsigned char result[61];
  std::memset(result, 0, sizeof result);

  //read bytes from decawave
  int counter =0;
  while (counter<61 && my_serial->isOpen()){
    counter+= my_serial->read(result+counter, 61-counter);
  }

  //convert 2 sets of 4 byte anchor distances to ints (measured in milimeters, not real precise)
  unsigned long int an1dist= (result[23]) | (result[24]<<8) | (result[25]<<16) | (result[26]<<24);
  unsigned long int an2dist= (result[43]) | (result[44]<<8) | (result[45]<<16) | (result[46]<<24);

  //store distnaces in array, keep 8 most recent
  if (index>7){
    index=0;
  }
  anchor1[index]=static_cast<double>(an1dist)/1000.0;
  anchor2[index]=static_cast<double>(an2dist)/1000.0;
  index+=1;
}

coordinate Decawave::getPos(){
  //coordinate tagPos; //caculated tag position

  //average the distances
  double r1=0;
  double r2=0;
  for(int i = 0; i < 8; ++i){
    r1+=anchor1[i];
    r2+=anchor2[i];
  }
  r1=r1/8.0;
  r2=r2/8.0;
  double d=anchorSeparation;
  tagPos.x=(d*d-r2*r2+r1*r1)/(2.0*d); //These assume anchor 1 is at (0,0)... adjust later
  tagPos.y=(1.0/(2.0*d))*sqrt(4.0*d*d*r1*r1-(d*d-r2*r2+r1*r1)*(d*d-r2*r2+r1*r1));
  /*
  //find angle
  double angleC= acos(((r1*r1)+(anchorSeparation*anchorSeparation)-(r2*r2))/(2.0*r1*anchorSeparation));//angle C in radians
  //x and y coords
  tagPos.x=r1*sin(angleC)+anchor1Pos.x;
  tagPos.y=r1*sin(angleC)+anchor1Pos.y;
  std::cout<<r1<<" "<<r2<<" "<<std::endl;
  std::cout<<(((r2*r2)-(anchorSeparation*anchorSeparation)-(r1*r1))/(-2.0*r1*anchorSeparation));
  */
  //std::cout<<tagPos.x<<" "<<tagPos.y<<std::endl;
  return tagPos;
}

Decawave::~Decawave(){
  my_serial->close();//close the serial port
}
