/*
 * decawave.cpp
 * Interfaces with Decawave 1001 module over USB
 * VERSION: 0.0
 * Last changed: 2020-01-22
 * Authors: favou005@umn.edu>
 * Maintainers: favou005@umn.edu
 * MIT License
 * Copyright (c) 2020 GOFIRST-Robotics
 */


#include <iostream>

#include "decawave/decawave.h"
#include "serial/serial.h"


namespace decawave{
Decawave::Decawave(std::string port_name){
  //Serial connection to decawave (tag)
  // /dev/ assumes a linux environment
  my_serial.reset(new serial::Serial(port_name, 115200, serial::Timeout::simpleTimeout(1000)));//opens the port
}

std::vector<Anchor> Decawave::updateSamples(){
  unsigned char coords_updated = 0;
  std::vector<Anchor> anchors = {};
  // Flush input buffer so that any detritus from previous commands doesn't mess up our reading
  my_serial->flushInput();
  if (my_serial->isOpen()) {
    //first check if the decawave has new information
    //sending command dwm_status_get uses code {0x32,0x00}
    my_serial->write((std::vector <unsigned char>){0x32,0x00});
    // Returns 1 if there is new data to read
    if(my_serial->read(&coords_updated, 1) == 0) {
      coords_updated = 0;
    }

    if (coords_updated) {
      //sending command: dwm_loc_get
      my_serial->write((std::vector <unsigned char>){0x0c,0x00});
    } 
    else { //no new data
      return anchors;
    }
  } 
  else {
    return anchors;
  }
  //array to hold bytes received from decawave
  unsigned char result[26];//maximum size to be read at once
  my_serial->read(result, 6); // These first six bytes are unused
  int bytesread = my_serial->read(result, 21);
  /* used for testing prints out first X bytes read */
  /*std::cout << '\n' << "====== message back ======" << '\n'<< '\n';
  for(int i =0; i<bytesread;i++){
      std::cout << "byte " << (i+1) << ": " << (int) result[i] << '\n';
  }*/
  
  if (bytesread == 21) {
    if (result[2] == 0) { // result[2] is error status, 0 is OK
      // Get the number of anchors that the tag sees
      int num_anchors = (int) result[20];
      Anchor anchor;
      for (int i=0; i<num_anchors; i++) {
        // Each anchor provides 20 bytes
        if (my_serial->read(result, 20) != 20) {
          // We ran out of data before reading all the anchors
          return anchors;
        }
        //read one anchor of data
        anchor.id = ((int)result[0]) | ((int)result[1]<<8);
        anchor.distance = (double) ((((int)result[2]) | ((int)result[3]<<8) | ((int)result[4]<<16) | ((int)result[5]<<24)));
        anchor.distance_quality = (int)result[6];
        anchor.position[0] = ((int)result[7]) | ((int)result[8]<<8) | ((int)result[9]<<16) | ((int)result[10]<<24); //x
        anchor.position[1] = ((int)result[11]) | ((int)result[12]<<8) | ((int)result[13]<<16) | ((int)result[14]<<24); //y
        anchor.position[2] = ((int)result[15]) | ((int)result[16]<<8) | ((int)result[17]<<16) | ((int)result[18]<<24); //z
        anchor.quality_factor = (int)result[19];

        // Convert to meters
        anchor.distance /= 1000.0;
        anchor.position[0] /= 1000.0;
        anchor.position[1] /= 1000.0;
        anchor.position[2] /= 1000.0;
        anchors.push_back(anchor);
      }
    }
  }
  else { // there was a transmission error
    return anchors;
  }
  return anchors;
}

Decawave::~Decawave(){
  my_serial->close();//close the serial port
}

}
//serial library
// http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html


//decawave manuals, (firmware API guide)
// https://www.decawave.com/wp-content/uploads/2019/01/DWM1001-API-Guide-2.2.pdf
