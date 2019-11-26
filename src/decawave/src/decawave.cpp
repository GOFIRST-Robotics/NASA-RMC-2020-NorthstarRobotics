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

Decawave::Decawave(std::string port_name){
  //connecting via serial
  std::string port =port_name; // could be something else (was /dev/serial0)
  // Find serial port with "SEGGER" (aka decawave attached)- currently doesn't work, defaults to ^
  // std::vector<serial::PortInfo> devices_found = serial::list_ports();
  // std::vector<serial::PortInfo>::iterator iter = devices_found.begin();
  // while(iter != devices_found.end()){
  //   serial::PortInfo device = *iter++;
  //   if(device.description.find("SEGGER") != std::string::npos){
  //     port = device.port;
  //   }
  // }

  //Serial connection to decawave (tag)
  // /dev/ assumes a linux environment
  my_serial.reset(new serial::Serial(("/dev/"+port), 115200, serial::Timeout::simpleTimeout(1000)));//opens the port
}

std::vector<anchor> Decawave::updateSamples(){
  unsigned char coords_updated=0;
  std::vector<anchor> anchors={};
  if(my_serial->isOpen()){
    //first check if the decawave has new information
    //sending command 4.3.19  dwm_status_get uses code {0x32,0x00}
    my_serial->write((std::vector <unsigned char>){0x32,0x00});
    //output is a single byte, read into class variable
    if(my_serial->read(&coords_updated, 1)==0){//handle no bytes read
      coords_updated=0;//false
    }
    if(coords_updated){
    //sending command: dwm_loc_get (see 4.3.9 in dwm1001 api)
      my_serial->write((std::vector <unsigned char>){0x0c,0x00});
    }else{//no new data
      return(anchors);
    }
  }
  //array to hold bytes received from decawave
  unsigned char result[26];//maximum size to be read at once
  int bytesread=my_serial->read(result, 26);
  std::cout << '\n' << "====== message back ======" << '\n'<< '\n';
  for(int i =0; i<bytesread;i++){
      std::cout << "byte " << (i+1) << ": " << (int) result[i] << '\n';
  }
  if (bytesread==26){
    if (result[1]==0){//check no error code
      int num_anchors=(int) result[25];
      std::cout << (int)result[25] << '\n';
      int i=0;
      anchor anchor;
      while(i<num_anchors){
        if (my_serial->read(result, 20)!=20){
          return (anchors);
        }//read one anchor of data
        anchor.id= ((int)result[0]) | ((int)result[1]<<8);
        anchor.distance= ((int)result[2]) | ((int)result[3]<<8) | ((int)result[4]<<16) | ((int)result[5]<<24);
        anchor.distance_quality= (int)result[6];
        anchor.position[0]=((int)result[7]) | ((int)result[8]<<8) | ((int)result[9]<<16) | ((int)result[10]<<24);//x
        anchor.position[1]=((int)result[11]) | ((int)result[12]<<8) | ((int)result[13]<<16) | ((int)result[14]<<24);//y
        anchor.position[2]=((int)result[15]) | ((int)result[16]<<8) | ((int)result[17]<<16) | ((int)result[18]<<24);//z
        anchor.quality_factor=(int)result[19];
        anchors.push_back(anchor);
        i++;

      }
    }
  }else{//error returns empty vector
    return(anchors);
  }
  return (anchors);
}


Decawave::~Decawave(){
  my_serial->close();//close the serial port
}

//serial library
// http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html


//decawave manuals, (firmware API guide)
// https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
//https://www.digikey.com/product-detail/en/decawave-limited/DWM1001-DEV/1479-1005-ND/7394533
