#include <cstdio>
#include <cstring>
#include <string>
#include <queue>
//http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html
#include "Telecomm.h"
#define ERR_CHECK \
  do { if (comm.status() != 0){ \
    fprintf(stdout, "Error: %s\n", comm.verboseStatus().c_str()); \
    return comm.status(); \
  } } while(0)
// Should add goto LBL_REBOOT to this, that's what it's really good for

int main(int argc, char *argv[]){
  if (argc != 4){
    fprintf(stderr, "usage: ./programname dst-hostname dst-udpport src-udpport\n");
    exit(1);
  }

  Telecomm comm(argv[1], atoi(argv[2]), atoi(argv[3]));
  comm.setFailureAction(false);
  comm.setBlockingTime(0,0);
  comm.enableMessageRecovery(false);
  ERR_CHECK;

  time_t timer;
  time(&timer);

  // Joystick js(); // #include "joystick.hh"
  // Exit if !js.isFound()
  // comm.fdAdd(js.fd());

  // Until user types, or receives from remote, "EOM\n", loop
  while(1){
    comm.update();
    ERR_CHECK;

    // JoystickEvent event;

    // Receive from remote
    if(comm.recvAvail()){
      std::string msg = comm.recv();
      //ERR_CHECK; // This makes it crash???

      if(!msg.empty())
        fprintf(stdout, "Received message: %s\n", msg.c_str());
      
      if(!msg.compare("EOM\n")){ break; } // delete if not want remote close
    }

    // Call restore to repair any broken connection if so
    // Sends dropped messages as well if enableMessageRecovery
    comm.restore();

    // Get user stdio input to send
    if(comm.stdioReadAvail()){
      std::string msg = comm.stdioRead();
      ERR_CHECK;

      if(!msg.compare("EOM\n")){ // Assume desired for connection to close
        comm.send(msg); // Delete for debug connection testing
        printf("Received EOM closing\n");
        break;
      }
      
      comm.send(msg);
      ERR_CHECK;
    }

    // Example if including joystick
    // if(comm.fdReadAvail(js.fd) && js.sample(&event)){
    //   ... process buttons and axis of event ... }

    // heartbeat
    if(difftime(time(NULL), timer) > 10){
      timer = time(NULL);
      printf(".\n");
    }
  }
}
