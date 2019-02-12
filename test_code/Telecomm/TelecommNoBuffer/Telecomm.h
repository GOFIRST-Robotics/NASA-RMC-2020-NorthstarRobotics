#ifndef TELECOMM_H
#define TELECOMM_H
// VERSION 1.1.1

#include <string>

/* 
  Telecomm defines macros and functions to send and recieve data between two 
  computers performing intensive operations in parallel. If in a live stream
  control scenario, or the adjustment of state space, mode handling and state
  handling is not handled here. 

  The handling of state is best done over another file, Protocal.h, which
  should have macros defining the content of the message and thus how to 
  handle the message. Ex. post to user, update motors, etc. 
*/

class Telecomm {
  public: 
    // Initialize communications: Telecomm comm = new Telecomm("127.0.0.1", 5005, 5006);
   // new Telecomm("192.168.1.117",5005,5005) is pi side, Telecomm("192.168.1.50",5005,5005) is laptop side
    Telecomm(std::string dst_addr, int dst_port, int src_port);
    ~Telecomm();

    // In loop, call update; if != 0, error
    int update();

    // Can send data at any time, no wait; if !=0, error
    int send(std::string msg);
    int sendBytes(char* bytes, int length);

    // Can recv data if available, check status
    bool recvAvail();
    std::string recv();
    // Checks for bytes
    int recv(char*& buf);

    // If connection is closed, do not continue, unless reboot'd
    bool isCommClosed();
    // reboot() fixes closed connection, but blocks until connectin is restored
    void reboot();

    // Can only read from STDIO if available
    bool stdioReadAvail();
    std::string stdioRead();

    // Set blocking time. If -1 (either), inf; if 0, immediate/no block
    void setBlockingTime(int sec, int usec);
    // Blocking: Telecomm uses file descriptors and select
    // Can check other file descriptors as well, choose non-blocking I think?
    bool fdReadAvail(int fd);
    // Add file descriptor to be checked
    void fdAdd(int fd);
    // Stop checking file descriptor; not removing when object is deleted is ok
    void fdRemove(int fd);

    // getErrno() returns the errno set by any socket functions
    int getErrno();
    // status() returns the internal state of the status, status() !=0 is error
    int status();
    // simpleStatus: a string code detailing the problem indicated by/of status
    std::string simpleStatus(int status);
    std::string simpleStatus();
    // verboseStatus: a more verbose version of simpleStatus if available
    std::string verboseStatus(int status);
    std::string verboseStatus();

    // Set failure to throw error, true/default, or not/false
    void setFailureAction(bool throwError);
};

#endif
