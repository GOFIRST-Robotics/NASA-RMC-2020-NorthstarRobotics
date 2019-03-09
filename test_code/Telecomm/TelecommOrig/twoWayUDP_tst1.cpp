//  https://stackoverflow.com/questions/9741392/can-you-bind-and-connect-both-ends-of-a-udp-connection#23635437 
/* 
Usage: ./<program_name> dst-hostname dst-udpport src-udpport

I tested this code opening two terminals. You should be able to send a message to the destination node and receive messages from it.

In terminal 1 run

./<program_name> 127.0.0.1 5555 5556

In terminal 2 run

./<program_name> 127.0.0.1 5556 5555

Even though I've tested it on a single machine I think it should also work on two different machines once you've setup the correct firewall settings

Here's a description of the flow:

    1) Setup hints indicated the type of destination address as that of a UDP connection
    2) Use getaddrinfo to obtain the address info structure dstinfo based on argument 1 which is the destination address and argument 2 which is the destination port
    3) Create a socket with the first valid entry in dstinfo
    4) Use getaddrinfo to obtain the address info structure srcinfo primarily for the source port details
    5) Use srcinfo to bind to the socket obtained
    6) Now connect to the first valid entry of dstinfo
    7) If all is well enter the loop
    8) The loop uses a select to block on a read descriptor list which consists of the STDIN and sockfd socket created
    9) If STDIN has an input it is sent to the destination UDP connection using sendall function
    10) If EOM is received the loop is exited.
    11) If sockfd has some data it is read through recv
    12) If recv returns -1 it is an error we try to decode it with perror
    13) If recv returns 0 it means the remote node has closed the connection. But I believe has no consequence with UDP a which is connectionless.

    A communication is a unique combination of 5 components: (protocol, src addr, src port, dst addr, dst port).
      The protocol is UDP/TCP, the src addr/port are set by bind(), and the dst addr/port are set by connect().
      UDP isn't automatically bound, the unique dst are set in the sendto recvfrom commands, but if connect()'d, can use the send recv commands. 
*/

//#include <stdio.h>
#include <cstdio>
//#include <stdlib.h>
#include <cstdlib>
#include <unistd.h>
//#include <errno.h>
#include <cerrno>
//#include <string.h>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define STDIN 0

int sendall(int s, char *buf, int *len)
{
    int total = 0;        // how many bytes we've sent
    int bytesleft = *len; // how many we have left to send
    int n;

    while(total < *len) {
        n = send(s, buf+total, bytesleft, 0);
        fprintf(stdout,"Sendall: %s\n",buf+total);
        if (n == -1) { break; }
        total += n;
        bytesleft -= n;
    }

    *len = total; // return number actually sent here

    return n==-1?-1:0; // return -1 on failure, 0 on success
} 

int main(int argc, char *argv[])
{
   int sockfd;
   struct addrinfo hints, *dstinfo = NULL, *srcinfo = NULL, *p = NULL;
   int rv = -1, ret = -1, len = -1,  numbytes = 0;
   struct timeval tv;
   char buffer[256] = {0};
   fd_set readfds;

   // don't care about writefds and exceptfds:
   //     select(STDIN+1, &readfds, NULL, NULL, &tv);

   if (argc != 4) {
      fprintf(stderr,"usage: ./programname dst-hostname dst-udpport src-udpport\n");
      ret = -1;
      goto LBL_RET;
   }

  // Step 1: dst info
   memset(&hints, 0, sizeof hints);
   hints.ai_family = AF_UNSPEC;
   hints.ai_socktype = SOCK_DGRAM;        //UDP communication

   /*For destination address*/ // Step 2
   if ((rv = getaddrinfo(argv[1], argv[2], &hints, &dstinfo)) != 0) {
      fprintf(stderr, "getaddrinfo for dest address: %s\n", gai_strerror(rv));
      ret = 1;
      goto LBL_RET;
   }

  // Step 3: socket for dst info, sockfd is dst
   // loop through all the results and make a socket
   for(p = dstinfo; p != NULL; p = p->ai_next) {

      if ((sockfd = socket(p->ai_family, p->ai_socktype,
                  p->ai_protocol)) == -1) {
         perror("socket");
         continue;
      }
      /*Taking first entry from getaddrinfo*/
      break;
   }

   /*Failed to get socket to all entries*/
   if (p == NULL) {
      fprintf(stderr, "error: Failed to get socket\n");
      ret = 2;
      goto LBL_RET;
   }

  // Step 4: src info
   /*For source address*/
   memset(&hints, 0, sizeof hints);
   hints.ai_family = AF_UNSPEC;
   hints.ai_socktype = SOCK_DGRAM;        //UDP communication
   hints.ai_flags = AI_PASSIVE;     // fill in my IP for me
   /*For source address*/
   if ((rv = getaddrinfo(NULL, argv[3], &hints, &srcinfo)) != 0) { // Note null ip
      fprintf(stderr, "getaddrinfo for src address: %s\n", gai_strerror(rv));
      ret = 3;
      goto LBL_RET;
   }

  // Step 5: bind for us, src
   /*Bind this datagram socket to source address info */
   if((rv = bind(sockfd, srcinfo->ai_addr, srcinfo->ai_addrlen)) != 0) {
      fprintf(stderr, "bind: %s\n", gai_strerror(rv));
      ret = 3;
      goto LBL_RET;
   }

  // Step 6, connect - created a unique communication: finished all 5 components
   /*Connect this datagram socket to destination address info */
   if((rv= connect(sockfd, p->ai_addr, p->ai_addrlen)) != 0) {
      fprintf(stderr, "connect: %s\n", gai_strerror(rv));
      ret = 3;
      goto LBL_RET;
   }

  // Step 7: The Loop
   while(1){
      // man fd_set; readfds is a set of file descriptors to read from
      FD_ZERO(&readfds);
      FD_SET(STDIN, &readfds); // STDIN = 0
      FD_SET(sockfd, &readfds);

      /*Select timeout at 10s*/
      tv.tv_sec = 10;
      tv.tv_usec = 0;
      // Step 8: Blocks for tv seconds
      select(sockfd + 1, &readfds, NULL, NULL, &tv); // arg0 is max fd of set plus 1

      // select() is used to efficiently monitor multiple file descriptors (fd's) to see
      // if any of them become "ready." This is most difficult thing here. 
      // select(int nfds/num_of_fds, fd_set *readfds, fd_set *writefds, 
      //    fd_set *exceptfds, struct timeval *utimeout);
      // nfds is the max int + 1 of all fds; 
      // readfds is a set loaded w/ all fds, and when select() returns, it will only 
      //    contain those available to read - this is syncronous io multiplexing
      // writefds watches for space to write to fds, not relevant for UDP, null
      // exceptfds is watched for "exceptional conditions:" 
      //    aka out-of-band (OOB) for TCP socket, not relevant for UDP, null
      // utimeout is the longest time before returning, it blocks until ready or timeout
      //    setting to null blocks indef, setting to 0 to return immediately

      // Step 9: receive user stdin input
      /*Obey your user, take his inputs*/
      if (FD_ISSET(STDIN, &readfds))
      {
         memset(buffer, 0, sizeof(buffer));
         len = 0;
         printf("A key was pressed!\n");
         if(0 >= (len = read(STDIN, buffer, sizeof(buffer))))
         {
            perror("read STDIN");
            ret = 4;
            goto LBL_RET;
         }

         fprintf(stdout, ">>%s\n", buffer);

         // Step 10: exit the loop on user input "EOM\n"
         /*EOM\n implies user wants to exit*/
         if(!strcmp(buffer,"EOM\n")){
            printf("Received EOM closing\n");
            break;
         }

         /*Sendall will use send to transfer to bound sockfd*/
         if (sendall(sockfd, buffer, &len) == -1) {
            perror("sendall");
            fprintf(stderr,"%s: We only sent %d bytes because of the error!\n", argv[0], len);
            ret = 5;
            goto LBL_RET;
         }  
      }

      // Step 11: receive data from socket
      /*We've got something on our socket to read */
      if(FD_ISSET(sockfd, &readfds))
      {
         memset(buffer, 0, sizeof(buffer));
         printf("Received something!\n");
         /*recv will use receive to connected sockfd */
         numbytes = recv(sockfd, buffer, sizeof(buffer), 0);
         if(0 == numbytes){
         // Step 13: recv closed; w/ UDP, we can still send (meaningless), and receive nada
            printf("Destination closed\n");
            break;
         }else if(-1 == numbytes){
         // Step 12: recv error, decode
            /*Could be an ICMP error from remote end*/
            perror("recv");
            printf("Receive error check your firewall settings\n");
            ret = 5;
            goto LBL_RET;
         }
         fprintf(stdout, "<<Number of bytes %d Message: %s\n", numbytes, buffer);
      }

      /*Heartbeat*/
      printf(".\n");
   }

   ret = 0;
LBL_RET:

   if(dstinfo)
      freeaddrinfo(dstinfo);

   if(srcinfo)
      freeaddrinfo(srcinfo);

   close(sockfd);

   return ret;
}
