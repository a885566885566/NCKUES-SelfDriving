#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

/*
 * Refer to https://www.kernel.org/doc/Documentation/networking/can.txt
 * This library can be used to setup can protocal including bitrate.
 * https://github.com/lalten/libsocketcan
 */

int main(int argc, char **argv){
    int s;  // socket descriptor
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s=socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Failed to open socket");
		return 1;
	}

    strcpy(ifr.ifr_name, "can0" );

    /* 
     * I/O control and management
     * https://www.man7.org/linux/man-pages/man2/ioctl.2.html 
     * 
     * @Args: 
     *      int fd: open file descriptor
     *      ulong request: device-dependent request code
     *      char* argp
     */
	if (ioctl(s, SIOCGIFINDEX, &ifr)<0){
		perror("Failed to call ioctl");
		return 1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    /*
     * To bind a socket to all(!) CAN interfaces the interface index must
     * be 0 (zero). In this case the socket receives CAN frames from every
     * enabled CAN interface. To determine the originating CAN interface
     * the system call recvfrom(2) may be used instead of read(2). To send
     * on a socket that is bound to 'any' interface sendto(2) is needed to
     * specify the outgoing interface.
     */
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		perror("Fail to Bind");
		return 1;
	}
}
