#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define COMMU_PORT 8787
#define TARGET_HOST "10.1.1.12"

typedef struct{
    int32_t mode;
    double a;
    double b;
    double c;
} MSG;

int main(int argc , char *argv[]){
    // Create socket file descriptor
    int sockfd = 0;
    sockfd = socket(AF_INET , SOCK_STREAM , 0);

    if (sockfd == -1){
        printf("Fail to create a socket.");
        return -1;
    }

    struct sockaddr_in info;
    bzero(&info,sizeof(info));
    info.sin_family = AF_INET;  // IPv4

    info.sin_addr.s_addr = inet_addr(TARGET_HOST);
    info.sin_port = htons(COMMU_PORT);

    int err = connect(sockfd, (struct sockaddr*)&info, sizeof(info));
    if(err==-1){
        printf("Connection error");
        return -1;
    }
    
    int len = 5 * sizeof(MSG);
    char buf[len];
    MSG msg;
    msg.mode = 1;
    msg.a = 100;
    msg.b = 101;
    msg.c = 102;
    for(int i=0; i<5; i++){
        msg.mode = i;
        memcpy(buf+i*sizeof(MSG), &msg, sizeof(MSG));
    }

    //Send a message to server
    char receiveMessage[100] = {};
    printf("Try send\n");
    send(sockfd, &buf, sizeof(buf), 0);
    recv(sockfd, receiveMessage, sizeof(receiveMessage), 0);

    printf("%s",receiveMessage);
    printf("close Socket\n");
    close(sockfd);
    return 0;
}
