#include "socket_pi.h"

int send_pose_msg(sl::Pose pose){
    MSG_POSE msg;
    msg.x = pose.getTranslation().tx;
    msg.y = pose.getTranslation().ty;
    msg.z = pose.getTranslation().tz;
    msg.ox = pose.getOrientation().ox;
    msg.oy = pose.getOrientation().oy;
    msg.oz = pose.getOrientation().oz;
    msg.ow = pose.getOrientation().ow;
    printf("%lf, %lf, %lf\n", msg.x, msg.y, msg.z);
    return send_msg(&msg, sizeof(msg));
}

int send_obs_msg(sl::Objects obs){
    MSG_OBSTACLE msg;
    const size_t unit_size = sizeof(msg);
    const size_t buf_len = unit_size * obs.object_list.size();
    uint8_t buf[buf_len];
    int idx = 0;
    //for (auto object: obs.object_list){
    for (int i=0; i< obs.object_list.size(); i++){
        auto object = obs.object_list[i];
        msg.id = object.id;
        if ( static_cast<int>(object.label) == 0 )
            msg.label = 1;
        else if( static_cast<int>(object.label) == 1 )
            msg.label = 2;
        else
            msg.label = 0;
        msg.x = object.position.x;
        msg.y = object.position.y;
        msg.z = object.position.z;
        msg.vx = object.velocity.x;
        msg.vy = object.velocity.y;
        msg.vz = object.velocity.z;
        memcpy(buf+idx*unit_size, &msg, unit_size);
        idx++;
    }
    return send_msg(buf, buf_len);
}

int send_msg(void* data, int len){
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
    //Send a message to server
    char receiveMessage[100] = {};
    send(sockfd, data, len, 0);
    recv(sockfd, receiveMessage, sizeof(receiveMessage), 0);

    printf("%s",receiveMessage);
    close(sockfd);
    return 0;
}
