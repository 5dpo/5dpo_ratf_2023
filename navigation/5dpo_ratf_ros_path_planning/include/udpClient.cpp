#include "../include/udpClient.h"

namespace udp{
    UdpClient::UdpClient(std::string ip_address, int port){
        memset(&serveraddr, 0, sizeof(serveraddr));
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_addr.s_addr = inet_addr(ip_address.c_str()); // Replace with server IP
        serveraddr.sin_port = htons(port); // Replace with server port

        localaddr.sin_family = AF_INET;
        localaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        localaddr.sin_port = htons(38689);

        tv.tv_sec = 10;
        tv.tv_usec = 0;
    }

    UdpClient::~UdpClient(){
        disconnect();
    }

    void UdpClient::connect(){
        std::string msg_rcv;

        socket_num = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_num < 0) {
           throw "UDP: Error creating socket.";
        }
        is_connected = true;

        if (setsockopt(socket_num, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv)) {
            throw "UDP: Error setting timeout options";
            close(socket_num);
        }

        if (setsockopt(socket_num, SOL_SOCKET, SO_REUSEADDR, &activate, sizeof(activate)) < 0) {
            throw "UDP: Error setting port options";
            close(socket_num);
        }      

        // if (setsockopt(socket_num, SOL_SOCKET, SO_KEEPALIVE, &activate, sizeof(activate)) < 0) {
        //     throw "UDP: Error setting port options";
        //     close(socket_num);
        // }    

        if (bind(socket_num, (struct sockaddr *)&localaddr, sizeof(localaddr)) < 0) {
            throw "Error binding socket";
            close(socket_num);
        } 

        for(int i=0; i< retries; i++){
            sendMsg("PING");
            msg_rcv = rcvMsg();

            if(!msg_rcv.compare("PONG")){
                is_connected = true;
                std::cout << "UDP connection established\n";
                return;
            }
            else{
                is_connected = false;
            }
        }
        
        close(socket_num);
        throw "UDP connection cannot be established";
    }

    void UdpClient::disconnect(){
        is_connected = false;
        close(socket_num);
    }

    void UdpClient::sendMsg(std::string msg){
        if (sendto(socket_num, msg.c_str(), msg.size(), 0, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
            std::cout << "UDP ERROR " << errno << std::endl;
            //throw "UDP: Error sending message.";
            close(socket_num);
        }
    }

    std::string UdpClient::rcvMsg(){
        socklen_t serverlen = sizeof(serveraddr);
        char buffer[kMaxBufferSize];

        int recvlen = recvfrom(socket_num, buffer, kMaxBufferSize, 0, (struct sockaddr*)&serveraddr, &serverlen);
        if (recvlen < 0) {
            return "";
        }

        buffer[recvlen] = '\0';

        return std::string(buffer);
    }
}
