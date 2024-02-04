#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <array>
#include <errno.h>

namespace udp{
    class UdpClient{
      private:
        int socket_num;
        struct sockaddr_in serveraddr, localaddr;
        const int kMaxBufferSize = 1024;
        struct timeval tv;
        const int activate = 1;
        const int retries = 3;

      public:
        bool is_connected = false;
        UdpClient(std::string ip_adress, int port);
        ~UdpClient();

        void connect();
        void disconnect();

        void sendMsg(std::string msg);
        std::string rcvMsg();
    };
}