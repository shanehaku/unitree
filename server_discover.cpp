#include <iostream>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

#define DISCOVERY_PORT 9999
#define SERVER_PORT 12345
#define DISCOVERY_MSG "WHERE_ARE_YOU?"
#define RESPONSE_MSG "I_AM_HERE"

void udp_discovery_responder() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DISCOVERY_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    bind(sock, (sockaddr*)&addr, sizeof(addr));

    char buf[1024];
    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);

    while (true) {
        int len = recvfrom(sock, buf, sizeof(buf) - 1, 0, (sockaddr*)&client_addr, &addr_len);
        buf[len] = '\0';

        if (std::string(buf) == DISCOVERY_MSG) {
            std::cout << "Client asked for discovery. Responding...\n";
            sendto(sock, RESPONSE_MSG, strlen(RESPONSE_MSG), 0, (sockaddr*)&client_addr, addr_len);
        }
    }

    close(sock);
}

