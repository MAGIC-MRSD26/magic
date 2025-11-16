// client.cpp
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

int main() {
    const char* SERVER_IP = "192.168.1.10"; // <-- change to Python machine's IP
    const int PORT = 5001;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation error\n";
        return -1;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address / Address not supported\n";
        return -1;
    }

    std::cout << "[CLIENT] Connecting...\n";
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection failed\n";
        return -1;
    }

    std::cout << "[CLIENT] Connected to server\n";

    // Send a test message
    uint8_t msg = 1;
    send(sock, &msg, 1, 0);
    std::cout << "[CLIENT] Sent: " <<(int)msg << "\n";

    // Receive response
    uint8_t buffer = 1;
    int valread = read(sock, &buffer, 1);
    if (valread > 0) {
        std::cout << "[CLIENT] Server says: " << (int)buffer << "\n";
    }
    if (buffer == 1) {
        std::cout << "[CLIENT] Received confirmation from server\n";
    }

    close(sock);
    return 0;
}
