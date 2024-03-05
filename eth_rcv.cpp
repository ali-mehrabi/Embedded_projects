#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT_NO  80

int main() {
    // Create a UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return 1;
    }

    // Set up the server address
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT_NO);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket
    if (bind(sock, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("Bind failed");
        close(sock);
        return 1;
    }

    // Receive data
    while (true) {
        unsigned char data[20];
        ssize_t bytesRead = recv(sock, data, sizeof(data), 0);

        if (bytesRead == -1) {
            perror("Receive failed");
            close(sock);
            return 1;
        }

        // Process the received data
        std::cout << "Received data: ";
        for (size_t i = 0; i < sizeof(data); ++i) {
            std::cout << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::endl;
    }

    // Close the socket (this part won't be reached in an infinite loop)
    close(sock);

    return 0;
}
