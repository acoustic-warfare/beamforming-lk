/** @file receiver.cpp
 * @author Irreq, Tuva
*/

#include "receiver.h"

#if RECEIVER_DEBUG

/**
 * Prints the binary representation of a 32-bit unsigned integer.
 */
void printBinary(uint32_t n) {
    for (int i = 31; i >= 0; --i) {
        // Bitwise AND operation to check if the ith bit is set
        if ((n >> i) & 1)
            std::cout << "1";
        else
            std::cout << "0";

        // Print space after every 4 bits for readability
        if (i % 8 == 0)
            std::cout << " ";
    }
    std::cout << " " << n << std::endl;
}
#endif

int init_receiver(const char *address, const int port) {
    int socket_desc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_desc < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return -1;
    }

    // Set port and IP:
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = inet_addr(address);

    // Bind to the set port and IP:
    if (bind(socket_desc, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Couldn't bind socket to the port" << std::endl;
        return -1;
    }

    return socket_desc;
}

int receive_message(int socket_desc, message *msg) {
    if (recv(socket_desc, msg, sizeof(message), 0) < 0) {
        std::cerr << "Couldn't receive" << std::endl;
        return -1;
    } else {
        return 0;
    }
}