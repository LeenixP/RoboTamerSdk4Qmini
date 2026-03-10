#include "user/custom.hpp"

int main(int argc, char const *argv[]) {
    std::cout << "Usage networkInterface: " << "loopback of Q1 robot " << std::endl;
    std::string networkInterface = "lo";
    G1 g1(networkInterface, false);

    while (true) sleep(10);

    return 0;
}
