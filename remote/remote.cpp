#include <iostream>
#include <sstream>
#include <vector>
#include <boost/mpi.hpp>
#include <zmq.hpp>

#include "cc1100_raspi.h"
#include <wiringPi.h>

const int RX_RANK = 0;
const int TX_RANK = 1;
const int REQ_TAG = 0;

uint8_t RF_TX_ADDR = 1;
uint8_t RF_RX_ADDR = 3;

uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
 int8_t rssi_dbm;

int cc1100_freq_select = 434;
int cc1100_mode_select = 3;
int cc1100_channel_select = 1;
CC1100 radio;

void transmitter() {
    // Set up MPI communications
    boost::mpi::communicator world;
    std::vector<int> data;

    // Set up Pi hardware
    wiringPiSetup();
    // Initialize CC1100 library
    radio.begin(RF_TX_ADDR);
    radio.sidle();
    radio.set_output_power_level(0);
    radio.receive();

    while (true) {
        // Receive data to transmit
        world.recv(RX_RANK, REQ_TAG, data);
        Tx_fifo[3] = data[0];
        Tx_fifo[4] = data[1];
        Tx_fifo[5] = data[2];
        Tx_fifo[6] = data[3];
        Pktlen = 0x07;
        // Send the data
        std::cout << "Sending data: " << data[0]
                << ", " << data[1] << ", " << data[2]
                << ", " << data[3] << ", " << std::endl;
        uint8_t res = radio.sent_packet(RF_TX_ADDR, RF_RX_ADDR, Tx_fifo, Pktlen, 1);
        if (res == 1) {
            std::cout << "Packet sent successfully" << std::endl;
        }
    }
}

void receiver() {
    boost::mpi::communicator world;

    zmq::context_t context(2);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    socket.bind("tcp://127.0.0.1:5555");

    while (true) {
        zmq::message_t request;
        
        // Wait for next request from client
        socket.recv(request, zmq::recv_flags::none);

        // Process request data
        std::string data = std::string(static_cast<char*>(request.data()), request.size());
        std::istringstream iss(data);
        std::vector<int> data(4);
        iss >> data[0] >> data[1] >> data[2] >> data[3];
        // Send data to transmitter process
        world.send(TX_RANK, REQ_TAG, data);

        // Reply to client with ACK
        zmq::message_t reply((void *)"ACK", 3, NULL);
        socket.send(reply, zmq::send_flags::none);
    }
}

int main(int argc, char* argv[]) {
    boost::mpi::environment env(argc, argv);
    boost::mpi::communicator world;

    if (world.size() < 2) {
        std::cerr << "This program must be run with at least 2 processes." << std::endl;
    } else {
        if (world.rank() == RX_RANK) {
            receiver();
        } else {
            transmitter();
        }
    }

    return 0;
}
