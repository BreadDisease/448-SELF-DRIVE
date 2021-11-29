#include <iostream>
#include <sstream>
#include <vector>
#include <RF24/RF24.h>
#include <boost/mpi.hpp>
#include <zmq.hpp>

const int RX_RANK = 0;
const int TX_RANK = 1;
const int REQ_TAG = 0;

struct Payload {
    uint8_t direction;
    uint8_t steering;
    uint8_t speed;
    bool    state;
};

void transmitter() {
    // Set up MPI communications
    boost::mpi::communicator world;
    std::vector<int> data;

    // Initialize payload
    Payload payload;

    // Set up radio hardware
    RF24 radio(17, 0);
    uint64_t address = 0x7878787878LL;
    if (!radio.begin()) {
        std::cerr << "Radio initialization failed." << std::endl;
        return;
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(sizeof(payload));

    // Put radio in TX mode
    radio.stopListening();
    radio.openWritingPipe(address);

    while (true) {
        // Receive data to transmit
        world.recv(RX_RANK, REQ_TAG, data);

        // Set payload data
        payload.direction = data[0];
        payload.steering  = data[1];
        payload.speed     = data[2];
        payload.state     = data[3];

        // Send the data
        if (radio.write(&payload, sizeof(payload))) {
            std::cout << "Delivered " << (int) sizeof(payload) << " bytes." << std::endl;
        } else {
            std::cout << "Payload failed to deliver." << std::endl;
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
        std::vector<int> cmdData(4);
        iss >> cmdData[0] >> cmdData[1] >> cmdData[2] >> cmdData[3];
        // Send data to transmitter process
        world.send(TX_RANK, REQ_TAG, cmdData);

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
