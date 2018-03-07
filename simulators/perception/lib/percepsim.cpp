#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

#include "percepsim.hpp"
#include "wire_protocol.hpp"

#define SOCKET "ipc:///tmp/percepsim.ipc"

inline void fatal(const char *func) {
    std::cerr << "[percepsim] " << func << ": " << nn_strerror(nn_errno()) << std::endl;
    std::exit(1);
}

Perception::SimulatedCamera::SimulatedCamera() {
    if ((this->sock_ = nn_socket(AF_SP, NN_PULL)) < 0) {
        fatal("nn_socket");
    }
    int rv;
    if ((rv = nn_bind(this->sock_, SOCKET)) < 0) {
        fatal("nn_bind");
    }
    int val = -1;
    if ((rv = nn_setsockopt(this->sock_, NN_SOL_SOCKET, NN_RCVMAXSIZE, &val, sizeof(int))) < 0) {
        fatal("nn_setsockopt");
    }
}

Perception::SimulatedCamera::~SimulatedCamera() {
    nn_close(this->sock_);
}

bool Perception::SimulatedCamera::grab() noexcept {
    try {
        char *reply_buf = nullptr;
        int bytes;
        if ((bytes = nn_recv(this->sock_, &reply_buf, NN_MSG, 0)) < 0) {
            fatal("nn_recv");
        }
        Protocol::Reply *rep = reinterpret_cast<Protocol::Reply *>(reply_buf);

        this->image_.create(rep->height, rep->width, rep->image_type);
        this->depth_.create(rep->height, rep->width, rep->depth_type);

        std::memcpy(this->image_.data, rep->data, rep->image_size);
        std::memcpy(this->depth_.data, &rep->data[rep->image_size], rep->depth_size);

        nn_freemsg(reply_buf);
        rep = nullptr;
    } catch (...) {
        return false;
    }
    return true;
}

cv::Mat Perception::SimulatedCamera::retrieve_image() noexcept {
    return this->image_;
}

cv::Mat Perception::SimulatedCamera::retrieve_depth() noexcept {
    return this->depth_;
}

Perception::Simulator::Simulator() {
    using namespace std::chrono_literals;
    if ((this->sock_ = nn_socket(AF_SP, NN_PUSH)) < 0) {
        fatal("nn_socket");
    }
    int rv;
    if ((rv = nn_connect(this->sock_, SOCKET)) < 0) {
        fatal("nn_connect");
    }
    std::this_thread::sleep_for(100ms);
}

Perception::Simulator::~Simulator() {
    nn_close(this->sock_);
}

void Perception::Simulator::publish(cv::Mat image, cv::Mat depth) {
    // Push reply
    int width = image.cols;
    int height = image.rows;

    image = (image.reshape(0, 1));
    depth = (depth.reshape(0, 1));

    size_t image_size = image.total()*image.elemSize();
    size_t depth_size = depth.total()*depth.elemSize();

    size_t message_size = sizeof(Protocol::Reply) + image_size + depth_size;
    void *msgbuf = nn_allocmsg(message_size, 0);
    Protocol::Reply * rep = reinterpret_cast<Protocol::Reply *>(msgbuf);

    rep->width = width;
    rep->height = height;

    rep->image_type = image.type();
    rep->depth_type = depth.type();

    rep->image_size = image_size;
    rep->depth_size = depth_size;

    std::memcpy(rep->data, image.data, rep->image_size);
    std::memcpy(&rep->data[rep->image_size], depth.data, rep->depth_size);

    int rv;
    if ((rv = nn_send(this->sock_, &msgbuf, NN_MSG, 0)) < 0) {
        fatal("nn_send");
    }
}
