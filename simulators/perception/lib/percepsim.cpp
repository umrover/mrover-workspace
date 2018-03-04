#include <nanomsg/reqrep.h>
#include <iostream>

#include "percepsim.hpp"
#include "wire_protocol.hpp"

#define SOCKET "ipc:///tmp/percepsim.ipc"

Perception::SimulatedCamera::SimulatedCamera() :
    sock_(AF_SP, NN_REQ)
{
    const int resend_interval = 100;
    this->sock_.setsockopt(NN_REQ, NN_REQ_RESEND_IVL, &resend_interval, sizeof(int));
    this->sock_.connect(SOCKET);
}

bool Perception::SimulatedCamera::grab() noexcept {
    try {
        auto req = Protocol::Request{};
        this->sock_.send(&req, sizeof(req), 0);

        void *reply_buf;
        std::cerr << "waiting on a message\n";
        this->sock_.recv(&reply_buf, NN_MSG, 0);
        std::cerr << "received a messsage\n";
        Protocol::Reply *rep = reinterpret_cast<Protocol::Reply *>(reply_buf);

        this->image_.create(rep->height, rep->width, rep->image_type);
        this->depth_.create(rep->height, rep->width, rep->depth_type);

        std::memcpy(this->image_.data, rep->data, rep->image_size);
        std::memcpy(this->depth_.data, &rep->data[rep->image_size], rep->depth_size);

        nn::freemsg(reply_buf);
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

Perception::Simulator::Simulator() :
    sock_(AF_SP, NN_REP)
{
    this->endpt_ = this->sock_.bind(SOCKET);
}

Perception::Simulator::~Simulator() {
    this->sock_.shutdown(this->endpt_);
}

void Perception::Simulator::publish(cv::Mat image, cv::Mat depth) {
    // Wait for request
    Protocol::Request req;
    this->sock_.recv(&req, sizeof(req), 0);

    // Push reply
    int width = image.cols;
    int height = image.rows;

    image = (image.reshape(0, 1));
    depth = (depth.reshape(0, 1));

    size_t image_size = image.total()*image.elemSize();
    size_t depth_size = depth.total()*depth.elemSize();

    size_t message_size = sizeof(Protocol::Reply) + image_size + depth_size;
    void *msgbuf = nn::allocmsg(message_size, 0);
    Protocol::Reply * rep = reinterpret_cast<Protocol::Reply *>(msgbuf);

    rep->width = width;
    rep->height = height;

    rep->image_type = image.type();
    rep->depth_type = depth.type();

    rep->image_size = image_size;
    rep->depth_size = depth_size;

    std::memcpy(rep->data, image.data, rep->image_size);
    std::memcpy(&rep->data[rep->image_size], depth.data, rep->depth_size);

    std::cerr << "sending " << message_size << " bytes\n";
    this->sock_.send(&msgbuf, NN_MSG, 0);
}
