#include <itomp_nlp/network/trajectory_publisher.h>


namespace itomp
{

TrajectoryPublisher::TrajectoryPublisher()
    : zmq_context_(1)
    , zmq_publisher_(zmq_context_, ZMQ_PUB)
{
    zmq_publisher_.bind("tcp://*:54321");
}

void TrajectoryPublisher::publish(const Trajectory& trajectory)
{
    std::string serial = trajectory.serialize();

    zmq::message_t message(serial.size());
    memcpy(message.data(), &serial[0], serial.size());

    zmq_publisher_.send(message);
}

}
