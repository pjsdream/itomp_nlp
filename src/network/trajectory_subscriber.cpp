#include <itomp_nlp/network/trajectory_subscriber.h>


namespace itomp
{

TrajectorySubscriber::TrajectorySubscriber(const std::string& ip)
    : zmq_context_(1)
    , zmq_subscriber_(zmq_context_, ZMQ_SUB)
{
    std::string address = "tcp://" + ip + ":54321";
    zmq_subscriber_.connect(address.c_str());
    zmq_subscriber_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
}

Trajectory TrajectorySubscriber::receive()
{
    zmq::message_t message;

    zmq_subscriber_.recv(&message);

    std::string serial;
    char* p = (char*)message.data();
    for (int i=0; i<message.size(); i++)
        serial.push_back(*p++);

    return Trajectory(serial);
}

}
