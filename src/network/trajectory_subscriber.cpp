#include <itomp_nlp/network/trajectory_subscriber.h>

#include <zmq.h>


namespace itomp
{

TrajectorySubscriber::TrajectorySubscriber(const std::string& ip)
{
    std::string address = "tcp://" + ip + ":54321";

    zmq_context_ = zmq_ctx_new();
    zmq_subscriber_ = zmq_socket(zmq_context_, ZMQ_SUB);
    zmq_connect(zmq_subscriber_, address.c_str());
    zmq_setsockopt(zmq_subscriber_, ZMQ_SUBSCRIBE, "", 0);
}

Trajectory TrajectorySubscriber::receiveSync()
{
    zmq_msg_t message;
    zmq_msg_init(&message);

    // wait until it receives a message, if flag is zero
    zmq_recvmsg(zmq_subscriber_, &message, 0);

    std::string serial;
    const int size = zmq_msg_size(&message);
    char* p = (char*)zmq_msg_data(&message);
    for (int i=0; i<size; i++)
        serial.push_back(*p++);

    zmq_msg_close(&message);

    return Trajectory(serial);
}

}
