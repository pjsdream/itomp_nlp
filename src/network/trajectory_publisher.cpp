#include <itomp_nlp/network/trajectory_publisher.h>

#include <zmq.h>


namespace itomp
{

TrajectoryPublisher::TrajectoryPublisher()
{
    zmq_context_ = zmq_ctx_new();
    zmq_publisher_ = zmq_socket(zmq_context_, ZMQ_PUB);
    zmq_bind(zmq_publisher_, "tcp://*:54321");
}

void TrajectoryPublisher::publish(const Trajectory& trajectory)
{
    std::string serial = trajectory.serialize();

    zmq_msg_t message;
    zmq_msg_init_size(&message, serial.size());
    memcpy(zmq_msg_data(&message), &serial[0], serial.size());

    zmq_sendmsg(zmq_publisher_, &message, 0);
    zmq_msg_close(&message);
}

}
