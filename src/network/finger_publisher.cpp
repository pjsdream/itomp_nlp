#include <itomp_nlp/network/finger_publisher.h>

#include <zmq.h>


namespace itomp
{

FingerPublisher::FingerPublisher()
{
    zmq_context_ = zmq_ctx_new();
    zmq_publisher_ = zmq_socket(zmq_context_, ZMQ_PUB);
    zmq_bind(zmq_publisher_, "tcp://*:54323");
}

void FingerPublisher::publish(double gap)
{
    char buf[256];
    sprintf(buf, "%lf ", gap);

    zmq_msg_t message;
    zmq_msg_init_size(&message, strlen(buf));
    memcpy(zmq_msg_data(&message), buf, strlen(buf));

    zmq_sendmsg(zmq_publisher_, &message, 0);
    zmq_msg_close(&message);
}

}
