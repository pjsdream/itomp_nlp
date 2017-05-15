#include <itomp_nlp/network/finger_subscriber.h>

#include <zmq.h>


namespace itomp
{

FingerSubscriber::FingerSubscriber(const std::string& ip)
{
    std::string address = "tcp://" + ip + ":54323";

    zmq_context_ = zmq_ctx_new();
    zmq_subscriber_ = zmq_socket(zmq_context_, ZMQ_SUB);
    zmq_connect(zmq_subscriber_, address.c_str());
    zmq_setsockopt(zmq_subscriber_, ZMQ_SUBSCRIBE, "", 0);
}

double FingerSubscriber::receiveAsync()
{
    zmq_msg_t message;
    zmq_msg_init(&message);

    if (zmq_recvmsg(zmq_subscriber_, &message, ZMQ_DONTWAIT) == -1)
        return -1.;

    std::string s;
    int size =zmq_msg_size(&message);
    char* p = (char*)zmq_msg_data(&message);
    for (int i=0; i<size; i++)
        s += *(p++);
    s += '\0';

    double gap = -1.;
    sscanf(s.c_str(), "%lf", &gap);

    return gap;
}

}
