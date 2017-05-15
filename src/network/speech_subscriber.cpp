#include <itomp_nlp/network/speech_subscriber.h>

#include <zmq.h>


namespace speech_network
{

SpeechSubscriber::SpeechSubscriber(const std::string& ip)
{
    std::string address = "tcp://" + ip + ":54322";

    zmq_context_ = zmq_ctx_new();
    zmq_subscriber_ = zmq_socket(zmq_context_, ZMQ_SUB);
    zmq_connect(zmq_subscriber_, address.c_str());
    zmq_setsockopt(zmq_subscriber_, ZMQ_SUBSCRIBE, "", 0);
}

SpeechSubscriber::~SpeechSubscriber()
{
    // TODO: destroy sockets
}

std::string SpeechSubscriber::receive()
{
    zmq_msg_t message;
    zmq_msg_init(&message);

    if (zmq_recvmsg(zmq_subscriber_, &message, ZMQ_DONTWAIT) != -1)
    {
        std::string string;
        const int size = zmq_msg_size(&message);
        char* p = (char*)zmq_msg_data(&message);

        for (int i=0; i<size; i++)
            string.push_back(*p++);

        return string;
    }

    return "";
}

}
