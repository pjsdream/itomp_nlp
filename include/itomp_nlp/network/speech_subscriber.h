#ifndef SPEECH_NETWORK_SPEECH_SUBSCRIBER_H
#define SPEECH_NETWORK_SPEECH_SUBSCRIBER_H


#include <string>


namespace speech_network
{

class SpeechSubscriber
{
public:

    SpeechSubscriber(const std::string& ip);
    ~SpeechSubscriber();

    std::string receive();

private:

    void* zmq_context_;
    void* zmq_subscriber_;
};

}


#endif // SPEECH_NETWORK_SPEECH_SUBSCRIBER_H
