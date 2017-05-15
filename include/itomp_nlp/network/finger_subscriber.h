#ifndef ITOMP_OPTIMIZATION_FINGER_SUBSCRIBER_H
#define ITOMP_OPTIMIZATION_FINGER_SUBSCRIBER_H


#ifdef _WIN32
#include <WinSock2.h>
#endif

#include <string>


namespace itomp
{

class FingerSubscriber
{
public:

    FingerSubscriber(const std::string& ip);

    double receiveAsync();

protected:

    void* zmq_context_;
    void* zmq_subscriber_;
};

}


#endif // ITOMP_OPTIMIZATION_FINGER_SUBSCRIBER_H
