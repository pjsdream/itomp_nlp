#ifndef ITOMP_OPTIMIZATION_FINGER_PUBLISHER_H
#define ITOMP_OPTIMIZATION_FINGER_PUBLISHER_H


#ifdef _WIN32
#include <WinSock2.h>
#endif

#include <itomp_nlp/optimization/trajectory.h>


namespace itomp
{

class FingerPublisher
{
public:

    FingerPublisher();

    void publish(double gap);

protected:

    void* zmq_context_;
    void* zmq_publisher_;
};

}


#endif // ITOMP_OPTIMIZATION_FINGER_PUBLISHER_H
