#ifndef ITOMP_OPTIMIZATION_TRAJECTORY_SUBSCRIBER_H
#define ITOMP_OPTIMIZATION_TRAJECTORY_SUBSCRIBER_H


#include <WinSock2.h>
#include <zmq.hpp>

#include <itomp_nlp/optimization/trajectory.h>


namespace itomp
{

class TrajectorySubscriber
{
public:

    TrajectorySubscriber(const std::string& ip);

    Trajectory receive();

protected:

    zmq::context_t zmq_context_;
    zmq::socket_t zmq_subscriber_;
};

}


#endif // ITOMP_OPTIMIZATION_TRAJECTORY_SUBSCRIBER_H