#ifndef ITOMP_OPTIMIZATION_TRAJECTORY_PUBLISHER_H
#define ITOMP_OPTIMIZATION_TRAJECTORY_PUBLISHER_H


#include <WinSock2.h>
#include <zmq.hpp>

#include <itomp_nlp/optimization/trajectory.h>


namespace itomp
{

class TrajectoryPublisher
{
public:

    TrajectoryPublisher();

    void publish(const Trajectory& trajectory);

protected:

    zmq::context_t zmq_context_;
    zmq::socket_t zmq_publisher_;
};

}


#endif // ITOMP_OPTIMIZATION_TRAJECTORY_PUBLISHER_H