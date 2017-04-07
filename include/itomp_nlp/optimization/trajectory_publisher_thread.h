#ifndef ITOMP_OPTIMIZATION_TRAJECTORY_PUBLISHER_THREAD_H
#define ITOMP_OPTIMIZATION_TRAJECTORY_PUBLISHER_THREAD_H


#include <itomp_nlp/optimization/optimizer_robot.h>
#include <itomp_nlp/optimization/cost.h>

#include <Eigen/Dense>

#include <thread>
#include <atomic>
#include <mutex>

#include <dlib/optimization.h>


namespace itomp
{

class Optimizer;

class TrajectoryPublisherThread
{
public:

    TrajectoryPublisherThread(Optimizer& optimizer);
    ~TrajectoryPublisherThread();

    void start(double timestep);
    void stop();

private:

    Optimizer& optimizer_;

    // should be called in the created thread
    void threadEnter();

    // thread
    std::thread thread_;
    std::atomic_bool thread_stop_requested_;

    // timer
    double start_time_;
    double timestep_;
};

}


#endif // ITOMP_OPTIMIZATION_TRAJECTORY_PUBLISHER_THREAD_H
