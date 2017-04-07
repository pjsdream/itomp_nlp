#include <itomp_nlp/optimization/trajectory_publisher_thread.h>

#include <itomp_nlp/utils/timing.h>


namespace itomp
{

TrajectoryPublisherThread::TrajectoryPublisherThread(Optimizer& optimizer)
    : optimizer_(optimizer)
{
}

TrajectoryPublisherThread::~TrajectoryPublisherThread()
{
}

void TrajectoryPublisherThread::start(double timestep)
{
    timestep_ = timestep;

    thread_stop_requested_ = false;

    start_time_ = getWallTime();
    thread_ = std::thread( std::bind(&TrajectoryPublisherThread::threadEnter, this) );
}

void TrajectoryPublisherThread::stop()
{
    thread_stop_requested_ = true;

    thread_.join();
}

void TrajectoryPublisherThread::threadEnter()
{
    // main loop
    while (!thread_stop_requested_)
    {
        double elapsed_time = getWallTime() - start_time_;
    }
}

}
