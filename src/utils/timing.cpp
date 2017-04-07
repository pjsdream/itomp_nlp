#include <itomp_nlp/utils/timing.h>

#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <time.h>
#include <sys/time.h>
#endif

namespace itomp
{

void timerStart()
{
    internal::start_time = clock();
}

void timerPrintElapsedTime()
{
    const double t = (double)(clock() - internal::start_time) / CLOCKS_PER_SEC;

    printf("elapsed time: %lf\n", t);

    timerStart();
}

//  Windows
#ifdef _WIN32
double getWallTime()
{
    LARGE_INTEGER time,freq;
    if (!QueryPerformanceFrequency(&freq))
    {
        //  Handle error
        return 0;
    }
    if (!QueryPerformanceCounter(&time))
    {
        //  Handle error
        return 0;
    }
    return (double)time.QuadPart / freq.QuadPart;
}
//  Posix/Linux
#else
double getWallTime()
{
    struct timeval time;
    if (gettimeofday(&time,NULL))
    {
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}
#endif

}
