#include <itomp_nlp/utils/timing.h>

#include <stdio.h>


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

}
