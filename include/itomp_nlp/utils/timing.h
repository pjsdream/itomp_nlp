#ifndef ITOMP_UTILS_TIMING_H
#define ITOMP_UTILS_TIMING_H


#include <time.h>


namespace itomp
{

namespace internal
{

static int start_time;

}

void timerStart();
void timerPrintElapsedTime();

double getWallTime();

}


#endif // ITOMP_UTILS_TIMING_H