#include "TimeMeasure.h"

void TimeMeasure::CaptureStart()
{
    startPoint = std::chrono::steady_clock::now();
}

void TimeMeasure::CaptureEnd()
{
    endPoint = std::chrono::steady_clock::now();
}

double TimeMeasure::GetTimeInSeconds()
{
    auto s = std::chrono::duration_cast<std::chrono::seconds>(endPoint - startPoint).count();
    return s;
}

double TimeMeasure::GetTimeInMilliseconds()
{
    auto s = std::chrono::duration_cast<std::chrono::milliseconds>(endPoint - startPoint).count();
    return s;
}

double TimeMeasure::GetTimeInNanoseconds()
{
    auto s = std::chrono::duration_cast<std::chrono::nanoseconds>(endPoint - startPoint).count();
    return s;
}
