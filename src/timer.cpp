#include "viewpoint_interface/timer.hpp"

// --- Timer ---
void Timer::init()
{
    start_ = std::chrono::high_resolution_clock::now();
    initialized_ = true;
}

void Timer::reset()
{
    init();
}

int64_t Timer::getDuration() const
{
    switch (type_)
    {
    case DurationType::NANOSECONDS:
        {
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(current_ - start_);
            return duration.count();
        }   break;

    case DurationType::MICROSECONDS:
        {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_ - start_);
            return duration.count();            
        }   break;

    case DurationType::MILLISECONDS:
        {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_ - start_);
            return duration.count();            
        }   break;
        
    case DurationType::SECONDS:
        {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_ - start_);
            return duration.count();            
        }   break;
    case DurationType::MINUTES:
        {
            auto duration = std::chrono::duration_cast<std::chrono::minutes>(current_ - start_);
            return duration.count();            
        }   break;
    }

    return 0;
}


// --- CountdownTimer ---
void CountdownTimer::reset()
{
    init();
    expired_ = false;
    acknowledged_ = false;
}

bool CountdownTimer::timerExpired()
{
    if (!initialized_) { return false; }
    if (expired_) { return true; }

    current_ = std::chrono::high_resolution_clock::now();
    int64_t duration = getDuration();

    if (duration >= time_) {
        expired_ = true;
    }

    return expired_;
}

bool CountdownTimer::acknowledgeExpiration()
{
    if (!expired_) { return false; }
    if (acknowledged_) { return false; } // Can't acknowledge twice

    acknowledged_ = true;
    return true;
}

int64_t CountdownTimer::getTimeRemaining() {
    if (!initialized_ || expired_) { return 0; }

    current_ = std::chrono::high_resolution_clock::now();
    int64_t duration = getDuration();

    return (int64_t)time_ - duration;
}


// --- Stopwatch ---
int64_t Stopwatch::getRunningTime()
{
    if (!initialized_) { return 0; }

    current_ = std::chrono::high_resolution_clock::now();

    return getDuration(); 
}
