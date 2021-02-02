#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <chrono>


class Timer
{
public:
    enum class DurationType
    {
        NANOSECONDS,
        MICROSECONDS,
        MILLISECONDS,
        SECONDS,
        MINUTES
    };

    inline bool isInitialized() const { return initialized_; }

    void init();
    virtual void reset();

protected:
    int time_;
    bool initialized_;
    DurationType type_;
    std::chrono::_V2::system_clock::time_point start_;
    std::chrono::_V2::system_clock::time_point current_;

    Timer(int time, DurationType type) : time_(time), type_(type), initialized_(false) {}

    int64_t getDuration() const;
};

class CountdownTimer : public Timer
{
public:
    CountdownTimer(int time, DurationType type) : Timer(time, type), expired_(false),
        acknowledged_(false) {}

    void reset() override;
    bool timerExpired();
    bool acknowledgeExpiration();
    int64_t getTimeRemaining();

private:
    bool expired_, acknowledged_;
};

class Stopwatch : public Timer
{
public:
    Stopwatch(DurationType type=DurationType::MILLISECONDS) : Timer(0, type) {}

    int64_t getRunningTime();
};

#endif // __TIMER_HPP__