#ifndef __SCOREBOARD_HPP__
#define __SCOREBOARD_HPP__

#include <string>
#include <vector>

#include <imgui/imgui.h>

#include "timer.hpp"


class Scoreboard
{
public:
    Scoreboard(int init_score=0) : score_(init_score) {}

    void addEventMessage(std::string message, int score_change=0, int timeout_secs=3);
    void draw();

private:
    struct ScoreMessage
    {
        std::string message_;
        int score_change_;
        CountdownTimer timer_;
    };

    const ImVec4 kIncrColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    const ImVec4 kDecrColor = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);

    int score_;
    std::vector<ScoreMessage> score_messages_;

    void checkMessageExpiration();
};

#endif //__SCOREBOARD_HPP__