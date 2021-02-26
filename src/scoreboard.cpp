#include "viewpoint_interface/scoreboard.hpp"

// ImGui menu management functions
static bool startMenu(std::string title, ImGuiWindowFlags window_flags)
{
    if (!ImGui::Begin(title.c_str(), (bool *)NULL, window_flags)) {
        ImGui::End();
        return false;    
    }

    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.35f);
    return true;
}

static void endMenu()
{
    ImGui::PopItemWidth();
    ImGui::End();
}

// Scoreboard functions
void Scoreboard::checkMessageExpiration()
{
    std::vector<ScoreMessage>::iterator message(score_messages_.begin());
    for ( ; message != score_messages_.end(); ) {
        if (message->timer_.timerExpired()) {
            score_messages_.erase(message);
        }
        else {
            ++message;
        }
    }
}

void Scoreboard::addEventMessage(std::string message, int score_change, int timeout_secs)
{
    score_ += score_change;

    ScoreMessage new_message {message, score_change, CountdownTimer(timeout_secs) };
    score_messages_.push_back(new_message);
}

void Scoreboard::draw()
{
    checkMessageExpiration();

    std::string title = "Score";
    ImGuiWindowFlags win_flags = 0;
    win_flags |= ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs;
    win_flags |= ImGuiWindowFlags_NoBackground;
    win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
    ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + (main_viewport->GetWorkSize().x / 2.0) - 25, 
            main_viewport->GetWorkPos().y + 20), ImGuiCond_Always);

    if (startMenu(title, win_flags)) {
        ImGui::Text("Score: %d", score_);
        for (ScoreMessage message : score_messages_) {
            ImGui::Text("%s - ", message.message_.c_str());
            ImGui::SameLine();
            ImVec4 score_color(message.score_change_ >= 0 ? kIncrColor : kDecrColor);
            ImGui::TextColored(score_color, std::to_string(message.score_change_).c_str());
        }
        endMenu();
    }
}
