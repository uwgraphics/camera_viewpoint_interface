#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <map>
#include <string>
#include <memory>
#include <algorithm>

#include <glm/vec2.hpp>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "display.hpp"
#include "timer.hpp"
#include "scoreboard.hpp"


namespace viewpoint_interface
{

class LayoutManager;

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

template <typename T>
int getItemIndexInVector(T item, std::vector<T> &vec)
{
    auto iter = std::find(vec.begin(), vec.end(), item);
    if(iter != vec.end()) {
        return iter - vec.begin();
    }

    return -1;
}

template <typename T>
int getItemIndexInVector(T item, const std::vector<T> &vec)
{
    auto iter = std::find(vec.begin(), vec.end(), item);
    if(iter != vec.end()) {
        return iter - vec.begin();
    }

    return -1;
}

template <typename T>
bool isItemInVector(T item, std::vector<T> &vec)
{
    return getItemIndexInVector(item, vec) != -1;
}

template <typename T>
bool isItemInVector(T item, const std::vector<T> &vec)
{
    return getItemIndexInVector(item, vec) != -1;
}


// To add/change layouts follow the following steps:
// - Add/change enum entry in LayoutType for layout
// - Edit num_layout_types and layout_names in Layout class
// - Create/change subclass inheriting from Layout class
// - Implement virtual functions for layout, if necessary
// - Add/change entry for layout in newLayout() function of LayoutManager
enum LayoutType
{
    INACTIVE = -1,
    DYNAMIC,
    WIDE,
    PIP,
    TIMED_PIP,
    SPLIT,
    TWINNED
};

enum LayoutCommand
{
    INVALID_COMMAND,
    PRIMARY_NEXT,
    PRIMARY_PREV,
    SECONDARY_NEXT,
    SECONDARY_PREV,
    TOGGLE
};

enum class LayoutDisplayRole
{
    Primary,
    Secondary
};

enum FrameMode
{
    CAMERA_FRAME = 0,
    WORLD_FRAME
};

struct DisplayImageRequest
{
    std::vector<uchar> &data;
    uint width, height;
    LayoutDisplayRole role;
    uint index;

    DisplayImageRequest(uint w, uint h, std::vector<uchar> &vec, uint ix, LayoutDisplayRole rl) :
            width(w), height(h), data(vec), index(ix), role(rl) {}
};

struct DisplayImageResponse
{
    LayoutDisplayRole role;
    uint id;
    uint index;

    DisplayImageResponse(uint gl_id, uint ix, LayoutDisplayRole rl) : id(gl_id), index(ix),
            role(rl) {}
};

    
class Layout
{
public:
    LayoutType getLayoutType() const { return layout_type_; } 
    const std::vector<std::string> getLayoutList() const { return kLayoutNames; }
    const std::string getLayoutName() const;

    // TODO: Make these private and make LayoutManager friend class
    static LayoutType intToLayoutType(int ix) {
        if (ix >= 0 && ix < kNumLayoutTypes) {
            return (LayoutType)ix;
        }

        return LayoutType::INACTIVE;
    }

    void setGrabbingState(bool state) { grabbing_ = state; }
    void setClutchingState(bool state) { clutching_ = state; }
    FrameMode getFrameMode() { return frame_mode_; }
    std::vector<DisplayImageRequest>& getImageRequestQueue();
    void pushImageResponse(const DisplayImageResponse &response);

    virtual void displayLayoutParams() = 0;
    virtual void draw() = 0;

    virtual void handleKeyInput(int key, int action, int mods);

    /**
     * This function is intended to serve as a universal translation table
     * for all the layouts. Anytime that a new command is desired for a
     * layout, a new entry should be added here and in the LayoutCommand
     * enum. This makes it easier to manage all the commands available
     * across layouts--preventing duplicates, serving as a reference, and
     * providing a single point to add/change commands.
     * 
     * Params:
     *      input - command string
     * 
     * Returns: command represented by input string. 
     */
    const LayoutCommand translateControllerInputToCommand(std::string input) const
    {
        if (input == "primary_next") {
            return LayoutCommand::PRIMARY_NEXT;
        }
        else if (input == "primary_prev") {
            return LayoutCommand::PRIMARY_PREV;
        }
        else if (input == "pip_next") {
            return LayoutCommand::SECONDARY_NEXT;
        }
        else if (input == "pip_prev") {
            return LayoutCommand::SECONDARY_PREV;
        }
        else if (input == "toggle") {
            return LayoutCommand::TOGGLE;
        }

        return LayoutCommand::INVALID_COMMAND;
    }

    virtual void handleControllerInput(std::string input);
    virtual void handleCollisionMessage(const std::string &message);

protected:
    DisplayManager &displays_;
    std::vector<DisplayImageRequest> display_image_queue_;
    std::vector<DisplayImageResponse> image_response_queue_;

    bool grabbing_, clutching_;
    FrameMode frame_mode_;
    Scoreboard scoreboard_;

    std::vector<uint> primary_displays_; // Stores display ID
    std::vector<uint> secondary_displays_; // Stores display ID
    std::vector<uint> prim_img_ids_; // Stores OpenGL ID for primary images

    struct ColorSet
    {
        ImVec4 base, hovered, active;
    };

    ColorSet color_cache_;
    ColorSet primary_color_;
    ColorSet secondary_color_;

    const ImVec4 kOnColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    const ImVec4 kOffColor = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);

    Layout(LayoutType type, DisplayManager &disp) : layout_type_(type), displays_(disp),
            frame_mode_(FrameMode::CAMERA_FRAME)
    {
        primary_color_.base =    ImVec4{10.0/255, 190.0/255, 10.0/255, 150.0/255};
        primary_color_.hovered = ImVec4{10.0/255, 190.0/255, 10.0/255, 200.0/255}; 
        primary_color_.active =  ImVec4{10.0/255, 190.0/255, 10.0/255, 255.0/255};

        secondary_color_.base =      ImVec4{165.0/255, 100.0/255, 100.0/255, 125.0/255};
        secondary_color_.hovered =   ImVec4{165.0/255, 100.0/255, 100.0/255, 200.0/255}; 
        secondary_color_.active =    ImVec4{165.0/255, 100.0/255, 100.0/255, 255.0/255};
    }
        
    virtual void handleImageResponse() = 0;

    void enableDisplayStyle(LayoutDisplayRole role);
    void disableDisplayStyle();
    void addPrimaryDisplayByIx(uint ix);
    void addPrimaryDisplayById(uint id);
    void toNextDisplay(uint vec_ix, LayoutDisplayRole role);
    void toPrevDisplay(uint vec_ix, LayoutDisplayRole role);
    void addImageRequestToQueue(DisplayImageRequest request);
    void displayInstructionsWindow(std::string text) const;
    void displayStateValues(std::map<std::string, bool> states) const;
    void drawDisplaysList();
    void drawDisplaySelector(uint num, std::string title="", LayoutDisplayRole role=LayoutDisplayRole::Primary);
    void drawDraggableRing();
    void displayPrimaryWindows() const;
    void displayPiPWindow(int width, int height, uint pip_id) const;
    void drawCarouselMenu() const;

private:
    static const uint kNumLayoutTypes = 6;
    const std::vector<std::string> kLayoutNames = {
        "Dynamic Camera", "Wide Angle", "Picture-in-Picture", "Timed Pic-in-Pic", "Split Screen", "Twinned"
    };

    LayoutType layout_type_;

    std::vector<uint> &getDisplaysVectorFromRole(LayoutDisplayRole role);
};
    

class InactiveLayout : public Layout
{
public:
    InactiveLayout(DisplayManager &displays) : Layout(LayoutType::INACTIVE, displays) {}

    virtual void displayLayoutParams() override
    {
        ImGui::Text("No layouts selected:");
        ImGui::BulletText("Please select a layout from the menu\n"
                        "at the top of the window.");
    }

    virtual void draw() override {}

    virtual void handleImageResponse() override {}
};

} // viewpoint_interface

#endif // __LAYOUT_HPP__