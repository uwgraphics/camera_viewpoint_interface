#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <map>
#include <list>
#include <string>
#include <memory>
#include <algorithm>

#include <glm/vec2.hpp>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "layout_component.hpp"
#include "display.hpp"
#include "display_ring.hpp"
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
    TWINNED,
    GRID,
    CAROUSEL,
    TWINNED_PIP
};

enum LayoutCommand
{
    INVALID_COMMAND,
    PRIMARY_NEXT,
    PRIMARY_PREV,
    SECONDARY_NEXT,
    SECONDARY_PREV,
    TOGGLE,
    ACTIVE_FRAME_NEXT,
    ACTIVE_FRAME_PREV
};

enum class LayoutDisplayRole
{
    Primary,
    Secondary
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
    void setActiveFrame(uint index);
    const std::vector<float>& getActiveDisplayMatrix() const;
    const std::vector<float> getDisplayBounds() const;
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
        else if (input == "active_next") {
            return LayoutCommand::ACTIVE_FRAME_NEXT;
        }
        else if (input == "active_prev") {
            return LayoutCommand::ACTIVE_FRAME_PREV;
        }

        return LayoutCommand::INVALID_COMMAND;
    }

    virtual void handleControllerInput(std::string input);
    virtual void handleCollisionMessage(const std::string &message);

protected:
    DisplayManager &displays_;
    std::vector<DisplayImageRequest> display_image_queue_;
    std::vector<DisplayImageResponse> image_response_queue_;
    Scoreboard scoreboard_;

    // States
    bool grabbing_, clutching_;

    DisplayRing primary_ring_;
    DisplayRing secondary_ring_;
    std::vector<uint> primary_img_ids_; // Stores OpenGL ID for primary display images
    std::vector<uint> secondary_img_ids_; // Stores OpenGL ID for secondary display images

    std::vector<LayoutComponent> layout_components_;
    std::vector<float> display_bounds_;

    struct ColorSet
    {
        ImVec4 base, hovered, active;
    };

    ColorSet color_cache_;
    ColorSet primary_color_;
    ColorSet secondary_color_;

    const ImVec4 kOnColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    const ImVec4 kOffColor = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    const ImVec4 kActiveBorderColor = ImVec4(30.0/255, 225.0/255, 0.0f, 0.5f);

    std::vector<float> kDummyMatrix;

    Layout(LayoutType type, DisplayManager &disp) : layout_type_(type), displays_(disp), kDummyMatrix(12, 0.0)
    {
        primary_color_.base =    ImVec4{10.0/255, 190.0/255, 10.0/255, 150.0/255};
        primary_color_.hovered = ImVec4{10.0/255, 190.0/255, 10.0/255, 200.0/255}; 
        primary_color_.active =  ImVec4{10.0/255, 190.0/255, 10.0/255, 255.0/255};

        secondary_color_.base =      ImVec4{165.0/255, 100.0/255, 100.0/255, 125.0/255};
        secondary_color_.hovered =   ImVec4{165.0/255, 100.0/255, 100.0/255, 200.0/255}; 
        secondary_color_.active =    ImVec4{165.0/255, 100.0/255, 100.0/255, 255.0/255};

        // Set up identity matrix
        kDummyMatrix[0] = 1.0;
        kDummyMatrix[5] = 1.0;
        kDummyMatrix[10] = 1.0;
    }
        
    virtual void handleImageResponse();

    void enableDisplayStyle(LayoutDisplayRole role);
    void disableDisplayStyle();
    void addDisplayByIxAndRole(uint ix, LayoutDisplayRole role);
    void addPrimaryDisplayById(uint id);
    void addSecondaryDisplayById(uint id);
    void activateDisplayAtIx(uint ix);
    void deactivateDisplayAtIx(uint ix);
    void toNextDisplay(LayoutDisplayRole role);
    void toPrevDisplay(LayoutDisplayRole role);
    void addImageRequestToQueue(DisplayImageRequest request);
    void addLayoutComponent(LayoutComponent::Type type, LayoutComponent::Spacing spacing=LayoutComponent::Spacing::Auto,
        LayoutComponent::Positioning positioning=LayoutComponent::ComponentPositioning_Auto, float width=0.0,
        float height=0.0, ImVec2 offset=ImVec2{-1.0, -1.0});
    void drawLayoutComponents();
    void displayStateValues(std::map<std::string, bool> states) const;
    void drawDisplaysList(uint display_limit=0);
    void drawDisplaySelector(uint num, std::string title="", LayoutDisplayRole role=LayoutDisplayRole::Primary);
    void drawDraggableRing();

private:
    static const uint kNumLayoutTypes = 9;
    const std::vector<std::string> kLayoutNames = {
        "Dynamic Camera", "Wide Angle", "Picture-in-Picture", "Timed Pic-in-Pic", "Split Screen",
        "Twinned", "Grid", "Carousel", "Twinned Pic-in-Pic"
    };

    LayoutType layout_type_;

    DisplayRing& getDisplaysVectorFromRole(LayoutDisplayRole role);

    friend class LayoutComponent;
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