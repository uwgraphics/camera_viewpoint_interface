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

#include "display.hpp"
#include "layout_component.hpp"
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
// - Edit num_layout_types and layout_names in Layout class (making sure that
// the ordering of the layout names matches the order in the LayoutType enum)
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
    TWINNED_PIP,
    DOUBLE_PIP,
    SPLIT,
    TWINNED,
    GRID,
    CAROUSEL
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
    ACTIVE_FRAME_PREV,
    ACTIVE_FRAME_UP,
    ACTIVE_FRAME_DOWN,
    ACTIVE_FRAME_LEFT,
    ACTIVE_FRAME_RIGHT    
};

enum class LayoutDisplayRole
{
    Primary,
    Secondary
};


struct DisplayImageRequest
{
public:
    DisplayImageRequest(uint w, uint h, std::vector<uchar> &data, uint id) :
            width_(w), height_(h), data_(data), disp_id_(id) {}

    uint getWidth() const { return width_; }
    uint getHeight() const { return height_; }
    uint getDisplayId() const { return disp_id_; }
    std::vector<uchar>& getDataVector() { return data_; }

private:
    std::vector<uchar> &data_;
    uint width_, height_, disp_id_;
};

struct DisplayImageResponse
{
public:
    DisplayImageResponse(uint gl_id, uint id) : gl_id_(gl_id), disp_id_(id) {}

    uint getGLId() const { return gl_id_; }
    uint getDisplayId() const { return disp_id_; }

private:
    uint gl_id_, disp_id_;
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
    const LayoutCommand translateStringInputToCommand(std::string input) const;
    virtual void handleStringInput(std::string input);
    virtual void handleCollisionMessage(const std::string &message);

protected:
    DisplayManager &displays_;
    std::vector<DisplayImageRequest> display_image_queue_;
    std::vector<DisplayImageResponse> image_response_queue_;
    Scoreboard scoreboard_;
    
    class DisplayStateCache
    {
    public:
        void touchDisplay(uint id);
        void reverseCache();
        void printCache();
        
        uint getFront() const { return cache_.front(); }
        uint getBack() const { return cache_.back(); }
        auto loopStart() const { return cache_.begin(); }
        auto loopEnd() const { return cache_.end(); }

    private:
        std::list<uint> cache_;
    };

    class DisplayRing
    {
    public:
        DisplayRing();

        auto loopStart() const { return ring_.begin(); }
        auto loopEnd() const { return ring_.end(); }

        void pushDisplay(uint id);
        void removeDisplay(uint id);
        void swapDisplays(uint id, float delta);
        void toNextDisplay(LayoutDisplayRole role);
        void toPrevDisplay(LayoutDisplayRole role);
        void setDisplayRole(uint id, LayoutDisplayRole role);
        void unsetDisplayRole(uint id, LayoutDisplayRole role);
        void unsetRoles(uint id);
        void unsetAllForRole(LayoutDisplayRole role);
        bool isPrimaryDisplay(uint id) const;
        bool isSecondaryDisplay(uint id) const;
        bool isDisplayRole(uint id, LayoutDisplayRole role) const;
        std::vector<uint> getDisplayRoleList(LayoutDisplayRole role);
        uint getNumPrimaryDisplays() const;
        uint getNumSecondaryDisplays() const;
        uint getNumForRole(LayoutDisplayRole role) const;
        uint getDisplayIdByIx(uint ix) const;
        void setActiveFrameByIndex(uint ix);
        void setActiveFrameById(uint id);
        uint getActiveFrameIndex() const;
        uint getActiveFrameDisplayId() const;
        void toNextActiveFrame();
        void toPrevActiveFrame();
        void addImageResponseForId(uint display_id, uint gl_id);
        uint getImageIdForDisplayId(uint id) const;

    private:
        std::vector<uint> ring_;
        uint active_frame_; // Active frame points to an index position within ring_
        std::map<uint, uint> gl_ids_; // Stores OpenGL ID for displays in ring
        std::map<uint, bool> primary_displays_;
        std::map<uint, bool> secondary_displays_;

        void setPrimaryDisplay(uint id);
        void setSecondaryDisplay(uint id);
        void unsetPrimaryDisplay(uint id);
        void unsetSecondaryDisplay(uint id);
        uint getNextIdWithoutRole(uint start_id, LayoutDisplayRole role);
        uint getPrevIdWithoutRole(uint start_id, LayoutDisplayRole role);
        uint getIndexForDisplayId(uint id);
    };

    class LayoutDisplayStates
    {
    public:
        LayoutDisplayStates() = delete;
        LayoutDisplayStates(DisplayManager &displays);

        uint size() const;
        bool empty() const;
        uint getNumActiveDisplays() const;
        bool noDisplaysActive() const;
        uint setActiveLimit(uint limit);
        uint setNumDisplaysForRole(int num, LayoutDisplayRole role);
        int getPrimaryLimitNum() const { return num_primary_; }
        int getSecondaryLimitNum() const { return num_secondary_; }
        std::map<uint, bool>::const_iterator loopStart() const;
        std::map<uint, bool>::const_iterator loopEnd() const;
        bool isDisplayActive(uint id);
        uint getOldestActiveDisplay();
        void activateDisplay(uint id);
        void deactivateDisplay(uint id);
        uint getDisplayIxById(uint id);
        bool isDisplayIxActive(uint ix);
        uint getNextActiveDisplayIx(int ix);
        uint getPrevActiveDisplayIx(int ix);
        uint getCacheFront();
        uint getCacheBack();

        // Ring functions
        DisplayRing& getDisplayRing();
        void setDisplayRole(uint id, LayoutDisplayRole role);
        void toNextDisplay(LayoutDisplayRole role);
        void toPrevDisplay(LayoutDisplayRole role);
        void setActiveFrameByIndex(uint ix);
        void setActiveFrameById(uint id);
        uint getActiveFrameDisplayId() const;
        void toNextActiveFrame();
        void toPrevActiveFrame();
        void handleActiveFrameDirectionInput(LayoutCommand command);
        void addImageResponseForId(uint display_id, uint gl_id);
        uint getImageIdForDisplayId(uint id) const;

    private:
        std::map<uint, bool> states_;
        uint num_active_displays_;
        uint active_limit_;
        int num_primary_;
        int num_secondary_;
        DisplayStateCache display_cache_;
        DisplayRing display_ring_;

        uint nextIx(uint ix, uint size) const;
        uint prevIx(uint ix, uint size) const;
    };

    DisplayStateCache cache;
    LayoutDisplayStates display_states_;
    // DisplayRing primary_ring_;
    // DisplayRing secondary_ring_;

    // Robot state data
    bool grabbing_, clutching_;


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

    Layout(LayoutType type, DisplayManager &disp) : layout_type_(type), displays_(disp), kDummyMatrix(12, 0.0),
            display_states_(displays_)
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

        clutching_ = false; grabbing_ = false;
    }
        
    virtual void handleImageResponse();

    void enableDisplayStyle(LayoutDisplayRole role);
    void disableDisplayStyle();
    void setNumDisplaysForRole(int num, LayoutDisplayRole role);
    void addDisplayByIxAndRole(uint ix, LayoutDisplayRole role);
    void addPrimaryDisplayById(uint id);
    void addSecondaryDisplayById(uint id);
    void activateDisplayAtIx(uint ix);
    void deactivateDisplayAtIx(uint ix);
    void toNextDisplay(LayoutDisplayRole role);
    void toPrevDisplay(LayoutDisplayRole role);
    void toNextDisplayWithPush(LayoutDisplayRole role);
    void toPrevDisplayWithPush(LayoutDisplayRole role);
    void addImageRequestToQueue(DisplayImageRequest request);
    void addLayoutComponent(LayoutComponent::Type type, LayoutComponent::Spacing spacing=LayoutComponent::Spacing::Auto,
        LayoutComponent::Positioning positioning=LayoutComponent::ComponentPositioning_Auto, float width=0.0,
        float height=0.0, ImVec2 offset=ImVec2{-1.0, -1.0});
    void drawLayoutComponents();
    void displayStateValues(std::map<std::string, bool> states) const;
    void drawDisplaysList(uint keep_active_num=0);
    void drawDisplaySelectors();
    void drawDraggableRing();

private:
    static const uint kNumLayoutTypes = 10;
    const std::vector<std::string> kLayoutNames = {
        "Dynamic Camera", "Wide Angle", "Picture-in-Picture", "Timed Pic-in-Pic", "Twinned Pic-in-Pic",
        "Double Pic-in-Pic", "Split Screen", "Twinned", "Grid", "Carousel"
    };

    LayoutType layout_type_;

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