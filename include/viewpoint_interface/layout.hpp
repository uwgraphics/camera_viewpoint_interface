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
// - Implement virtual classes for layout, if necessary
// - Add/change entry for layout in newLayout() function of LayoutManager
enum LayoutType
{
    INACTIVE = -1,
    PIP,
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
    PIP_TOGGLE
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
        else if (input == "pip_toggle") {
            return LayoutCommand::PIP_TOGGLE;
        }

        return LayoutCommand::INVALID_COMMAND;
    }

    virtual void handleControllerInput(std::string input);

protected:
    DisplayManager &displays_;
    std::vector<DisplayImageRequest> display_image_queue_;
    std::vector<DisplayImageResponse> image_response_queue_;

    bool grabbing_, clutching_;

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

    Layout(LayoutType type, DisplayManager &disp) : layout_type_(type), displays_(disp)
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
    static const uint kNumLayoutTypes = 3;
    const std::vector<std::string> kLayoutNames = {
        "Picture-in-Picture", "Split Screen", "Twinned"
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


struct PiPParams
{
    uint start_primary_display = 0;
    uint start_pip_display = 1;
    uint max_num_displays = 3;

    int pip_window_dims[2] = { 400, 225 };
    int pip_aspect_ratio[2] = { 16, 9 };
};

class PiPLayout : public Layout
{
public:
    PiPLayout(DisplayManager &displays, PiPParams params=PiPParams()) : Layout(LayoutType::PIP, displays),
            parameters_(params), keep_aspect_ratio_(true), pip_enabled_(true), 
            countdown_(5, Timer::DurationType::SECONDS)
    {
        if (parameters_.max_num_displays > displays.size() || parameters_.max_num_displays == 0) {
            parameters_.max_num_displays = displays.size();
        }

        addPrimaryDisplayByIx(parameters_.start_primary_display);
        secondary_displays_.push_back(displays.getDisplayId(parameters_.start_pip_display));
    }

    virtual void displayLayoutParams() override
    {
        static uint cur_num_displays = parameters_.max_num_displays;

        drawDisplaysList();

        ImGui::SliderInt("# Displays", (int *) &cur_num_displays, 1, parameters_.max_num_displays);

        drawDraggableRing();

        drawDisplaySelector(0, "Main Display", LayoutDisplayRole::Primary);
        drawDisplaySelector(0, "Pic-in-Pic Display", LayoutDisplayRole::Secondary);

        ImGui::Separator();

        ImGui::Text("Time to Hide: %is", countdown_.getTimeRemaining());

        ImGui::Text("Picture-in-Picture Settings:\n");
        uint max_size = 600;
        int start_pip_dims[2] = { parameters_.pip_window_dims[0], parameters_.pip_window_dims[1] };
        bool pip_dims_changed = ImGui::DragInt2("Dimensions", parameters_.pip_window_dims, 1.0f, 100, max_size);
        bool ar_changed = ImGui::Checkbox("Keep Aspect Ratio", &keep_aspect_ratio_);

        if (keep_aspect_ratio_) {
            ImGui::Text("Aspect Ratio");
            ImGui::SameLine();
            ar_changed = ImGui::InputInt2("##PiP AR", parameters_.pip_aspect_ratio) || ar_changed;

            if (pip_dims_changed || ar_changed) {
                float aspect_ratio = (float)parameters_.pip_aspect_ratio[0] / parameters_.pip_aspect_ratio[1];
                
                // Clamp both axes to max_size
                int max_width(max_size), max_height(max_size);
                if (aspect_ratio > 1.0) {
                    // Width is largest
                    max_height = max_size * (1.0 / aspect_ratio);
                }
                else {
                    // Height is largest or same
                    max_width = max_size * aspect_ratio;
                }

                if (parameters_.pip_window_dims[0] > max_width || 
                        parameters_.pip_window_dims[1] > max_height) {
                    parameters_.pip_window_dims[0] = max_width;
                    parameters_.pip_window_dims[1] = max_height;
                }
                else {
                    if (parameters_.pip_window_dims[1]-start_pip_dims[1] != 0) {
                        // Height changed
                        parameters_.pip_window_dims[0] = parameters_.pip_window_dims[1] * aspect_ratio;
                    }
                    else {
                        // Width changed
                        aspect_ratio = 1.0 / aspect_ratio;
                        parameters_.pip_window_dims[1] = parameters_.pip_window_dims[0] * aspect_ratio;
                    }
                }
            }
        }
    }

    virtual void draw() override
    {
        handleImageResponse();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);

        // We only have one primary and one Pic-in-pic display
        displayPrimaryWindows();

        std::vector<uchar> &prim_data = displays_.getDisplayDataById(primary_displays_.at(0));
        const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(0)));
        addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                prim_data, 0, LayoutDisplayRole::Primary});

        if (countdown_.timerExpired() && countdown_.acknowledgeExpiration()) {
            pip_enabled_ = false;
        }
        else if (!countdown_.timerExpired() && countdown_.isInitialized()) {
            pip_enabled_ = true;
        }

        if (pip_enabled_) {
            displayPiPWindow(parameters_.pip_window_dims[0], parameters_.pip_window_dims[1], pip_id_);

            std::vector<uchar> &sec_data = displays_.getDisplayDataById(secondary_displays_[0]);
            const DisplayInfo &sec_info(displays_.getDisplayInfoById(primary_displays_.at(0)));
            addImageRequestToQueue(DisplayImageRequest{sec_info.dimensions.width, sec_info.dimensions.height, 
                    sec_data, 0, LayoutDisplayRole::Secondary});
        }
    }

    virtual void handleImageResponse() override
    {
        for (int i = 0; i < image_response_queue_.size(); i++) {
            DisplayImageResponse &response(image_response_queue_.at(i));

            switch (response.role)
            {
                case LayoutDisplayRole::Primary:
                {
                    prim_img_ids_[response.index] = response.id;
                }   break;

                case LayoutDisplayRole::Secondary:
                {
                    pip_id_ = response.id;
                }   break;
            }
        }
    }

    virtual void handleKeyInput(int key, int action, int mods) override
    {
        if (action == GLFW_PRESS) {
            switch (key) {
                case GLFW_KEY_P:
                {
                    pip_enabled_ = !pip_enabled_;
                } break;

                case GLFW_KEY_S:
                {
                    countdown_.reset();
                }   break;

                case GLFW_KEY_RIGHT:
                {
                    toNextDisplay(0, LayoutDisplayRole::Primary);
                } break;

                case GLFW_KEY_LEFT:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Primary);
                } break;

                case GLFW_KEY_UP:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Secondary);
                } break;

                case GLFW_KEY_DOWN:
                {
                    toNextDisplay(0, LayoutDisplayRole::Secondary);
                } break;

                default:
                {
                } break;
            }
        }
    }

    virtual void handleControllerInput(std::string input) override
    {
        LayoutCommand command(translateControllerInputToCommand(input));

        switch(command)
        {
            case LayoutCommand::PIP_TOGGLE:
            {
                pip_enabled_ = !pip_enabled_;
            }   break;

            default:
            {}  break;
        }
    }


private:
    PiPParams parameters_;
    
    uint pip_id_;
    bool keep_aspect_ratio_, pip_enabled_;
    CountdownTimer countdown_;
};


struct SplitParams
{
    uint first_primary_display = 0;
    uint second_primary_display = 1;
};

class SplitLayout : public Layout
{
public:

    SplitLayout(DisplayManager &displays, SplitParams params=SplitParams()) : Layout(LayoutType::SPLIT, displays),
            parameters_(params) 
    {
        addPrimaryDisplayByIx(parameters_.first_primary_display);
        addPrimaryDisplayByIx(parameters_.second_primary_display);
    }

    virtual void displayLayoutParams() override
    {
        // static uint num_displays = parameters.start_num_displays;
        // static uint max_bound = num_displays < parameters.max_displays ? num_displays : parameters.max_displays;

        drawDisplaysList(); 

        // ImGui::SliderInt("# Displays", (int *) &num_displays, 1, max_bound);

        drawDraggableRing();

        drawDisplaySelector(0, "Left Display", LayoutDisplayRole::Primary);
        drawDisplaySelector(1, "Right Display", LayoutDisplayRole::Primary);
    }

    virtual void draw() override
    {
        handleImageResponse();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);

        displayPrimaryWindows();

        for (int i = 0; i < 2; i++) {
            std::vector<uchar> &prim_data = displays_.getDisplayDataById(primary_displays_.at(i));
            const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(i)));
            addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                    prim_data, (uint)i, LayoutDisplayRole::Primary});
        }
    }

    virtual void handleImageResponse() override
    {
        for (int i = 0; i < image_response_queue_.size(); i++) {
            DisplayImageResponse &response(image_response_queue_.at(i));
            prim_img_ids_[response.index] = response.id;
        }
    }

private:
    SplitParams parameters_;
};

// TODO: Make sure to use the same button to switch displays as PiP layout
// uses for toggling PiP window
struct TwinnedParams
{
    uint primary_display = 0;
};

class TwinnedLayout : public Layout
{
public:
    TwinnedLayout(DisplayManager &displays, TwinnedParams params=TwinnedParams()) : 
            Layout(LayoutType::TWINNED, displays), parameters_(params) 
    {
        addPrimaryDisplayByIx(parameters_.primary_display);
    }

    virtual void displayLayoutParams() override
    {
        // TODO: This should limit to only two displays available
        drawDisplaysList();

        drawDraggableRing();

        drawDisplaySelector(0, "Main Display", LayoutDisplayRole::Primary);
    }

    virtual void draw() override
    {
        handleImageResponse();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);

        displayPrimaryWindows();

        std::vector<uchar> &prim_data = displays_.getDisplayDataById(primary_displays_.at(0));
        const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(0)));
        addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                prim_data, (uint)0, LayoutDisplayRole::Primary});           
    }

    virtual void handleImageResponse() override
    {
        for (int i = 0; i < image_response_queue_.size(); i++) {
            DisplayImageResponse &response(image_response_queue_.at(i));
            prim_img_ids_[response.index] = response.id;
        }
    }

    virtual void handleControllerInput(std::string input) override
    {
        LayoutCommand command(translateControllerInputToCommand(input));

        switch(command)
        {
            case LayoutCommand::PRIMARY_NEXT:
            {
                toNextDisplay(0, LayoutDisplayRole::Primary);
            }   break;

            default:
            {}  break;
        }
    }

private:
    TwinnedParams parameters_;
};



    class LayoutManager
    {
    public:
        const std::string cp_title = "Layouts Control Panel";

        LayoutManager() : active_layout(new InactiveLayout(displays)), previous_layout(active_layout) {}

        void setGrabbingState(bool state) { active_layout->setGrabbingState(state); }
        void setClutchingState(bool state) { active_layout->setClutchingState(state); }

        void toggleControlPanel()
        {
            control_panel_active = !control_panel_active;
        }

        void draw()
        {
            previous_layout = active_layout;

            if (control_panel_active)
            {
                ImGuiWindowFlags win_flags = 0;
                win_flags |= ImGuiWindowFlags_NoScrollbar;
                win_flags |= ImGuiWindowFlags_NoResize;
                win_flags |= ImGuiWindowFlags_MenuBar;
                win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
                ImGuiViewport* main_viewport = ImGui::GetMainViewport();
                ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + 30, 
                        main_viewport->GetWorkPos().y + 30), ImGuiCond_Once);
                if (startMenu(cp_title, win_flags)) {
                    buildControlPanel();
                    endMenu();
                }
            }

            active_layout->draw();
        }

        void handleKeyInput(int key, int action, int mods)
        {
            active_layout->handleKeyInput(key, action, mods);
        }

        void handleControllerInput(std::string input)
        {
            active_layout->handleControllerInput(input);
        }

        void addDisplay(const Display &disp) { displays.addDisplay(disp); }

        const DisplayInfo& getDisplayInfo(uint ix) const
        {
            return displays.getDisplayInfo(ix);
        }

        uint getNumTotalDisplays() const { return displays.getNumTotalDisplays(); }

        void forwardImageForDisplayId(uint id, const cv::Mat &image)
        {
            displays.copyImageToDisplay(id, image);
        }

        bool wasLayoutChanged() const
        {
            return previous_layout->getLayoutType() != active_layout->getLayoutType();
        }

        std::vector<DisplayImageRequest>& getImageRequestQueue()
        {
            return active_layout->getImageRequestQueue();
        }

        void pushImageResponse(const DisplayImageResponse &response)
        {
            // Skip responses since they may no longer apply to new layout
            if (wasLayoutChanged()) {
                return;
            }

            active_layout->pushImageResponse(response);
        }

    private:
        std::shared_ptr<Layout> active_layout;
        std::shared_ptr<Layout> previous_layout;
        DisplayManager displays;

        bool control_panel_active = true;

        std::vector<std::shared_ptr<Layout>> layouts_cache;
        std::vector<LayoutType> excluded_layouts;

        std::shared_ptr<Layout> newLayout(LayoutType type)
        {
            std::shared_ptr<Layout> layout;
            switch(type) {
                case LayoutType::PIP:
                {
                    layout = std::shared_ptr<Layout>(new PiPLayout(displays));
                } break;

                case LayoutType::SPLIT:
                {
                    layout = std::shared_ptr<Layout>(new SplitLayout(displays));
                } break;

                case LayoutType::TWINNED:
                {
                    layout = std::shared_ptr<Layout>(new TwinnedLayout(displays));
                } break;

                default: 
                {
                    layout = std::shared_ptr<Layout>(new InactiveLayout(displays));
                } break;
            }
            
            return layout;
        }

        void activateLayout(LayoutType type)
        {
            // It's already active
            if (active_layout->getLayoutType() == type) {
                return;
            }

            // We cache previously active layouts so that their params are not reset
            // This also allows us to initialize all the layouts with custom params
            // at the start of the program
            if (!isInCache(active_layout->getLayoutType())) {
                layouts_cache.push_back(active_layout);
            }

            if (isInCache(type)) {
                active_layout = getLayoutFromCache(type);
            }
            else {
                active_layout = newLayout(type);
            }
        }

        bool isLayoutActive(LayoutType type) const
        {
            if (active_layout->getLayoutType() == type) {
                return true;
            }

            return false;
        }

        void excludeLayout(LayoutType type)
        {
            if (!isLayoutExcluded(type)) {
                excluded_layouts.push_back(type);
            }
        }

        bool isLayoutExcluded(LayoutType type) const
        {
            const std::vector<LayoutType> &vec = excluded_layouts;
            if (std::find(vec.begin(), vec.end(), type) != vec.end()) {
                return true;
            }

            return false;
        }

        bool isInCache(LayoutType type) const
        {
            for (const std::shared_ptr<Layout> layout : layouts_cache) {
                if (layout->getLayoutType() == type) {
                    return true;
                }
            }

            return false;
        }

        std::shared_ptr<Layout> getLayoutFromCache(LayoutType type) const
        {
            // LayoutManager is initialized with an InactiveLayout, so it should always
            // should be the first layout in the cache
            std::shared_ptr<Layout> none_layout(layouts_cache[0]);

            if (type == LayoutType::INACTIVE) {
                return none_layout;
            }

            for (const std::shared_ptr<Layout> layout : layouts_cache) {
                if (layout->getLayoutType() == type) {
                    return layout;
                }
            }

            return none_layout;
        }
    
        void buildControlPanel()
        {
            if (ImGui::BeginMenuBar())
            {
                if (ImGui::BeginMenu(active_layout->getLayoutName().c_str())) {
                    std::vector<std::string> layout_names = active_layout->getLayoutList();
                    bool selected;
                    bool inactivate = false;

                    selected = isLayoutActive(LayoutType::INACTIVE);
                    ImGui::MenuItem("Inactivate", NULL, &selected);
                    if (selected)
                    {
                        activateLayout(LayoutType::INACTIVE);
                    }

                    for (int i = 0; i < layout_names.size(); i++) {
                        LayoutType layout_type = Layout::intToLayoutType(i);
                        
                        if (isLayoutExcluded(layout_type)) {
                            continue;
                        }

                        selected = isLayoutActive(layout_type);
                        ImGui::MenuItem(layout_names[i].c_str(), NULL, &selected);

                        if (selected) {
                            activateLayout(layout_type);
                        }
                    }

                    ImGui::EndMenu();
                }

                ImGui::EndMenuBar();
            }

            if (!isLayoutActive(LayoutType::INACTIVE)) {
                ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
                ImGui::Text("Parameters for %s:", active_layout->getLayoutName().c_str());
            }
            ImGui::Spacing();
            ImGui::Spacing();

            active_layout->displayLayoutParams();
        }

    };

} // viewpoint_interface

#endif // __LAYOUT_HPP__