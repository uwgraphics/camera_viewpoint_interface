#ifndef __LAYOUT_MANAGER_HPP__
#define __LAYOUT_MANAGER_HPP__

#include "viewpoint_interface/layout.hpp"
#include "viewpoint_interface/layouts/dynamic.hpp"
#include "viewpoint_interface/layouts/wide.hpp"
#include "viewpoint_interface/layouts/pip.hpp"
#include "viewpoint_interface/layouts/timed_pip.hpp"
#include "viewpoint_interface/layouts/split.hpp"
#include "viewpoint_interface/layouts/twinned.hpp"
#include "viewpoint_interface/layouts/grid.hpp"
#include "viewpoint_interface/layouts/carousel.hpp"

#include "viewpoint_interface/instance.hpp"

namespace viewpoint_interface
{

class LayoutManager
{
public:
    const std::string cp_title = "Layouts Control Panel";

    LayoutManager() : active_layout(new InactiveLayout(displays)), previous_layout(active_layout) {}

    void addModelData(std::vector<Model>* vec) { models_ = vec; }
    void setGrabbingState(bool state) { active_layout->setGrabbingState(state); }
    void setClutchingState(bool state) { active_layout->setClutchingState(state); }
    void handleCollisionMessage(const std::string& message) { active_layout->handleCollisionMessage(message); }
    void setActiveWindow(const uint& index) { active_layout->setActiveWindow(index); }
    const std::vector<float>& getActiveDisplayMatrix() const { return active_layout->getActiveDisplayMatrix(); }
    const std::vector<float> getDisplayBounds() const { return active_layout->getDisplayBounds(); }

    void toggleControlPanel()
    {
        control_panel_active = !control_panel_active;
    }

    void draw(float aspect_ratio)
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
                    main_viewport->GetWorkPos().y + 100), ImGuiCond_Once);
            if (startMenu(cp_title, win_flags)) {
                buildControlPanel();
                endMenu();
            }
        }

        // active_layout->draw();

        for (auto& instance : model_instances_) {
            instance.draw(aspect_ratio);
        }
    }

    void handleKeyInput(int key, int action, int mods)
    {
        active_layout->handleKeyInput(key, action, mods);
    }

    glm::vec2 getMouseCoordinates(glm::vec2 mouse_pos, glm::ivec2 window_dims)
    {
        return glm::vec2((mouse_pos.x - (window_dims.x/2.0)) / (window_dims.x/2.0), 
            -(mouse_pos.y - (window_dims.y/2.0)) / (window_dims.y/2.0));
    }

    Instance* getHoveredInstance(glm::vec2 mouse_coords)
    {
        for (auto& instance : model_instances_) {
            if (instance.isWithinSelectionRange(mouse_coords)) {
                return &instance;
            }
        }

        return NULL;
    }

    void handleMouseInput(glm::vec2 pos, glm::ivec2 window_dims, glm::ivec3 buttons)
    {
        static glm::ivec3 prev_buttons(GLFW_RELEASE, GLFW_RELEASE, GLFW_RELEASE);
        static Instance* prev_hovered_instance(NULL);
        static bool dragging(false);

        enum MouseButtons
        {
            LEFT_BUTTON = 0,
            MIDDLE_BUTTON = 1,
            RIGHT_BUTTON = 2
        };

        glm::vec2 mouse_coords(getMouseCoordinates(pos, window_dims));
        bool left_click_event(buttons[LEFT_BUTTON] == GLFW_PRESS && prev_buttons[LEFT_BUTTON] == GLFW_RELEASE);
        bool left_release_event(buttons[LEFT_BUTTON] == GLFW_RELEASE && prev_buttons[LEFT_BUTTON] == GLFW_PRESS);

        // Clear previous hovered (but not selected) instance state
        if (prev_hovered_instance && prev_hovered_instance->getSelectionState() == SelectionState::Hovered) {
            prev_hovered_instance->setSelectionState(SelectionState::None);
        }

        if (left_release_event) { dragging = false; }

        Instance* hovered_instance(getHoveredInstance(mouse_coords));
        if (dragging) { // We only start dragging when active not null and mouse not released
            active_instance_->setPositionOnScreen(mouse_coords);
        }
        else if (left_click_event) {
            if (active_instance_) {
                active_instance_->setSelectionState(SelectionState::None);
            }

            active_instance_ = hovered_instance;
            
            if (active_instance_) {
                dragging = true;
                active_instance_->setSelectionState(SelectionState::Selected);
            }
        }
        else if (hovered_instance && hovered_instance != active_instance_) {
            hovered_instance->setSelectionState(SelectionState::Hovered);
        }

        prev_buttons = buttons;
        prev_hovered_instance = hovered_instance;
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

    void forwardMatrixForDisplayId(uint id, const std::vector<float> &matrix)
    {
        displays.copyMatrixToDisplay(id, matrix);
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

    std::vector<Model>* models_;
    std::vector<Instance> model_instances_;
    Instance* active_instance_ = NULL;

    std::shared_ptr<Layout> newLayout(LayoutType type)
    {
        std::shared_ptr<Layout> layout;
        switch(type) {
            case LayoutType::DYNAMIC:
            {
                layout = std::shared_ptr<Layout>(new DynamicLayout(displays));
            } break;

            case LayoutType::WIDE:
            {
                layout = std::shared_ptr<Layout>(new WideLayout(displays));
            } break;

            case LayoutType::PIP:
            {
                layout = std::shared_ptr<Layout>(new PiPLayout(displays));
            } break;

            case LayoutType::TIMED_PIP:
            {
                layout = std::shared_ptr<Layout>(new TimedPiPLayout(displays));
            } break;

            case LayoutType::SPLIT:
            {
                layout = std::shared_ptr<Layout>(new SplitLayout(displays));
            } break;

            case LayoutType::TWINNED:
            {
                layout = std::shared_ptr<Layout>(new TwinnedLayout(displays));
            } break;

            case LayoutType::GRID:
            {
                layout = std::shared_ptr<Layout>(new GridLayout(displays));
            } break;

            case LayoutType::CAROUSEL:
            {
                layout = std::shared_ptr<Layout>(new CarouselLayout(displays));
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
            ImGui::Spacing();

            if (ImGui::CollapsingHeader("Loadable Models")) {
                ImGui::Indent();
                for (auto& model : *models_) {
                    if (ImGui::Button(model.getName().c_str())) {
                        model_instances_.push_back(Instance(model));
                    }
                }
                ImGui::Unindent();
            }
            ImGui::Spacing();

            ImGui::Text("Parameters for %s:", active_layout->getLayoutName().c_str());
        }
        ImGui::Spacing();
        ImGui::Spacing();

        active_layout->displayLayoutParams();
    }

};

} // viewpoint_interface

#endif // __LAYOUT_MANAGER_HPP__