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
#include "viewpoint_interface/layouts/twinned_pip.hpp"
#include "viewpoint_interface/layouts/double_pip.hpp"

namespace viewpoint_interface
{

class LayoutManager
{
public:
    const std::string cp_title = "Layouts Control Panel";

    LayoutManager() : active_layout_(new InactiveLayout(displays_)), previous_layout_(active_layout_) {}

    void setGrabbingState(bool state) { active_layout_->setGrabbingState(state); }
    void setClutchingState(bool state) { active_layout_->setClutchingState(state); }
    void handleCollisionMessage(const std::string& message) { active_layout_->handleCollisionMessage(message); }
    void setActiveFrame(const uint& index) { active_layout_->setActiveFrame(index); }
    const std::vector<float>& getActiveDisplayMatrix() const { return active_layout_->getActiveDisplayMatrix(); }
    const std::vector<float> getDisplayBounds() const { return active_layout_->getDisplayBounds(); }

    void toggleControlPanel()
    {
        control_panel_active_ = !control_panel_active_;
    }

    void draw()
    {
        previous_layout_ = active_layout_;

        if (control_panel_active_)
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

        active_layout_->draw();
    }

    void handleKeyInput(int key, int action, int mods)
    {
        active_layout_->handleKeyInput(key, action, mods);
    }

    void handleControllerInput(std::string input)
    {
        active_layout_->handleControllerInput(input);
    }

    void addDisplay(const Display &disp) { displays_.addDisplay(disp); }

    const DisplayInfo& getDisplayInfo(uint ix) const
    {
        return displays_.getDisplayInfo(ix);
    }

    uint getNumTotalDisplays() const { return displays_.getNumTotalDisplays(); }

    void forwardImageForDisplayId(uint id, const cv::Mat &image)
    {
        displays_.copyImageToDisplay(id, image);
    }

    void forwardMatrixForDisplayId(uint id, const std::vector<float> &matrix)
    {
        displays_.copyMatrixToDisplay(id, matrix);
    }

    bool wasLayoutChanged() const
    {
        return previous_layout_->getLayoutType() != active_layout_->getLayoutType();
    }

    std::vector<DisplayImageRequest>& getImageRequestQueue()
    {
        return active_layout_->getImageRequestQueue();
    }

    void pushImageResponse(const DisplayImageResponse &response)
    {
        // Skip responses since they may no longer apply to new layout
        if (wasLayoutChanged()) {
            return;
        }

        active_layout_->pushImageResponse(response);
    }

private:
    std::shared_ptr<Layout> active_layout_;
    std::shared_ptr<Layout> previous_layout_;
    DisplayManager displays_;

    bool control_panel_active_ = true;

    std::vector<std::shared_ptr<Layout>> layouts_cache_;
    std::vector<LayoutType> excluded_layouts_;

    std::shared_ptr<Layout> newLayout(LayoutType type)
    {
        std::shared_ptr<Layout> layout;
        switch(type) {
            case LayoutType::DYNAMIC:
            {
                layout = std::shared_ptr<Layout>(new DynamicLayout(displays_));
            } break;

            case LayoutType::WIDE:
            {
                layout = std::shared_ptr<Layout>(new WideLayout(displays_));
            } break;

            case LayoutType::PIP:
            {
                layout = std::shared_ptr<Layout>(new PiPLayout(displays_));
            } break;

            case LayoutType::TIMED_PIP:
            {
                layout = std::shared_ptr<Layout>(new TimedPiPLayout(displays_));
            } break;

            case LayoutType::SPLIT:
            {
                layout = std::shared_ptr<Layout>(new SplitLayout(displays_));
            } break;

            case LayoutType::TWINNED:
            {
                layout = std::shared_ptr<Layout>(new TwinnedLayout(displays_));
            } break;

            case LayoutType::GRID:
            {
                layout = std::shared_ptr<Layout>(new GridLayout(displays_));
            } break;

            case LayoutType::CAROUSEL:
            {
                layout = std::shared_ptr<Layout>(new CarouselLayout(displays_));
            } break;

            case LayoutType::TWINNED_PIP:
            {
                layout = std::shared_ptr<Layout>(new TwinnedPipLayout(displays_));
            } break;

            case LayoutType::DOUBLE_PIP:
            {
                layout = std::shared_ptr<Layout>(new DoublePipLayout(displays_));
            } break;

            default: 
            {
                layout = std::shared_ptr<Layout>(new InactiveLayout(displays_));
            } break;
        }
        
        return layout;
    }

    void activateLayout(LayoutType type)
    {
        // It's already active
        if (active_layout_->getLayoutType() == type) {
            return;
        }

        // We cache previously active layouts so that their params are not reset
        // This also allows us to initialize all the layouts with custom params
        // at the start of the program
        if (!isInCache(active_layout_->getLayoutType())) {
            layouts_cache_.push_back(active_layout_);
        }

        if (isInCache(type)) {
            active_layout_ = getLayoutFromCache(type);
        }
        else {
            active_layout_ = newLayout(type);
        }
    }

    bool isLayoutActive(LayoutType type) const
    {
        if (active_layout_->getLayoutType() == type) {
            return true;
        }

        return false;
    }

    void excludeLayout(LayoutType type)
    {
        if (!isLayoutExcluded(type)) {
            excluded_layouts_.push_back(type);
        }
    }

    bool isLayoutExcluded(LayoutType type) const
    {
        const std::vector<LayoutType> &vec = excluded_layouts_;
        if (std::find(vec.begin(), vec.end(), type) != vec.end()) {
            return true;
        }

        return false;
    }

    bool isInCache(LayoutType type) const
    {
        for (const std::shared_ptr<Layout> layout : layouts_cache_) {
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
        std::shared_ptr<Layout> none_layout(layouts_cache_[0]);

        if (type == LayoutType::INACTIVE) {
            return none_layout;
        }

        for (const std::shared_ptr<Layout> layout : layouts_cache_) {
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
            if (ImGui::BeginMenu(active_layout_->getLayoutName().c_str())) {
                std::vector<std::string> layout_names = active_layout_->getLayoutList();
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
            ImGui::Text("Parameters for %s:", active_layout_->getLayoutName().c_str());
        }
        ImGui::Spacing();
        ImGui::Spacing();

        active_layout_->displayLayoutParams();
    }

};

} // viewpoint_interface

#endif // __LAYOUT_MANAGER_HPP__