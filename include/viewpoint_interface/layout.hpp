#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <string>
#include <algorithm>

#include <imgui/imgui.h>
#include "display.hpp"

namespace viewpoint_interface
{
    class LayoutManager;

    // To add/change layouts follow the following steps:
    // - Add/change enum entry for layout
    // - Edit num_layout_types and layout_names in Layout class
    // - Create/change subclass inheriting from Layout class
    // - Add/change entry for layout in newLayout() function of LayoutManager
    enum LayoutType
    {
        NONE = -1,
        PILOT,
        STACK,
        GRID,
        PIP,
        CAROUSEL
    };
    
    class Layout
    {
    public:
        LayoutType getLayoutType() const { return layout_type; } 

        static LayoutType intToLayoutType(int ix) {
            if (ix >= 0 && ix < num_layout_types) {
                return (LayoutType)ix;
            }

            return LayoutType::NONE;
        }

        const std::vector<std::string> getLayoutList() const 
        { 
            return layout_names; 
        }

        const std::string getLayoutName() const
        {
            if (layout_type == LayoutType::NONE) {
                return "Layouts Inactive";
            }

            return layout_names[(uint)layout_type];
        }

        virtual void displayLayoutParams() const = 0;

    protected:
        DisplayManager &displays;

        Layout(LayoutType type, DisplayManager &disp) : layout_type(type), displays(disp) {}

        virtual void displayInstructionsWindow() const
        {
            
        }

        virtual void drawDisplaysList() const
        {
            // TODO: Add min and max number of displays that can be active
            // If max is one, just use standard ListBox

            const int max_items = 5; // Max number of displays to list w/o scrolling

            int total_displays = displays.size();
            int opt_shown = total_displays > max_items ? max_items : total_displays;
            if (ImGui::ListBoxHeader("Available\nDisplays", total_displays, opt_shown)) {
                for (int i = 0; i < total_displays; i++) {
                    bool active = displays.isDisplayActive(i);

                    if (ImGui::Selectable(displays[i].getInternalName().c_str(), active)) {
                        if (!displays.isDisplayActive(i) || displays.getNumActiveDisplays() > 1) {
                            displays.flipDisplayState(i);
                        }
                    }
                }

                ImGui::ListBoxFooter();
            }

            ImGui::Separator();
        }

        virtual void drawDisplaySelector() const
        {
            static ImGuiComboFlags flags = 0;
            flags |= ImGuiComboFlags_PopupAlignLeft;

            std::string combo_label = displays.getPrimaryDisplay().getInternalName();
            if (ImGui::BeginCombo("Primary Display", combo_label.c_str(), flags))
            {
                for (int i = 0; i < displays.getNumActiveDisplays(); i++)
                {
                    const bool is_primary = (i == displays.getPrimaryDisplayIx());
                    std::string disp_name = displays[i].getInternalName();
                    if (ImGui::Selectable(disp_name.c_str(), is_primary))
                        displays.setPrimaryDisplay(i);

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_primary) {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }
        }

        virtual void drawDraggableRing() const
        {
            ImGuiStyle& style = ImGui::GetStyle();
            uint disps_num = displays.getNumActiveDisplays();
            if (ImGui::ListBoxHeader("Display\nOrder", disps_num)) {
                for (int i = 0; i < displays.size(); i++) {
                    if (!displays.isDisplayActive(i)) {
                        continue;
                    }

                    if (i == displays.getPrimaryDisplayIx()) {
                        ImVec4 active_color = style.Colors[ImGuiCol_HeaderActive];
                        ImVec4 hovered_color = style.Colors[ImGuiCol_HeaderHovered];
                        ImVec4 header_color = style.Colors[ImGuiCol_Header];
                        style.Colors[ImGuiCol_HeaderActive] = displays.getPrimaryColorActive();
                        style.Colors[ImGuiCol_HeaderHovered] = displays.getPrimaryColorHovered();
                        style.Colors[ImGuiCol_Header] = displays.getPrimaryColorBase();
                        ImGui::Selectable(displays[i].getInternalName().c_str(), true);
                        style.Colors[ImGuiCol_Header] = header_color;
                        style.Colors[ImGuiCol_HeaderHovered] = hovered_color;
                        style.Colors[ImGuiCol_HeaderActive] = active_color;
                    }
                    else {
                        ImGui::Selectable(displays[i].getInternalName().c_str());
                    }

                    if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
                        int next = i + (ImGui::GetMouseDragDelta(0).y < 0.f ? -1 : 1);
                        if (next >= 0 && next < displays.size())
                        {
                            displays.swapDisplays(i, next);
                            ImGui::ResetMouseDragDelta();
                        }
                    }
                }

                ImGui::ListBoxFooter();
            }
            
        }

        virtual void drawCarouselMenu() const
        {

        }

    private:
        static const uint num_layout_types = 5;
        const std::vector<std::string> layout_names = {
            "Pilot", "Stack", "Grid", "Picture-in-Picture", "Carousel"
        };

        LayoutType layout_type;
    };
    

    class NoneLayout : public Layout
    {
    public:
        NoneLayout(DisplayManager &displays) : Layout(LayoutType::NONE, displays) {}

        virtual void displayLayoutParams() const override
        {
            ImGui::Text("No layouts selected:");
            ImGui::BulletText("Please select a layout from the menu\n"
                            "at the top of the window.");
        }

    };


    struct PilotParams
    {
        uint start_num_displays = 3;
        uint max_displays = DisplayManager::max_buffers;
        uint primary_display = 0;
    };

    class PilotLayout : public Layout
    {
    public:
        PilotLayout(DisplayManager &displays, PilotParams params=PilotParams()) : Layout(LayoutType::PILOT, displays),
                parameters(params) 
        {
            if (parameters.start_num_displays > displays.size()) {
                parameters.start_num_displays = displays.size();
            }
            if (parameters.start_num_displays > parameters.max_displays) {
                parameters.start_num_displays = parameters.max_displays;
            }
            else if (parameters.start_num_displays == 0) {
                parameters.start_num_displays = 1;
            }

            primary_display = parameters.primary_display;
        }

        virtual void displayLayoutParams() const override
        {
            static uint num_displays = parameters.start_num_displays;
            static uint max_bound = num_displays < parameters.max_displays ? num_displays : parameters.max_displays;

            drawDisplaysList();

            ImGui::SliderInt("# Displays", (int *) &num_displays, 1, max_bound);

            drawDraggableRing();

            drawDisplaySelector();
        }


    private:
        PilotParams parameters;
        uint primary_display;
    };


    struct StackParams
    {
        uint start_num_displays = 3;
        uint max_displays = DisplayManager::max_buffers;
        uint primary_display = 0;
    };

    class StackLayout : public Layout
    {
    public:

        StackLayout(DisplayManager &displays, StackParams params=StackParams()) : Layout(LayoutType::STACK, displays), 
                parameters(params) 
        {
            if (parameters.start_num_displays > displays.size()) {
                parameters.start_num_displays = displays.size();
            }
            if (parameters.start_num_displays > parameters.max_displays) {
                parameters.start_num_displays = parameters.max_displays;
            }
            else if (parameters.start_num_displays == 0) {
                parameters.start_num_displays = 1;
            }

            primary_display = parameters.primary_display;
        }

        virtual void displayLayoutParams() const override
        {
            static uint num_displays = parameters.start_num_displays;
            static uint max_bound = num_displays < parameters.max_displays ? num_displays : parameters.max_displays;

            drawDisplaysList(); 

            ImGui::SliderInt("# Displays", (int *) &num_displays, 1, max_bound);

            drawDraggableRing();

            drawDisplaySelector();
        }

    private:
        StackParams parameters;
        uint primary_display;
    };

    class GridLayout : public Layout
    {
    public:
        GridLayout(DisplayManager &displays) : Layout(LayoutType::GRID, displays) {}

        virtual void displayLayoutParams() const override
        {

        }

    };

    class PipLayout : public Layout
    {
    public:
        PipLayout(DisplayManager &displays) : Layout(LayoutType::PIP, displays) {}

        virtual void displayLayoutParams() const override
        {

        }

    };

    class CarouselLayout : public Layout
    {
    public:
        CarouselLayout(DisplayManager &displays) : Layout(LayoutType::CAROUSEL, displays) {}

        virtual void displayLayoutParams() const override
        {

        }

    };


    class LayoutManager
    {
    public:
        const std::string window_title = "Layouts Control Panel";

        LayoutManager() : active_layout(new NoneLayout(displays)) {}

        inline static LayoutType intToLayoutType(int ix) {
            return Layout::intToLayoutType(ix);
        }

        // TODO: Add function to draw layout windows
        // May need to pass OpenGL info about window size

        const std::vector<std::string> getLayoutList() const 
        { 
            return active_layout->getLayoutList(); 
        }

        std::shared_ptr<Layout> getActiveLayout() const
        {
            return active_layout;
        }

        void activateLayout(LayoutType type)
        {
            // Already active
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

        DisplayManager &getDisplays()
        {
            return displays;
        }

    private:
        std::shared_ptr<Layout> active_layout;
        DisplayManager displays;

        std::vector<std::shared_ptr<Layout>> layouts_cache;
        std::vector<LayoutType> excluded_layouts;

        std::shared_ptr<Layout> newLayout(LayoutType type)
        {
            std::shared_ptr<Layout> layout;
            switch(type) {
                case LayoutType::PILOT:
                {
                    layout = std::shared_ptr<Layout>(new PilotLayout(displays));
                } break;

                case LayoutType::STACK:
                {
                    layout = std::shared_ptr<Layout>(new StackLayout(displays));
                } break;

                case LayoutType::GRID:
                {
                    layout = std::shared_ptr<Layout>(new GridLayout(displays));
                } break;

                case LayoutType::PIP:
                {
                    layout = std::shared_ptr<Layout>(new PipLayout(displays));
                } break;

                case LayoutType::CAROUSEL:
                {
                    layout = std::shared_ptr<Layout>(new CarouselLayout(displays));
                } break;

                default: 
                {
                    layout = std::shared_ptr<Layout>(new NoneLayout(displays));
                } break;
            }
            
            return layout;
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
            // LayoutManager is initialized with a NoneLayout, so it should always
            // should be the first layout in the cache
            std::shared_ptr<Layout> none_layout(layouts_cache[0]);

            if (type == LayoutType::NONE) {
                return none_layout;
            }

            for (const std::shared_ptr<Layout> layout : layouts_cache) {
                if (layout->getLayoutType() == type) {
                    return layout;
                }
            }

            return none_layout;
        }
    };

} // viewpoint_interface

#endif // __LAYOUT_HPP__