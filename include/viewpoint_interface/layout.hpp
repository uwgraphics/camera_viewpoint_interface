#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <string>
#include <algorithm>

#include <imgui/imgui.h>
#include "display.hpp"

namespace viewpoint_interface
{
    class LayoutManager;

    enum LayoutType
    {
        NONE = -1,
        STACK,
        GRID,
        PIP,
        CAROUSEL
    };
    
    class Layout
    {
    public:
        LayoutType getLayoutType() const { return layout_type; } 

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

        virtual void drawDisplaysList() const
        {
            const int max_items = 5; // Max number of displays to list w/o scrolling

            int total_displays = displays.size();
            if (total_displays > 1) {

                int opt_shown = total_displays > max_items ? max_items : total_displays;
                if (ImGui::ListBoxHeader("Available\nDisplays", total_displays, opt_shown)) {
                    for (int i = 0; i < total_displays; i++) {
                        bool active = displays[i].isActive();

                        if (ImGui::Selectable(displays[i].getInternalName().c_str(), active)) {
                            // TODO: Shouldn't be able to turn off all the displays
                            
                            displays[i].flipState();
                        }
                    }
                }

                ImGui::ListBoxFooter();
            }

            ImGui::Separator();
        }

    private:
        const std::vector<std::string> layout_names = {
            "Stack", "Grid", "Picture-in-Picture", "Carousel"
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


    struct StackParams
    {
        uint start_num_displays = 3;
        uint max_displays = DisplayManager::max_buffers;

    };

    class StackLayout : public Layout
    {
    public:

        StackLayout(DisplayManager &displays, StackParams params=StackParams()) : Layout(LayoutType::STACK, displays), 
                parameters(params) 
        {
            if (parameters.start_num_displays > parameters.max_displays) {
                parameters.start_num_displays = parameters.max_displays;
            }
            else if (parameters.start_num_displays == 0) {
                parameters.start_num_displays = 1;
            }
        }

        virtual void displayLayoutParams() const override
        {
            static uint num_displays = parameters.start_num_displays;

            drawDisplaysList(); // Add min and max number of displays that can be active
            // If max is one, just use standard ListBox

            ImGui::SliderInt("# Displays", (int *) &num_displays, 1, parameters.max_displays);

            // TODO:
            // List all active displays in a ListBox and allow dragging and dropping to
            // rearrange the display ring
            // <algorithm> std::iter_swap
        }

    private:
        StackParams parameters;
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

        static LayoutType intToLayoutType(int ix) {
            if (ix >= 0 && ix < num_layout_types) {
                return (LayoutType)ix;
            }

            return LayoutType::NONE;
        }

        const std::vector<std::string> getLayoutList() const 
        { 
            return active_layout->getLayoutList(); 
        }

        std::shared_ptr<Layout> getActiveLayout() const
        {
            return active_layout;
        }

        void addDisplay(const Display &disp)
        {
            displays.addDisplay(disp);
        }

        const DisplayManager &getDisplays() const
        {
            return displays;
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

    private:
        static const uint num_layout_types = 4;

        std::shared_ptr<Layout> active_layout;
        DisplayManager displays;

        std::vector<std::shared_ptr<Layout>> layouts_cache;
        std::vector<LayoutType> excluded_layouts;

        std::shared_ptr<Layout> newLayout(LayoutType type)
        {
            std::shared_ptr<Layout> layout;
            switch(type) {
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