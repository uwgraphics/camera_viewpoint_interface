#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <string>
#include <algorithm>

#include <imgui/imgui.h>
#include "display.hpp"

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

        // TODO: Make these private and make LayoutManager friend class
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

        inline bool hasPrimaryDisplays() const { return has_primary; }

        virtual void displayLayoutParams() = 0;
        virtual void draw() = 0;

        virtual void handleKeyInput(int key, int action, int mods)
        {
            if (action == GLFW_PRESS) {
                switch (key) {
                    case GLFW_KEY_RIGHT:
                    {
                        toNextPrimaryDisplay(0);
                    } break;

                    case GLFW_KEY_LEFT:
                    {
                        toPrevPrimaryDisplay(0);
                    } break;

                    default:
                    {
                    } break;
                }
            }
        }

    protected:
        DisplayManager &displays;
        std::vector<uint> primary_displays; // Stores display ID
        std::vector<uint> secondary_displays;

        struct ColorSet
        {
            ImVec4 base, hovered, active;
        };

        ColorSet color_cache;
        ColorSet primary_color;

        Layout(LayoutType type, DisplayManager &disp, bool prim=true) : layout_type(type), displays(disp),
                has_primary(prim)
            {
                primary_color.base = ImVec4{10.0/255, 190.0/255, 10.0/255, 150.0/255};
                primary_color.hovered = ImVec4{10.0/255, 190.0/255, 10.0/255, 200.0/255}; 
                primary_color.active = ImVec4{10.0/255, 190.0/255, 10.0/255, 150.0/255};
            }

        template <typename T>
        int getItemIndexInVector(T item, std::vector<T> &vec) const
        {
            auto iter = std::find(vec.begin(), vec.end(), item);
            if(iter != vec.end()) {
                return iter - vec.begin();
            }

            return -1;
        }

        template <typename T>
        int getItemIndexInVector(T item, const std::vector<T> &vec) const
        {
            auto iter = std::find(vec.begin(), vec.end(), item);
            if(iter != vec.end()) {
                return iter - vec.begin();
            }

            return -1;
        }

        template <typename T>
        bool isItemInVector(T item, std::vector<T> &vec) const
        {
            return getItemIndexInVector(item, vec) != -1;
        }

        template <typename T>
        bool isItemInVector(T item, const std::vector<T> &vec) const
        {
            return getItemIndexInVector(item, vec) != -1;
        }

        void enablePrimaryDisplayStyle()
        {
            ImGuiStyle& style = ImGui::GetStyle();
            color_cache.base = style.Colors[ImGuiCol_Header];
            color_cache.hovered = style.Colors[ImGuiCol_HeaderHovered];
            color_cache.active = style.Colors[ImGuiCol_HeaderActive];
            style.Colors[ImGuiCol_Header] = primary_color.base;
            style.Colors[ImGuiCol_HeaderHovered] = primary_color.hovered;
            style.Colors[ImGuiCol_HeaderActive] = primary_color.active;
        }

        void disablePrimaryDisplayStyle()
        {
            ImGuiStyle& style = ImGui::GetStyle();
            style.Colors[ImGuiCol_Header] = color_cache.base;
            style.Colors[ImGuiCol_HeaderHovered] = color_cache.hovered;
            style.Colors[ImGuiCol_HeaderActive] = color_cache.active;
        }

        void toNextPrimaryDisplay(uint primary_ix)
        {
            uint cur_ix = displays.getDisplayIxById(primary_displays.at(primary_ix));
            uint next_ix = displays.getNextActiveDisplayIx(cur_ix);
            primary_displays[primary_ix] = displays.getDisplayId(next_ix);
        }

        void toPrevPrimaryDisplay(uint primary_ix)
        {
            uint cur_ix = displays.getDisplayIxById(primary_displays.at(primary_ix));
            uint prev_ix = displays.getPrevActiveDisplayIx(cur_ix);
            primary_displays[primary_ix] = displays.getDisplayId(prev_ix);
        }

        virtual void displayInstructionsWindow(std::string text) const
        {
            std::string title = "Instructions";
            ImGuiWindowFlags win_flags = 0;
            win_flags |= ImGuiWindowFlags_NoScrollbar;
            win_flags |= ImGuiWindowFlags_NoResize;
            win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
            ImGuiViewport* main_viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + (main_viewport->GetWorkSize().x - 350), 
                    main_viewport->GetWorkPos().y + 30), ImGuiCond_Always);

            if (startMenu(title, win_flags)) {
                ImGui::Text("%s", text.c_str());
                endMenu();
            }
        }

        virtual void drawDisplaysList()
        {
            // TODO: Add min and max number of displays that can be active
            // If max is one, just use standard ListBox

            const int max_items = 5; // Max number of displays to list w/o scrolling

            int total_displays = displays.size();
            int opt_shown = total_displays > max_items ? max_items : total_displays;
            if (ImGui::ListBoxHeader("Available\nDisplays", total_displays, opt_shown)) {
                for (int i = 0; i < total_displays; i++) {
                    bool active = displays.isDisplayActive(i);

                    if (ImGui::Selectable(displays.getDisplayInternalName(i).c_str(), active)) {
                        if (!displays.isDisplayActive(i) || displays.getNumActiveDisplays() > 1) {
                            displays.flipDisplayState(i);

                            int item_ix = getItemIndexInVector(displays.getDisplayId(i), primary_displays);
                            if (item_ix != -1) {
                                toNextPrimaryDisplay(item_ix);
                            }
                        }
                    }
                }

                ImGui::ListBoxFooter();
            }

            ImGui::Separator();
        }

        virtual void drawPrimaryDisplaySelector(uint num)
        {
            if (!hasPrimaryDisplays()) {
                return;
            }

            static ImGuiComboFlags flags = 0;
            flags |= ImGuiComboFlags_PopupAlignLeft;

            std::string combo_label = displays.getDisplayInternalName(displays.getDisplayIxById(primary_displays.at(num)));
            if (ImGui::BeginCombo("Primary Display", combo_label.c_str(), flags))
            {
                int cur_ix = -1;
                for (int i = 0; i < displays.getNumActiveDisplays(); i++)
                {
                    cur_ix = displays.getNextActiveDisplayIx(cur_ix);

                    const bool is_primary = (displays.getDisplayId(cur_ix) == primary_displays[num]);
                    std::string disp_name = displays.getDisplayInternalName(cur_ix);
                    if (ImGui::Selectable(disp_name.c_str(), is_primary)) {
                        primary_displays[num] = displays.getDisplayId(cur_ix);
                    }

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_primary) {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }
        }

        virtual void drawDraggableRing()
        {
            uint disps_num = displays.getNumActiveDisplays();
            if (ImGui::ListBoxHeader("Display\nOrder", disps_num)) {

                for (int i = 0; i < displays.size(); i++) {
                    if (!displays.isDisplayActive(i)) {
                        continue;
                    }

                    if (isItemInVector(displays.getDisplayId(i), primary_displays)) {
                        enablePrimaryDisplayStyle();
                        ImGui::Selectable(displays.getDisplayInternalName(i).c_str(), true);
                        disablePrimaryDisplayStyle();
                    }
                    else {
                        ImGui::Selectable(displays.getDisplayInternalName(i).c_str());
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
        bool has_primary;
    };
    

    class NoneLayout : public Layout
    {
    public:
        NoneLayout(DisplayManager &displays) : Layout(LayoutType::NONE, displays) {}

        virtual void displayLayoutParams() override
        {
            ImGui::Text("No layouts selected:");
            ImGui::BulletText("Please select a layout from the menu\n"
                            "at the top of the window.");
        }

        virtual void draw() override {}
    };


    struct PilotParams
    {
        uint start_primary_display = 0;
        uint max_num_displays = 3;
    };

    class PilotLayout : public Layout
    {
    public:
        PilotLayout(DisplayManager &displays, PilotParams params=PilotParams()) : Layout(LayoutType::PILOT, displays),
                parameters(params) 
        {
            if (parameters.max_num_displays > displays.size() || parameters.max_num_displays == 0) {
                parameters.max_num_displays = displays.size();
            }

            

            primary_displays.push_back(displays.getDisplayId(parameters.start_primary_display));
        }

        virtual void displayLayoutParams() override
        {
            static uint cur_num_displays = parameters.max_num_displays;

            drawDisplaysList();

            ImGui::SliderInt("# Displays", (int *) &cur_num_displays, 1, parameters.max_num_displays);

            drawDraggableRing();

            drawPrimaryDisplaySelector(0);
        }

        virtual void draw() override
        {
            std::string instr_text = "Instructions:";
            displayInstructionsWindow(instr_text);
        }


    private:
        PilotParams parameters;
    };


    struct StackParams
    {
        uint start_num_displays = 3;
        uint max_displays = 9; // TODO
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

        virtual void displayLayoutParams() override
        {
            static uint num_displays = parameters.start_num_displays;
            static uint max_bound = num_displays < parameters.max_displays ? num_displays : parameters.max_displays;

            drawDisplaysList(); 

            ImGui::SliderInt("# Displays", (int *) &num_displays, 1, max_bound);

            drawDraggableRing();

            drawPrimaryDisplaySelector(0);
        }

        virtual void draw() override
        {

        }

    private:
        StackParams parameters;
        uint primary_display;
    };

    class GridLayout : public Layout
    {
    public:
        GridLayout(DisplayManager &displays) : Layout(LayoutType::GRID, displays) {}

        virtual void displayLayoutParams() override
        {

        }

        virtual void draw() override
        {
            
        }

    };

    class PipLayout : public Layout
    {
    public:
        PipLayout(DisplayManager &displays) : Layout(LayoutType::PIP, displays) {}

        virtual void displayLayoutParams() override
        {

        }

        virtual void draw() override
        {
            
        }

    };

    class CarouselLayout : public Layout
    {
    public:
        CarouselLayout(DisplayManager &displays) : Layout(LayoutType::CAROUSEL, displays) {}

        virtual void displayLayoutParams() override
        {

        }

        virtual void draw() override
        {
            
        }

    };


    class LayoutManager
    {
    public:
        const std::string cp_title = "Layouts Control Panel";

        LayoutManager() : active_layout(new NoneLayout(displays)) {}


        // TODO: Add function to draw layout windows
        // May need to pass OpenGL info about window size

        void draw()
        {
            // if control_panel_active
            ImGuiWindowFlags win_flags = 0;
            win_flags |= ImGuiWindowFlags_NoScrollbar;
            win_flags |= ImGuiWindowFlags_NoResize;
            win_flags |= ImGuiWindowFlags_MenuBar;
            win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
            ImGuiViewport* main_viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + 30, main_viewport->GetWorkPos().y + 30), ImGuiCond_Always);
            if (startMenu(cp_title, win_flags)) {
                buildControlPanel();
                endMenu();
            }

            active_layout->draw();
        }

        void handleKeyInput(int key, int action, int mods)
        {
            active_layout->handleKeyInput(key, action, mods);
        }

        void addDisplay(const Display &disp) { displays.addDisplay(disp); }

        const DisplayInfo& getDisplayInfo(uint ix) const
        {
            return displays.getDisplayInfo(ix);
        }

        uint getNumTotalDisplays() const { return displays.getNumTotalDisplays(); }

        void forwardImageForDisplayIx(uint ix, const cv::Mat &image)
        {
            displays.copyImageToDisplay(ix, image);
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
    
        void buildControlPanel()
        {
            if (ImGui::BeginMenuBar())
            {
                if (ImGui::BeginMenu(active_layout->getLayoutName().c_str())) {
                    std::vector<std::string> layout_names = active_layout->getLayoutList();
                    bool selected;
                    bool inactivate = false;

                    selected = isLayoutActive(LayoutType::NONE);
                    ImGui::MenuItem("Inactivate", NULL, &selected);
                    if (selected)
                    {
                        activateLayout(LayoutType::NONE);
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

            if (!isLayoutActive(LayoutType::NONE)) {
                ImGui::Text("Parameters for %s:", active_layout->getLayoutName().c_str());
            }
            ImGui::Spacing();
            ImGui::Spacing();

            active_layout->displayLayoutParams();
        }

    };

} // viewpoint_interface

#endif // __LAYOUT_HPP__