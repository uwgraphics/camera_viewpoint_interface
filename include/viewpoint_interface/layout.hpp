#ifndef __LAYOUT_HPP__
#define __LAYOUT_HPP__

#include <map>
#include <string>
#include <algorithm>

#include <glm/vec2.hpp>
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
        PILOT,
        STACK,
        GRID,
        PIP,
        CAROUSEL
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
        LayoutType getLayoutType() const { return layout_type; } 

        // TODO: Make these private and make LayoutManager friend class
        static LayoutType intToLayoutType(int ix) {
            if (ix >= 0 && ix < num_layout_types) {
                return (LayoutType)ix;
            }

            return LayoutType::INACTIVE;
        }

        const std::vector<std::string> getLayoutList() const 
        { 
            return layout_names; 
        }

        const std::string getLayoutName() const
        {
            if (layout_type == LayoutType::INACTIVE) {
                return "Layouts Inactive";
            }

            return layout_names[(uint)layout_type];
        }

        void setGrabbingState(bool state) { grabbing = state; }
        void setClutchingState(bool state) { clutching = state; }

        std::vector<DisplayImageRequest>& getImageRequestQueue()
        {
            return display_image_queue;
        }

        void pushImageResponse(const DisplayImageResponse &response)
        {
            image_response_queue.push_back(response);
        }

        virtual void displayLayoutParams() = 0;
        virtual void draw() = 0;

        virtual void handleKeyInput(int key, int action, int mods)
        {
            if (action == GLFW_PRESS) {
                switch (key) {
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

        virtual void handleControllerInput(std::string input)
        {
            LayoutCommand command(translateControllerInputToCommand(input));

            switch(command)
            {
                case LayoutCommand::PRIMARY_NEXT:
                {
                    toNextDisplay(0, LayoutDisplayRole::Primary);
                }   break;

                case LayoutCommand::PRIMARY_PREV:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Primary);
                }   break;

                case LayoutCommand::SECONDARY_NEXT:
                {
                    toNextDisplay(0, LayoutDisplayRole::Secondary);
                }   break;

                case LayoutCommand::SECONDARY_PREV:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Secondary);
                }   break;

                default:
                {

                }   break;
            }
        }

    protected:
        DisplayManager &displays;
        std::vector<DisplayImageRequest> display_image_queue;
        std::vector<DisplayImageResponse> image_response_queue;

        bool grabbing, clutching;

        std::vector<uint> primary_displays; // Stores display ID
        std::vector<uint> secondary_displays; // Stores display ID
        std::vector<uint> prim_img_ids; // Stores OpenGL ID for primary images

        struct ColorSet
        {
            ImVec4 base, hovered, active;
        };

        ColorSet color_cache;
        ColorSet primary_color;
        ColorSet secondary_color;

        const ImVec4 on_color = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
        const ImVec4 off_color = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);

        Layout(LayoutType type, DisplayManager &disp) : layout_type(type), displays(disp)
        {
            primary_color.base =    ImVec4{10.0/255, 190.0/255, 10.0/255, 150.0/255};
            primary_color.hovered = ImVec4{10.0/255, 190.0/255, 10.0/255, 200.0/255}; 
            primary_color.active =  ImVec4{10.0/255, 190.0/255, 10.0/255, 255.0/255};

            secondary_color.base =      ImVec4{165.0/255, 100.0/255, 100.0/255, 125.0/255};
            secondary_color.hovered =   ImVec4{165.0/255, 100.0/255, 100.0/255, 200.0/255}; 
            secondary_color.active =    ImVec4{165.0/255, 100.0/255, 100.0/255, 255.0/255};
        }

        void enableDisplayStyle(LayoutDisplayRole role)
        {
            ColorSet color_set;
            switch (role)
            {
                case LayoutDisplayRole::Primary:
                {
                    color_set = primary_color;
                }   break;

                case LayoutDisplayRole::Secondary:
                {
                    color_set = secondary_color;
                }   break;
            }

            ImGuiStyle& style = ImGui::GetStyle();
            color_cache.base = style.Colors[ImGuiCol_Header];
            color_cache.hovered = style.Colors[ImGuiCol_HeaderHovered];
            color_cache.active = style.Colors[ImGuiCol_HeaderActive];
            style.Colors[ImGuiCol_Header] = color_set.base;
            style.Colors[ImGuiCol_HeaderHovered] = color_set.hovered;
            style.Colors[ImGuiCol_HeaderActive] = color_set.active;
        }

        void disableDisplayStyle()
        {
            ImGuiStyle& style = ImGui::GetStyle();
            style.Colors[ImGuiCol_Header] = color_cache.base;
            style.Colors[ImGuiCol_HeaderHovered] = color_cache.hovered;
            style.Colors[ImGuiCol_HeaderActive] = color_cache.active;
        }

        void addPrimaryDisplayByIx(uint ix)
        {
            uint id = displays.getDisplayId(ix);
            addPrimaryDisplayById(id);
        }

        void addPrimaryDisplayById(uint id)
        {
            primary_displays.push_back(id);
            prim_img_ids.resize(primary_displays.size());
        }

        void toNextDisplay(uint vec_ix, LayoutDisplayRole role)
        {
            std::vector<uint> &role_vec(getDisplaysVectorFromRole(role));

            if (role_vec.size() <= vec_ix) {
                return;
            }

            uint cur_ix = displays.getDisplayIxById(role_vec.at(vec_ix));
            uint next_ix = displays.getNextActiveDisplayIx(cur_ix);
            role_vec[vec_ix] = displays.getDisplayId(next_ix);
        }

        void toPrevDisplay(uint vec_ix, LayoutDisplayRole role)
        {
            std::vector<uint> &role_vec(getDisplaysVectorFromRole(role));

            if (role_vec.size() <= vec_ix) {
                return;
            }

            uint cur_ix = displays.getDisplayIxById(role_vec.at(vec_ix));
            uint prev_ix = displays.getPrevActiveDisplayIx(cur_ix);
            role_vec[vec_ix] = displays.getDisplayId(prev_ix);
        }

        void addImageRequestToQueue(DisplayImageRequest request)
        {
            display_image_queue.push_back(request);
        }

        virtual void handleImageResponse() = 0;

        virtual void displayInstructionsWindow(std::string text) const
        {
            std::string title = "Instructions";
            ImGuiWindowFlags win_flags = 0;
            win_flags |= ImGuiWindowFlags_NoScrollbar;
            win_flags |= ImGuiWindowFlags_NoResize;
            win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
            ImGuiViewport* main_viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + (main_viewport->GetWorkSize().x - 600), 
                    main_viewport->GetWorkPos().y + 50), ImGuiCond_Always);

            if (startMenu(title, win_flags)) {
                ImGui::Text("%s", text.c_str());
                endMenu();
            }
        }

        void displayStateValues(std::map<std::string, bool> states) const
        {
            std::string title = "States";
            ImGuiWindowFlags win_flags = 0;
            win_flags |= ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs;
            win_flags |= ImGuiWindowFlags_NoBackground;
            win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
            ImGuiViewport* main_viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + (main_viewport->GetWorkSize().x - 300), 
                    main_viewport->GetWorkPos().y + 50), ImGuiCond_Always);

            if (startMenu(title, win_flags)) {
                std::map<std::string, bool>::iterator itr;
                for (itr = states.begin(); itr != states.end(); itr++) {
                    ImGui::Text("%s: ", (itr->first).c_str());
                    ImGui::SameLine();
                    bool state(itr->second);
                    ImVec4 state_color(state ? on_color : off_color);
                    std::string state_val(state ? "on" : "off");
                    ImGui::TextColored(state_color, state_val.c_str());
                }
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
                                toNextDisplay(item_ix, LayoutDisplayRole::Primary);
                            }
                            else {
                                item_ix = getItemIndexInVector(displays.getDisplayId(i), secondary_displays);
                                if (item_ix != -1) {
                                    toNextDisplay(item_ix, LayoutDisplayRole::Secondary);
                                }
                            }
                        }
                    }
                }

                ImGui::ListBoxFooter();
            }

            ImGui::Separator();
        }

        virtual void drawDisplaySelector(uint num, std::string title="", 
                LayoutDisplayRole role=LayoutDisplayRole::Primary)
        {
            std::vector<uint> &role_vec(getDisplaysVectorFromRole(role));
            std::string label(title);
            switch (role)
            {
                case LayoutDisplayRole::Primary:
                {
                    if (title.compare("") == 0) {
                        label = "Primary Display " + num;
                    }
                }   break;

                case LayoutDisplayRole::Secondary:
                {
                    if (title.compare("") == 0) {
                        label = "Secondary Display " + num;
                    }
                }   break;
            }

            if (role_vec.size() <= num) {
                return;
            }

            static ImGuiComboFlags flags = 0;
            flags |= ImGuiComboFlags_PopupAlignLeft;

            std::string preview = displays.getDisplayInternalNameById(role_vec.at(num));
            if (ImGui::BeginCombo(label.c_str(), preview.c_str(), flags))
            {
                int cur_ix = -1;
                for (int i = 0; i < displays.getNumActiveDisplays(); i++)
                {
                    cur_ix = displays.getNextActiveDisplayIx(cur_ix);

                    const bool in_vec = (displays.getDisplayId(cur_ix) == role_vec.at(num));
                    std::string disp_name = displays.getDisplayInternalName(cur_ix);
                    if (ImGui::Selectable(disp_name.c_str(), in_vec)) {
                        role_vec[num] = displays.getDisplayId(cur_ix);
                    }

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (in_vec) {
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
                        enableDisplayStyle(LayoutDisplayRole::Primary);
                        ImGui::Selectable(displays.getDisplayInternalName(i).c_str(), true);
                        disableDisplayStyle();
                    }
                    else if(isItemInVector(displays.getDisplayId(i), secondary_displays)) {
                        enableDisplayStyle(LayoutDisplayRole::Secondary);
                        ImGui::Selectable(displays.getDisplayInternalName(i).c_str(), true);
                        disableDisplayStyle();
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

        virtual void displayPrimaryWindows() {
            for (int i = 0; i < prim_img_ids.size(); i++) {
                ImGuiWindowFlags win_flags = 0;
                win_flags |= ImGuiWindowFlags_NoDecoration;
                win_flags |= ImGuiWindowFlags_NoInputs;
                win_flags |= ImGuiWindowFlags_NoSavedSettings;
                win_flags |= ImGuiWindowFlags_NoMove;
                win_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus; // Otherwise, it overlays everything
                ImGuiViewport* main_viewport = ImGui::GetMainViewport();

                // TODO: Figure out grid spacing when there are multiple primary displays
                ImGui::SetNextWindowPos(ImVec2(0.0, 0.0), ImGuiCond_Once);
                ImGui::SetNextWindowSize(main_viewport->GetWorkSize());

                // Maintain the right aspect ratio
                const DisplayInfo &disp_info(displays.getDisplayInfoById(primary_displays.at(i)));
                float aspect_ratio = (float)disp_info.dimensions.width / disp_info.dimensions.height;

                ImVec2 image_dims = main_viewport->GetWorkSize();
                ImVec2 padding{5, 5};

                float img_width = image_dims.x - (2*padding.x);
                float img_height = img_width * (1.0/aspect_ratio);

                if (img_height > image_dims.y) {
                    // Converted height is too large--convert width instead
                    img_height = image_dims.y - (2*padding.y);
                    img_width = img_height * aspect_ratio;
                }

                if (startMenu("Primary Display", win_flags)) {
                    // Center the image on the window
                    ImVec2 image_pos = ImVec2{(ImGui::GetWindowSize().x - img_width) * 0.5f, 
                                              (ImGui::GetWindowSize().y - img_height) * 0.5f};
                    ImGui::SetCursorPos(image_pos);

                    ImGui::Image(reinterpret_cast<ImTextureID>(prim_img_ids.at(i)), ImVec2 {img_width, img_height});
                    
                    // Show camera external name on top of image
                    ImGui::SetCursorPos({image_pos.x + 10, image_pos.y + 5});
                    const std::string &title(displays.getDisplayExternalNameById(primary_displays.at(i)));
                    ImGui::Text(title.c_str());

                    endMenu();
                }
            }
        }

        virtual void displayPiPWindow(int width, int height, uint pip_id)
        {
            ImVec2 offset{150, 150};

            std::string title = displays.getDisplayExternalNameById(secondary_displays.at(0));
            ImGuiWindowFlags win_flags = 0;
            win_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse;
            win_flags |= ImGuiWindowFlags_NoTitleBar;
            win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
            ImGuiViewport* main_viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + (main_viewport->GetWorkSize().x - width-offset.x), 
                    main_viewport->GetWorkPos().y + (main_viewport->GetWorkSize().y - height-offset.y)), ImGuiCond_Once);

            if (startMenu("Picture-in-Picture", win_flags)) {
                ImGui::Text("%s", title.c_str());
                ImGui::Image(reinterpret_cast<ImTextureID>(pip_id), ImVec2(width, height));
                endMenu();
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

        std::vector<uint> &getDisplaysVectorFromRole(LayoutDisplayRole role)
        {
            switch (role)
            {
                case LayoutDisplayRole::Primary:
                {
                    return primary_displays;
                }   break;

                case LayoutDisplayRole::Secondary:
                {
                    return secondary_displays;
                }   break;
            }

            return primary_displays;
        }
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


    struct PilotParams
    {
        uint start_primary_display = 0;
        uint start_pip_display = 1;
        uint max_num_displays = 3;

        int pip_window_dims[2] = { 400, 225 };
        int pip_aspect_ratio[2] = { 16, 9 };
    };

    class PilotLayout : public Layout
    {
    public:
        PilotLayout(DisplayManager &displays, PilotParams params=PilotParams()) : Layout(LayoutType::PILOT, displays),
                parameters(params), keep_aspect_ratio(true), pip_enabled(true)
        {
            if (parameters.max_num_displays > displays.size() || parameters.max_num_displays == 0) {
                parameters.max_num_displays = displays.size();
            }

            addPrimaryDisplayByIx(parameters.start_primary_display);
            secondary_displays.push_back(displays.getDisplayId(parameters.start_pip_display));
        }

        virtual void displayLayoutParams() override
        {
            static uint cur_num_displays = parameters.max_num_displays;

            drawDisplaysList();

            ImGui::SliderInt("# Displays", (int *) &cur_num_displays, 1, parameters.max_num_displays);

            drawDraggableRing();

            drawDisplaySelector(0, "Main Camera", LayoutDisplayRole::Primary);
            drawDisplaySelector(0, "Pic-in-Pic Camera", LayoutDisplayRole::Secondary);

            ImGui::Separator();

            ImGui::Text("Picture-in-Picture Settings:\n");
            uint max_size = 600;
            int start_pip_dims[2] = { parameters.pip_window_dims[0], parameters.pip_window_dims[1] };
            bool pip_dims_changed = ImGui::DragInt2("Dimensions", parameters.pip_window_dims, 1.0f, 100, max_size);
            bool ar_changed = ImGui::Checkbox("Keep Aspect Ratio", &keep_aspect_ratio);

            if (keep_aspect_ratio) {
                ImGui::Text("Aspect Ratio");
                ImGui::SameLine();
                ar_changed = ImGui::InputInt2("##PiP AR", parameters.pip_aspect_ratio) || ar_changed;

                if (pip_dims_changed || ar_changed) {
                    float aspect_ratio = (float)parameters.pip_aspect_ratio[0] / parameters.pip_aspect_ratio[1];
                    
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

                    if (parameters.pip_window_dims[0] > max_width || 
                            parameters.pip_window_dims[1] > max_height) {
                        parameters.pip_window_dims[0] = max_width;
                        parameters.pip_window_dims[1] = max_height;
                    }
                    else {
                        if (parameters.pip_window_dims[1]-start_pip_dims[1] != 0) {
                            // Height changed
                            parameters.pip_window_dims[0] = parameters.pip_window_dims[1] * aspect_ratio;
                        }
                        else {
                            // Width changed
                            aspect_ratio = 1.0 / aspect_ratio;
                            parameters.pip_window_dims[1] = parameters.pip_window_dims[0] * aspect_ratio;
                        }
                    }
                }
            }
        }

        virtual void draw() override
        {
            handleImageResponse();

            std::map<std::string, bool> states;
            states["Suction Pad"] = grabbing;
            states["Clutching"] = clutching;
            displayStateValues(states);

            // We only have one primary and one Pic-in-pic display
            displayPrimaryWindows();

            std::vector<uchar> &prim_data = displays.getDisplayDataById(primary_displays.at(0));
            const DisplayInfo &prim_info(displays.getDisplayInfoById(primary_displays.at(0)));
            addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                    prim_data, 0, LayoutDisplayRole::Primary});

            if (pip_enabled) {
                displayPiPWindow(parameters.pip_window_dims[0], parameters.pip_window_dims[1], pip_id);

                std::vector<uchar> &sec_data = displays.getDisplayDataById(secondary_displays[0]);
                const DisplayInfo &sec_info(displays.getDisplayInfoById(primary_displays.at(0)));
                addImageRequestToQueue(DisplayImageRequest{sec_info.dimensions.width, sec_info.dimensions.height, 
                        sec_data, 0, LayoutDisplayRole::Secondary});
            }
        }

        virtual void handleImageResponse() override
        {
            for (int i = 0; i < image_response_queue.size(); i++) {
                DisplayImageResponse &response(image_response_queue.at(i));

                switch (response.role)
                {
                    case LayoutDisplayRole::Primary:
                    {
                        prim_img_ids[response.index] = response.id;
                    }   break;

                    case LayoutDisplayRole::Secondary:
                    {
                        pip_id = response.id;
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
                        pip_enabled = !pip_enabled;
                    } break;

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
                case LayoutCommand::PRIMARY_NEXT:
                {
                    toNextDisplay(0, LayoutDisplayRole::Primary);
                }   break;

                case LayoutCommand::PRIMARY_PREV:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Primary);
                }   break;

                case LayoutCommand::SECONDARY_NEXT:
                {
                    toNextDisplay(0, LayoutDisplayRole::Secondary);
                }   break;

                case LayoutCommand::SECONDARY_PREV:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Secondary);
                }   break;

                case LayoutCommand::PIP_TOGGLE:
                {
                    pip_enabled = !pip_enabled;
                }   break;

                default:
                {

                }   break;
            }
        }


    private:
        PilotParams parameters;
        
        uint pip_id;
        bool keep_aspect_ratio, pip_enabled;
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

            drawDisplaySelector(0);
        }

        virtual void draw() override
        {

        }

        virtual void handleImageResponse() override
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

        virtual void handleImageResponse() override
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

        virtual void handleImageResponse() override
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

        virtual void handleImageResponse() override
        {
            
        }

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