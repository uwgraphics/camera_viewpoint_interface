#include "viewpoint_interface/layout.hpp"


namespace viewpoint_interface {

// --- Public ---

const std::string Layout::getLayoutName() const
{
    if (layout_type_ == LayoutType::INACTIVE) {
        return "Layouts Inactive";
    }

    return kLayoutNames[(uint)layout_type_];
}

std::vector<DisplayImageRequest>& Layout::getImageRequestQueue() 
{ 
    return display_image_queue_; 
}

void Layout::pushImageResponse(const DisplayImageResponse &response)
{
    image_response_queue_.push_back(response);
}

void Layout::handleKeyInput(int key, int action, int mods)
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

void Layout::handleControllerInput(std::string input)
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


// --- Protected ---

void Layout::enableDisplayStyle(LayoutDisplayRole role)
{
    ColorSet color_set;
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            color_set = primary_color_;
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            color_set = secondary_color_;
        }   break;
    }

    ImGuiStyle& style = ImGui::GetStyle();
    color_cache_.base = style.Colors[ImGuiCol_Header];
    color_cache_.hovered = style.Colors[ImGuiCol_HeaderHovered];
    color_cache_.active = style.Colors[ImGuiCol_HeaderActive];
    style.Colors[ImGuiCol_Header] = color_set.base;
    style.Colors[ImGuiCol_HeaderHovered] = color_set.hovered;
    style.Colors[ImGuiCol_HeaderActive] = color_set.active;
}

void Layout::disableDisplayStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();
    style.Colors[ImGuiCol_Header] = color_cache_.base;
    style.Colors[ImGuiCol_HeaderHovered] = color_cache_.hovered;
    style.Colors[ImGuiCol_HeaderActive] = color_cache_.active;
}

void Layout::addPrimaryDisplayByIx(uint ix)
{
    uint id = displays_.getDisplayId(ix);
    addPrimaryDisplayById(id);
}

void Layout::addPrimaryDisplayById(uint id)
{
    primary_displays_.push_back(id);
    prim_img_ids_.resize(primary_displays_.size());
}

void Layout::toNextDisplay(uint vec_ix, LayoutDisplayRole role)
{
    std::vector<uint> &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() <= vec_ix) {
        return;
    }

    uint cur_ix = displays_.getDisplayIxById(role_vec.at(vec_ix));
    uint next_ix = displays_.getNextActiveDisplayIx(cur_ix);
    role_vec[vec_ix] = displays_.getDisplayId(next_ix);
}

void Layout::toPrevDisplay(uint vec_ix, LayoutDisplayRole role)
{
    std::vector<uint> &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() <= vec_ix) {
        return;
    }

    uint cur_ix = displays_.getDisplayIxById(role_vec.at(vec_ix));
    uint prev_ix = displays_.getPrevActiveDisplayIx(cur_ix);
    role_vec[vec_ix] = displays_.getDisplayId(prev_ix);
}

void Layout::addImageRequestToQueue(DisplayImageRequest request)
{
    display_image_queue_.push_back(request);
}

void Layout::displayInstructionsWindow(std::string text) const
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

void Layout::displayStateValues(std::map<std::string, bool> states) const
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
            ImVec4 state_color(state ? kOnColor : kOffColor);
            std::string state_val(state ? "ACTIVE" : "INACTIVE");
            ImGui::TextColored(state_color, state_val.c_str());
        }
        endMenu();
    }
}

void Layout::drawDisplaysList()
{
    // TODO: If num is one, just use standard ListBox

    const int max_items = 5; // Max number of displays to list w/o scrolling

    int total_displays = displays_.size();
    int opt_shown = total_displays > max_items ? max_items : total_displays;
    if (ImGui::ListBoxHeader("Available\nDisplays", total_displays, opt_shown)) {
        for (int i = 0; i < total_displays; i++) {
            bool active = displays_.isDisplayActive(i);

            if (ImGui::Selectable(displays_.getDisplayInternalName(i).c_str(), active)) {
                if (!displays_.isDisplayActive(i) || displays_.getNumActiveDisplays() > 1) {
                    displays_.flipDisplayState(i);

                    int item_ix = getItemIndexInVector(displays_.getDisplayId(i), primary_displays_);
                    if (item_ix != -1) {
                        toNextDisplay(item_ix, LayoutDisplayRole::Primary);
                    }
                    else {
                        item_ix = getItemIndexInVector(displays_.getDisplayId(i), secondary_displays_);
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

void Layout::drawDisplaySelector(uint num, std::string title, LayoutDisplayRole role)
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

    std::string preview = displays_.getDisplayInternalNameById(role_vec.at(num));
    if (ImGui::BeginCombo(label.c_str(), preview.c_str(), flags))
    {
        int cur_ix = -1;
        for (int i = 0; i < displays_.getNumActiveDisplays(); i++)
        {
            cur_ix = displays_.getNextActiveDisplayIx(cur_ix);

            const bool in_vec = (displays_.getDisplayId(cur_ix) == role_vec.at(num));
            std::string disp_name = displays_.getDisplayInternalName(cur_ix);
            if (ImGui::Selectable(disp_name.c_str(), in_vec)) {
                role_vec[num] = displays_.getDisplayId(cur_ix);
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (in_vec) {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }
}

void Layout::drawDraggableRing()
{
    uint disps_num = displays_.getNumActiveDisplays();
    if (ImGui::ListBoxHeader("Display\nOrder", disps_num)) {

        for (int i = 0; i < displays_.size(); i++) {
            if (!displays_.isDisplayActive(i)) {
                continue;
            }

            if (isItemInVector(displays_.getDisplayId(i), primary_displays_)) {
                enableDisplayStyle(LayoutDisplayRole::Primary);
                ImGui::Selectable(displays_.getDisplayInternalName(i).c_str(), true);
                disableDisplayStyle();
            }
            else if(isItemInVector(displays_.getDisplayId(i), secondary_displays_)) {
                enableDisplayStyle(LayoutDisplayRole::Secondary);
                ImGui::Selectable(displays_.getDisplayInternalName(i).c_str(), true);
                disableDisplayStyle();
            }
            else {
                ImGui::Selectable(displays_.getDisplayInternalName(i).c_str());
            }

            if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
                int next = i + (ImGui::GetMouseDragDelta(0).y < 0.f ? -1 : 1);
                if (next >= 0 && next < displays_.size())
                {
                    displays_.swapDisplays(i, next);
                    ImGui::ResetMouseDragDelta();
                }
            }
        }

        ImGui::ListBoxFooter();
    }
}

void Layout::displayPrimaryWindows() const
{
    ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImVec2 work_size(main_viewport->GetWorkSize());
    uint num_displays(prim_img_ids_.size());

    bool displays_even(num_displays % 2 == 0); // Even # of displays?
    uint half_num_displays(std::ceil(num_displays / 2.0)); // Account for horizontal split
    uint vert_slices(half_num_displays > 3 ? 3 : half_num_displays); // Clamp vertical slices
    float width_split(work_size.x / (float)vert_slices);
    float height_split(work_size.y / (num_displays > 1 ? 2.0 : 1.0));

    for (int i = 0; i < num_displays; i++) {
        ImGuiWindowFlags win_flags = 0;
        win_flags |= ImGuiWindowFlags_NoDecoration;
        win_flags |= ImGuiWindowFlags_NoInputs;
        win_flags |= ImGuiWindowFlags_NoSavedSettings;
        win_flags |= ImGuiWindowFlags_NoMove;
        win_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus; // Otherwise, it overlays everything

        // TODO: Handle more than 6 (3 slices * 2 halves) displays

        float width_pos, height_pos, width_pad;
        if (displays_even) {
            width_pad = 0.0;
        }
        else {
            // Add padding to bottom windows if number of displays is odd
            width_pad = (width_split / 2.0) * (i % 2);
        }
        // Move further to the right every other display
        width_pos = (width_split * std::floor(i/2.0)) + width_pad;
        // Jump back and forth b/w top and bottom halves
        height_pos = height_split * (i % 2);
        ImGui::SetNextWindowPos(ImVec2(width_pos, height_pos), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(width_split, height_split));

        // Maintain the right aspect ratio
        const DisplayInfo &disp_info(displays_.getDisplayInfoById(primary_displays_.at(i)));
        float aspect_ratio = (float)disp_info.dimensions.width / disp_info.dimensions.height;

        ImVec2 padding{5, 5};

        float img_width = width_split - (2*padding.x);
        float img_height = img_width * (1.0/aspect_ratio);

        if (img_height > height_split) {
            // If converted height is too large, convert width instead
            img_height = height_split - (2*padding.y);
            img_width = img_height * aspect_ratio;
        }

        std::string menu_name("Primary Display##" + i);
        if (startMenu(menu_name, win_flags)) {
            // Center the image on the window
            ImVec2 image_pos = ImVec2{(ImGui::GetWindowSize().x - img_width) * 0.5f, 
                                        (ImGui::GetWindowSize().y - img_height) * 0.5f};
            ImGui::SetCursorPos(image_pos);

            ImGui::Image(reinterpret_cast<ImTextureID>(prim_img_ids_.at(i)), ImVec2 {img_width, img_height});
            
            // TODO: Fix this for split screen
            // Show camera external name on top of image
            ImGui::SetCursorPos({image_pos.x + 10, image_pos.y + 5});
            const std::string &title(displays_.getDisplayExternalNameById(primary_displays_.at(i)));
            ImVec4 label_color(ImVec4(0.9, 0.2, 0.9, 1.0));
            ImGui::TextColored(label_color, title.c_str());

            endMenu();
        }
    }
}

void Layout::displayPiPWindow(int width, int height, uint pip_id) const
{
    ImVec2 offset{150, 150};

    std::string title = displays_.getDisplayExternalNameById(secondary_displays_.at(0));
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

void Layout::drawCarouselMenu() const
{

}


// --- Private ---

std::vector<uint> &Layout::getDisplaysVectorFromRole(LayoutDisplayRole role)
{
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            return primary_displays_;
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            return secondary_displays_;
        }   break;
    }

    return primary_displays_;
}


} // namespace viewpoint_interface