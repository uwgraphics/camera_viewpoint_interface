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

void Layout::setActiveWindow(uint index)
{
    if (index >= getPrimaryDisplayCount(false)) {
        return;
    }

    active_window_ix_ = index;
}

const std::vector<float>& Layout::getActiveDisplayMatrix() const
{
    if (primary_displays_.empty()) {
        return kDummyMatrix;
    }

    uint active_id(primary_displays_[active_window_ix_]);

    return displays_.getDisplayMatrixById(active_id);
}

const std::vector<float> Layout::getDisplayBounds() const
{
    std::vector<float> bounds;
    ImGuiViewport* main_viewport(ImGui::GetMainViewport());

    float x_pos, y_pos, width, height;
    uint total_displays(getPrimaryDisplayCount(false));
    for (uint i = 0; i < total_displays; ++i) {
        getDisplayPositionAndSize(i, total_displays, x_pos, y_pos, width, height);
        
        // Top left corner
        bounds.push_back(x_pos); bounds.push_back(y_pos);
        
        // Bottom right corner
        bounds.push_back(x_pos + width); bounds.push_back(y_pos + height);
    }

    return bounds;
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
            }   break;

            case GLFW_KEY_LEFT:
            {
                toPrevDisplay(0, LayoutDisplayRole::Primary);
            }   break;

            case GLFW_KEY_UP:
            {
                toPrevDisplay(0, LayoutDisplayRole::Secondary);
            }   break;

            case GLFW_KEY_DOWN:
            {
                toNextDisplay(0, LayoutDisplayRole::Secondary);
            }   break;

            case GLFW_KEY_TAB:
            {
                if (mods && GLFW_MOD_SHIFT) {
                    toPrevActiveWindow();
                }
                else {
                    toNextActiveWindow();
                }
            }   break;

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

        case LayoutCommand::ACTIVE_WINDOW_NEXT:
        {
            toNextActiveWindow();
        }   break;

        case LayoutCommand::ACTIVE_WINDOW_PREV:
        {
            toPrevActiveWindow();
        }   break;

        default:
        {

        }   break;
    }
}

void Layout::handleCollisionMessage(const std::string &message)
{
    // TODO: Implement this
}


// --- Protected ---

void Layout::handleImageResponse()
{
    for (int i = 0; i < image_response_queue_.size(); ++i) {
        DisplayImageResponse &response(image_response_queue_.at(i));

        switch (response.role)
        {
            case LayoutDisplayRole::Primary:
            {
                primary_img_ids_[response.index] = response.id;
            }   break;

            case LayoutDisplayRole::Secondary:
            {
                secondary_img_ids_[response.index] = response.id;
            }   break;
        }
    }
}

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
    ImGuiStyle& style(ImGui::GetStyle());
    style.Colors[ImGuiCol_Header] = color_cache_.base;
    style.Colors[ImGuiCol_HeaderHovered] = color_cache_.hovered;
    style.Colors[ImGuiCol_HeaderActive] = color_cache_.active;
}

uint Layout::getPrimaryDisplayCount(bool include_inactive) const
{
    if (include_inactive) {
        return primary_displays_.size();
    }

    uint count(0);
    for (int i = 0; i < primary_displays_.size(); ++i) {
        if (displays_.isDisplayIdActive(primary_displays_[i])) {
            ++count;
        }
    }
    return count;
}

void Layout::addDisplayByIxAndRole(uint ix, LayoutDisplayRole role)
{
    uint id(displays_.getDisplayId(ix));
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {    
            addPrimaryDisplayById(id);
        }   break;

        case LayoutDisplayRole::Secondary:
        {    
            addSecondaryDisplayById(id);
        }   break;
    }
}

void Layout::addPrimaryDisplayById(uint id)
{
    primary_displays_.push_back(id);
    primary_img_ids_.resize(primary_displays_.size());
}

void Layout::addSecondaryDisplayById(uint id)
{
    secondary_displays_.push_back(id);
    secondary_img_ids_.resize(secondary_displays_.size());
}

void Layout::toNextDisplay(uint vec_ix, LayoutDisplayRole role)
{
    std::vector<uint> &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() <= vec_ix) {
        return;
    }

    uint cur_ix(displays_.getDisplayIxById(role_vec.at(vec_ix)));
    uint next_ix(displays_.getNextActiveDisplayIx(cur_ix));
    role_vec[vec_ix] = displays_.getDisplayId(next_ix);
}

void Layout::toPrevDisplay(uint vec_ix, LayoutDisplayRole role)
{
    std::vector<uint> &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() <= vec_ix) {
        return;
    }

    uint cur_ix(displays_.getDisplayIxById(role_vec.at(vec_ix)));
    uint prev_ix(displays_.getPrevActiveDisplayIx(cur_ix));
    role_vec[vec_ix] = displays_.getDisplayId(prev_ix);
}

void Layout::toNextActiveWindow()
{
    uint window_count(getPrimaryDisplayCount(false));

    if (window_count == 0) { return; }

    ++active_window_ix_ %= window_count;
}

void Layout::toPrevActiveWindow()
{
    uint window_count(getPrimaryDisplayCount(false));

    if (window_count == 0) { return; }

    if (active_window_ix_ == 0) {
        active_window_ix_ = window_count - 1;
    }
    else {
        --active_window_ix_;
    }
}

void Layout::addImageRequestToQueue(DisplayImageRequest request)
{
    display_image_queue_.push_back(request);
}

void Layout::addLayoutComponent(LayoutComponent::Type type, LayoutComponent::Spacing spacing,
        LayoutComponent::Positioning positioning, float width, float height, ImVec2 offset)
{
    layout_components_.push_back(LayoutComponent(*this, type, spacing, positioning, width, height,
        offset));
}

void displayWarningMessage(std::string message)
{
    ImGuiWindowFlags win_flags(0);
    win_flags |= ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs;
    win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
    ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + 50, 
            main_viewport->GetWorkPos().y + 20), ImGuiCond_Always);

    if (startMenu("Warnings", win_flags)) {
        ImVec4 warning_color(1.0f, 0.0f, 0.0f, 1.0f);
        ImGui::TextColored(warning_color, message.c_str());
        endMenu();
    }
}

void Layout::drawLayoutComponents()
{
    handleImageResponse();

    // Check that there is exactly one primary window (which could contain
    // multiple displays)
    LayoutComponent *primary_window;
    bool primary_found(false);
    for (LayoutComponent &component : layout_components_) {
        if (component.getType() == LayoutComponent::Type::Primary) {
            if (primary_found) {
                displayWarningMessage("Multiple primary windows specified.");
                return;
            }
            primary_found = true;
            primary_window = &component;
        }
    }
    if (!primary_found) {
        displayWarningMessage("No primary window specified.");
        return;
    }

    // Initial parameters for work area
    ImGuiViewport* main_viewport(ImGui::GetMainViewport());
    ImVec2 work_size(main_viewport->GetWorkSize());
    ImVec2 work_offset(0.0, 0.0);

    // TODO/NOTE: If new components are added that may conflict in 
    // positioning/spacing, will need to add another loop to validate first

    // Adjust parameters based on other components
    for (LayoutComponent &component : layout_components_) {
        switch (component.getType())
        {
            case LayoutComponent::Type::Carousel:
            {
                if (component.getSpacing() == LayoutComponent::Spacing::Horizontal) {
                    float new_height(work_size.y - component.getHeight());
                    work_size.y = new_height;
                    component.setWidth(work_size.x);

                    if (component.getPositioning() == LayoutComponent::Positioning::Top) {
                        component.setOffset(ImVec2{0.0, 0.0});
                        work_offset.y = component.getHeight();
                    }
                    else { // Bottom
                        float new_offset(main_viewport->GetWorkSize().y - component.getHeight());
                        component.setOffset(ImVec2{0.0, new_offset});
                    }
                }
                else { // Vertical
                    float new_width(work_size.x - component.getWidth());
                    work_size.x = new_width;
                    component.setHeight(work_size.y);

                    if (component.getPositioning() == LayoutComponent::Positioning::Left) {
                        component.setOffset(ImVec2{0.0, 0.0});
                        work_offset.x = component.getWidth();
                    }
                    else { // Right
                        float new_offset(main_viewport->GetWorkSize().x - component.getWidth());
                        component.setOffset(ImVec2{new_offset, 0.0});
                    }
                }
            }   break;

            default:
            {
            }   break;
        }
    }

    primary_window->setWidth(work_size.x);
    primary_window->setHeight(work_size.y);
    primary_window->setOffset(work_offset);

    // Draw all components
    for (LayoutComponent &component : layout_components_) {
        component.draw();
    }

    // Clean up and prepare for next frame
    for (int i = 0; i < primary_displays_.size(); ++i) {
        std::vector<uchar> &prim_data(displays_.getDisplayDataById(primary_displays_.at(i)));
        const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(i)));
        addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                prim_data, (uint)0, LayoutDisplayRole::Primary});
    } 

    for (int i = 0; i < secondary_displays_.size(); ++i) {
            std::vector<uchar> &sec_data(displays_.getDisplayDataById(secondary_displays_.at(i)));
            const DisplayInfo &sec_info(displays_.getDisplayInfoById(secondary_displays_.at(i)));
            addImageRequestToQueue(DisplayImageRequest{sec_info.dimensions.width, sec_info.dimensions.height,
                    sec_data, (uint)i, LayoutDisplayRole::Secondary});
    }

    layout_components_.clear();
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
    ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + 25, 
            main_viewport->GetWorkPos().y + 40), ImGuiCond_Always);

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
            bool active = displays_.isDisplayIxActive(i);

            if (ImGui::Selectable(displays_.getDisplayInternalName(i).c_str(), active)) {
                if (!displays_.isDisplayIxActive(i) || displays_.getNumActiveDisplays() > 1) {
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

    std::string preview(displays_.getDisplayInternalNameById(role_vec.at(num)));
    if (ImGui::BeginCombo(label.c_str(), preview.c_str(), flags))
    {
        int cur_ix = -1;
        for (int i = 0; i < displays_.getNumActiveDisplays(); ++i)
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
            if (!displays_.isDisplayIxActive(i)) {
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

void Layout::getDisplayPositionAndSize(uint cur_display, uint num_displays, float &x_pos, float &y_pos, 
        float &width, float &height) const
{
    const static uint max_vertical_slices(4); // Note: This is half of total possible displays b/c of horizontal slicing
    
    ImGuiViewport* main_viewport(ImGui::GetMainViewport());
    ImVec2 work_size(main_viewport->GetWorkSize());

    // Calculate slices
    bool displays_even(num_displays % 2 == 0); // Even # of displays?
    uint half_num_displays(std::ceil(num_displays / 2.0)); // Account for horizontal split
    uint vert_slices(half_num_displays > max_vertical_slices ? max_vertical_slices : half_num_displays); // Clamp vertical slices
    
    // Calculate dimensions for each slice
    width = work_size.x / (float)vert_slices;
    height = work_size.y / (num_displays > 1 ? 2.0 : 1.0);

    // TODO: Handle more than [max_vert_slices * 2] displays
    // In particular, add functionality to switch between 'pages' of displays

    float width_pad;
    if (displays_even) {
        width_pad = 0.0;
    }
    else {
        // Add padding to bottom windows if number of displays is odd
        width_pad = (width / 2.0) * (cur_display % 2);
    }
    // Move further to the right every other display
    x_pos = (width * std::floor(cur_display / 2.0)) + width_pad;
    // Jump back and forth b/w top and bottom halves
    y_pos = height * (cur_display % 2);
}

void Layout::displayPrimaryWindows() const
{
    ImGuiStyle& style(ImGui::GetStyle());
    float orig_border_size(style.WindowBorderSize);
    ImVec4 orig_border_color(style.Colors[ImGuiCol_Border]);

    uint num_displays(getPrimaryDisplayCount(false));

    for (int i = 0; i < num_displays; ++i) {
        ImGuiWindowFlags win_flags = 0;
        win_flags |= ImGuiWindowFlags_NoDecoration;
        win_flags |= ImGuiWindowFlags_NoInputs;
        win_flags |= ImGuiWindowFlags_NoSavedSettings;
        win_flags |= ImGuiWindowFlags_NoMove;
        win_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus; // Otherwise, it overlays everything

        float x_pos, y_pos, win_width, win_height;
        getDisplayPositionAndSize(i, num_displays, x_pos, y_pos, win_width, win_height);
        ImGui::SetNextWindowPos(ImVec2(x_pos, y_pos), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(win_width, win_height));

        // Maintain the right aspect ratio
        const DisplayInfo &disp_info(displays_.getDisplayInfoById(primary_displays_.at(i)));
        float aspect_ratio = (float)disp_info.dimensions.width / disp_info.dimensions.height;

        ImVec2 padding{5, 5};

        float img_width = win_width - (2*padding.x);
        float img_height = img_width * (1.0/aspect_ratio);

        if (img_height > win_height) {
            // If converted height is too large, convert width instead
            img_height = win_height - (2*padding.y);
            img_width = img_height * aspect_ratio;
        }

        if (num_displays > 1 && i == active_window_ix_) {
            style.WindowBorderSize = 3;
            style.Colors[ImGuiCol_Border] = kActiveBorderColor;
        }

        std::string menu_name("Primary Display##" + i);
        if (startMenu(menu_name, win_flags)) {
            // Center the image on the window
            ImVec2 image_pos(ImVec2{(ImGui::GetWindowSize().x - img_width) * 0.5f, 
                                        (ImGui::GetWindowSize().y - img_height) * 0.5f});
            ImGui::SetCursorPos(image_pos);

            ImGui::Image(reinterpret_cast<ImTextureID>(primary_img_ids_.at(i)), ImVec2 {img_width, img_height});
            
            // Show camera external name on top of image
            ImGui::SetCursorPos({image_pos.x + 10, image_pos.y + 5});
            const std::string &title(displays_.getDisplayExternalNameById(primary_displays_.at(i)));
            ImGui::Text(title.c_str());

            endMenu();
        }

        if (num_displays > 1 && i == active_window_ix_) {
            style.WindowBorderSize = orig_border_size;
            style.Colors[ImGuiCol_Border] = orig_border_color;
        }
    }
}

void Layout::displayPiPWindow(int width, int height) const
{
    ImVec2 offset{150, 150};

    std::string title = displays_.getDisplayExternalNameById(secondary_displays_.at(0));
    ImGuiWindowFlags win_flags = 0;
    win_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse;
    win_flags |= ImGuiWindowFlags_NoTitleBar;
    win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
    ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + (main_viewport->GetWorkSize().x - width-offset.x), 
            main_viewport->GetWorkPos().y + height-offset.y), ImGuiCond_Once);

    if (startMenu("Picture-in-Picture", win_flags)) {
        ImGui::Text("%s", title.c_str());
        ImGui::Image(reinterpret_cast<ImTextureID>(secondary_img_ids_.at(0)), ImVec2(width, height));
        endMenu();
    }
}

void Layout::displayCarouselRibbon() const
{
    ImVec2 display_size{250, 140};
    uint height_offset(display_size.y + 100);
    
    // Calculate number of displays in carousel and spacing
    const float ribbon_padding(50.0);
    const float display_padding(15.0);
    const uint max_displays(5);

    // The ribbon is displayed as follows:
    // rp = ribbon_padding; dp = display_padding
    //
    //   ~~~~~~~~~~ Top of screen snipped ~~~~~~~~~~
    //
    // |    ------------------------------------
    // |    |  -------  -------  -------       |    |
    // | rp |dp|:::::|dp|:::::|dp|:::::| ~~~ dp| rp |
    // |    |  -------  -------  -------       |    |
    // |    ------------------------------------    |
    // ----------------------------------------------
    
    ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    float ribbon_width(main_viewport->GetWorkSize().x);
    ribbon_width -= ribbon_padding*2; // Padding from ribbon edges of screen

    uint num_displays((ribbon_width - display_padding) / (display_size.x + display_padding));
    if (secondary_displays_.size()-1 < num_displays) {
        num_displays = secondary_displays_.size()-1;
    }
    else {
        num_displays = num_displays > max_displays ? max_displays : num_displays;
    }

    ImGuiWindowFlags win_flags = 0;
    win_flags |= ImGuiWindowFlags_NoDecoration;
    win_flags |= ImGuiWindowFlags_NoInputs;
    win_flags |= ImGuiWindowFlags_NoSavedSettings;
    win_flags |= ImGuiWindowFlags_NoMove;
    ImVec2 ribbon_pos(ImVec2(main_viewport->GetWorkPos().x + ribbon_padding, 
        main_viewport->GetWorkPos().y + (main_viewport->GetWorkSize().y - height_offset)));
    ImGui::SetNextWindowPos(ribbon_pos);
    ImGui::SetNextWindowSize(ImVec2(ribbon_width, display_size.y + (2 * display_padding)));
    
    if (startMenu("Carousel", win_flags)) {
        for (uint i(0), count(0); count < num_displays; ++i) {
            if (secondary_displays_.at(i) == primary_displays_.at(0)) {
                continue;
            }

            ImVec2 image_pos(ImVec2{display_padding + (count * (display_size.x + display_padding)), display_padding});
            ImGui::SetCursorPos(image_pos);

            ImGui::Image(reinterpret_cast<ImTextureID>(secondary_img_ids_.at(i)), ImVec2{display_size.x, display_size.y});
            
            // Show camera external name on top of image
            ImGui::SetCursorPos({image_pos.x + 10, image_pos.y + 5});
            const std::string &title(displays_.getDisplayExternalNameById(secondary_displays_.at(i)));
            ImGui::Text(title.c_str());

            ++count;
        }

        endMenu();
    }
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


} // viewpoint_interface