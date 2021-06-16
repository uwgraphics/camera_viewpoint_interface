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
    return display_bounds_;
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
    for (int i(0); i < image_response_queue_.size(); ++i) {
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

    image_response_queue_.clear();
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
    for (int i(0); i < primary_displays_.size(); ++i) {
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

void Layout::toNextDisplayIfInRole(uint ix, LayoutDisplayRole role)
{
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            int item_ix(getItemIndexInVector(displays_.getDisplayId(ix), primary_displays_));
            if (item_ix != -1) {
                toNextDisplay(item_ix, LayoutDisplayRole::Primary);
            }
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            int item_ix(getItemIndexInVector(displays_.getDisplayId(ix), secondary_displays_));
            if (item_ix != -1) {
                toNextDisplay(item_ix, LayoutDisplayRole::Secondary);
            }
        }   break;    
    }
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

                    if (component.getPositioning() == LayoutComponent::ComponentPositioning_Top) {
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

                    if (component.getPositioning() == LayoutComponent::ComponentPositioning_Left) {
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

    // Update bounds and draw all components
    std::vector<float> bounds;
    for (LayoutComponent &component : layout_components_) {
        std::vector<float> cur_bounds(component.getDisplayBounds());
        for (int j(0); j < cur_bounds.size(); ++j) {
            bounds.push_back(cur_bounds[j]);
        }

        component.draw();
    }
    display_bounds_ = bounds;

    // Clean up and prepare for next frame
    for (int i(0); i < primary_displays_.size(); ++i) {
        std::vector<uchar> &prim_data(displays_.getDisplayDataById(primary_displays_.at(i)));
        const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(i)));
        addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                prim_data, (uint)i, LayoutDisplayRole::Primary});
    }

    for (int i(0); i < secondary_displays_.size(); ++i) {
        std::vector<uchar> &sec_data(displays_.getDisplayDataById(secondary_displays_.at(i)));
        const DisplayInfo &sec_info(displays_.getDisplayInfoById(secondary_displays_.at(i)));
        addImageRequestToQueue(DisplayImageRequest{sec_info.dimensions.width, sec_info.dimensions.height,
                sec_data, (uint)i, LayoutDisplayRole::Secondary});
    }

    layout_components_.clear();
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

void Layout::drawDisplaysList(uint display_limit)
{
    static std::list<uint> active_ixs_cache;
    static bool cache_init(false);
    
    int total_displays(displays_.getNumTotalDisplays());
    
    // Initial population of active display cache
    if (!cache_init) {
        for (uint i(0); i < total_displays; ++i) {
            if (displays_.isDisplayIxActive(i)) {
                active_ixs_cache.emplace_back(i);
            }
        }

        cache_init = true;
    }

    if (display_limit > 0) { // Pare down to active display number limit
        // Identify newly-active ixs
        std::list<uint> newly_active_ixs; // Store ixs of displays activated since last loop
        std::list<uint> still_active_ixs; // Store ixs that stayed active across loops
        for (uint i(0); i < total_displays; ++i) {
            if (displays_.isDisplayIxActive(i)) {
                if (std::find(active_ixs_cache.begin(), active_ixs_cache.end(), i) == active_ixs_cache.end()) {
                    newly_active_ixs.emplace_back(i); // Display was activated since last loop
                }
                else {
                    still_active_ixs.emplace_back(i); // Display remained active
                }
            }
        }

        uint num_active_displays(newly_active_ixs.size() + still_active_ixs.size());
        while (num_active_displays > display_limit && !still_active_ixs.empty()) {
            // Draw down the more 'stale' displays first to get to the limit
            uint ix(still_active_ixs.front());
            displays_.flipDisplayState(ix);

            // Switch to next active display if this one was primary or secondary
            toNextDisplayIfInRole(ix, LayoutDisplayRole::Primary);
            toNextDisplayIfInRole(ix, LayoutDisplayRole::Secondary);

            still_active_ixs.pop_front();
            --num_active_displays;
        }
        while (num_active_displays > display_limit) {
            // Draw down the newly-activated displays
            // NOTE: This should probably never be necessary since there is probably no more 
            // than one display activated since the previous loop. There is also no check on
            // the list size since an empty list would be problematic either way.
            uint ix(newly_active_ixs.front());
            displays_.flipDisplayState(ix);

            // Switch to next active display if this one was primary or secondary
            toNextDisplayIfInRole(ix, LayoutDisplayRole::Primary);
            toNextDisplayIfInRole(ix, LayoutDisplayRole::Secondary);

            newly_active_ixs.pop_front();
            --num_active_displays; 
        }

        // Update active display cache
        active_ixs_cache.clear();
        for (uint ix : still_active_ixs) {
            active_ixs_cache.emplace_back(ix);
        }
        for (uint ix : newly_active_ixs) {
            active_ixs_cache.emplace_back(ix);
        }
    }
    else { // display_limit == 0 indicates no limit on number of active displays
        // Update active display cache
        active_ixs_cache.clear();
        for (uint i(0); i < total_displays; ++i) {
            if (displays_.isDisplayIxActive(i)) {
                active_ixs_cache.emplace_back(i);
            }
        }
    }

    const int max_items(5); // Max number of displays to list w/o scrolling
    int opt_shown(total_displays > max_items ? max_items : total_displays);
    if (ImGui::ListBoxHeader("Available\nDisplays", total_displays, opt_shown)) {
        for (uint i(0); i < total_displays; ++i) {
            bool active(displays_.isDisplayIxActive(i));

            if (ImGui::Selectable(displays_.getDisplayInternalName(i).c_str(), active)) {
                if (displays_.isDisplayIxActive(i) && displays_.getNumActiveDisplays() > 1) {
                    displays_.flipDisplayState(i);

                    // Switch to next active display if this one was primary or secondary,
                    // unless it's the last active display
                    toNextDisplayIfInRole(i, LayoutDisplayRole::Primary);
                    toNextDisplayIfInRole(i, LayoutDisplayRole::Secondary);
                }
                else if (!displays_.isDisplayIxActive(i)) {
                    displays_.flipDisplayState(i);
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

    static ImGuiComboFlags flags(0);
    flags |= ImGuiComboFlags_PopupAlignLeft;

    std::string preview(displays_.getDisplayInternalNameById(role_vec.at(num)));
    if (ImGui::BeginCombo(label.c_str(), preview.c_str(), flags))
    {
        int cur_ix(-1);
        for (int i(0); i < displays_.getNumActiveDisplays(); ++i)
        {
            cur_ix = displays_.getNextActiveDisplayIx(cur_ix);

            const bool in_vec(displays_.getDisplayId(cur_ix) == role_vec.at(num));
            std::string disp_name(displays_.getDisplayInternalName(cur_ix));
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
    uint disps_num(displays_.getNumActiveDisplays());
    if (ImGui::ListBoxHeader("Display\nOrder", disps_num)) {

        for (int i(0); i < displays_.getNumTotalDisplays(); i++) {
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
                if (next >= 0 && next < displays_.getNumTotalDisplays())
                {
                    displays_.swapDisplays(i, next);
                    ImGui::ResetMouseDragDelta();
                }
            }
        }

        ImGui::ListBoxFooter();
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