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

void Layout::setActiveFrame(uint index)
{
    if (index >= primary_ring_.size()) {
        return;
    }

    primary_ring_.setActiveFrameByIndex(index);
}

const std::vector<float>& Layout::getActiveDisplayMatrix() const
{
    if (primary_ring_.empty()) {
        return kDummyMatrix;
    }

    return displays_.getDisplayMatrixById(primary_ring_.getActiveFrameDisplayId());
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
                toNextDisplay(LayoutDisplayRole::Primary);
            }   break;

            case GLFW_KEY_LEFT:
            {
                toPrevDisplay(LayoutDisplayRole::Primary);
            }   break;

            case GLFW_KEY_UP:
            {
                toPrevDisplay(LayoutDisplayRole::Secondary);
            }   break;

            case GLFW_KEY_DOWN:
            {
                toNextDisplay(LayoutDisplayRole::Secondary);
            }   break;

            case GLFW_KEY_TAB:
            {
                if (mods && GLFW_MOD_SHIFT) {
                    primary_ring_.toPrevActiveFrame();
                }
                else {
                    primary_ring_.toNextActiveFrame();
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
            toNextDisplay(LayoutDisplayRole::Primary);
        }   break;

        case LayoutCommand::PRIMARY_PREV:
        {
            toPrevDisplay(LayoutDisplayRole::Primary);
        }   break;

        case LayoutCommand::SECONDARY_NEXT:
        {
            toNextDisplay(LayoutDisplayRole::Secondary);
        }   break;

        case LayoutCommand::SECONDARY_PREV:
        {
            toPrevDisplay(LayoutDisplayRole::Secondary);
        }   break;

        case LayoutCommand::ACTIVE_FRAME_NEXT:
        {
            primary_ring_.toNextActiveFrame();
        }   break;

        case LayoutCommand::ACTIVE_FRAME_PREV:
        {
            primary_ring_.toPrevActiveFrame();
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

        switch (response.getRole())
        {
            case LayoutDisplayRole::Primary:
            {
                primary_ring_.addImageResponseForId(response.getDisplayId(), response.getGLId());
            }   break;

            case LayoutDisplayRole::Secondary:
            {
                secondary_ring_.addImageResponseForId(response.getDisplayId(), response.getGLId());
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
    primary_ring_.addDisplayId(id);
}

void Layout::addSecondaryDisplayById(uint id)
{
    secondary_ring_.addDisplayId(id);
}

void Layout::activateDisplayAtIx(uint ix)
{
    displays_.activateDisplay(ix);
    primary_ring_.markDisplayActive(displays_.getDisplayId(ix));
    secondary_ring_.markDisplayActive(displays_.getDisplayId(ix));
}

void Layout::deactivateDisplayAtIx(uint ix)
{
    displays_.deactivateDisplay(ix);

    uint display_id(displays_.getDisplayId(ix));
    if (primary_ring_.isDisplayInRing(display_id)) {
        toNextDisplay(LayoutDisplayRole::Primary);
    }
    if (secondary_ring_.isDisplayInRing(display_id)) {
        toNextDisplay(LayoutDisplayRole::Secondary);
    }

    primary_ring_.markDisplayInactive(display_id);
    secondary_ring_.markDisplayInactive(display_id);
}

void Layout::toNextDisplay(LayoutDisplayRole role)
{
    DisplayRing &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() == 0) {
        return;
    }

    uint cur_ix(displays_.getDisplayIxById(role_vec.getActiveFrameDisplayId()));
    uint next_ix(displays_.getNextActiveDisplayIx(cur_ix));

    if (cur_ix == next_ix) {
        return;
    }

    role_vec.switchActiveDisplay(displays_.getDisplayId(next_ix));
}

void Layout::toNextDisplayWithPush(LayoutDisplayRole role)
{
    DisplayRing &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() == 0) {
        return;
    }

    for (int i(0); i < role_vec.size(); ++i) {
        uint cur_ix(displays_.getDisplayIxById(role_vec.getDisplayIdAt(i)));
        uint next_ix(displays_.getNextActiveDisplayIx(cur_ix));

        role_vec.switchDisplayAtIx(i, displays_.getDisplayId(next_ix));
    }
}

void Layout::toPrevDisplayWithPush(LayoutDisplayRole role)
{
    DisplayRing &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() == 0) {
        return;
    }

    for (int i(0); i < role_vec.size(); ++i) {
        uint cur_ix(displays_.getDisplayIxById(role_vec.getDisplayIdAt(i)));
        uint prev_ix(displays_.getPrevActiveDisplayIx(cur_ix));

        role_vec.switchDisplayAtIx(i, displays_.getDisplayId(prev_ix));
    }
}

void Layout::toPrevDisplay(LayoutDisplayRole role)
{
    DisplayRing &role_vec(getDisplaysVectorFromRole(role));

    if (role_vec.size() == 0) {
        return;
    }

    uint cur_ix(displays_.getDisplayIxById(role_vec.getActiveFrameDisplayId()));
    uint prev_ix(displays_.getPrevActiveDisplayIx(cur_ix));

    if (cur_ix == prev_ix) {
        return;
    }

    role_vec.switchActiveDisplay(displays_.getDisplayId(prev_ix));
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
    for (LayoutComponent& component : layout_components_) {
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
    for (LayoutComponent& component : layout_components_) {
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
    for (LayoutComponent& component : layout_components_) {
        std::vector<float> cur_bounds(component.getDisplayBounds());
        for (int j(0); j < cur_bounds.size(); ++j) {
            bounds.push_back(cur_bounds[j]);
        }

        component.draw();
    }
    display_bounds_ = bounds;

    // Clean up and prepare for next frame
    for (int i(0); i < primary_ring_.size(); ++i) {
        uint prim_id(primary_ring_.getDisplayIdAt(i));
        std::vector<uchar>& prim_data(displays_.getDisplayDataById(prim_id));
        const DisplayInfo& prim_info(displays_.getDisplayInfoById(prim_id));
        addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                prim_data, prim_id, LayoutDisplayRole::Primary});
    }

    for (int i(0); i < secondary_ring_.size(); ++i) {
        uint sec_id(secondary_ring_.getDisplayIdAt(i));
        std::vector<uchar>& sec_data(displays_.getDisplayDataById(sec_id));
        const DisplayInfo& sec_info(displays_.getDisplayInfoById(sec_id));
        addImageRequestToQueue(DisplayImageRequest{sec_info.dimensions.width, sec_info.dimensions.height,
                sec_data, sec_id, LayoutDisplayRole::Secondary});
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
            deactivateDisplayAtIx(ix);

            still_active_ixs.pop_front();
            --num_active_displays;
        }
        while (num_active_displays > display_limit) {
            // Draw down the newly-activated displays
            // NOTE: This should probably never be necessary since there is probably no more 
            // than one display activated since the previous loop. There is also no check on
            // the list size since an empty list would be problematic either way.
            uint ix(newly_active_ixs.front());
            deactivateDisplayAtIx(ix);

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
                    deactivateDisplayAtIx(i);
                }
                else if (!displays_.isDisplayIxActive(i)) {
                    activateDisplayAtIx(i);
                }
            }
        }

        ImGui::ListBoxFooter();
    }

    ImGui::Separator();
}

void Layout::drawDisplaySelector(uint num, std::string title, LayoutDisplayRole role)
{
    DisplayRing &role_vec(getDisplaysVectorFromRole(role));
    if (role_vec.size() <= num) {
        return;
    }

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

    static ImGuiComboFlags flags(0);
    flags |= ImGuiComboFlags_PopupAlignLeft;

    uint display_id(role_vec.getDisplayIdAt(num));
    std::string preview(displays_.getDisplayInternalNameById(display_id));
    if (ImGui::BeginCombo(label.c_str(), preview.c_str(), flags))
    {
        int cur_ix(-1);
        for (int i(0); i < displays_.getNumActiveDisplays(); ++i)
        {
            cur_ix = displays_.getNextActiveDisplayIx(cur_ix);

            const bool in_vec(displays_.getDisplayId(cur_ix) == display_id);
            std::string disp_name(displays_.getDisplayInternalName(cur_ix));
            if (ImGui::Selectable(disp_name.c_str(), in_vec)) {
                role_vec.switchDisplayAtIx(num, displays_.getDisplayId(cur_ix));
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
    // Primary displays
    uint prim_num(primary_ring_.size());
    if (ImGui::ListBoxHeader("Primary", prim_num)) {
        for (int i(0); i < prim_num; ++i) {
            uint id(primary_ring_.getDisplayIdAt(i));
            if (!displays_.isDisplayIdActive(id)) {
                continue;
            }

            enableDisplayStyle(LayoutDisplayRole::Primary);
            ImGui::Selectable(displays_.getDisplayInternalNameById(id).c_str(), true);
            disableDisplayStyle();

            if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
                int delta(ImGui::GetMouseDragDelta(0).y);
                if (std::abs(delta) > 1e-5) {
                    primary_ring_.swapDisplayPositionsInRing(id, delta);
                    ImGui::ResetMouseDragDelta();
                }
            }
        }

        ImGui::ListBoxFooter();
    }

    // Secondary displays
    uint sec_num(secondary_ring_.size());
    if (sec_num == 0) {
        return;
    }
    if (ImGui::ListBoxHeader("Secondary", sec_num)) {
        for (int i(0); i < sec_num; ++i) {
            uint id(secondary_ring_.getDisplayIdAt(i));
            if (!displays_.isDisplayIdActive(id)) {
                continue;
            }

            enableDisplayStyle(LayoutDisplayRole::Secondary);
            ImGui::Selectable(displays_.getDisplayInternalNameById(id).c_str(), true);
            disableDisplayStyle();

            if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
                int delta(ImGui::GetMouseDragDelta(0).y);
                if (std::abs(delta) > 1e-5) {
                    secondary_ring_.swapDisplayPositionsInRing(id, delta);
                    ImGui::ResetMouseDragDelta();
                }
            }
        }

        ImGui::ListBoxFooter();
    }
}


// --- Private ---

DisplayRing& Layout::getDisplaysVectorFromRole(LayoutDisplayRole role)
{
    switch (role)
    {
        case LayoutDisplayRole::Primary:
        {
            return primary_ring_;
        }   break;

        case LayoutDisplayRole::Secondary:
        {
            return secondary_ring_;
        }   break;
    }

    return primary_ring_; // Not expected to hit this
}


} // viewpoint_interface