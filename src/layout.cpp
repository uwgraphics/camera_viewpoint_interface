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
    if (index >= display_states_.getNumActiveDisplays()) {
        return;
    }

    display_states_.setActiveFrameByIndex(index);
}

const std::vector<float>& Layout::getActiveDisplayMatrix() const
{
    if (display_states_.empty()) {
        return kDummyMatrix;
    }

    return displays_.getDisplayMatrixById(display_states_.getActiveFrameDisplayId());
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
                    display_states_.toPrevActiveFrame();
                }
                else {
                    display_states_.toNextActiveFrame();
                }
            }   break;

            default:
            {
            } break;
        }
    }
}

const LayoutCommand Layout::translateStringInputToCommand(std::string input) const
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
    else if (input == "toggle") {
        return LayoutCommand::TOGGLE;
    }
    else if (input == "active_next") {
        return LayoutCommand::ACTIVE_FRAME_NEXT;
    }
    else if (input == "active_prev") {
        return LayoutCommand::ACTIVE_FRAME_PREV;
    }
    else if (input == "active_up") {
        return LayoutCommand::ACTIVE_FRAME_UP;
    }
    else if (input == "active_down") {
        return LayoutCommand::ACTIVE_FRAME_DOWN;
    }
    else if (input == "active_left") {
        return LayoutCommand::ACTIVE_FRAME_LEFT; 
    }
    else if (input == "active_right") {
        return LayoutCommand::ACTIVE_FRAME_RIGHT;       
    }

    return LayoutCommand::INVALID_COMMAND;
}

void Layout::handleStringInput(std::string input)
{
    LayoutCommand command(translateStringInputToCommand(input));

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
            display_states_.toNextActiveFrame();
        }   break;

        case LayoutCommand::ACTIVE_FRAME_PREV:
        {
            display_states_.toPrevActiveFrame();
        }   break;

        case LayoutCommand::ACTIVE_FRAME_UP:
        case LayoutCommand::ACTIVE_FRAME_DOWN:
        case LayoutCommand::ACTIVE_FRAME_LEFT:
        case LayoutCommand::ACTIVE_FRAME_RIGHT:
        {
            display_states_.handleActiveFrameDirectionInput(command);
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
        display_states_.addImageResponseForId(response.getDisplayId(), response.getGLId());
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

void Layout::setNumDisplaysForRole(int num, LayoutDisplayRole role)
{
    display_states_.setNumDisplaysForRole(num, role);
}

void Layout::addDisplayByIxAndRole(uint ix, LayoutDisplayRole role)
{
    // uint id(displays_.getDisplayId(ix));
    // switch (role)
    // {
    //     case LayoutDisplayRole::Primary:
    //     {    
    //         addPrimaryDisplayById(id);
    //     }   break;

    //     case LayoutDisplayRole::Secondary:
    //     {    
    //         addSecondaryDisplayById(id);
    //     }   break;
    // }
}

// void Layout::addPrimaryDisplayById(uint id)
// {
//     display_states_.addPrimaryDisplay(id);
// }

// void Layout::addSecondaryDisplayById(uint id)
// {
//     display_states_.addSecondaryDisplay(id);
// }

void Layout::toNextDisplay(LayoutDisplayRole role)
{
    display_states_.toNextDisplay(role);
}

void Layout::toPrevDisplay(LayoutDisplayRole role)
{
    display_states_.toPrevDisplay(role);
}

void Layout::toNextDisplayWithPush(LayoutDisplayRole role)
{
    // TODO: These should push secondaries as well
    display_states_.toNextDisplay(role);
}

void Layout::toPrevDisplayWithPush(LayoutDisplayRole role)
{
    // TODO:
    display_states_.toPrevDisplay(role);
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
    auto it(display_states_.loopStart());
    for (; it != display_states_.loopEnd(); ++it) {
        uint disp_id(it->first);
        std::vector<uchar>& disp_data(displays_.getDisplayDataById(disp_id));
        const DisplayInfo& disp_info(displays_.getDisplayInfoById(disp_id));
        addImageRequestToQueue(DisplayImageRequest{disp_info.dimensions.width, disp_info.dimensions.height,
                disp_data, disp_id});
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

void Layout::drawDisplaysList(uint keep_active_num)
{
    int total_displays(display_states_.size());
    // If a limit is set, displays can only be manually actived (the least recently touched
    // display is automatically deactivated). If there is no limit, the displays can be
    // toggled on demand.
    bool active_limited(keep_active_num > 0);
    
    // The for-loop below only runs once, at layout initialization
    static bool cache_init(false);
    uint active_num(display_states_.getNumActiveDisplays());
    if (!cache_init && keep_active_num > 0 && active_num > keep_active_num) {
        uint excess_num(active_num - keep_active_num);
        for (int i(0); i < excess_num; ++i) {
            display_states_.deactivateDisplay(display_states_.getCacheBack());
        }

        display_states_.setActiveLimit(keep_active_num);
        cache_init = true;
    }

    const int max_items(5); // Max number of displays to list w/o scrolling
    int opt_shown(total_displays > max_items ? max_items : total_displays);
    if (ImGui::ListBoxHeader("Available\nDisplays", total_displays, opt_shown)) {
        auto it(display_states_.loopStart());
        for (; it != display_states_.loopEnd(); ++it) {
            uint id(it->first);
            bool active(display_states_.isDisplayActive(id));

            if (ImGui::Selectable(displays_.getDisplayInternalNameById(id).c_str(), active)) {
                if (active && !active_limited) {
                    display_states_.deactivateDisplay(id);
                }
                else if (!active) {
                    display_states_.activateDisplay(id);
                }
            }
        }

        ImGui::ListBoxFooter();
    }

    ImGui::Separator();
}

void Layout::drawDisplaySelectors()
{
    DisplayRing& ring(display_states_.getDisplayRing());

    static ImGuiComboFlags flags(0);
    flags |= ImGuiComboFlags_PopupAlignLeft;
    if (display_states_.getPrimaryLimitNum() != -1) {
        std::vector<uint> prim_list(ring.getDisplayRoleList(LayoutDisplayRole::Primary));
        for (int i(0); i < prim_list.size(); ++i) {
            std::string label("Primary Display " + std::to_string(i));
            int cur_prim_id(prim_list.at(i));
            std::string preview(displays_.getDisplayInternalNameById(cur_prim_id));
            if (ImGui::BeginCombo(label.c_str(), preview.c_str(), flags))
            {
                auto it(ring.loopStart());
                for (; it != ring.loopEnd(); ++it) {
                    uint id_to_list(*it);
                    bool cur_active_id(id_to_list == cur_prim_id);
                    if (ring.isPrimaryDisplay(id_to_list) && !cur_active_id) {
                        continue;
                    }

                    std::string disp_name(displays_.getDisplayInternalNameById(id_to_list));
                    if (ImGui::Selectable(disp_name.c_str(), cur_active_id) && !cur_active_id) {
                        ring.unsetDisplayRole(cur_prim_id, LayoutDisplayRole::Primary);
                        ring.setDisplayRole(id_to_list, LayoutDisplayRole::Primary);

                        if (ring.getActiveFrameDisplayId() == cur_prim_id) {
                            ring.setActiveFrameById(id_to_list);
                        }
                    }

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (cur_active_id) {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }
        }
    }

    if (display_states_.getSecondaryLimitNum() != -1) {
        std::vector<uint> sec_list(ring.getDisplayRoleList(LayoutDisplayRole::Secondary));
        for (int i(0); i < sec_list.size(); ++i) {
            std::string label("Secondary Display " + std::to_string(i));
            int cur_sec_id(sec_list.at(i));
            std::string preview(displays_.getDisplayInternalNameById(cur_sec_id));
            if (ImGui::BeginCombo(label.c_str(), preview.c_str(), flags))
            {
                auto it(ring.loopStart());
                for (; it != ring.loopEnd(); ++it) {
                    uint id_to_list(*it);
                    bool cur_active_id(id_to_list == cur_sec_id);
                    if (ring.isSecondaryDisplay(id_to_list) && !cur_active_id) {
                        continue;
                    }

                    std::string disp_name(displays_.getDisplayInternalNameById(id_to_list));
                    if (ImGui::Selectable(disp_name.c_str(), cur_active_id) && !cur_active_id) {
                        ring.unsetDisplayRole(cur_sec_id, LayoutDisplayRole::Secondary);
                        ring.setDisplayRole(id_to_list, LayoutDisplayRole::Secondary);
                    }

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (cur_active_id) {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }
        }
    }
}

void Layout::drawDraggableRing()
{
    uint num_displays(display_states_.getNumActiveDisplays());
    if (ImGui::ListBoxHeader("Display Ring", num_displays)) {
        DisplayRing& ring(display_states_.getDisplayRing());
        auto it(ring.loopStart());
        for (; it != ring.loopEnd(); ++it) {
            uint id(*it);
            bool has_role(false);
            if (ring.isPrimaryDisplay(id)) {
                enableDisplayStyle(LayoutDisplayRole::Primary);
                has_role = true;
            }
            else if (ring.isSecondaryDisplay(id)) {
                enableDisplayStyle(LayoutDisplayRole::Secondary);
                has_role = true;
            }

            ImGui::Selectable(displays_.getDisplayInternalNameById(id).c_str(), true);                
            
            if (has_role) {
                disableDisplayStyle();
            }

            if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
                int delta(ImGui::GetMouseDragDelta(0).y);
                if (std::abs(delta) > 1e-5) {
                    ring.swapDisplays(id, delta);
                }
            }

            if (ImGui::IsItemHovered()) {
                ImGui::ResetMouseDragDelta();  
            }
        }

        ImGui::ListBoxFooter();
    }

    drawDisplaySelectors();
}


} // viewpoint_interface