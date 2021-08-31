#include "viewpoint_interface/layout.hpp"
#include "viewpoint_interface/layout_component.hpp"


namespace viewpoint_interface {

// --- Public ---
const std::vector<float> LayoutComponent::getDisplayBounds() const
{
    std::vector<float> bounds;

    switch (type_)
    {
        case Type::Primary:
        {
            float x_pos, y_pos, width, height;
            uint total_displays(layout_.display_states_.getDisplayRing().getNumPrimaryDisplays());
            for (uint i = 0; i < total_displays; ++i) {
                getPrimaryDisplayPositionAndSize(i, total_displays, x_pos, y_pos, width, height);
                
                // Top left corner
                bounds.push_back(x_pos); bounds.push_back(y_pos);
                
                // Bottom right corner
                bounds.push_back(x_pos + width); bounds.push_back(y_pos + height);
            }
        }   break;
    
        case Type::Pic_In_Pic:
        {
            ImVec2 pos;
            getPiPWindowPosition(pos);

            // Top left corner
            bounds.push_back(pos.x); bounds.push_back(pos.y);

            // Bottom right corner
            bounds.push_back(pos.x + width_); bounds.push_back(pos.y + height_);
        }   break;
    
        case Type::Carousel:
        {
            ImVec2 ribbon_pos;
            std::vector<ImVec4> display_dim_data;
            getCarouselRibbonPosAndDisplaysPosAndSize(ribbon_pos, display_dim_data);

            for (int i(0); i < display_dim_data.size(); ++i) {
                float x_pos(ribbon_pos.x + display_dim_data[i].x);
                float y_pos(ribbon_pos.y + display_dim_data[i].y);
                
                // Top left corner
                bounds.push_back(x_pos);
                bounds.push_back(y_pos);

                // Bottom right corner
                bounds.push_back(x_pos + display_dim_data[i].z);
                bounds.push_back(y_pos + display_dim_data[i].w);
            }
        }   break;
    
        default:
        {
        }   break;
    }

    return bounds;
}

void LayoutComponent::draw()
{
    switch (type_)
    {
        case Type::Primary:
        {
            drawPrimaryWindows();
        }   break;
    
        case Type::Pic_In_Pic:
        {
            drawPiPWindow();
        }   break;

        case Type::Double_PiP:
        {
            drawDoublePiPWindows();
        }   break;
    
        case Type::Carousel:
        {
            drawCarouselRibbon();
        }   break;
    
        default:
        {
        }   break;
        }
}


// --- Private ---
void LayoutComponent::checkParameters()
{
    switch (type_)
    {
        case Type::Primary:
        {
            // NOTE: For Full spacing, the width and height is always the
            // entire work area minus secondary component areas (e.g., 
            // sidebars), so the dimensions and offset will get recalculated
            // every frame

            spacing_ = Spacing::Full;
            positioning_ = ComponentPositioning_Full;
        }   break;
        
        case Type::Pic_In_Pic:
        {
            ImVec2 default_dims{400.0, 225.0};
            ImVec2 default_offset{75.0, 75.0};
            // NOTE: The offset indicates how far to be from the edges of
            // the screen, not the absolute position of the window

            spacing_ = Spacing::Floating;

            if (positioning_ != ComponentPositioning_Top_Left &&
                    positioning_ != ComponentPositioning_Top_Right &&
                    positioning_ != ComponentPositioning_Bottom_Left &&
                    positioning_ != ComponentPositioning_Bottom_Right) {
                if (positioning_ == ComponentPositioning_Bottom) {
                    positioning_ = ComponentPositioning_Bottom_Right;
                }
                else {
                    positioning_ = ComponentPositioning_Top_Right;
                }
            }

            if (width_ <= 1e-5 || height_ <= 1e-5) {
                width_ = default_dims.x;
                height_ = default_dims.y;
            }

            if (offset_.x < 0.0 || offset_.y < 0.0) {
                offset_ = default_offset;
            }
        }   break;
        
        case Type::Double_PiP:
        {
            ImVec2 default_dims{400.0, 225.0};
            ImVec2 default_offset{75.0, 75.0};
            // NOTE: The offset indicates how far to be from the edges of
            // the screen, not the absolute position of the window

            spacing_ = Spacing::Floating;

            if (positioning_ != ComponentPositioning_Left &&
                    positioning_ != ComponentPositioning_Right) {
                positioning_ = ComponentPositioning_Right;
            }

            if (width_ <= 1e-5 || height_ <= 1e-5) {
                width_ = default_dims.x;
                height_ = default_dims.y;
            }

            if (offset_.x < 0.0 || offset_.y < 0.0) {
                offset_ = default_offset;
            }
        }   break;
        
        case Type::Carousel:
        {
            // Takes padding into account--make sure that this matches the
            // display_size + (2 * padding) from the carousel dimensions
            // calculation function specified below
            ImVec2 default_dims{280, 170};

            // NOTE: The carousel is presented as a bar along an edge of
            // the screen, so the offset will be recalculated every frame.
            // The width or height is also recalculated based on the
            // positioning

            if (spacing_ != Spacing::Horizontal &&
                    spacing_ != Spacing::Vertical) {
                spacing_ = Spacing::Horizontal;
            }

            if (spacing_ == Spacing::Horizontal) {
                if (height_ <= 1e-5) {
                    height_ = default_dims.y;
                }

                if (positioning_ != ComponentPositioning_Top &&
                        positioning_ != ComponentPositioning_Bottom) {
                    positioning_ = ComponentPositioning_Bottom;
                }
            }
            else {
                if (width_ <= 1e-5) {
                    width_ = default_dims.x;
                }

                if (positioning_ != ComponentPositioning_Left &&
                        positioning_ != ComponentPositioning_Right) {
                    positioning_ = ComponentPositioning_Right;
                }
            }
        }   break;
        
        default:
        {
        }   break;
    }
}

void LayoutComponent::getPrimaryDisplayPositionAndSize(uint cur_display, uint num_displays, float &x_pos, float &y_pos, 
        float &width, float &height) const
{
    const static uint max_vertical_slices(4); // Note: This is half of total possible displays b/c of horizontal slicing

    // Calculate slices
    bool displays_even(num_displays % 2 == 0); // Even # of displays?
    uint half_num_displays(std::ceil(num_displays / 2.0)); // Account for horizontal split
    uint vert_slices(half_num_displays > max_vertical_slices ? max_vertical_slices : half_num_displays); // Clamp vertical slices
    
    // Calculate dimensions for each slice
    width = width_ / (float)vert_slices;
    height = height_ / (num_displays > 1 ? 2.0 : 1.0);

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

void LayoutComponent::drawPrimaryWindows() const
{
    Layout::DisplayRing& ring(layout_.display_states_.getDisplayRing());
    auto primary_displays(ring.getDisplayRoleList(LayoutDisplayRole::Primary));
    uint num_displays(primary_displays.size());
    
    uint cur_num(0);
    for (uint display_id : primary_displays) {
        ImGuiWindowFlags win_flags = 0;
        win_flags |= ImGuiWindowFlags_NoDecoration;
        win_flags |= ImGuiWindowFlags_NoInputs;
        win_flags |= ImGuiWindowFlags_NoSavedSettings;
        win_flags |= ImGuiWindowFlags_NoMove;
        win_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus; // Otherwise, it overlays everything

        float x_pos, y_pos, win_width, win_height;
        getPrimaryDisplayPositionAndSize(cur_num, num_displays, x_pos, y_pos, win_width, win_height);
        ImGui::SetNextWindowPos(ImVec2(x_pos, y_pos), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(win_width, win_height));

        // Maintain the right aspect ratio
        const DisplayInfo &disp_info(layout_.displays_.getDisplayInfoById(display_id));
        float aspect_ratio((float)disp_info.dimensions.width / disp_info.dimensions.height);

        ImVec2 padding{5, 5};

        float img_width(win_width - (2*padding.x));
        float img_height(img_width * (1.0/aspect_ratio));

        if (img_height > win_height) {
            // If converted height is too large, convert width instead
            img_height = win_height - (2*padding.y);
            img_width = img_height * aspect_ratio;
        }

        bool active_frame(cur_num == ring.getActiveFrameIndex());
        if (num_displays > 1 && active_frame) {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 3.0);
            ImGui::PushStyleColor(ImGuiCol_Border, layout_.kActiveBorderColor);
        }

        std::string menu_name("Primary Display " + std::to_string(cur_num));
        if (startMenu(menu_name, win_flags)) {
            // Center the image on the window
            ImVec2 image_pos(ImVec2{(ImGui::GetWindowSize().x - img_width) * 0.5f, 
                                        (ImGui::GetWindowSize().y - img_height) * 0.5f});
            ImGui::SetCursorPos(image_pos);

            ImGui::Image(reinterpret_cast<ImTextureID>(ring.getImageIdForDisplayId(display_id)), ImVec2 {img_width, img_height});
            
            // Show camera external name on top of image
            ImGui::SetCursorPos({image_pos.x + 10, image_pos.y + 5});
            const std::string &title(layout_.displays_.getDisplayExternalNameById(display_id));
            ImGui::Text(title.c_str());

            endMenu();
        }

        if (num_displays > 1 && active_frame) {
            ImGui::PopStyleVar();
            ImGui::PopStyleColor();
        }

        ++cur_num;
    }
}

void LayoutComponent::getCarouselRibbonPosAndDisplaysPosAndSize(ImVec2 &ribbon_pos, 
        std::vector<ImVec4> &display_dim_data) const
{    
    ImVec2 display_size{250.0, 140.0};

    ImGuiViewport* main_viewport(ImGui::GetMainViewport());
    ImVec2 work_pos(main_viewport->GetWorkPos());
    ribbon_pos = ImVec2(work_pos.x + offset_.x, work_pos.y + offset_.y);

    // Calculate number of displays in carousel
    const float padding(15.0); // Padding b/w displays as well as edges
    uint num_displays(0); // Number of displays that can fit w/in ribbon
    if (spacing_ == Spacing::Horizontal) {
        num_displays = (width_ - padding) / (display_size.x + padding);
    }
    else {
        num_displays = (height_ - padding) / (display_size.y + padding);
    }

    Layout::DisplayRing& ring(layout_.display_states_.getDisplayRing());
    if (ring.getNumSecondaryDisplays() < num_displays) {
        num_displays = ring.getNumSecondaryDisplays();
    }

    // Calculate the display position within the ribbon. In the future, 
    // this could also calculate a variable size for certain displays
    for (int i(0); i < num_displays; ++i) {
        uint spacing_factor(std::ceil(i / 2.0));
        int side(i % 2 == 0 ? -1 : 1); // -1 = left/top, 1 = right/bottom

        // Calculate spacing b/w displays
        float disp_spacing(0);
        if (side == 1) { // Right side
            disp_spacing = (spacing_factor * padding) + ((spacing_factor-1) * display_size.x);
        }
        else if (side == -1 && spacing_factor != 0) {
            disp_spacing = spacing_factor * (padding + display_size.x);
        }

        // Calculate position of display
        ImVec2 display_pos;
        if (spacing_ == Spacing::Vertical) {
            float ribbon_middle(height_ / 2.0);
            display_pos = ImVec2(padding, ribbon_middle + (side * ((display_size.y / 2) + disp_spacing)));
        }
        else {
            float ribbon_middle(width_ / 2.0);
            display_pos = ImVec2(ribbon_middle + (side * ((display_size.x / 2) + disp_spacing)), padding);
        }

        // Add position and size data to display dimensions vector
        display_dim_data.push_back(ImVec4(display_pos.x, display_pos.y, display_size.x, display_size.y));
    }
}

void LayoutComponent::drawCarouselRibbon() const
{
    ImGuiWindowFlags win_flags = 0;
    win_flags |= ImGuiWindowFlags_NoDecoration;
    win_flags |= ImGuiWindowFlags_NoInputs;
    win_flags |= ImGuiWindowFlags_NoSavedSettings;
    win_flags |= ImGuiWindowFlags_NoMove;

    std::vector<ImVec4> display_dim_data;
    ImVec2 ribbon_pos;
    getCarouselRibbonPosAndDisplaysPosAndSize(ribbon_pos, display_dim_data);
    ImGui::SetNextWindowPos(ribbon_pos);
    ImGui::SetNextWindowSize(ImVec2(width_, height_));

    ImGuiStyle& style(ImGui::GetStyle());
    float orig_border_size(style.WindowBorderSize);
    ImVec4 orig_border_color(style.Colors[ImGuiCol_Border]);
    Layout::DisplayRing& ring(layout_.display_states_.getDisplayRing());
    if (startMenu("Carousel", win_flags)) {
        win_flags |= ImGuiWindowFlags_NoBackground;
        win_flags |= ImGuiWindowFlags_AlwaysAutoResize;

        for (uint i(0); i < display_dim_data.size(); ++i) {
            ImVec2 display_pos(ribbon_pos.x + display_dim_data[i].x, ribbon_pos.y + display_dim_data[i].y);
            ImGui::SetNextWindowPos(display_pos);
            uint display_id(ring.getDisplayIdByIx(i));

            if (display_id == ring.getActiveFrameDisplayId()) {
                ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 3.0);
                ImGui::PushStyleColor(ImGuiCol_Border, layout_.kActiveBorderColor);
                win_flags ^= ImGuiWindowFlags_NoBackground;
            }

            std::string title("Carousel Display " + std::to_string(i));
            if (startMenu(title, win_flags)) {
                ImGui::SetCursorPos(ImVec2(0.0, 0.0));

                ImGui::Image(reinterpret_cast<ImTextureID>(ring.getImageIdForDisplayId(display_id)), 
                    ImVec2{display_dim_data[i].z, display_dim_data[i].w});
                
                // Show camera external name on top of image
                ImGui::SetCursorPos(ImVec2(10, 5));
                const std::string &title(layout_.displays_.getDisplayExternalNameById(display_id));
                ImGui::Text(title.c_str());

                endMenu();
            }

            if (display_id == ring.getActiveFrameDisplayId()) {
                ImGui::PopStyleVar();
                ImGui::PopStyleColor();
                win_flags |= ImGuiWindowFlags_NoBackground;
            }
        }

        endMenu();
    }
}

void LayoutComponent::getPiPWindowPosition(ImVec2 &pos) const
{
    ImGuiViewport* main_viewport(ImGui::GetMainViewport());
    ImVec2 work_pos(main_viewport->GetWorkPos());
    ImVec2 work_size(main_viewport->GetWorkSize());
    pos = ImVec2(0.0, 0.0);
    if (positioning_ & ComponentPositioning_Top) {
        pos.y = work_pos.y + offset_.y;

        if (positioning_ & ComponentPositioning_Left) {
            pos.x = work_pos.x + offset_.x;
        }
        else if (positioning_ & ComponentPositioning_Right) {
            pos.x = work_pos.x + (work_size.x - width_ - offset_.x);
        }
    }
    else if (positioning_ & ComponentPositioning_Bottom) {
        pos.y = work_pos.y + (work_size.y - height_ - offset_.y);

        if (positioning_ & ComponentPositioning_Left) {
            pos.x = work_pos.x + offset_.x;
        }
        else if (positioning_ & ComponentPositioning_Right) {
            pos.x = work_pos.x + (work_size.x - width_ - offset_.x);
        }
    }
}

void LayoutComponent::drawPiPWindow() const
{
    ImGuiWindowFlags win_flags(0);
    win_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse;
    win_flags |= ImGuiWindowFlags_NoTitleBar;
    win_flags |= ImGuiWindowFlags_AlwaysAutoResize;

    ImVec2 window_pos;
    getPiPWindowPosition(window_pos);
    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always);

    if (startMenu("Picture-in-Picture", win_flags)) {
        Layout::DisplayRing& ring(layout_.display_states_.getDisplayRing());
        if (ring.getNumSecondaryDisplays() == 0) { 
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "No secondary displays specified.");
            endMenu();
            return;
        }

        uint active_id(ring.getDisplayRoleList(LayoutDisplayRole::Secondary).at(0));
        std::string title(layout_.displays_.getDisplayExternalNameById(active_id));
        ImGui::Text("%s", title.c_str());
        ImGui::Image(reinterpret_cast<ImTextureID>(ring.getImageIdForDisplayId(active_id)),
            ImVec2(width_, height_));
        endMenu();
    }
}

void LayoutComponent::getDoublePiPWindowPositions(ImVec4 &pos) const
{
    ImGuiViewport* main_viewport(ImGui::GetMainViewport());
    ImVec2 work_pos(main_viewport->GetWorkPos());
    ImVec2 work_size(main_viewport->GetWorkSize());

    const int window_gap(75);

    // NOTE: pos.z represents the second PiP window's x-val, pos.w is its y-val

    if (positioning_ & ComponentPositioning_Left) {
        pos.x = work_pos.x + offset_.x;
        pos.z = pos.x;
    }
    else if (positioning_ & ComponentPositioning_Right) {
        pos.x = work_pos.x + (work_size.x - width_ - offset_.x);
        pos.z = pos.x;
    }

    pos.y = work_pos.y + offset_.y;
    pos.w = pos.y + height_ + window_gap;
}

void LayoutComponent::drawDoublePiPWindows() const
{
    // ImGuiWindowFlags win_flags(0);
    // win_flags |= ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse;
    // win_flags |= ImGuiWindowFlags_NoTitleBar;
    // win_flags |= ImGuiWindowFlags_AlwaysAutoResize;

    // ImVec4 windows_pos;
    // getDoublePiPWindowPositions(windows_pos);

    // ImGui::SetNextWindowPos(ImVec2{windows_pos.x, windows_pos.y}, ImGuiCond_Once);
    // if (startMenu("Picture-in-Picture 1", win_flags)) {
    //     if (layout_.secondary_ring_.empty()) { 
    //         ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "No secondary displays specified.");
    //         endMenu();
    //         return;
    //     }

    //     uint active_id(layout_.secondary_ring_.getActiveFrameDisplayId());
    //     std::string title(layout_.displays_.getDisplayExternalNameById(active_id));
    //     ImGui::Text("%s", title.c_str());
    //     ImGui::Image(reinterpret_cast<ImTextureID>(layout_.secondary_ring_.getImageIdForDisplayId(active_id)),
    //         ImVec2(width_, height_));
    //     endMenu();
    // }

    // // Since window above returns when there are no secondary displays, it shouldn't be
    // // necessary to check again
    // ImGui::SetNextWindowPos(ImVec2{windows_pos.z, windows_pos.w}, ImGuiCond_Once);
    // if (startMenu("Picture-in-Picture 2", win_flags)) {
    //     // This will get the active display again if there's only one active
    //     uint next_id(layout_.secondary_ring_.getNextActiveFrameDisplayId());
    //     std::string title(layout_.displays_.getDisplayExternalNameById(next_id));
    //     ImGui::Text("%s", title.c_str());
    //     ImGui::Image(reinterpret_cast<ImTextureID>(layout_.secondary_ring_.getImageIdForDisplayId(next_id)),
    //         ImVec2(width_, height_));
    //     endMenu();
    // }
}

} // viewpoint_interface