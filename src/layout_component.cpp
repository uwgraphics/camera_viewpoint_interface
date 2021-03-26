#include "viewpoint_interface/layout.hpp"
#include "viewpoint_interface/layout_component.hpp"


namespace viewpoint_interface {

// --- Public ---
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
            positioning_ = Positioning::Full;
        }   break;
        
        case Type::Pic_In_Pic:
        {
            ImVec2 default_dims{400, 225};
            ImVec2 default_offset{150, 150};
            // NOTE: The offset indicates how far to be from the edges of
            // the screen, not the absolute position of the window

            spacing_ = Spacing::Floating;

            if (positioning_ != Positioning::Top_Left &&
                    positioning_ != Positioning::Top_Right &&
                    positioning_ != Positioning::Bottom_Left &&
                    positioning_ != Positioning::Bottom_Right) {
                if (positioning_ == Positioning::Bottom) {
                    positioning_ = Positioning::Bottom_Right;
                }
                else {
                    positioning_ = Positioning::Top_Right;
                }
            }

            if (width_ <= 1e-10 || height_ <= 1e-10) {
                width_ = default_dims.x;
                height_ = default_dims.y;
            }

            if (offset_.x < 0.0 || offset_.y < 0.0) {
                offset_ = default_offset;
            }
        }   break;
        
        case Type::Carousel:
        {
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
                if (height_ <= 1e-10) {
                    height_ = default_dims.y;
                }

                if (positioning_ != Positioning::Top &&
                        positioning_ != Positioning::Bottom) {
                    positioning_ = Positioning::Bottom;
                }
            }
            else {
                if (width_ <= 1e-10) {
                    width_ = default_dims.x;
                }

                if (positioning_ != Positioning::Left &&
                        positioning_ != Positioning::Right) {
                    positioning_ = Positioning::Right;
                }
            }
        }   break;
        
        default:
        {
        }   break;
    }
}

void LayoutComponent::getDisplayPositionAndSize(uint cur_display, uint num_displays, float &x_pos, float &y_pos, 
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
    ImGuiStyle& style(ImGui::GetStyle());
    float orig_border_size(style.WindowBorderSize);
    ImVec4 orig_border_color(style.Colors[ImGuiCol_Border]);

    uint num_displays(layout_.getPrimaryDisplayCount(false));

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
        const DisplayInfo &disp_info(layout_.displays_.getDisplayInfoById(layout_.primary_displays_.at(i)));
        float aspect_ratio = (float)disp_info.dimensions.width / disp_info.dimensions.height;

        ImVec2 padding{5, 5};

        float img_width = win_width - (2*padding.x);
        float img_height = img_width * (1.0/aspect_ratio);

        if (img_height > win_height) {
            // If converted height is too large, convert width instead
            img_height = win_height - (2*padding.y);
            img_width = img_height * aspect_ratio;
        }

        if (num_displays > 1 && i == layout_.active_window_ix_) {
            style.WindowBorderSize = 3;
            style.Colors[ImGuiCol_Border] = layout_.kActiveBorderColor;
        }

        std::string menu_name("Primary Display##" + i);
        if (startMenu(menu_name, win_flags)) {
            // Center the image on the window
            ImVec2 image_pos(ImVec2{(ImGui::GetWindowSize().x - img_width) * 0.5f, 
                                        (ImGui::GetWindowSize().y - img_height) * 0.5f});
            ImGui::SetCursorPos(image_pos);

            ImGui::Image(reinterpret_cast<ImTextureID>(layout_.primary_img_ids_.at(i)), ImVec2 {img_width, img_height});
            
            // Show camera external name on top of image
            ImGui::SetCursorPos({image_pos.x + 10, image_pos.y + 5});
            const std::string &title(layout_.displays_.getDisplayExternalNameById(layout_.primary_displays_.at(i)));
            ImGui::Text(title.c_str());

            endMenu();
        }

        if (num_displays > 1 && i == layout_.active_window_ix_) {
            style.WindowBorderSize = orig_border_size;
            style.Colors[ImGuiCol_Border] = orig_border_color;
        }
    }
}

void LayoutComponent::drawCarouselRibbon() const
{
    ImVec2 display_size{250, 140};

    // Calculate number of displays in carousel and spacing
    const float display_padding(15.0);
    const uint max_displays(5);
    
    // TODO: Change num display calculation to account for primary thumbnail
    uint num_displays(0);
    if (spacing_ == Spacing::Horizontal) {
        num_displays = (width_ - display_padding) / (display_size.x + display_padding);
    }
    else {
        num_displays = (height_ - display_padding) / (display_size.y + display_padding);
    }

    if (layout_.secondary_displays_.size()-1 < num_displays) {
        num_displays = layout_.secondary_displays_.size()-1;
    }
    else {
        num_displays = num_displays > max_displays ? max_displays : num_displays;
    }

    ImGuiWindowFlags win_flags = 0;
    win_flags |= ImGuiWindowFlags_NoDecoration;
    win_flags |= ImGuiWindowFlags_NoInputs;
    win_flags |= ImGuiWindowFlags_NoSavedSettings;
    win_flags |= ImGuiWindowFlags_NoMove;
    ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImVec2 work_pos(main_viewport->GetWorkPos());
    ImVec2 ribbon_pos(ImVec2(work_pos.x + offset_.x, work_pos.y + offset_.y));
    ImGui::SetNextWindowPos(ribbon_pos);
    ImGui::SetNextWindowSize(ImVec2(width_, height_));
    
    if (startMenu("Carousel", win_flags)) {
        for (uint i(0), count(0); count < num_displays; ++i) {
            if (layout_.secondary_displays_.at(i) == layout_.primary_displays_.at(0)) {
                continue;
            }

            ImVec2 image_pos(ImVec2{display_padding + (count * (display_size.x + display_padding)), display_padding});
            ImGui::SetCursorPos(image_pos);

            ImGui::Image(reinterpret_cast<ImTextureID>(layout_.secondary_img_ids_.at(i)), ImVec2{display_size.x, display_size.y});
            
            // Show camera external name on top of image
            ImGui::SetCursorPos({image_pos.x + 10, image_pos.y + 5});
            const std::string &title(layout_.displays_.getDisplayExternalNameById(layout_.secondary_displays_.at(i)));
            ImGui::Text(title.c_str());

            ++count;
        }

        endMenu();
    }
}


} // viewpoint_interface