#ifndef __LAYOUT_COMPONENT_HPP__
#define __LAYOUT_COMPONENT_HPP__

#include <string>
#include <cmath>
#include <vector>

#include <iostream>

#include <glm/vec2.hpp>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>


namespace viewpoint_interface
{

class Layout;

class LayoutComponent
{
public:
    // To add new layout components follow the following steps:
    // - Add a new entry to LayoutComponent::Type (immediately below this)
    // - Add drawing function signature to this file
    // - Implement drawing function in layout_component.cpp
    // - Add an entry to call the new component draw function to the main LayoutComponent::draw()
    // function

    enum class Type
    {
        Primary,
        Pic_In_Pic,
        Double_PiP,
        Carousel
    };

    enum class Spacing
    {
        Auto,
        Full,
        Horizontal,
        Vertical,
        Floating
    };

    enum Positioning
    {
        ComponentPositioning_Auto            = 1 << 5,
        ComponentPositioning_Full            = 0,
        ComponentPositioning_Top             = 1 << 0,
        ComponentPositioning_Bottom          = 1 << 1,
        ComponentPositioning_Left            = 1 << 2,
        ComponentPositioning_Right           = 1 << 3,
        
        ComponentPositioning_Top_Left        = ComponentPositioning_Top | ComponentPositioning_Left,
        ComponentPositioning_Top_Right       = ComponentPositioning_Top | ComponentPositioning_Right,
        ComponentPositioning_Bottom_Left     = ComponentPositioning_Bottom | ComponentPositioning_Left,
        ComponentPositioning_Bottom_Right    = ComponentPositioning_Bottom | ComponentPositioning_Right
    };


    LayoutComponent(Layout &layout, Type type, Spacing spacing, Positioning positioning, float width,
            float height, ImVec2 offset) : layout_(layout), type_(type), spacing_(spacing),
            positioning_(positioning), width_(width), height_(height), offset_(offset)
    {
        checkParameters();
    }

    inline Type getType() { return type_; }
    inline Spacing getSpacing() { return spacing_; }
    inline Positioning getPositioning() { return positioning_; }
    inline float getWidth() { return width_; }
    inline float getHeight() { return height_; }
    inline ImVec2 getOffset() { return offset_; }
    const std::vector<float> getDisplayBounds() const;

    void setWidth(float width) { 
        if (width > 0.0) {
            width_ = width;
        }
    }

    void setHeight(float height) {
        if (height > 0.0) {
            height_ = height;
        }
    }

    void setOffset(ImVec2 offset) { offset_ = offset; }

    void draw();

private:
    Layout &layout_;
    Type type_;
    Spacing spacing_;
    Positioning positioning_;
    ImVec2 offset_;
    float width_;
    float height_;

    void checkParameters();

    void getPrimaryDisplayPositionAndSize(uint cur_display, uint num_displays, float &x_pos, float &y_pos,
        float &width, float &height) const;
    void drawPrimaryWindows() const;
    void getCarouselRibbonPosAndDisplaysPosAndSize(ImVec2 &ribbon_pos, std::vector<ImVec4> &display_dim_data) const;
    void drawCarouselRibbon() const;
    void getPiPWindowPosition(ImVec2 &window_pos) const;
    void drawPiPWindow() const;
    void getDoublePiPWindowPositions(ImVec4 &window_pos) const;
    void drawDoublePiPWindows() const;
};

} // viewpoint_interface

#endif // __LAYOUT_COMPONENT_HPP__
