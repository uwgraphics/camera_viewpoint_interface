#ifndef __LAYOUT_COMPONENT_HPP__
#define __LAYOUT_COMPONENT_HPP__

#include <string>
#include <cmath>

#include <glm/vec2.hpp>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>


namespace viewpoint_interface
{

class Layout;

class LayoutComponent
{
public:
    enum class Type
    {
        Primary,
        Pic_In_Pic,
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

    enum class Positioning
    {
        Auto,
        Full,
        Top,
        Bottom,
        Left,
        Right,
        Top_Left,
        Top_Right,
        Bottom_Left,
        Bottom_Right
    };


    LayoutComponent(Layout &layout, Type type, Spacing spacing, Positioning positioning, float width,
            float height, ImVec2 offset) : layout_(layout), type_(type), spacing_(spacing),
            positioning_(positioning), width_(width), height_(height_), offset_(offset)
    {
        checkParameters();
    }

    inline Type getType() { return type_; }
    inline Spacing getSpacing() { return spacing_; }
    inline Positioning getPositioning() { return positioning_; }
    inline float getWidth() { return width_; }
    inline float getHeight() { return height_; }
    inline ImVec2 getOffset() { return offset_; }

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

    void getDisplayPositionAndSize(uint cur_display, uint num_displays, float &x_pos, float &y_pos, 
        float &width, float &height) const;
    void drawPrimaryWindows() const;
    void drawCarouselRibbon() const;
};

} // viewpoint_interface

#endif // __LAYOUT_COMPONENT_HPP__
