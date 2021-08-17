#ifndef __LAYOUT_SPLIT_HPP__
#define __LAYOUT_SPLIT_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct SplitParams
{
    uint first_primary_display = 0;
    uint second_primary_display = 1;
};

class SplitLayout final : public Layout
{
public:

    SplitLayout(DisplayManager &displays, SplitParams params=SplitParams()) : Layout(LayoutType::SPLIT, displays),
            parameters_(params) 
    {
        setNumDisplaysForRole(2, LayoutDisplayRole::Primary);

        addDisplayByIxAndRole(parameters_.first_primary_display, LayoutDisplayRole::Primary);
        addDisplayByIxAndRole(parameters_.second_primary_display, LayoutDisplayRole::Primary);
    }

    virtual void displayLayoutParams() override
    {
        // static uint num_displays = parameters.start_num_displays;
        // static uint max_bound = num_displays < parameters.max_displays ? num_displays : parameters.max_displays;

        drawDisplaysList(); 
        // ImGui::SliderInt("# Displays", (int *) &num_displays, 1, max_bound);
        drawDraggableRing();
    }

    virtual void draw() override
    {
        addLayoutComponent(LayoutComponent::Type::Primary);
        drawLayoutComponents();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);
    }

private:
    SplitParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_SPLIT_HPP__