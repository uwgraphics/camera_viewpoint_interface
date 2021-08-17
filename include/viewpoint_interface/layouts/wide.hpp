#ifndef __LAYOUT_WIDE_HPP__
#define __LAYOUT_WIDE_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct WideParams
{
    uint primary_display = 1;
};

class WideLayout final : public Layout
{
public:
    WideLayout(DisplayManager &displays, WideParams params=WideParams()) : 
            Layout(LayoutType::WIDE, displays), parameters_(params) 
    {
        setNumDisplaysForRole(1, LayoutDisplayRole::Primary);
        
        addDisplayByIxAndRole(parameters_.primary_display, LayoutDisplayRole::Primary);
    }

    virtual void displayLayoutParams() override
    {
        drawDisplaysList();
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

    virtual void handleStringInput(std::string input) override
    {
        LayoutCommand command(translateStringInputToCommand(input));

        switch(command)
        {
            default:
            {}  break;
        }
    }

private:
    WideParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_WIDE_HPP__