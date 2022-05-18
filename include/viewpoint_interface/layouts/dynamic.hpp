#ifndef __LAYOUT_DYNAMIC_HPP__
#define __LAYOUT_DYNAMIC_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct DynamicParams
{
    uint primary_display = 0;
};

class DynamicLayout final : public Layout
{
public:
    DynamicLayout(DisplayManager &displays, DynamicParams params=DynamicParams()) : 
            Layout(LayoutType::DYNAMIC, displays), parameters_(params)
    {
        setNumDisplaysForRole(1, LayoutDisplayRole::Primary);
        // addDisplayByIxAndRole(parameters_.primary_display, LayoutDisplayRole::Primary);
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
        // displayStateValues(states);
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
    DynamicParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_DYNAMIC_HPP__