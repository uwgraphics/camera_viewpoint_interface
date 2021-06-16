#ifndef __LAYOUT_GRID_HPP__
#define __LAYOUT_GRID_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct GridParams
{

};

class GridLayout final : public Layout
{
public:

    GridLayout(DisplayManager &displays, GridParams params=GridParams()) : Layout(LayoutType::GRID, displays),
            parameters_(params) 
    {
        for (int i = 0; i < displays_.getNumTotalDisplays(); ++i) {
            addDisplayByIxAndRole(i, LayoutDisplayRole::Primary);
        }
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
        displayStateValues(states);
    }

private:
    GridParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_GRID_HPP__