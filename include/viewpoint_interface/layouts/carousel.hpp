#ifndef __LAYOUT_CAROUSEL_HPP__
#define __LAYOUT_CAROUSEL_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct CarouselParams
{
    uint primary_display = 0;
};

class CarouselLayout final : public Layout
{
public:
    CarouselLayout(DisplayManager &displays, CarouselParams params=CarouselParams()) : 
            Layout(LayoutType::CAROUSEL, displays), parameters_(params) 
    {
        setNumDisplaysForRole(1, LayoutDisplayRole::Primary);
        setNumDisplaysForRole(-1, LayoutDisplayRole::Secondary);        

        // addDisplayByIxAndRole(parameters_.primary_display, LayoutDisplayRole::Primary);

        // for (int i = 0; i < displays.getNumTotalDisplays(); ++i) {
        //     addDisplayByIxAndRole(i, LayoutDisplayRole::Secondary);
        // }
    }

    virtual void displayLayoutParams() override
    {
        drawDisplaysList();
        drawDraggableRing();
    }

    virtual void draw() override
    {
        addLayoutComponent(LayoutComponent::Type::Primary);
        addLayoutComponent(LayoutComponent::Type::Carousel);
        drawLayoutComponents();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);
    }

private:
    CarouselParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_CAROUSEL_HPP__