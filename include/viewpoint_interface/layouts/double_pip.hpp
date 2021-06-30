#ifndef __LAYOUT_DOUBLE_PIP_HPP__
#define __LAYOUT_DOUBLE_PIP_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct DoublePipParams
{
    uint start_primary_display = 0;
    uint first_pip_display = 1;
    uint second_pip_display = 2;
    uint max_num_displays = 3;
};

class DoublePipLayout final : public Layout
{
public:
    DoublePipLayout(DisplayManager &displays, DoublePipParams params=DoublePipParams()) : Layout(LayoutType::DOUBLE_PIP, displays),
            parameters_(params)
    {
        addDisplayByIxAndRole(parameters_.start_primary_display, LayoutDisplayRole::Primary);
        addDisplayByIxAndRole(parameters_.first_pip_display, LayoutDisplayRole::Secondary);
        addDisplayByIxAndRole(parameters_.second_pip_display, LayoutDisplayRole::Secondary);
    }

    virtual void displayLayoutParams() override
    {
        drawDisplaysList(parameters_.max_num_displays);
        drawDraggableRing();

        drawDisplaySelector(0, "Main Display", LayoutDisplayRole::Primary);
        drawDisplaySelector(0, "Pic-in-Pic 1", LayoutDisplayRole::Secondary);
        drawDisplaySelector(1, "Pic-in-Pic 2", LayoutDisplayRole::Secondary);
    }

    virtual void draw() override
    {
        addLayoutComponent(LayoutComponent::Type::Primary);

        addLayoutComponent(LayoutComponent::Type::Double_PiP, LayoutComponent::Spacing::Floating,
            LayoutComponent::ComponentPositioning_Right);
        drawLayoutComponents();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);
    }

    virtual void handleKeyInput(int key, int action, int mods) override
    {
        if (action == GLFW_PRESS) {
            switch (key) {

                case GLFW_KEY_RIGHT:
                case GLFW_KEY_UP:
                {
                    toNextDisplay(LayoutDisplayRole::Primary);
                    toNextDisplayWithPush(LayoutDisplayRole::Secondary);
                } break;

                case GLFW_KEY_LEFT:
                case GLFW_KEY_DOWN:
                {
                    toPrevDisplay(LayoutDisplayRole::Primary);
                    toPrevDisplayWithPush(LayoutDisplayRole::Secondary);
                } break;

                default:
                {
                } break;
            }
        }
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
    DoublePipParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_DOUBLE_PIP_HPP__