#ifndef __LAYOUT_TWINNED_HPP__
#define __LAYOUT_TWINNED_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct TwinnedParams
{
    uint primary_display = 0;
};

class TwinnedLayout : public Layout
{
public:
    TwinnedLayout(DisplayManager &displays, TwinnedParams params=TwinnedParams()) : 
            Layout(LayoutType::TWINNED, displays), parameters_(params) 
    {
        addDisplayByIxAndRole(parameters_.primary_display, LayoutDisplayRole::Primary);
    }

    virtual void displayLayoutParams() override
    {
        // TODO: This should limit to only two displays available
        drawDisplaysList();
        drawDraggableRing();
        drawDisplaySelector(0, "Main Display", LayoutDisplayRole::Primary);
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

    void nextDisplayAndFrame() {
        toNextDisplay(LayoutDisplayRole::Primary);
    }

    virtual void handleKeyInput(int key, int action, int mods) override
    {
        if (action == GLFW_PRESS) {
            switch (key) {
                case GLFW_KEY_RIGHT:
                case GLFW_KEY_LEFT:
                {
                    nextDisplayAndFrame();
                } break;

                default:
                {
                } break;
            }
        }
    }

    virtual void handleControllerInput(std::string input) override
    {
        LayoutCommand command(translateControllerInputToCommand(input));

        switch(command)
        {
            case LayoutCommand::TOGGLE:
            {
                nextDisplayAndFrame();
            }   break;

            default:
            {}  break;
        }
    }

private:
    TwinnedParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_TWINNED_HPP__