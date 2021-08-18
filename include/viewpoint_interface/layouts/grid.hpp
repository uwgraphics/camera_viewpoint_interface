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
        setNumDisplaysForRole(-1, LayoutDisplayRole::Primary);

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
        states["Gripper#on_off"] = grabbing_;
        displayStateValues(states);
    }

    virtual void handleStringInput(std::string input) override
    {
        LayoutCommand command(translateStringInputToCommand(input));

        switch(command)
        {
            case LayoutCommand::TOGGLE:
            {
                toNextDisplay(LayoutDisplayRole::Primary);
            }   break;

            case LayoutCommand::ACTIVE_FRAME_UP:
            case LayoutCommand::ACTIVE_FRAME_DOWN:
            case LayoutCommand::ACTIVE_FRAME_LEFT:
            case LayoutCommand::ACTIVE_FRAME_RIGHT:
            {
                display_states_.handleActiveFrameDirectionInput(command);
            }   break;

            case LayoutCommand::ACTIVE_FRAME_UP_RIGHT:
            {
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_UP);
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_RIGHT);
            }   break;

            case LayoutCommand::ACTIVE_FRAME_UP_LEFT:
            {
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_UP);
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_LEFT);
            }   break;

            case LayoutCommand::ACTIVE_FRAME_DOWN_RIGHT:
            {
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_DOWN);
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_RIGHT);
            }   break;

            case LayoutCommand::ACTIVE_FRAME_DOWN_LEFT:
            {
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_DOWN);
                display_states_.handleActiveFrameDirectionInput(LayoutCommand::ACTIVE_FRAME_LEFT);
            }   break;

        }
    }
    
private:
    GridParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_GRID_HPP__