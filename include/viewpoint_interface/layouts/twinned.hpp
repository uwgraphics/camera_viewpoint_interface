#ifndef __LAYOUT_TWINNED_HPP__
#define __LAYOUT_TWINNED_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct TwinnedParams
{
    FrameMode starting_frame = FrameMode::CAMERA_FRAME;
    uint primary_display = 0;
};

class TwinnedLayout : public Layout
{
public:
    TwinnedLayout(DisplayManager &displays, TwinnedParams params=TwinnedParams()) : 
            Layout(LayoutType::TWINNED, displays), parameters_(params) 
    {
        addPrimaryDisplayByIx(parameters_.primary_display);
        frame_mode_ = parameters_.starting_frame;
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
        handleImageResponse();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);

        displayPrimaryWindows();

        std::vector<uchar> &prim_data = displays_.getDisplayDataById(primary_displays_.at(0));
        const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(0)));
        addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                prim_data, (uint)0, LayoutDisplayRole::Primary});           
    }

    virtual void handleImageResponse() override
    {
        for (int i = 0; i < image_response_queue_.size(); i++) {
            DisplayImageResponse &response(image_response_queue_.at(i));
            prim_img_ids_[response.index] = response.id;
        }
    }

    void nextDisplayAndFrame() {
        toNextDisplay(0, LayoutDisplayRole::Primary);
        if (frame_mode_ == FrameMode::CAMERA_FRAME) {
            frame_mode_ = FrameMode::WORLD_FRAME;
        }
        else if (frame_mode_ == FrameMode::WORLD_FRAME) {
            frame_mode_ = FrameMode::CAMERA_FRAME;
        }
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