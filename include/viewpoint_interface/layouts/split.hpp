#ifndef __LAYOUT_SPLIT_HPP__
#define __LAYOUT_SPLIT_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct SplitParams
{
    FrameMode starting_frame = FrameMode::CAMERA_FRAME;
    uint first_primary_display = 0;
    uint second_primary_display = 1;
};

class SplitLayout : public Layout
{
public:

    SplitLayout(DisplayManager &displays, SplitParams params=SplitParams()) : Layout(LayoutType::SPLIT, displays),
            parameters_(params) 
    {
        addPrimaryDisplayByIx(parameters_.first_primary_display);
        addPrimaryDisplayByIx(parameters_.second_primary_display);
        frame_mode_ = parameters_.starting_frame;
    }

    virtual void displayLayoutParams() override
    {
        // static uint num_displays = parameters.start_num_displays;
        // static uint max_bound = num_displays < parameters.max_displays ? num_displays : parameters.max_displays;

        drawDisplaysList(); 

        // ImGui::SliderInt("# Displays", (int *) &num_displays, 1, max_bound);

        drawDraggableRing();

        drawDisplaySelector(0, "Left Display", LayoutDisplayRole::Primary);
        drawDisplaySelector(1, "Right Display", LayoutDisplayRole::Primary);
    }

    virtual void draw() override
    {
        handleImageResponse();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);

        displayPrimaryWindows();

        for (int i = 0; i < 2; i++) {
            std::vector<uchar> &prim_data = displays_.getDisplayDataById(primary_displays_.at(i));
            const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(i)));
            addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                    prim_data, (uint)i, LayoutDisplayRole::Primary});
        }
    }

    virtual void handleImageResponse() override
    {
        for (int i = 0; i < image_response_queue_.size(); i++) {
            DisplayImageResponse &response(image_response_queue_.at(i));
            prim_img_ids_[response.index] = response.id;
        }
    }

private:
    SplitParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_SPLIT_HPP__