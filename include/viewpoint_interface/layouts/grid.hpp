#ifndef __LAYOUT_GRID_HPP__
#define __LAYOUT_GRID_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct GridParams
{

};

class GridLayout : public Layout
{
public:

    GridLayout(DisplayManager &displays, GridParams params=GridParams()) : Layout(LayoutType::GRID, displays),
            parameters_(params) 
    {
        for (int i = 0; i < displays_.size(); ++i) {
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
        handleImageResponse();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        displayStateValues(states);

        displayPrimaryWindows();

        for (int i = 0; i < primary_displays_.size(); ++i) {
            std::vector<uchar> &prim_data(displays_.getDisplayDataById(primary_displays_.at(i)));
            const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(i)));
            addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                    prim_data, (uint)i, LayoutDisplayRole::Primary});
        }
    }

private:
    GridParams parameters_;
};

} // viewpoint_interface

#endif //__LAYOUT_GRID_HPP__