#ifndef __LAYOUT_PIP_HPP__
#define __LAYOUT_PIP_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct PiPParams
{
    uint start_primary_display = 0;
    uint start_pip_display = 1;
    uint max_num_displays = 3;

    int pip_window_dims[2] = { 400, 225 };
    int pip_aspect_ratio[2] = { 16, 9 };
};

class PiPLayout final : public Layout
{
public:
    PiPLayout(DisplayManager &displays, PiPParams params=PiPParams()) : Layout(LayoutType::PIP, displays),
            parameters_(params), keep_aspect_ratio_(true), pip_enabled_(true)
    {
        if (parameters_.max_num_displays > displays.size() || parameters_.max_num_displays == 0) {
            parameters_.max_num_displays = displays.size();
        }

        addDisplayByIxAndRole(parameters_.start_primary_display, LayoutDisplayRole::Primary);
        addDisplayByIxAndRole(parameters_.start_pip_display, LayoutDisplayRole::Secondary);
    }

    virtual void displayLayoutParams() override
    {
        static uint cur_num_displays = parameters_.max_num_displays;

        drawDisplaysList();

        ImGui::SliderInt("# Displays", (int *) &cur_num_displays, 1, parameters_.max_num_displays);

        drawDraggableRing();

        drawDisplaySelector(0, "Main Display", LayoutDisplayRole::Primary);
        drawDisplaySelector(0, "Pic-in-Pic Display", LayoutDisplayRole::Secondary);

        ImGui::Separator();

        ImGui::Text("Picture-in-Picture Settings:\n");
        uint max_size = 600;
        int start_pip_dims[2] = { parameters_.pip_window_dims[0], parameters_.pip_window_dims[1] };
        bool pip_dims_changed = ImGui::DragInt2("Dimensions", parameters_.pip_window_dims, 1.0f, 100, max_size);
        bool ar_changed = ImGui::Checkbox("Keep Aspect Ratio", &keep_aspect_ratio_);

        if (keep_aspect_ratio_) {
            ImGui::Text("Aspect Ratio");
            ImGui::SameLine();
            ar_changed = ImGui::InputInt2("##PiP AR", parameters_.pip_aspect_ratio) || ar_changed;

            if (pip_dims_changed || ar_changed) {
                float aspect_ratio = (float)parameters_.pip_aspect_ratio[0] / parameters_.pip_aspect_ratio[1];
                
                // Clamp both axes to max_size
                int max_width(max_size), max_height(max_size);
                if (aspect_ratio > 1.0) {
                    // Width is largest
                    max_height = max_size * (1.0 / aspect_ratio);
                }
                else {
                    // Height is largest or same
                    max_width = max_size * aspect_ratio;
                }

                if (parameters_.pip_window_dims[0] > max_width || 
                        parameters_.pip_window_dims[1] > max_height) {
                    parameters_.pip_window_dims[0] = max_width;
                    parameters_.pip_window_dims[1] = max_height;
                }
                else {
                    if (parameters_.pip_window_dims[1]-start_pip_dims[1] != 0) {
                        // Height changed
                        parameters_.pip_window_dims[0] = parameters_.pip_window_dims[1] * aspect_ratio;
                    }
                    else {
                        // Width changed
                        aspect_ratio = 1.0 / aspect_ratio;
                        parameters_.pip_window_dims[1] = parameters_.pip_window_dims[0] * aspect_ratio;
                    }
                }
            }
        }
    }

    virtual void draw() override
    {
        handleImageResponse();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);

        // We only have one primary and one Pic-in-pic display
        displayPrimaryWindows();

        std::vector<uchar> &prim_data = displays_.getDisplayDataById(primary_displays_.at(0));
        const DisplayInfo &prim_info(displays_.getDisplayInfoById(primary_displays_.at(0)));
        addImageRequestToQueue(DisplayImageRequest{prim_info.dimensions.width, prim_info.dimensions.height,
                prim_data, 0, LayoutDisplayRole::Primary});

        if (pip_enabled_) {
            displayPiPWindow(parameters_.pip_window_dims[0], parameters_.pip_window_dims[1]);

            std::vector<uchar> &sec_data = displays_.getDisplayDataById(secondary_displays_[0]);
            const DisplayInfo &sec_info(displays_.getDisplayInfoById(primary_displays_.at(0)));
            addImageRequestToQueue(DisplayImageRequest{sec_info.dimensions.width, sec_info.dimensions.height, 
                    sec_data, 0, LayoutDisplayRole::Secondary});
        }
    }

    virtual void handleKeyInput(int key, int action, int mods) override
    {
        if (action == GLFW_PRESS) {
            switch (key) {
                case GLFW_KEY_P:
                {
                    pip_enabled_ = !pip_enabled_;
                } break;

                case GLFW_KEY_RIGHT:
                {
                    toNextDisplay(0, LayoutDisplayRole::Primary);
                } break;

                case GLFW_KEY_LEFT:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Primary);
                } break;

                case GLFW_KEY_UP:
                {
                    toPrevDisplay(0, LayoutDisplayRole::Secondary);
                } break;

                case GLFW_KEY_DOWN:
                {
                    toNextDisplay(0, LayoutDisplayRole::Secondary);
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
            default:
            {}  break;
        }
    }


private:
    PiPParams parameters_;
    
    bool keep_aspect_ratio_, pip_enabled_;
};

} // viewpoint_interface

#endif //__LAYOUT_PIP_HPP__