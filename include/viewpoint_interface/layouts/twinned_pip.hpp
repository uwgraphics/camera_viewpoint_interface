#ifndef __LAYOUT_TWINNED_PIP_HPP__
#define __LAYOUT_TWINNED_PIP_HPP__

#include "viewpoint_interface/layout.hpp"

namespace viewpoint_interface
{

struct TwinnedPipParams
{
    uint start_primary_display = 0;
    uint start_pip_display = 1;
    uint max_num_displays = 2;

    int pip_window_dims[2] = { 400, 225 };
    int pip_aspect_ratio[2] = { 16, 9 };
};

class TwinnedPipLayout final : public Layout
{
public:
    TwinnedPipLayout(DisplayManager &displays, TwinnedPipParams params=TwinnedPipParams()) : 
            Layout(LayoutType::TWINNED_PIP, displays), parameters_(params), keep_aspect_ratio_(true), 
            pip_enabled_(true)
    {
        addDisplayByIxAndRole(parameters_.start_primary_display, LayoutDisplayRole::Primary);
        addDisplayByIxAndRole(parameters_.start_pip_display, LayoutDisplayRole::Secondary);
    }

    virtual void displayLayoutParams() override
    {
        drawDisplaysList(parameters_.max_num_displays);
        drawDraggableRing();

        drawDisplaySelector(0, "Main Display", LayoutDisplayRole::Primary);
        drawDisplaySelector(0, "Pic-in-Pic Display", LayoutDisplayRole::Secondary);

        ImGui::Separator();

        ImGui::Text("Picture-in-Picture Settings:\n");
        uint max_size(600);
        int start_pip_dims[2] = { parameters_.pip_window_dims[0], parameters_.pip_window_dims[1] };
        bool pip_dims_changed(ImGui::DragInt2("Dimensions", parameters_.pip_window_dims, 1.0f, 100, max_size));
        bool ar_changed(ImGui::Checkbox("Keep Aspect Ratio", &keep_aspect_ratio_));

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
        addLayoutComponent(LayoutComponent::Type::Primary);

        if (pip_enabled_) {
            addLayoutComponent(LayoutComponent::Type::Pic_In_Pic, LayoutComponent::Spacing::Floating,
                LayoutComponent::ComponentPositioning_Top_Right, (float)(parameters_.pip_window_dims[0]),
                (float)(parameters_.pip_window_dims[1]));
        }
        drawLayoutComponents();

        std::map<std::string, bool> states;
        states["Robot"] = !clutching_;
        states["Suction"] = grabbing_;
        displayStateValues(states);
    }

    virtual void handleKeyInput(int key, int action, int mods) override
    {
        // TODO: Add function to swap active primary with active secondary
        if (action == GLFW_PRESS) {
            switch (key) {
                case GLFW_KEY_P:
                {
                    pip_enabled_ = !pip_enabled_;
                } break;

                case GLFW_KEY_RIGHT:
                {
                    toNextDisplay(LayoutDisplayRole::Primary);
                    toNextDisplay(LayoutDisplayRole::Secondary);
                } break;

                case GLFW_KEY_LEFT:
                {
                    toPrevDisplay(LayoutDisplayRole::Primary);
                    toPrevDisplay(LayoutDisplayRole::Secondary);
                } break;

                case GLFW_KEY_UP:
                {
                    toPrevDisplay(LayoutDisplayRole::Secondary);
                    toPrevDisplay(LayoutDisplayRole::Primary);
                } break;

                case GLFW_KEY_DOWN:
                {
                    toNextDisplay(LayoutDisplayRole::Secondary);
                    toNextDisplay(LayoutDisplayRole::Primary);
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
    TwinnedPipParams parameters_;
    
    bool keep_aspect_ratio_, pip_enabled_;
};

} // viewpoint_interface

#endif //__LAYOUT_TWINNED_PIP_HPP__