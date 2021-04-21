#ifndef __INSTANCE_HPP__
#define __INSTANCE_HPP__

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include "instance_info.hpp"
#include "shader.hpp"
#include "model.hpp"

namespace viewpoint_interface
{

class Instance
{
public:
    Instance(Model& model) : model_(model)
    {
        position_ = model_.getDefaultPosition();
        scale_ = model_.getDefaultScale();
        // The coordinate system for models in the interface is
        // X-right, Y-forward, Z-up
        glm::vec3 def_angles = model_.getDefaultOrientation();
        orientation_ = { 90.0 + def_angles.x, 0.0 + def_angles.y, 0.0 + def_angles.z };
        model_.initializeModel();
    }

    InstanceState& getState() { return state_; }
    SelectionState getSelectionState() { return state_.selection_; }
    void setSelectionState(SelectionState select) { state_.selection_ = select; }

    const glm::vec2 getPositionOnScreen() const
    {
        return glm::vec2(position_.x, position_.y);
    }

    void setPositionOnScreen(glm::vec2 new_pos)
    {
        position_.x = new_pos.x;
        position_.y = new_pos.y;
    }

    bool isWithinSelectionRange(glm::vec2 mouse_coords) const
    {
        float scale_factor(glm::length(scale_));
        glm::vec2 anchor_pos(model_.getAnchorPosition() * scale_factor);
        glm::vec2 cur_anchor(getPositionOnScreen() + anchor_pos);

        float scaled_range(model_.getAnchorRange() * scale_factor);

        return glm::distance(mouse_coords, cur_anchor) < scaled_range;
    }

    void draw(float aspect_ratio) 
    { 
        glm::mat4 model_mat(1.0f);

        model_mat = glm::translate(model_mat, position_);
        model_mat = glm::rotate(model_mat, glm::radians(orientation_.z), glm::vec3(0.0, 0.0, 1.0));
        model_mat = glm::rotate(model_mat, glm::radians(orientation_.y), glm::vec3(0.0, 1.0, 0.0));
        model_mat = glm::rotate(model_mat, glm::radians(orientation_.x), glm::vec3(1.0, 0.0, 0.0));
        model_mat = glm::scale(model_mat, scale_);

        // The model should look the same regardless of aspect ratio
        model_mat = glm::scale(model_mat, glm::vec3(1.0/aspect_ratio, aspect_ratio, 1.0));

        model_.draw(model_mat, state_); 
    }

private:
    InstanceState state_;
    glm::vec3 position_;
    glm::vec3 orientation_;
    glm::vec3 scale_;
    Model& model_;
};

} // viewpoint_interface

#endif // __INSTANCE_HPP__