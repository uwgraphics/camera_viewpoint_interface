#ifndef __OBJECT_HPP__
#define __OBJECT_HPP__

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "shader.hpp"
#include "model.hpp"

namespace viewpoint_interface
{

class Object
{
public:
    Object(Model& model, glm::vec3 pos=glm::vec3(0.0), glm::vec3 scale=glm::vec3(0.2, 0.2, 0.2)) : 
            model_(model), position_(pos), scale_(scale) 
    {
        // The coordinate system for models in the interface is
        // X-right, Y-forward, Z-up
        orientation_ = { 90.0, 0.0, 0.0 };
        model_.initializeModel();
    }

    void draw() 
    { 
        glm::mat4 model_mat(1.0f);
        model_mat = glm::rotate(model_mat, glm::radians(orientation_.z), glm::vec3(0.0, 0.0, 1.0));
        model_mat = glm::rotate(model_mat, glm::radians(orientation_.y), glm::vec3(0.0, 1.0, 0.0));
        model_mat = glm::rotate(model_mat, glm::radians(orientation_.x), glm::vec3(1.0, 0.0, 0.0));
        model_mat = glm::scale(model_mat, scale_);
        model_mat = glm::translate(model_mat, position_);

        model_.draw(model_mat); 
    }

private:
    glm::vec3 position_;
    glm::vec3 orientation_;
    glm::vec3 scale_;
    Model& model_;
};

} // viewpoint_interface

#endif // __OBJECT_HPP__