#ifndef __OBJECT_HPP__
#define __OBJECT_HPP__

#include <glm/vec3.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "shader.hpp"
#include "camera.hpp"
#include "model.hpp"


class Object
{
public:
    Object(Shader &shdr, Camera &cam, Model &mdl, glm::vec3 pos, glm::vec3 scl)
            : shader(shdr), camera(cam), model(mdl), position(pos), scale(scl) {}

private:
    glm::vec3 position;
    glm::vec3 scale;
    Shader &shader;
    Camera &camera;
    Model &model;
};

#endif // __OBJECT_HPP__