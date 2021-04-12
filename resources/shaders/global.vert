#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec3 position;
out vec3 normal;
out vec2 tex_coord;

uniform mat4 model_mat;

void main()
{
    tex_coord = aTexCoord;
    normal = mat3(transpose(inverse(model_mat))) * aNormal;

    position = vec3(model_mat * vec4(aPos, 1.0));
    gl_Position = model_mat * vec4(aPos, 1.0);
}
