#version 330 core
out vec4 frag_color;

in vec3 position;
in vec3 normal;
in vec2 tex_coord;

uniform vec3 diffuse;

void main()
{
    // Hard-coded light dir--it always emanates from the 'camera'
    vec3 light_dir = vec3(0.0, 0.0, -1.0);
    vec3 norm = normalize(normal);
    float diffuse_factor = max(dot(norm, light_dir), 0.0);

    frag_color = vec4(diffuse * diffuse_factor, 1.0);
}
