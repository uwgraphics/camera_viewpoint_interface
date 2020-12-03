#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D diffuse;

void main()
{    
    FragColor = vec4(TexCoords, 0.3, 1.0);
    // FragColor = texture(diffuse, TexCoords);
}