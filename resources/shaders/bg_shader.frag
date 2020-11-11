#version 330 core
out vec4 FragColor;

in vec3 Color;
in vec2 TexCoord;

uniform bool overlay_on;

// texture samplers
uniform sampler2D Texture;
uniform sampler2D Overlay;

void main()
{
	vec4 main_tex = texture(Texture, TexCoord);
    vec4 over_tex = texture(Overlay, TexCoord);
    float overlay_factor = 0.9;
    // FragColor = over_tex;
    if (overlay_on) {
        FragColor = (main_tex / 1.5) + (over_tex / 1.5);
    }
    else {
        FragColor = main_tex;
    }

    // FragColor = vec4(TexCoord, 0.5, 1.0);
}
