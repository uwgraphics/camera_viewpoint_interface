#ifndef __PUBLISH_GOALS_HPP__
#define __PUBLISH_GOALS_HPP__

#include <string>

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/ext/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale
#include <glm/ext/matrix_clip_space.hpp> // glm::perspective
#include <glm/ext/scalar_constants.hpp> // glm::pi
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>


namespace goal_publisher
{

struct Switch
{
    enum class Type 
    {
        HOLD, // Turn on while holding button
        SINGLE, // Toggle with each press
        DOUBLE // Double-press to toggle
    };

    Switch(bool state=false, Type type=Type::HOLD) : m_state(state),  m_type(type) {}

    bool is_on() { return m_state; }
    bool is_flipping() { return m_flipping; }
    std::string to_str() { return (m_state ? "on" : "off"); }

    void turn_on() { m_state = true; }
    void turn_off() { m_state = false; }
    void flip() { m_state = !m_state; m_flipping = true; }
    void set_state(bool state) { m_state = state; }

    // TODO: Consider breaking this out into button object
    bool button_pressed() { return m_cur_signal && !m_prev_signal; }
    bool button_depressed() { return !m_cur_signal && m_prev_signal; }
    void set_signal(bool signal) 
    { 
        m_prev_signal = m_cur_signal;
        m_cur_signal = signal;

        m_flipping = false;
        switch(m_type)
        {
            case Type::HOLD:
            {
                m_state = m_cur_signal;
                if (button_pressed() || button_depressed()) {
                    m_flipping = true;
                }
            } break;
            case Type::SINGLE:
            {
                if (button_pressed()) {
                    flip();
                }
            } break;
            case Type::DOUBLE:
            {
                // TODO:
                // When first press happens:
                // - Set sentinel 'wait' variable
                // - Start timer (1.5sec?)
                // - Check time:
                //      * Timer expired--set wait to false, end
                // - If button pressed again, flip switch
            } break;
        }
    }

    void operator =(const bool val) { set_signal(val); }

private:
    bool m_state;
    bool m_cur_signal;
    bool m_prev_signal;
    Type m_type;
    bool m_flipping; // Is switch flipping this cycle?
};

struct ControllerInput
{
    enum AxisOptions
    {
        FRONT,
        BACK,
        MIRROR
    };

    glm::vec3 init_pos, position, prev_pos, debug_pos;
    glm::quat init_orient, orientation;
    glm::vec3 cam_pos, manual_offset;
    glm::quat cam_orient;
    glm::quat inv_init_quat;
    bool initialized;
    Switch gripping;
    Switch camera_mode;
    Switch reset;
    Switch clutching;
    glm::vec3 clutch_offset;
    AxisOptions axis_mode;

    ControllerInput()
    {
        init_pos, position, prev_pos, debug_pos = glm::vec3();
        cam_pos, manual_offset = glm::vec3();
        init_orient, orientation, cam_orient = glm::quat();
        initialized = false;
        gripping = Switch(); // Gripper starts open
        camera_mode = Switch(false, Switch::Type::SINGLE);
        reset = Switch();
        clutching = Switch(false, Switch::Type::SINGLE);
        clutch_offset = glm::vec3();
    }

    std::string to_str(bool show_euler=false)
    {
        std::string content;
        content  = "Position: " + glm::to_string(position) + "\n";
        content += "Orientation: " + glm::to_string(orientation) + "\n";
        content += "Camera Mode: \t" + camera_mode.to_str();
        content +=  "\t" + glm::to_string(manual_offset) + "\n";
        content += "Grip: " + gripping.to_str() + "\n";
        content += "Reset: " + reset.to_str() + "\n";
        content += "Clutch: " + clutching.to_str() + "\n";

        return content;
    }
};

} // goal_publisher

void parseInput(std::string data, goal_publisher::ControllerInput &input);

#endif // __PUBLISH_GOALS_HPP__