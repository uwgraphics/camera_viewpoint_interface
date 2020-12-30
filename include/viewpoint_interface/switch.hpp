#ifndef __SWITCH_HPP__
#define __SWITCH_HPP__

#include <string>

namespace viewpoint_interface
{

    struct Switch
    {
        enum class Type 
        {
            HOLD, // Turn on while holding button
            SINGLE, // Toggle with each press
            DOUBLE // Double-press to toggle
        };

        Switch(bool state=false, Type type=Type::HOLD) : m_state(state),  m_type(type),
                m_cur_signal(false), m_prev_signal(false), m_flipping(false), m_unconfirmed(false) {}

        bool is_on() { return m_state; }
        bool is_flipping() { return m_flipping; }
        std::string to_str() { return (m_state ? "on" : "off"); }

        void turn_on() { m_state = true; m_flipping = true; m_unconfirmed = true; }
        void turn_off() { m_state = false; m_flipping = true; m_unconfirmed = true; }
        void flip() { m_state = !m_state; m_flipping = true; m_unconfirmed = true; }
        void set_state(bool state) { m_state = state; }
        bool confirm_flip() {
            if (m_unconfirmed) {
                m_unconfirmed = false;
                return true;
            }

            return false;
        }
        bool confirm_flip_on() {
            return confirm_flip() && is_on();
        }
        bool confirm_flip_off() {
            return confirm_flip() && !is_on();    
        }

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
                        flip();
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
        bool m_unconfirmed;
    };


} // viewpoint_interface

#endif // __SWITCH_HPP__