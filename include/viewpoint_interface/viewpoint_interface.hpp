#ifndef __MULTICAM_HPP__
#define __MULTICAM_HPP__

#include <string>
#include <vector>

#include "ros/ros.h"

// TEST
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/ext/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale
#include <glm/ext/matrix_clip_space.hpp> // glm::perspective
#include <glm/ext/scalar_constants.hpp> // glm::pi
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "viewpoint_interface/shader.hpp"
#include "viewpoint_interface/camera.hpp"


namespace multicam
{
    struct Socket
    {
        uint const PORT = 8080;
        uint static const DATA_SIZE = 2048;

        int sock;
        sockaddr_in addr;
        socklen_t len;
        char buffer[DATA_SIZE];
    };

    struct AppParams
    {
        uint loop_rate = 60;

        // Starting window dimensions
        uint const WINDOW_WIDTH = 1200;
        uint const WINDOW_HEIGHT = 900;

        uint pip_width = WINDOW_WIDTH * 0.25;
        uint pip_height = WINDOW_HEIGHT * 0.25;

        uint def_cam_height = 1024;
        uint def_cam_width = 1024;
        uint def_cam_channels = 3;
        uint total_cameras = 2;
        std::string cam_config_file = "resources/config/cam_config.json";
        const std::string CONTR_NAME = "vive_controller";
    };

    struct Image
    {
        uint width, height, channels, size, id;
        std::vector<uchar> data;

        Image() : width(0), height(0), channels(0), size(0) {}
        Image(uint w, uint h, uint c) : width(w), height(h), channels(c)
        {
            size = w * h * c;
            data = std::vector<uchar>(size);
        }

        void copy_data(const cv::Mat &mat)
        {
            std::vector<uchar> new_data;
            if (mat.isContinuous()) {
                new_data.assign(mat.data, mat.data + mat.total()*mat.channels());
            } else {
                for (int i = 0; i < mat.rows; ++i) {
                    new_data.insert(new_data.end(), mat.ptr<uchar>(i), mat.ptr<uchar>(i)+mat.cols*mat.channels());
                }
            }
            data = new_data;
        }

        void createTexture(uint tex_num);

        void resize(uint w, uint h, uint c)  
        {
            width = w; height = h; channels = c;
            size = width * height * channels;

            data.reserve(size);
        }
    };

    struct CCamera
    {
        std::string name, topic_name, display_name;
        Image image;

        CCamera() : name(""), topic_name(""), display_name("") {}
        CCamera(std::string n, std::string t, std::string d, uint w, uint h, uint c) : name(n), topic_name(t), 
                display_name(d), image(Image(w, h, c)) {}
    };

    struct Mesh
    {
        uint VBO, VAO, EBO, tex;
    };

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

    struct Input
    {
        glm::vec3 init_pos, position, prev_pos, debug_pos;
        glm::quat init_orient, orientation;
        glm::vec3 cam_pos, manual_offset;
        glm::quat cam_orient;
        glm::quat inv_init_quat;
        bool initialized;
        Switch gripping;
        Switch manual_adj;
        Switch reset;
        Switch clutching;
        glm::vec3 clutch_offset;
        cv_bridge::CvImagePtr cur_img;
        bool dyn_valid, stat_valid;


        Input()
        {
            init_pos, position, prev_pos, debug_pos = glm::vec3();
            cam_pos, manual_offset = glm::vec3();
            init_orient, orientation, cam_orient = glm::quat();
            initialized = false;
            gripping = Switch(); // Gripper starts open
            manual_adj = Switch(false, Switch::Type::HOLD);
            reset = Switch();
            clutching = Switch(false, Switch::Type::SINGLE);
            clutch_offset = glm::vec3();
            dyn_valid, stat_valid = false;
        }

        std::string to_str(bool show_euler=false)
        {
            std::string content;
            content  = "Position: " + glm::to_string(position) + "\n";
            content += "Orientation: " + glm::to_string(orientation) + "\n";
            content += "Manual Adj: \t" + manual_adj.to_str();
            content +=  "\t" + glm::to_string(manual_offset) + "\n";
            content += "Grip: " + gripping.to_str() + "\n";
            content += "Reset: " + reset.to_str() + "\n";
            content += "Clutch: " + clutching.to_str() + "\n";

            return content;
        }
    };

    class App
    {
    public:
        App(AppParams params = AppParams()) : app_params(params)
        {
            pip_enabled = clutch_mode = false;
            active_camera = pip_camera = 0;
            scene_cam = Camera(glm::vec3(0.0f, 0.0f, 3.0f));

            // TODO: Add config for cams
        }

        int run(int argc, char *argv[]);

    private:
        // General items
        AppParams app_params;
        Input input;
        Socket sock;
        Camera scene_cam;
        std::map<uint, CCamera> cam_info;
        uint active_camera;
        bool pip_enabled; // pip = Picture-in-picture
        uint pip_tex;
        uint pip_camera;
        bool clutch_mode;

        // ROS
        ros::Publisher ee_pub;
        ros::Publisher gripper_pub;
        ros::Publisher reset_pub;

        ros::Subscriber cam_pos_sub;
        std::vector<ros::Subscriber> cam_subs;

        // GUI
        GLFWwindow* window;
        ImGuiIO io;
        Image out_img;
        Image pip_img;


        // General program flow
        bool initialize(int argc, char *argv[]);
        bool parseCameraFile();
        bool initializeSocket();
        void initializeROS(int argc, char *argv[]);
        bool initializeGlfw();
        void initializeImGui();
        void shutdownApp();

        // Robot control
        void parseControllerInput(std::string data);
        void publishRobotData();
        void handleRobotControl();

        void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg, int index);
        static void keyCallbackForwarding(GLFWwindow* window, int key, int scancode, int action, int mods);
        void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
        void updateOutputImage();
        void updatePipImage();
        
        // Dear ImGui
        void buildMenu(const char *title, void (App::*build_func)(void), ImGuiWindowFlags window_flags = 0);
        void buildCameraSelectors();
        void buildPiPWindow();
    };

} // multicam

void printText(std::string text="", int newlines=1, bool flush=false);
uint getNextIndex(uint ix, uint size);
uint getPreviousIndex(uint ix, uint size);
void mismatchCameras(uint &cam1, uint &cam2, uint size);
void nextCamera(uint &cam1, uint &cam2, uint size, bool bump=true);
void previousCamera(uint &cam1, uint &cam2, uint size, bool bump=true);

void glfwErrorCallback(int code, const char* description);
void framebufferSizeCallback(GLFWwindow* window, int width, int height);

glm::vec3 positionToRobotFrame(glm::vec3 v);
glm::quat orientationToRobotFrame(glm::quat quat_in);
glm::mat4 translation_matrix(glm::vec3 coords);
glm::vec3 translation_from_matrix(glm::mat4 mat);
std::string getData(multicam::Socket &sock);

multicam::Mesh generateSquare();


#endif // __MULTICAM_HPP__
