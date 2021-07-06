#ifndef __VIEWPOINT_INTERFACE_HPP__
#define __VIEWPOINT_INTERFACE_HPP__

#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"

#include <GLFW/glfw3.h>

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtx/string_cast.hpp>

#include "viewpoint_interface/shader.hpp"
#include "viewpoint_interface/switch.hpp"
#include "viewpoint_interface/display.hpp"
#include "viewpoint_interface/layout.hpp"
#include "viewpoint_interface/layout_manager.hpp"
#include "viewpoint_interface/scene_camera.hpp"


namespace viewpoint_interface
{
    struct Socket
    {
        uint const PORT = 8080;
        uint static const DATA_SIZE = 2048;

        int socket;
        sockaddr_in address;
        socklen_t len;
        char buffer[DATA_SIZE];
    };

    struct AppParams
    {
        uint loop_rate = 60;

        // Starting window dimensions - 0 indicates fullscreen
        uint const WINDOW_WIDTH = 0;
        uint const WINDOW_HEIGHT = 0;

        uint pip_width = WINDOW_WIDTH * 0.25;
        uint pip_height = WINDOW_HEIGHT * 0.25;

        uint def_disp_width = 1280;
        uint def_disp_height = 720;
        uint def_disp_channels = 3;
        const std::string CONTR_NAME = "vive_controller";
    };


    class App
    {
    public:
        static const int FRAME_X = 0;
        static const int FRAME_Y = 0;
        static constexpr float WIDTH_FAC = 1.0f;
        static constexpr float HEIGHT_FAC = 1.0f;

        App(AppParams params=AppParams()) : node_("~"), app_params_(params), spinner_(ros::AsyncSpinner(0)) {}

        int run(int argc, char *argv[]);

        static void transformFramebufferDims(int *x, int *y, int *width, int *height);

    private:
        // General items
        AppParams app_params_;
        Socket socket_;
        LayoutManager layouts_;
        bool clutch_mode_;

        // ROS
        ros::NodeHandle node_;
        ros::AsyncSpinner spinner_;
        ros::Subscriber grasping_sub_;
        ros::Subscriber clutching_sub_;
        ros::Subscriber collision_sub_;
        ros::Subscriber active_display_sub_;
        ros::Subscriber manual_command_sub_;
        std::vector<ros::Subscriber> disp_subs_;
        std::vector<ros::Subscriber> cam_matrix_subs_;
        ros::Publisher frame_matrix_pub_;
        ros::Publisher display_bounds_pub_;
        ros::Publisher mouse_pos_raw_;
        ros::Publisher mouse_pos_normalized_;
        ros::Publisher mouse_buttons_;
        ros::Publisher mouse_scroll_;

        // GUI
        GLFWwindow* window_;
        GLFWmonitor *monitor_;
        ImGuiIO io_;

        enum AppCommand
        {
            NONE,
            CLOSE_WINDOW,
            TOGGLE_CONTROL_PANEL
        };

        // General program flow
        bool initialize();
        bool parseCameraFile(std::string cam_config_data);
        bool initializeSocket();
        void initializeROS();
        bool initializeGlfw();
        void initializeImGui();
        void shutdownApp();

        // Input handling
        void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
        const AppCommand translateStringInputToCommand(std::string input) const;
        void handleCommandString(std::string command);
        void parseControllerInput(std::string data);
        void handleControllerInput();
        static glm::ivec2 getWindowDimensions(GLFWwindow* window);
        static void handleMousePosition(GLFWwindow* window, double x_pos, double y_pos);
        static void handleMouseButtons(GLFWwindow* window, int button, int action, int mods);
        static void handleMouseScroll(GLFWwindow* window, double x_offset, double y_offset);

        void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg, uint id);
        void cameraMatrixCallback(const std_msgs::Float32MultiArrayConstPtr& msg, uint id);
        void graspingCallback(const std_msgs::BoolConstPtr& msg);
        void clutchingCallback(const std_msgs::BoolConstPtr& msg);
        void collisionCallback(const std_msgs::StringConstPtr& msg);
        void activeDisplayCallback(const std_msgs::UInt8ConstPtr& msg);
        void handleManualCommand(const std_msgs::StringConstPtr& msg);
        void publishControlFrameMatrix();
        void publishDisplayData();
        void publishDisplayBounds();
        static void keyCallbackForwarding(GLFWwindow* window, int key, int scancode, int action, int mods);
        void handleDisplayImageQueue();
    };

} // viewpoint_interface

void printText(std::string text="", int newlines=1, bool flush=false);

void glfwErrorCallback(int code, const char* description);
void framebufferSizeCallback(GLFWwindow* window, int width, int height);
uint generateGLTextureId();

std::string getSocketData(viewpoint_interface::Socket &sock);


#endif // __VIEWPOINT_INTERFACE_HPP__
