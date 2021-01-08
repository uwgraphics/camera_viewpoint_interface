#ifndef __VIEWPOINT_INTERFACE_HPP__
#define __VIEWPOINT_INTERFACE_HPP__

#include <string>
#include <vector>

#include "ros/ros.h"

#include <GLFW/glfw3.h>

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtx/string_cast.hpp>

#include "viewpoint_interface/shader.hpp"
#include "viewpoint_interface/switch.hpp"
#include "viewpoint_interface/layout.hpp"
#include "viewpoint_interface/display.hpp"
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
        std::string cam_config_file = "resources/config/cam_config.json";
        const std::string CONTR_NAME = "vive_controller";
    };


    class App
    {
    public:
        static const int FRAME_X = 0;
        static const int FRAME_Y = 0;
        static constexpr float WIDTH_FAC = 1.0f;
        static constexpr float HEIGHT_FAC = 1.0f;

        App(AppParams params=AppParams()) : app_params(params), spinner(ros::AsyncSpinner(0)) {}

        int run(int argc, char *argv[]);

        static void transformFramebufferDims(int *x, int *y, int *width, int *height);

    private:
        // General items
        AppParams app_params;
        Socket sock;
        LayoutManager layouts;
        bool clutch_mode;

        // ROS
        ros::NodeHandle n;
        ros::AsyncSpinner spinner;
        std::vector<ros::Subscriber> disp_subs;

        // GUI
        GLFWwindow* window;
        GLFWmonitor *monitor;
        ImGuiIO io;

        // General program flow
        bool initialize(int argc, char *argv[]);
        bool parseCameraFile();
        bool initializeSocket();
        void initializeROS(int argc, char *argv[]);
        bool initializeGlfw();
        void initializeImGui();
        void shutdownApp();

        // Input handling
        void parseControllerInput(std::string data);
        void handleRobotControl();

        void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg, uint index);
        static void keyCallbackForwarding(GLFWwindow* window, int key, int scancode, int action, int mods);
        void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
        void handleDisplayImageQueue();
    };

} // viewpoint_interface

void printText(std::string text="", int newlines=1, bool flush=false);

void glfwErrorCallback(int code, const char* description);
void framebufferSizeCallback(GLFWwindow* window, int width, int height);
uint generateGLTextureId();

std::string getData(viewpoint_interface::Socket &sock);


#endif // __VIEWPOINT_INTERFACE_HPP__
