#ifndef __VIEWPOINT_INTERFACE_HPP__
#define __VIEWPOINT_INTERFACE_HPP__

#include <string>
#include <vector>

#include "ros/ros.h"

// TEST
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

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

        int sock;
        sockaddr_in addr;
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

        uint def_disp_height = 1024;
        uint def_disp_width = 1024;
        uint def_disp_channels = 3;
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

        void generateEmptyTexture(uint tex_num);

        void resize(uint w, uint h, uint c)  
        {
            width = w; height = h; channels = c;
            size = width * height * channels;

            data.reserve(size);
        }
    };


    struct Mesh
    {
        uint VBO, VAO, EBO, tex;
    };

    struct Input
    {
        glm::vec3 manual_offset;
        bool initialized;
        Switch manual_adj;
        Switch clutching;
        glm::vec3 clutch_offset;
        cv_bridge::CvImagePtr cur_img;
        bool dyn_valid, stat_valid;


        Input()
        {
            manual_offset = glm::vec3();
            initialized = false;
            manual_adj = Switch(false, Switch::Type::HOLD);
            clutching = Switch(false, Switch::Type::SINGLE);
            clutch_offset = glm::vec3();
            dyn_valid, stat_valid = false;
        }

        std::string to_str(bool show_euler=false)
        {
            std::string content;
            content += "Manual Adj: \t" + manual_adj.to_str();
            content +=  "\t" + glm::to_string(manual_offset) + "\n";
            content += "Clutch: " + clutching.to_str() + "\n";

            return content;
        }
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
        Input input;
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

        // Input handling
        void parseControllerInput(std::string data);
        void handleRobotControl();

        void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg, int index);
        static void keyCallbackForwarding(GLFWwindow* window, int key, int scancode, int action, int mods);
        void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
        void updateOutputImage();
        void updatePipImage();
        
        // Dear ImGui
        void buildMenu(std::string title, void (App::*build_func)(void), ImGuiWindowFlags window_flags = 0);
        void buildLayoutsMenu();
        void buildDisplaySelectors();
        void buildPiPWindow();
    };

} // viewpoint_interface

void printText(std::string text="", int newlines=1, bool flush=false);

void glfwErrorCallback(int code, const char* description);
void framebufferSizeCallback(GLFWwindow* window, int width, int height);

std::string getData(viewpoint_interface::Socket &sock);

viewpoint_interface::Mesh generateSquare();


#endif // __VIEWPOINT_INTERFACE_HPP__
