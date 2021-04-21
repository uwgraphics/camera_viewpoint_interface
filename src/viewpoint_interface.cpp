// Standard libraries
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <poll.h>
#include <netinet/in.h>

// ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// OpenGL and Dear ImGui
#include <glad/glad.h> 
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// Custom headers
#include "viewpoint_interface/helpers.hpp"
#include "viewpoint_interface/json.hpp"
#include "viewpoint_interface/viewpoint_interface.hpp"

using json = nlohmann::json;
using App = viewpoint_interface::App;
using AppParams = viewpoint_interface::AppParams;
using Socket = viewpoint_interface::Socket;


/**
 * Entry point to the application.
 */
int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "viewpoint_interface");

    // Change working directory so we can specify resources more easily
    // NOTE: This depends on the 'cwd' param of the launch file being set to "node"
    chdir("../../../src/camera_viewpoint_interface");

    App app = App();
    app.run(argc, argv);
    
    return 0;
}


// App init/shutdown
bool App::parseCameraFile(std::string cam_config_data)
{
    if (cam_config_data.empty()) {
        printText("No camera config data.");
        return false;
    }

    json j(json::parse(cam_config_data));

    for (json::iterator it(j.begin()); it != j.end(); ++it) {
        std::string int_name, ext_name, topic_name;
        uint w, h, c;

        int_name = (*it)["internal_name"];
        ext_name = (*it)["external_name"];
        topic_name = (*it)["topic"];
        w = (*it)["width"];
        h = (*it)["height"];
        c = (*it)["channels"];

        layouts_.addDisplay(Display(int_name, ext_name, topic_name, DisplayDims(w, h, c)));
    }

    return true;
}

bool App::initialize()
{
    std::string cam_config_data;
    n_.getParam("cam_config_data", cam_config_data);

    // This must run first so that display settings are initialized
    if (!parseCameraFile(cam_config_data)) {
        return false;
    } 

    if (!initializeSocket()) {
        return false;
    }
    initializeROS();
    if (!initializeGlfw()) {
        return false;
    }
    initializeImGui();

    return true;
}

bool App::initializeSocket()
{
    if ((sock_.socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
        printText("Could not initialize socket.");
        return false;
    }

    memset(&sock_.address, 0, sizeof(sock_.address));
    sock_.address.sin_family = AF_INET; 
    sock_.address.sin_addr.s_addr = INADDR_ANY;
    sock_.address.sin_port = htons(sock_.PORT);

    if (bind(sock_.socket, (const sockaddr *)&sock_.address, sizeof(sock_.address)) < 0) { 
        printText("Socket binding failed."); 
        return false;
    }

    return true;
}

void App::initializeROS()
{
    spinner_.start();
    
    // Init display image callbacks
    for (int i = 0; i < layouts_.getNumTotalDisplays(); ++i) {
        ros::Subscriber disp_sub(n_.subscribe<sensor_msgs::Image>(layouts_.getDisplayInfo(i).topic, 1, 
                boost::bind(&App::cameraImageCallback, this, _1, layouts_.getDisplayInfo(i).id)));
        disp_subs_.push_back(disp_sub);
    }

    // Init camera pose matrix callbacks
    for (int i = 0; i < layouts_.getNumTotalDisplays(); ++i) {
        ros::Subscriber cam_matrix_sub(n_.subscribe<std_msgs::Float32MultiArray>(layouts_.getDisplayInfo(i).topic + "_matrix", 1, 
                boost::bind(&App::cameraMatrixCallback, this, _1, layouts_.getDisplayInfo(i).id)));
        cam_matrix_subs_.push_back(cam_matrix_sub);
    }

    grasping_sub_ = n_.subscribe<std_msgs::Bool>("/robot_state/grasping", 3, boost::bind(&App::graspingCallback, this, _1));
    clutching_sub_ = n_.subscribe<std_msgs::Bool>("/robot_state/clutching", 3, boost::bind(&App::clutchingCallback, this, _1));
    collision_sub_ = n_.subscribe<std_msgs::String>("/robot_state/collisions", 3, boost::bind(&App::collisionCallback, this, _1));
    active_display_sub_ = n_.subscribe<std_msgs::UInt8>("/viewpoint_interface/active_display", 3, 
            boost::bind(&App::activeDisplayCallback, this, _1));

    frame_matrix_pub_ = n_.advertise<std_msgs::Float32MultiArray>("/viewpoint_interface/frame_matrix", 3);
    display_bounds_pub_ = n_.advertise<std_msgs::Float32MultiArray>("/viewpoint_interface/display_bounds", 3);
}

bool App::initializeGlfw()
{
    if (!glfwInit()) {
        printText("Could not initialize GLFW!");
        return false;
    }
    glfwSetErrorCallback(glfwErrorCallback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    monitor_ = glfwGetPrimaryMonitor();
    if (!monitor_) {
        printText("Could not find primary monitor.");
        return false;
    }

    uint win_width, win_height;
    if (app_params_.WINDOW_WIDTH == 0 || app_params_.WINDOW_HEIGHT == 0) {
        const GLFWvidmode* mode(glfwGetVideoMode(monitor_));
        win_width = mode->width;
        win_height = mode->height;
    }
    else {
        win_width = app_params_.WINDOW_WIDTH;
        win_height = app_params_.WINDOW_HEIGHT;
    }

    window_ = glfwCreateWindow(win_width, win_height, app_params_.WINDOW_TITLE.c_str(), NULL, NULL);
    if (!window_) {
        printText("Could not create window.");
        return false;
    }
    glfwMakeContextCurrent(window_);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        printText("Failed to initialize GLAD");
        return false;
    }

    // Tells stb_image.h to flip loaded textures on y-axis
    stbi_set_flip_vertically_on_load(true);

    int frame_width, frame_height, x, y;
    glfwGetFramebufferSize(window_, &frame_width, &frame_height);
    transformFramebufferDims(&x, &y, &frame_width, &frame_height);
    glViewport(0, 0, frame_width, frame_height);
    glEnable(GL_DEPTH_TEST);
    glfwSetFramebufferSizeCallback(window_, framebufferSizeCallback);
    glfwSetWindowSizeLimits(window_, 800, 600, GLFW_DONT_CARE, GLFW_DONT_CARE);
    
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, keyCallbackForwarding);
    glfwSetInputMode(window_, GLFW_STICKY_MOUSE_BUTTONS, GLFW_TRUE);

    return true;
}

void App::initializeImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io_ = ImGui::GetIO(); (void)io_;
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    style.ItemInnerSpacing = ImVec2(7.0, 2.0);
    style.WindowRounding = 6.0;

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    if (!io_.Fonts->AddFontFromFileTTF("resources/fonts/Ubuntu-Regular.ttf", 18.0f)) {
        printText("Could not load font.");
    }
}

void App::shutdownApp()
{
    // Triggers when we're shutting down due to window being closed
    if (ros::ok()) {
        ros::shutdown();
    }
    spinner_.stop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
}


void App::loadMeshes()
{
    std::string settings_data;
    n_.getParam("settings_data", settings_data);

    if (settings_data.empty()) {
        printText("No settings data.");
    }

    json j(json::parse(settings_data));
    std::string model_dir("resources/models/");
    std::string shader_dir("resources/shaders/");

    for (auto model_data : j["models_to_load"]) {
        std::string model_path(model_dir + std::string(model_data["model"]));
        std::string fragment_path(shader_dir + std::string(model_data["fragment_shader"]));
        std::string vertex_path(shader_dir + std::string(model_data["vertex_shader"]));
        
        std::vector<float> anchor_vec(model_data["anchor_position"].get<std::vector<float>>());
        glm::vec2 anchor_pos(anchor_vec[0], anchor_vec[1]);

        float anchor_range(model_data["anchor_range"]);

        std::vector<float> pos_vec(model_data["default_position"].get<std::vector<float>>());
        glm::vec3 def_pos(pos_vec[0], pos_vec[1], pos_vec[2]);

        std::vector<float> angles_vec(model_data["default_angles_degrees"].get<std::vector<float>>());
        glm::vec3 def_angles(angles_vec[0], angles_vec[1], angles_vec[2]);

        float def_scale(model_data["default_scale"]);

        models_.push_back(Model(model_path, fragment_path, vertex_path, anchor_pos, anchor_range,
            def_pos, def_angles, def_scale));
    }

    layouts_.addModelData(&models_);
}


// Input handling
void App::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
            {
                glfwSetWindowShouldClose(window, true);
            } break;

            case GLFW_KEY_C:
            {
                layouts_.toggleControlPanel();
            } break;

            default:
            {
                layouts_.handleKeyInput(key, action, mods);
            } break;
        }
    }
    else
    {
        layouts_.handleKeyInput(key, action, mods);
    }
}

glm::vec2 App::getMousePosition()
{
    double x_pos, y_pos;
    glfwGetCursorPos(window_, &x_pos, &y_pos);

    return glm::vec2(x_pos, y_pos);
}

glm::ivec2 App::getWindowDimensions()
{
    int window_w, window_h;
    glfwGetFramebufferSize(window_, &window_w, &window_h);

    return glm::ivec2(window_w, window_h);
}

float App::getAspectRatio()
{
    glm::ivec2 window_dims(getWindowDimensions());
    return (float)window_dims.x / window_dims.y;
}

void App::drawOverlays()
{
    glm::vec2 mouse_pos(getMousePosition());
    glm::ivec2 window_dims(getWindowDimensions());
    glm::ivec3 mouse_buttons(glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT),
                             glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE),
                             glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT));
    layouts_.handleMouseInput(mouse_pos, window_dims, mouse_buttons);

    layouts_.draw(getAspectRatio());
}

std::string getSocketData(Socket &sock)
{
    int len_data;
    len_data = recvfrom(sock.socket, sock.buffer, sock.DATA_SIZE, MSG_WAITALL, (sockaddr *) &(sock.address), &(sock.len)); 
    // while (len_data == -1 && ros::ok()) {
    //     len_data = recvfrom(sock.socket, sock.buffer, sock.DATA_SIZE, MSG_WAITALL, (sockaddr *) &(sock.address), &(sock.len));   
    // }
    sock.buffer[len_data] = '\0';
    std::string data = sock.buffer;

    return data;
}

const App::AppCommand App::translateControllerInputToCommand(std::string input) const
{
    if (input == "shutdown") {
        return AppCommand::CLOSE_WINDOW;
    }
    else if (input == "toggle_control_panel") {
        return AppCommand::TOGGLE_CONTROL_PANEL;
    }

    return AppCommand::NONE;
}

void App::parseControllerInput(std::string data)
{
    json j = json::parse(data);

    if (j.is_null()) {
        return;
    }

    for (json::iterator it(j.begin()); it != j.end(); ++it) {
        AppCommand command(translateControllerInputToCommand(it.key()));

        switch (command)
        {
            case AppCommand::CLOSE_WINDOW:
            {
                glfwSetWindowShouldClose(window_, true);                
            }   break;

            case AppCommand::TOGGLE_CONTROL_PANEL:
            {
                layouts_.toggleControlPanel();
            }   break;
        
            default:
            {
                layouts_.handleControllerInput(it.key());
            }   break;
        }
    }
}

void App::handleControllerInput()
{
    pollfd poll_fds;
    poll_fds.fd = sock_.socket;
    poll_fds.events = POLLIN; // Wait until there's data to read

    while (!exiting_)
    {
        if (poll(&poll_fds, 1, 1000.0/(float)app_params_.loop_rate) > 0) {
            std::string input_data = getSocketData(sock_);
            parseControllerInput(input_data);
        }
    }

    shutdown(sock_.socket, SHUT_RDWR);  
}


// -- Window handling --
void glfwErrorCallback(int code, const char* description)
{
    printText("GLFW Error: " + code);
    printText(description);
}

void App::transformFramebufferDims(int *x, int *y, int *width, int *height)
{
    *x = FRAME_X;
    *y = FRAME_Y;
    *width = (int)(*width * WIDTH_FAC);
    *height = (int)(*height * HEIGHT_FAC);
}

void framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    int x, y;
    App::transformFramebufferDims(&x, &y, &width, &height);
    glViewport(x, y, width, height);
}

void App::keyCallbackForwarding(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    App *app = (App *)glfwGetWindowUserPointer(window);
    app->keyCallback(window, key, scancode, action, mods);
}


// -- Display image handling --
uint generateGLTextureId()
{
    uint id;
    glGenTextures(1, &id);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    return id;
}

void App::handleDisplayImageQueue()
{
    static std::vector<uint> tex_ids;

    std::vector<DisplayImageRequest> &queue(layouts_.getImageRequestQueue());

    if (layouts_.wasLayoutChanged()) {
        return;
    }

    if (queue.size() > tex_ids.size()) {
        for (uint i = tex_ids.size(); i < queue.size(); ++i) {
            uint new_id(generateGLTextureId());
            tex_ids.push_back(new_id);
        }
    }

    for (int i = 0; i < queue.size(); ++i) {
        DisplayImageRequest &request(queue.at(i));
        uint cur_id(tex_ids.at(i));

        int width, height, x, y;
        if (request.width == 0 || request.height == 0) {
            glfwGetFramebufferSize(window_, &width, &height);
            transformFramebufferDims(&x, &y, &width, &height);
        }
        else {
            width = request.width;
            height = request.height;
        }

        glActiveTexture(0);
        glBindTexture(GL_TEXTURE_2D, cur_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                (GLvoid*)request.data.data());

        layouts_.pushImageResponse(DisplayImageResponse{cur_id, request.index, request.role});
    }

    queue.clear();
}


// -- ROS Handling --
void App::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg, uint id)
{
    cv_bridge::CvImageConstPtr cur_img;
    try
    {
        cur_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        printText("cv_bridge exception: %s", 0);
        printText(e.what());
        return;
    }

    cv::Mat unflipped_mat;
    cv::flip(cur_img->image, unflipped_mat, 0);

    layouts_.forwardImageForDisplayId(id, unflipped_mat);
}

void App::cameraMatrixCallback(const std_msgs::Float32MultiArrayConstPtr& msg, uint id)
{
    return layouts_.forwardMatrixForDisplayId(id, msg->data);
}

void App::graspingCallback(const std_msgs::BoolConstPtr& msg)
{
    layouts_.setGrabbingState(msg->data);
}

void App::clutchingCallback(const std_msgs::BoolConstPtr& msg)
{
    layouts_.setClutchingState(msg->data);
}

void App::collisionCallback(const std_msgs::StringConstPtr& msg)
{
    layouts_.handleCollisionMessage(msg->data);
}

void App::activeDisplayCallback(const std_msgs::UInt8ConstPtr& msg)
{
    layouts_.setActiveWindow(msg->data);
}

void App::publishControlFrameMatrix()
{
    std_msgs::Float32MultiArray matrix_msg;

    // TODO: Consider adding camera label to message
    matrix_msg.data = layouts_.getActiveDisplayMatrix();
    frame_matrix_pub_.publish(matrix_msg);
}

void App::publishDisplayBounds()
{
    std_msgs::Float32MultiArray bounds_msg;
    bounds_msg.data = layouts_.getDisplayBounds();

    if (!bounds_msg.data.empty()) {
        display_bounds_pub_.publish(bounds_msg);
    }
}

void App::publishDisplayData()
{
    ros::Rate loop_rate(app_params_.loop_rate);
    while (!exiting_)
    {
        publishControlFrameMatrix();
        publishDisplayBounds();

        loop_rate.sleep();
    }
}

int App::run(int argc, char *argv[])
{
    if (!initialize()) {
        return -1;
    }

    std::thread controller_input(&App::handleControllerInput, this);
    std::thread publish_display_data(&App::publishDisplayData, this);
    std::thread load_meshes(&App::loadMeshes, this);

    ros::Rate loop_rate(app_params_.loop_rate);
    while (ros::ok() && !glfwWindowShouldClose(window_))
    {
        glfwPollEvents();

        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ImGui::ShowDemoWindow();
        
        drawOverlays();

        handleDisplayImageQueue();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        ImGui::EndFrame();

        glfwSwapBuffers(window_);

        loop_rate.sleep();
    }

    exiting_ = true;

    controller_input.join();
    publish_display_data.join();
    load_meshes.join();
    shutdownApp();

    return 0;
}
