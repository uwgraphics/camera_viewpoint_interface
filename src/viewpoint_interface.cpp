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
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point32.h>

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
#include "viewpoint_interface/shader.hpp"
#include "viewpoint_interface/mesh.hpp"
#include "viewpoint_interface/model.hpp"
#include "viewpoint_interface/object.hpp"

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

    json j = json::parse(cam_config_data);

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
    node_.getParam("cam_config_data", cam_config_data);

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
    if ((socket_.socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
        printText("Could not initialize socket.");
        return false;
    }

    memset(&socket_.address, 0, sizeof(socket_.address));
    socket_.address.sin_family = AF_INET; 
    socket_.address.sin_addr.s_addr = INADDR_ANY;
    socket_.address.sin_port = htons(socket_.PORT);

    if (bind(socket_.socket, (const sockaddr *)&socket_.address, sizeof(socket_.address)) < 0) { 
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
        ros::Subscriber disp_sub(node_.subscribe<sensor_msgs::Image>(layouts_.getDisplayInfo(i).topic, 1, 
                boost::bind(&App::cameraImageCallback, this, _1, layouts_.getDisplayInfo(i).id)));
        disp_subs_.push_back(disp_sub);
    }

    // Init camera pose matrix callbacks
    for (int i = 0; i < layouts_.getNumTotalDisplays(); ++i) {
        ros::Subscriber cam_matrix_sub(node_.subscribe<std_msgs::Float32MultiArray>(layouts_.getDisplayInfo(i).topic + "_matrix", 1,
                boost::bind(&App::cameraMatrixCallback, this, _1, layouts_.getDisplayInfo(i).id)));
        cam_matrix_subs_.push_back(cam_matrix_sub);
    }

    grasping_sub_ = node_.subscribe<std_msgs::Bool>("/robot_state/grasping", 10, boost::bind(&App::graspingCallback, this, _1));
    clutching_sub_ = node_.subscribe<std_msgs::Bool>("/robot_state/clutching", 10, boost::bind(&App::clutchingCallback, this, _1));
    collision_sub_ = node_.subscribe<std_msgs::String>("/robot_state/collisions", 10, boost::bind(&App::collisionCallback, this, _1));
    active_display_sub_ = node_.subscribe<std_msgs::UInt8>("/viewpoint_interface/active_display", 10, 
            boost::bind(&App::activeDisplayCallback, this, _1));
    manual_command_sub_ = node_.subscribe<std_msgs::String>("/viewpoint_interface/manual_command", 10,
            boost::bind(&App::handleManualCommand, this, _1));

    frame_matrix_pub_ = node_.advertise<std_msgs::Float32MultiArray>("/viewpoint_interface/frame_matrix", 10);
    display_bounds_pub_ = node_.advertise<std_msgs::Float32MultiArray>("/viewpoint_interface/display_bounds", 10);
    mouse_pos_raw_ = node_.advertise<geometry_msgs::Point32>("/viewpoint_interface/mouse_pos_raw", 10);
    mouse_pos_normalized_ = node_.advertise<geometry_msgs::Point32>("/viewpoint_interface/mouse_pos_normalized", 10);
    mouse_buttons_ = node_.advertise<sensor_msgs::Joy>("/viewpoint_interface/mouse_buttons", 10);
    mouse_scroll_ = node_.advertise<geometry_msgs::Point32>("/viewpoint_interface/mouse_scroll", 10);
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
        const GLFWvidmode* mode = glfwGetVideoMode(monitor_);
        win_width = mode->width;
        win_height = mode->height;
    }
    else {
        win_width = app_params_.WINDOW_WIDTH;
        win_height = app_params_.WINDOW_HEIGHT;
    }

    window_ = glfwCreateWindow(win_width, win_height, "HRI Study", NULL, NULL);
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
    glfwSetCursorPosCallback(window_, handleMousePosition);
    glfwSetMouseButtonCallback(window_, handleMouseButtons);
    glfwSetScrollCallback(window_, handleMouseScroll);

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

glm::ivec2 App::getWindowDimensions(GLFWwindow* window)
{
    int window_w, window_h;
    glfwGetFramebufferSize(window, &window_w, &window_h);

    return glm::ivec2(window_w, window_h);
}

void App::handleMousePosition(GLFWwindow* window, double x_pos, double y_pos)
{
    App *app = (App *)glfwGetWindowUserPointer(window);

    geometry_msgs::Point32 raw_pos;
    raw_pos.x = x_pos;
    raw_pos.y = y_pos;
    app->mouse_pos_raw_.publish(raw_pos);

    glm::ivec2 window_dims(getWindowDimensions(window));
    geometry_msgs::Point32 normalized_pos;
    normalized_pos.x = x_pos / window_dims.x;
    normalized_pos.y = y_pos / window_dims.y;
    app->mouse_pos_normalized_.publish(normalized_pos);
}

void App::handleMouseButtons(GLFWwindow* window, int button, int action, int mods)
{
    App *app = (App *)glfwGetWindowUserPointer(window);

    // TODO: Figure out if we just need to care about button action or if we should
    // poll all the buttons
    sensor_msgs::Joy buttons;
    buttons.buttons.emplace_back(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS);
    buttons.buttons.emplace_back(button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS);
    buttons.buttons.emplace_back(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);
    app->mouse_buttons_.publish(buttons);
}

void App::handleMouseScroll(GLFWwindow* window, double x_offset, double y_offset)
{
    App *app = (App *)glfwGetWindowUserPointer(window);

    geometry_msgs::Point32 scroll;
    scroll.x = x_offset;
    scroll.y = y_offset;
    app->mouse_scroll_.publish(scroll);
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

const App::AppCommand App::translateStringInputToCommand(std::string input) const
{
    if (input == "shutdown") {
        return AppCommand::CLOSE_WINDOW;
    }
    else if (input == "toggle_control_panel") {
        return AppCommand::TOGGLE_CONTROL_PANEL;
    }

    return AppCommand::NONE;
}

void App::handleCommandString(std::string in_string)
{
    AppCommand command(translateStringInputToCommand(in_string));

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
            layouts_.handleStringInput(in_string);
        }   break;
    }
}

void App::parseControllerInput(std::string data)
{
    json j = json::parse(data);

    if (j.is_null()) {
        return;
    }

    for (json::iterator it(j.begin()); it != j.end(); ++it) {
        handleCommandString(it.key());
    }
}

void App::handleControllerInput()
{
    pollfd poll_fds;
    poll_fds.fd = socket_.socket;
    poll_fds.events = POLLIN; // Wait until there's data to read

    while (ros::ok() && !glfwWindowShouldClose(window_))
    {
        if (poll(&poll_fds, 1, 1000.0/(float)app_params_.loop_rate) > 0) {
            std::string input_data = getSocketData(socket_);
            parseControllerInput(input_data);
        }
    }

    shutdown(socket_.socket, SHUT_RDWR);
}

void App::handleManualCommand(const std_msgs::StringConstPtr& msg)
{
    handleCommandString(msg->data);
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
        if (request.getWidth() == 0 || request.getHeight() == 0) {
            glfwGetFramebufferSize(window_, &width, &height);
            transformFramebufferDims(&x, &y, &width, &height);
        }
        else {
            width = request.getWidth();
            height = request.getHeight();
        }

        glActiveTexture(0);
        glBindTexture(GL_TEXTURE_2D, cur_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                (GLvoid*)request.getDataVector().data());


        layouts_.pushImageResponse(DisplayImageResponse{cur_id, request.getDisplayId(), request.getRole()});
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
    layouts_.setActiveFrame(msg->data);
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
    while (ros::ok() && !glfwWindowShouldClose(window_))
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

    ros::Rate loop_rate(app_params_.loop_rate);
    while (ros::ok() && !glfwWindowShouldClose(window_))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ImGui::ShowDemoWindow();

        layouts_.draw();

        handleDisplayImageQueue();

        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);

        // ImGui::EndFrame();

        loop_rate.sleep();
    }

    controller_input.join();
    publish_display_data.join();
    shutdownApp();

    return 0;
}
