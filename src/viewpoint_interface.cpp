// Standard libraries
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <poll.h>
#include <netinet/in.h>

// ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

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
bool App::parseCameraFile()
{
    std::ifstream cam_file;
    std::stringstream config_data;
    
    cam_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try { 
        cam_file.open(app_params.cam_config_file);
        config_data << cam_file.rdbuf();
        cam_file.close();
    }
    catch (std::ifstream::failure& e)
    {
        printText("Error reading camera config file: ", 0);
        printText(e.what());
        return false;
    }


    json j = json::parse(config_data.str());

    for (json::iterator it = j.begin(); it != j.end(); it++) {
        std::string int_name, ext_name, topic_name;
        uint w, h, c;

        int_name = (*it)["internal_name"];
        ext_name = (*it)["external_name"];
        topic_name = (*it)["topic"];
        w = (*it)["width"];
        h = (*it)["height"];
        c = (*it)["channels"];

        layouts.addDisplay(Display(int_name, ext_name, topic_name, DisplayDims(w, h, c)));
    }

    return true;
}

bool App::initialize(int argc, char *argv[])
{
    // This must run first so that display settings are initialized
    if (!parseCameraFile()) {
        return false;
    } 

    if (!initializeSocket()) {
        return false;
    }
    initializeROS(argc, argv);
    if (!initializeGlfw()) {
        return false;
    }
    initializeImGui();

    return true;
}

bool App::initializeSocket()
{
    if ((sock.socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
        printText("Could not initialize socket.");
        return false;
    }

    memset(&sock.address, 0, sizeof(sock.address));
    sock.address.sin_family = AF_INET; 
    sock.address.sin_addr.s_addr = INADDR_ANY;
    sock.address.sin_port = htons(sock.PORT);

    if (bind(sock.socket, (const sockaddr *)&sock.address, sizeof(sock.address)) < 0) { 
        printText("Socket binding failed."); 
        return false;
    }

    return true;
}

void App::initializeROS(int argc, char *argv[])
{
    spinner.start();
    
    for (int i = 0; i < layouts.getNumTotalDisplays(); i++) {
        ros::Subscriber disp_sub(n.subscribe<sensor_msgs::Image>(layouts.getDisplayInfo(i).topic, 10, 
                boost::bind(&App::cameraImageCallback, this, _1, layouts.getDisplayInfo(i).id)));
        disp_subs.push_back(disp_sub);
    }
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

    monitor = glfwGetPrimaryMonitor();
    if (!monitor) {
        printText("Could not find primary monitor.");
        return false;
    }

    uint win_width, win_height;
    if (app_params.WINDOW_WIDTH == 0 || app_params.WINDOW_HEIGHT == 0) {
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        win_width = mode->width;
        win_height = mode->height;
    }
    else {
        win_width = app_params.WINDOW_WIDTH;
        win_height = app_params.WINDOW_HEIGHT;
    }

    window = glfwCreateWindow(win_width, win_height, "HRI Study", NULL, NULL);
    if (!window) {
        printText("Could not create window.");
        return false;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        printText("Failed to initialize GLAD");
        return false;
    }

    // Tells stb_image.h to flip loaded textures on y-axis
    stbi_set_flip_vertically_on_load(true);

    int frame_width, frame_height, x, y;
    glfwGetFramebufferSize(window, &frame_width, &frame_height);
    transformFramebufferDims(&x, &y, &frame_width, &frame_height);
    glViewport(0, 0, frame_width, frame_height);
    glEnable(GL_DEPTH_TEST);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetWindowSizeLimits(window, 800, 600, GLFW_DONT_CARE, GLFW_DONT_CARE);
    
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, keyCallbackForwarding);

    return true;
}

void App::initializeImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    style.ItemInnerSpacing = ImVec2(7.0, 2.0);
    style.WindowRounding = 6.0;

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    if (!io.Fonts->AddFontFromFileTTF("resources/fonts/Ubuntu-Regular.ttf", 18.0f)) {
        printText("Could not load font.");
    }
}

void App::shutdownApp()
{
    // Triggers when we're shutting down due to window being closed
    if (ros::ok()) {
        ros::shutdown();
    }
    spinner.stop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
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
                layouts.toggleControlPanel();
            } break;

            default:
            {
                layouts.handleKeyInput(key, action, mods);
            } break;
        }
    }
    else
    {
        layouts.handleKeyInput(key, action, mods);
    }
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

    for (json::iterator it = j.begin(); it != j.end(); it++) {
        AppCommand command(translateControllerInputToCommand(it.key()));

        switch (command)
        {
            case AppCommand::CLOSE_WINDOW:
            {
                glfwSetWindowShouldClose(window, true);                
            }   break;

            case AppCommand::TOGGLE_CONTROL_PANEL:
            {
                layouts.toggleControlPanel();
            }   break;
        
            default:
            {
                layouts.handleControllerInput(it.key());
            }   break;
        }
    }
}

void App::handleControllerInput()
{
    pollfd poll_fds;
    poll_fds.fd = sock.socket;
    poll_fds.events = POLLIN; // Wait until there's data to read

    while (ros::ok() && !glfwWindowShouldClose(window))
    {
        if (poll(&poll_fds, 1, 1000.0/(float)app_params.loop_rate) > 0) {
            std::string input_data = getSocketData(sock);
            parseControllerInput(input_data);
        }
    }

    shutdown(sock.socket, SHUT_RDWR);  
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

    std::vector<DisplayImageRequest> &queue = layouts.getImageRequestQueue();

    if (layouts.wasLayoutChanged()) {
        return;
    }

    if (queue.size() > tex_ids.size()) {
        for (uint i = tex_ids.size(); i < queue.size(); i++) {
            uint new_id = generateGLTextureId();
            tex_ids.push_back(new_id);
        }
    }

    for (int i = 0; i < queue.size(); i++) {
        DisplayImageRequest &request(queue.at(i));
        uint cur_id = tex_ids.at(i);

        int width, height, x, y;
        if (request.width == 0.0 || request.height == 0.0) {
            glfwGetFramebufferSize(window, &width, &height);
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


        layouts.pushImageResponse(DisplayImageResponse{cur_id, request.index, request.role});
    }

    queue.clear();
}


// -- ROS Callbacks --
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

    layouts.forwardImageForDisplayId(id, unflipped_mat);
}


int App::run(int argc, char *argv[])
{
    if (!initialize(argc, argv)) {
        return -1;
    }

    // uint overlay = TextureFromFile("clutch_mode_overlay.png", "resources/textures/");
    // glActiveTexture(GL_TEXTURE2);
    // glBindTexture(GL_TEXTURE_2D, overlay);
    
    // -- Set up shaders --
    // std::string base_path("resources/shaders/");

    // Shader bg_shader((base_path + "bg_shader.vert").c_str(), (base_path + "bg_shader.frag").c_str());
    // bg_shader.use();
    // bg_shader.setInt("Texture", 1);
    // bg_shader.setInt("Overlay", 2);
    // ----


    std::thread controller_input(&App::handleControllerInput, this);

    ros::Rate loop_rate(app_params.loop_rate);
    while (ros::ok() && !glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ImGui::ShowDemoWindow();

        layouts.draw();
        
        handleDisplayImageQueue();

        // bg_shader.setBool("overlay_on", clutch_mode);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // glBindVertexArray(img_surface.VAO);
        // glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        // ImGui::EndFrame();

        loop_rate.sleep();
    }

    controller_input.join();
    shutdownApp();

    return 0;
}
