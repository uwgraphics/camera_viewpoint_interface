// Standard libraries
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>

// ROS
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <relaxed_ik/EEPoseGoals.h>
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
#include "multicam_mimicry/json.hpp"
#include "multicam_mimicry/multicam.hpp"
#include "multicam_mimicry/shader.hpp"


using json = nlohmann::json;
using EEPoseGoals = relaxed_ik::EEPoseGoals;
using Pose = geometry_msgs::Pose;
using Bool = std_msgs::Bool;
using App = multicam::App;
using Mesh = multicam::Mesh;
using Image = multicam::Image;
using Socket = multicam::Socket;


/**
 * Entry point to the application.
 */
int main(int argc, char *argv[])
{    
    App app = App();
    app.run(argc, argv);
    return 0;
}

// -- Helper functions --
/**
 * Print a string to the screen.
 * 
 * Params:
 * 		text - text to print
 * 		newlines - number of newlines to print at the end of input string
 * 		flush - whether to flush text. Generally, only needed when
 * 			printing a large body of text with manual newlines
 */
void printText(std::string text, int newlines, bool flush)
{
    // TODO: Consider adding param for width of text line
    std::cout << text;

    for (int i = 0; i < newlines; i++) {
        if (i > 1) {
            std::cout << "\n";
        }
        else {
            std::cout << std::endl;
        }
    }

    if (flush) {
        std::cout.flush();
    }

    return;
}

void mismatchCameras(uint &cam1, uint &cam2, uint size)
{
    if (cam1 == cam2) {
        cam2 = (cam2 + 1) % size;
    }
}

void nextCamera(uint &active, uint &pip, uint size)
{
    active = (active + 1) % size;
    mismatchCameras(active, pip, size);
}

void previousCamera(uint &active, uint &pip, uint size)
{
    active = (active == 0) ? size-1 : active-1;
    mismatchCameras(active, pip, size);
}

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

    uint max_width = 0, max_height = 0, max_channels = 0;
    uint num_cams = j["_num_cams"];
    for (uint i = 0; i < num_cams; i++) {
        std::string cam_id = "cam" + std::to_string(i);
        std::string name, topic_name;
        uint w, h, c;

        name = j[cam_id]["name"];
        topic_name = j[cam_id]["topic"];
        w = j[cam_id]["width"];
        h = j[cam_id]["height"];
        c = j[cam_id]["channels"];

        if (w > max_width) {
            max_width = w;
        }
        if (h > max_height) {
            max_height = h;
        }
        if (c > max_channels) {
            max_channels = c;
        }

        cam_info.insert(std::pair<uint, Camera>(i, Camera(name, topic_name, w, h, c)));
    }

    out_img = Image(max_width, max_height, max_channels);

    return true;
}

bool App::initialize(int argc, char *argv[])
{
    // This must run first so that camera settings are initialized
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
    if ((sock.sock = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
        printText("Could not initialize socket.");
        return false;
    }

    memset(&sock.addr, 0, sizeof(sock.addr));
    sock.addr.sin_family = AF_INET; 
    sock.addr.sin_addr.s_addr = INADDR_ANY;
    sock.addr.sin_port = htons(sock.PORT);

    if (bind(sock.sock, (const sockaddr *)&sock.addr, sizeof(sock.addr)) < 0) { 
        printText("Socket binding failed."); 
        return false;
    }

    return true;
}

void App::initializeROS(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_openvr");
    ros::NodeHandle n;

    ee_pub = n.advertise<relaxed_ik::EEPoseGoals>("/relaxed_ik/ee_pose_goals", 1000);
    gripper_pub = n.advertise<std_msgs::Bool>("/relaxed_ik/gripper_state", 1000);
    reset_pub = n.advertise<std_msgs::Bool>("/relaxed_ik/reset_pose", 1000);
    
    // cam_pos_sub(n.subscribe<geometry_msgs::Pose>("/cam/pose_actual", 10, cameraCallback);
    for (int i = 0; i < cam_info.size(); i++) {
        ros::Subscriber cam_sub(n.subscribe<sensor_msgs::Image>(cam_info[i].topic_name, 10, boost::bind(&App::cameraImageCallback, this, _1, i)));
        cam_subs.push_back(cam_sub);
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

    window = glfwCreateWindow(app_params.WINDOW_WIDTH, app_params.WINDOW_HEIGHT, "Camera Feed", NULL, NULL);
    if (!window) {
        printText("Could not create window.");
        return false;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        printText("Failed to initialize GLAD");
        return false;
    }
    glViewport(0, 0, app_params.WINDOW_WIDTH, app_params.WINDOW_HEIGHT);
    // glEnable(GL_DEPTH_TEST);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, keyCallbackForwarding);

    return true;
}

void App::initializeImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void App::shutdown()
{
    if (ros::ok()) {
        ros::shutdown();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}


// -- Robot control --
glm::vec3 positionToRobotFrame(glm::vec3 v)
{
    return glm::vec3(-v.y, -v.z, v.x);
}

glm::quat orientationToRobotFrame(glm::quat quat_in)
{
    glm::vec3 new_euler = glm::vec3(glm::pitch(quat_in), -glm::roll(quat_in), -glm::yaw(quat_in));
    glm::quat new_quat = glm::quat(new_euler);

    return new_quat;
}

glm::mat4 translation_matrix(glm::vec3 coords)
{
    glm::mat4 mat = glm::mat4();
    mat[0][3] = coords.x;
    mat[1][3] = coords.y;
    mat[2][3] = coords.z;

    return mat;
}

glm::vec3 translation_from_matrix(glm::mat4 mat)
{
    return glm::vec3(mat[0][3], mat[1][3], mat[2][3]);
}

std::string getSocketData(Socket &sock)
{
    int len_data;
    len_data = recvfrom(sock.sock, sock.buffer, sock.DATA_SIZE, MSG_WAITALL, (sockaddr *) &(sock.addr), &(sock.len)); 
    sock.buffer[len_data] = '\0';
    std::string data = sock.buffer;

    return data;
}

void App::parseControllerInput(std::string data)
{
    std::string CONTR_NAME = app_params.CONTR_NAME;

    json j = json::parse(data);

    auto pos = j[CONTR_NAME]["pose"]["position"];
    glm::vec3 pos_vec = glm::vec3(pos["x"], pos["y"], pos["z"]);

    auto quat = j[CONTR_NAME]["pose"]["orientation"];
    glm::quat quat_vec = glm::quat(quat["w"], quat["x"], quat["y"], quat["z"]);

    input.gripping = j[CONTR_NAME]["gripper"]["boolean"];
    input.clutching = j[CONTR_NAME]["clutch"]["boolean"];

    input.manual_adj = j[CONTR_NAME]["manual_adj"]["boolean"];
    input.manual_offset.x = j[CONTR_NAME]["manual_adj"]["2d"]["x"];
    input.manual_offset.z = j[CONTR_NAME]["manual_adj"]["2d"]["y"];

    input.reset = j[CONTR_NAME]["reset"]["boolean"];

    if (!input.initialized) {
        input.init_pos = pos_vec;
        input.prev_pos = pos_vec;
        input.init_orient = quat_vec;

        input.inv_init_quat = glm::inverse(quat_vec);

        input.initialized = true;
    }

    // Handle clutching enabled by keyboard
    if (clutch_mode && !input.clutching.is_flipping() && !input.clutching.is_on()) {
        input.clutching.turn_on();
    }

    if (input.clutching.is_flipping()) {
        if (input.clutching.is_on()) { // When just turned on
            clutch_mode = true;
            input.clutch_offset = pos_vec - input.init_pos;
            // TODO: Add orientation handling
        }
        else {
            clutch_mode = false;
            input.init_pos = pos_vec - input.clutch_offset;
        }
    }

    if (input.manual_adj.confirm_flip_on()) {
        if (input.manual_offset.x >= 0.5) {
            nextCamera(active_camera, pip_camera, cam_info.size());
        }
        else if (input.manual_offset.x <= -0.5) {
            previousCamera(active_camera, pip_camera, cam_info.size());
        }
        else {
            pip_enabled = !pip_enabled;
        }
    }

    // TODO: Consider how this should interact with clutching
    if (!input.reset.is_on() && input.reset.is_flipping()) {
        input.init_pos = pos_vec;
        input.prev_pos = pos_vec;
        input.init_orient = quat_vec;

         input.inv_init_quat = glm::inverse(quat_vec);
    }

    // TODO: **Add mode for using camera frame**
    if (!input.clutching.is_on() && !input.reset.is_on()) {
        input.position = pos_vec - input.init_pos;
        input.position = glm::rotate(input.init_orient, input.position);
        glm::mat4 trans_mat = glm::toMat4(input.cam_orient) * translation_matrix(input.position);
        input.position = translation_from_matrix(trans_mat);
        input.orientation = input.inv_init_quat * quat_vec; // Displacement b/w quats

        input.position = positionToRobotFrame(input.position);
        // input.manual_offset = positionToRobotFrame(input.manual_offset);
        input.orientation = orientationToRobotFrame(input.orientation);
    }
    else {
        // Clutching mode handling
        // Note that the behavior of buttons changes while in this mode
    }
}

void App::publishRobotData()
{
    EEPoseGoals goal;
    Pose pose;
    pose.position.x = input.position.x;
    pose.position.y = input.position.y;
    pose.position.z = input.position.z;

    pose.orientation.x = input.orientation.x;
    pose.orientation.y = input.orientation.y;
    pose.orientation.z = input.orientation.z;
    pose.orientation.w = input.orientation.w;

    Pose pose_cam;
    pose_cam.position.x = input.manual_offset.x;
    pose_cam.position.y = input.manual_offset.y;
    pose_cam.position.z = input.manual_offset.z;

    pose_cam.orientation.x = 0.0;
    pose_cam.orientation.y = 0.0;
    pose_cam.orientation.z = 0.0;
    pose_cam.orientation.w = 1.0;

    goal.header.stamp = ros::Time::now();
    goal.ee_poses.push_back(pose);
    goal.ee_poses.push_back(pose_cam);

    ee_pub.publish(goal);

    input.prev_pos = glm::vec3(input.position.x, input.position.y, input.position.z);


    Bool gripping;
    gripping.data = input.gripping.is_on();

    gripper_pub.publish(gripping);        
}

void App::handleRobotControl()
{
    // Get initial values
    std::string input_data;
    input_data = getSocketData(sock);
    parseControllerInput(input_data);

    while (ros::ok() && !glfwWindowShouldClose(window))
    {
        input_data = getSocketData(sock);
        parseControllerInput(input_data);
        printText(input.to_str(true));

        publishRobotData();
    }
}


// -- Window handling --
void glfwErrorCallback(int code, const char* description)
{
    printText("GLFW Error: " + code);
    printText(description);
}

void framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void App::keyCallbackForwarding(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    App *app = (App *)glfwGetWindowUserPointer(window);
    app->keyCallback(window, key, scancode, action, mods);
}

void App::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
    else if (key == GLFW_KEY_TAB && action == GLFW_PRESS) {
        nextCamera(active_camera, pip_camera, cam_info.size());
    }
    else if (key == GLFW_KEY_C && action == GLFW_PRESS) {
        clutch_mode = !clutch_mode;
    }
    else if (key == GLFW_KEY_P && action == GLFW_PRESS) {
        pip_enabled = !pip_enabled;
    }
}


// -- OpenGL and Dear ImGui --
void Image::createTexture(uint tex_num)
{
    glGenTextures(1, &id);
    glActiveTexture(GL_TEXTURE0 + tex_num);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
}

void App::updateOutputImage()
{
    Camera cur_cam = cam_info.at(active_camera);
    uint cam_size = cur_cam.image.size;

    if (cam_size != out_img.size) {
        // TODO: Figure out how to deal with different sizes
    }

    out_img.data.assign(cur_cam.image.data.data(), cur_cam.image.data.data() + cur_cam.image.data.size());
    glActiveTexture(0);
    glBindTexture(GL_TEXTURE_2D, out_img.id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, out_img.width, out_img.height, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)out_img.data.data());
}

void App::updatePipImage()
{
    Camera cur_cam = cam_info.at(pip_camera);
    uint cam_size = cur_cam.image.size;

    if (cam_size != out_img.size) {
        // TODO: Figure out how to deal with different sizes
    }

    cv::Mat flip_mat = cv::Mat(cur_cam.image.width, cur_cam.image.height, CV_8UC3, cur_cam.image.data.data());
    cv::flip(flip_mat, flip_mat, 0);

    pip_img.resize(cur_cam.image.width, cur_cam.image.height, cur_cam.image.channels);
    pip_img.copy_data(flip_mat);
    glActiveTexture(2);
    glBindTexture(GL_TEXTURE_2D, pip_img.id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pip_img.width, pip_img.height, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)pip_img.data.data());
}

Mesh generateSquare()
{
    float vertices[] = {
        // positions          // colors           // texture coords
         1.0f,  1.0f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,   // top right
         1.0f, -1.0f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,   // bottom right
        -1.0f, -1.0f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,   // bottom left
        -1.0f,  1.0f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f    // top left 
    };
    unsigned int indices[] = {
        0, 1, 3, // first triangle
        1, 2, 3  // second triangle
    };

    Mesh mesh;

    glGenVertexArrays(1, &mesh.VAO);
    glGenBuffers(1, &mesh.VBO);
    glGenBuffers(1, &mesh.EBO);

    // Binds
    glBindVertexArray(mesh.VAO);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // Texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    
    return mesh;
}

void App::buildMenu(const char* title, void (App::*build_func)(void), ImGuiWindowFlags window_flags)
{

    ImGui::Begin(title, (bool *)NULL, window_flags);
    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.6f);
    (this->*build_func)();
    ImGui::PopItemWidth();
    ImGui::End();
}

void App::buildCameraSelectors()
{
    const char *cam_preview = cam_info.at(active_camera).name.c_str(); 
    if (ImGui::BeginCombo("Camera", cam_preview, ImGuiComboFlags_None))
    {
        for (int n = 0; n < cam_info.size(); n++)
        {
            const char *cam_name = cam_info.at(n).name.c_str();
            const bool is_selected = (active_camera == n);
            if (ImGui::Selectable(cam_name, is_selected)) {
                active_camera = n;
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }

    mismatchCameras(active_camera, pip_camera, cam_info.size());

    if (cam_info.size() > 1) {
        ImGui::Checkbox("PiP Enabled", &pip_enabled);
        if (pip_enabled) {
            const char *pip_preview = pip_preview = cam_info.at(pip_camera).name.c_str(); 
            if (ImGui::BeginCombo("PiP Camera", pip_preview, ImGuiComboFlags_None))
            {
                for (int n = 0; n < cam_info.size(); n++)
                {
                    // Skip currently active main camera
                    if (active_camera == n) {
                        continue;
                    }

                    const char *cam_name = cam_info.at(n).name.c_str();
                    const bool is_selected = (pip_camera == n);
                    if (ImGui::Selectable(cam_name, is_selected)) {
                        pip_camera = n;
                    }

                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }
        }
    }
}

void App::buildPiPWindow()
{
    ImTextureID img_id = (ImTextureID)(pip_img.id);
    ImGui::Image(img_id, ImVec2(app_params.pip_width, app_params.pip_height));
}


// -- ROS Callbacks --
// void cameraPoseCallback(const geometry_msgs::Pose::ConstPtr& msg, ControllerInput& input)
// {
//     auto pos = msg->position;
//     auto quat = msg->orientation;
//     input.cam_pos = glm::vec3(pos.x, pos.y, pos.z);
//     input.cam_orient = glm::quat(quat.w, quat.x, quat.y, quat.z);
// }

void App::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg, int index)
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

    cv::Mat image = cur_img->image;
    int data_size = image.cols * image.rows * image.channels();
    if (data_size > cam_info.at(index).image.size) {
        cam_info.at(index).image.resize(image.cols, image.rows, image.channels());
    }
    cam_info.at(index).image.copy_data(image);
}


int App::run(int argc, char *argv[])
{
    // Change working directory so we can specify resources more easily
    // Note that this depends on the 'cwd' param of the launch file being set to "node"
    chdir("../../../src/multicam_mimicry");

    if (!initialize(argc, argv)) {
        return -1;
    }

    // Set up camera textures
    Mesh img_surface = generateSquare();
    out_img.createTexture(0);

    cv::Mat overlay_img = cv::imread("resources/textures/multicam_overlay.png");
    if (overlay_img.empty()) {
        printText("Could not load overlay image.");
        return -1;
    }
    cv::flip(overlay_img, overlay_img, 0);
    Image overlay = Image(overlay_img.cols, overlay_img.rows, overlay_img.channels());
    overlay.copy_data(overlay_img);
    overlay.createTexture(1);
    overlay_img.release();

    pip_img = Image();
    pip_img.createTexture(2);


    Shader bg_shader("resources/shaders/bg_shader.vert", "resources/shaders/bg_shader.frag");
    bg_shader.use();
    bg_shader.setInt("Texture", 0);
    bg_shader.setInt("Overlay", 1);

    // Split robot control into its own thread to improve performance
    std::thread robot_control(&App::handleRobotControl, this);


    ros::Rate loop_rate(app_params.loop_rate);
    while (ros::ok() && !glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // ImGui::ShowDemoWindow();
        
        ImGuiWindowFlags win_flags = 0;
        win_flags |= ImGuiWindowFlags_NoScrollbar;
        win_flags |= ImGuiWindowFlags_NoResize;
        buildMenu("Camera Feed", &App::buildCameraSelectors, win_flags);

        if (pip_enabled) { 
            win_flags |= ImGuiWindowFlags_NoTitleBar;
            ImGui::SetNextWindowSize(ImVec2(app_params.pip_width+15, app_params.pip_height+15), ImGuiCond_Once);

            buildMenu("PiP Feed", &App::buildPiPWindow, win_flags);
        }

        updateOutputImage();
        glActiveTexture(1);
        glBindTexture(GL_TEXTURE_2D, overlay.id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, overlay.width, overlay.height, 0, GL_BGR, GL_UNSIGNED_BYTE, (GLvoid*)overlay.data.data());

        if (pip_enabled) {
            updatePipImage();
        }

        bg_shader.setBool("overlay_on", clutch_mode);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glBindVertexArray(img_surface.VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);

        // ImGui::EndFrame();

        ros::spinOnce();
        loop_rate.sleep();
    }
    robot_control.join();
    shutdown();

    return 0;
}
