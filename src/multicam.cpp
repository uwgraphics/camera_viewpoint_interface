#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <relaxed_ik/EEPoseGoals.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// TEST
#include <visualization_msgs/Marker.h>

#include <glad/glad.h> 
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "multicam_mimicry/json.hpp"
#include "multicam_mimicry/multicam.hpp"
#include "multicam_mimicry/shader.hpp"


#define PORT        8080 
#define MAXLINE     2048
#define LOOPRATE    120
#define CONTR_NAME  "vive_controller"

#define WIN_HEIGHT  900
#define WIN_WIDTH   1200

using json = nlohmann::json;
using EEPoseGoals = relaxed_ik::EEPoseGoals;
using Pose = geometry_msgs::Pose;
using Bool = std_msgs::Bool;
using ControllerInput = goal_publisher::ControllerInput;

int tex_size = 1024 * 1024;
GLubyte* tex_buffer;
ControllerInput::ActiveCamera active_cam = ControllerInput::STATIC;


/**
 * Print a string to the screen.
 * 
 * Params:
 * 		text - text to print
 * 		newlines - number of newlines to print at the end of input string
 * 		flush - whether to flush text. Generally, only needed when
 * 			printing a large body of text with manual newlines
 */
void printText(std::string text="", int newlines=1, bool flush=false)
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

void glfw_error_callback(int code, const char* description)
{
    printText("GLFW Error: " + code);
    printText(description);
}

glm::vec3 positionToRobotFrame(glm::vec3 v)
{
    return glm::vec3(-v.y, -v.z, v.x);
}

glm::quat orientationToRobotFrame(ControllerInput input)
{
    glm::quat in = input.orientation;

    glm::vec3 new_euler = glm::vec3(glm::pitch(in), -glm::roll(in), -glm::yaw(in));
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

void parseInput(std::string data, ControllerInput& input)
{
    json j = json::parse(data);

    auto pos = j[CONTR_NAME]["pose"]["position"];
    glm::vec3 pos_vec = glm::vec3(pos["x"], pos["y"], pos["z"]);

    auto quat = j[CONTR_NAME]["pose"]["orientation"];
    glm::quat quat_vec = glm::quat(quat["w"], quat["x"], quat["y"], quat["z"]);

    input.gripping = j[CONTR_NAME]["gripper"]["boolean"];
    input.clutching = j[CONTR_NAME]["clutch"]["boolean"];

    input.camera_mode = j[CONTR_NAME]["manual_adj"]["boolean"];
    input.manual_offset.z = j[CONTR_NAME]["manual_adj"]["2d"]["x"];
    input.manual_offset.x = j[CONTR_NAME]["manual_adj"]["2d"]["y"];

    input.reset = j[CONTR_NAME]["reset"]["boolean"];
    // input.switch_cam = j[CONTR_NAME]["switch_cam"]["boolean"];

    if (!input.initialized) {
        input.init_pos = pos_vec;
        input.prev_pos = pos_vec;
        input.init_orient = quat_vec;

        input.inv_init_quat = glm::inverse(quat_vec);

        input.initialized = true;
    }

    if (input.clutching.is_flipping()) {
        if (input.clutching.is_on()) { // When just turned on
            input.clutch_offset = pos_vec - input.init_pos;
            // TODO: Add orientation handling
        }
        else {
            input.init_pos = pos_vec - input.clutch_offset;
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
        input.manual_offset = positionToRobotFrame(input.manual_offset);
        input.orientation = orientationToRobotFrame(input);
    }

    if (input.change_cam.is_flipping()) {
        if (active_cam == input.DYNAMIC) {
            active_cam = input.STATIC;
        }
        else {
            active_cam = input.DYNAMIC;
        }
    }
}

std::string getData(int sock, char *buffer, sockaddr_in &addr, socklen_t len)
{
    int len_data;

    len_data = recvfrom(sock, buffer, MAXLINE, MSG_WAITALL, (sockaddr *) &addr, &len); 
    buffer[len_data] = '\0';
    std::string data = buffer;

    return data;
}

void cameraCallback(const geometry_msgs::Pose::ConstPtr& msg, ControllerInput& input)
{
    auto pos = msg->position;
    auto quat = msg->orientation;
    input.cam_pos = glm::vec3(pos.x, pos.y, pos.z);
    input.cam_orient = glm::quat(quat.w, quat.x, quat.y, quat.z);
}

void dynamicImageCb(const sensor_msgs::ImageConstPtr& msg, ControllerInput& input)
{
    if (!input.dyn_valid) {
        input.dyn_valid = true;
    }
    
    if (active_cam != input.DYNAMIC) {
        return;
    }

    cv_bridge::CvImageConstPtr cur_img;
    try
    {
        cur_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        printText("cv_bridge exception: %s", 0);
        printText(e.what());
        input.dyn_valid = false;
        return;
    }

    cv::Mat image = cur_img->image;
    int data_size = image.cols * image.rows * image.channels();
    if (data_size > tex_size) {
        delete[] tex_buffer;
        tex_size = data_size;
        tex_buffer = new GLubyte[tex_size];
    }
    memcpy(tex_buffer, cur_img->image.data, data_size);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)tex_buffer);         
}

void staticImageCb(const sensor_msgs::ImageConstPtr& msg, ControllerInput& input)
{
    if (!input.stat_valid) {
        input.stat_valid = true;
    }
    
    if (active_cam != input.STATIC) {
        return;
    }

    cv_bridge::CvImageConstPtr cur_img;
    try
    {
        cur_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        printText("cv_bridge exception: %s", 0);
        printText(e.what());
        input.stat_valid = false;
        return;
    }

    cv::Mat image = cur_img->image;
    int data_size = image.cols * image.rows * image.channels();
    if (data_size > tex_size) {
        delete[] tex_buffer;
        tex_size = data_size;
        tex_buffer = new GLubyte[tex_size];
    }
    memcpy(tex_buffer, cur_img->image.data, data_size);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)tex_buffer);
}


void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window, ControllerInput input)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    if (glfwGetKey(window,GLFW_KEY_1) == GLFW_PRESS) {
        active_cam = input.DYNAMIC;
    }
    else if (glfwGetKey(window,GLFW_KEY_2) == GLFW_PRESS) {
        active_cam = input.STATIC;
    }
    
}

bool showCameraSelector(ControllerInput input)
{
    static int cam_idx = 1;
    const char* label = "Camera";

    if (active_cam == input.DYNAMIC) {
        cam_idx = 0;
    }
    else {
        cam_idx = 1;
    }

    if (ImGui::Combo(label, &cam_idx, "Dynamic\0Static\0"))
    {
        switch (cam_idx)
        {
            case 0: 
            {
                active_cam = input.DYNAMIC;
            }   break;
            case 1:
            {
                active_cam = input.STATIC;
            }   break;
        }

        return true;
    }

    return false;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_openvr");
    ros::NodeHandle n;
    ControllerInput input;

    // Change cwd so we can specify resources more easily
    chdir("../../../src/openvr_ros");

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

    // TEST
    ros::Publisher marker_pub(n.advertise<visualization_msgs::Marker>("visualization_marker", 1000));
    ros::Publisher ee_pub(n.advertise<relaxed_ik::EEPoseGoals>("/relaxed_ik/ee_pose_goals", 1000));
    ros::Publisher gripper_pub(n.advertise<std_msgs::Bool>("/relaxed_ik/gripper_state", 1000));
    ros::Publisher reset_pub(n.advertise<std_msgs::Bool>("/relaxed_ik/reset_pose", 1000));

    ros::Subscriber cam_sub(n.subscribe<geometry_msgs::Pose>("/cam/pose_actual", 10, boost::bind(cameraCallback, _1, std::ref(input))));
    ros::Subscriber dyn_sub(n.subscribe<sensor_msgs::Image>("/cam/dyn_image", 10, boost::bind(dynamicImageCb, _1, std::ref(input))));
    ros::Subscriber stat_sub(n.subscribe<sensor_msgs::Image>("/cam/stat_image", 10, boost::bind(staticImageCb, _1, std::ref(input))));

    if (!glfwInit()) {
        printText("Could not initialize GLFW!");
        return -1;
    }
    glfwSetErrorCallback(glfw_error_callback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "OpenVR ROS", NULL, NULL);
    if (!window) {
        printText("Could not create window.");
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        printText("Failed to initialize GLAD");
        return -1;
    }
    glViewport(0, 0, WIN_WIDTH, WIN_HEIGHT);
    // glEnable(GL_DEPTH_TEST);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);


    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Init data feed socket
    // TODO: Consider putting these into a struct
    // int sock;
    // sockaddr_in addr;
    // socklen_t len;
    // char buffer[MAXLINE];
    // {
    //     if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
    //         printText("Could not initialize socket.");
    //         return 1;
    //     }

    //     memset(&addr, 0, sizeof(addr));
    //     addr.sin_family = AF_INET; 
    //     addr.sin_addr.s_addr = INADDR_ANY;
    //     addr.sin_port = htons(PORT);

    //     // TODO: Allow exiting while this blocks
    //     if (bind(sock, (const sockaddr *)&addr, sizeof(addr)) < 0) { 
    //         printText("Binding failed"); 
    //         return 1;
    //     }
    // }


    // Get initial values
    // std::string data;
    // data = getData(sock, buffer, addr, len);
    // parseInput(data, input);

    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    uint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture); 
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    
    Shader bg_shader("resources/shaders/bg_shader.vert", "resources/shaders/bg_shader.frag");
    bg_shader.use();
    bg_shader.setInt("Texture", 0);

    tex_buffer = new GLubyte[tex_size];
    memset(tex_buffer, 0, tex_size);

    ros::Rate loop_rate(LOOPRATE);
    while (ros::ok() && !glfwWindowShouldClose(window))
    {
        // Window rendering
        processInput(window, input);

        // if (input.dyn_valid && input.stat_valid) {   
        //     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, input.texture.width, input.texture.height, 0, GL_BGR, GL_UNSIGNED_BYTE, (GLvoid*)input.texture.data);         
        //     // glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, input.texture.width, input.texture.height, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)input.texture.data);
        //     printText("Error: " + std::to_string(glGetError()));
        //     // glGenerateMipmap(GL_TEXTURE_2D);
        // }


        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // ImGui::ShowDemoWindow();

        // Camera switching window
        ImGui::Begin("Camera Feed");
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.6f);
        showCameraSelector(input);
        ImGui::PopItemWidth();
        ImGui::End();


        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();


        // data = getData(sock, buffer, addr, len);
        // parseInput(data, input);

        // printText(input.to_str(true));

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

        ros::spinOnce();
        loop_rate.sleep();
    }
    if (ros::ok()) {
        ros::shutdown();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
