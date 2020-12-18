// Standard libraries
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <poll.h>
#include <netinet/in.h>
#include <thread>

#include <chrono> // TEST

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
#include "viewpoint_interface/json.hpp"
#include "viewpoint_interface/viewpoint_interface.hpp"
#include "viewpoint_interface/shader.hpp"
#include "viewpoint_interface/mesh.hpp"
#include "viewpoint_interface/model.hpp"
#include "viewpoint_interface/object.hpp"

using json = nlohmann::json;
using App = viewpoint_interface::App;
using AppParams = viewpoint_interface::AppParams;
using MMesh = viewpoint_interface::Mesh;
using Image = viewpoint_interface::Image;
using Socket = viewpoint_interface::Socket;


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


// TEST
auto start = std::chrono::high_resolution_clock::now();
void startTiming()
{
    start = std::chrono::high_resolution_clock::now();
}

void stopTiming()
{
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Time: " << duration.count() << std::endl;
}


// void mismatchDisplays(uint &disp1, uint &disp2, uint size)
// {
//     if (disp1 == disp2) {
//         disp2 = getNextIndex(disp2, size);
//     }
// }

// void nextDisplay(uint &disp1, uint &disp2, uint size, bool bump)
// {
//     if (size < 2) {
//         disp1 = disp2 = 0;
//     }
//
//     if (bump) {
//         disp1 = getNextIndex(disp1, size);
//         mismatchDisplays(disp1, disp2, size);
//     }
//     else {
//         uint next_disp = getNextIndex(disp1, size);
//         if (next_disp == disp2) {
//             next_disp = getNextIndex(next_disp, size);
//         }
//
//         disp1 = next_disp;
//     }
// }

// void previousDisplay(uint &disp1, uint &disp2, uint size, bool bump)
// {
//     if (size < 2) {
//         disp1 = disp2 = 0;
//     }
//
//     if (bump) {
//         disp1 = getPreviousIndex(disp1, size);
//         mismatchDisplays(disp1, disp2, size);
//     }
//     else {
//         uint next_disp = getPreviousIndex(disp1, size);
//         if (next_disp == disp2) {
//             next_disp = getPreviousIndex(next_disp, size);
//         }
//
//         disp1 = next_disp;
//     }
// }

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
        std::string int_name, ext_name, topic_name;
        uint w, h, c;

        int_name = j[cam_id]["internal_name"];
        ext_name = j[cam_id]["external_name"];
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

        layouts.getDisplays().addDisplay(Display(int_name, ext_name, topic_name, DisplayDims(w, h, c)));
    }

    out_img = Image(max_width, max_height, max_channels);

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
    ros::init(argc, argv, "viewpoint_interface");
    ros::NodeHandle n;

    DisplayManager displays(layouts.getDisplays());
    
    for (int i = 0; i < displays.size(); i++) {
        ros::Subscriber cam_sub(n.subscribe<sensor_msgs::Image>(displays[i].getTopicName(), 10, 
                boost::bind(&App::cameraImageCallback, this, _1, i)));
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

    window = glfwCreateWindow(win_width, win_height, "Viewpoint Selection Interface", NULL, NULL);
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
    style.ItemInnerSpacing = ImVec2(7.0, 2.0);
    style.WindowRounding = 6.0;
    // if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    //     style.WindowRounding = 0.0f;
    //     style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    // }

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void App::shutdownApp()
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


// Input handling
std::string getSocketData(Socket &sock)
{
    int len_data;
    len_data = recvfrom(sock.sock, sock.buffer, sock.DATA_SIZE, MSG_WAITALL, (sockaddr *) &(sock.addr), &(sock.len)); 
    sock.buffer[len_data] = '\0';
    std::string data = sock.buffer;

    return data;
}

// TODO: Change all robot control code
// void App::parseControllerInput(std::string data)
// {
//     std::string CONTR_NAME = app_params.CONTR_NAME;
//
//     json j = json::parse(data);
//
//     input.clutching = j[CONTR_NAME]["clutch"]["boolean"];
//
//     input.manual_adj = j[CONTR_NAME]["manual_adj"]["boolean"];
//     input.manual_offset.x = j[CONTR_NAME]["manual_adj"]["2d"]["x"];
//     input.manual_offset.z = j[CONTR_NAME]["manual_adj"]["2d"]["y"];
//
//     if (!input.initialized) {
//     }
//
//     // Handle clutching enabled by keyboard
//     if (clutch_mode && !input.clutching.is_flipping() && !input.clutching.is_on()) {
//         input.clutching.turn_on();
//     }
//
//     if (input.clutching.is_flipping()) {
//         if (input.clutching.is_on()) { // When just turned on
//             clutch_mode = true;
//             // TODO: Add orientation handling
//         }
//         else {
//             clutch_mode = false;
//         }
//     }
//
//     if (input.manual_adj.confirm_flip_on()) {
//         if (input.manual_offset.x >= 0.5) {
//             nextDisplay(active_display, pip_display, disp_info.size());
//         }
//         else if (input.manual_offset.x <= -0.5) {
//             previousDisplay(active_display, pip_display, disp_info.size());
//         }
//         else if (input.manual_offset.z >= 0.5) {
//             nextDisplay(pip_display, active_display, disp_info.size(), false);
//         }
//         else if (input.manual_offset.z <= -0.5) {
//             previousDisplay(pip_display, active_display, disp_info.size(), false);
//         }
//         else {
//             pip_enabled = !pip_enabled;
//         }
//     }
// }

// TODO: No longer robot control--only handles controller input
// void App::handleRobotControl()
// {
//     pollfd poll_fds;
//     poll_fds.fd = sock.sock;
//     poll_fds.events = POLLIN; // Wait until there's data to read
//
//     while (ros::ok() && !glfwWindowShouldClose(window))
//     {
//         if (poll(&poll_fds, 1, app_params.loop_rate) > 0) {
//             std::string input_data = getSocketData(sock);
//             parseControllerInput(input_data);
//             printText(input.to_str(true));        
//         }
//     }
//
//     shutdown(sock.sock, SHUT_RDWR);
// }


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

void App::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
    else if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
        layouts.getDisplays().toNextPrimaryDisplay();
    }
    else if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
        layouts.getDisplays().toPrevPrimaryDisplay();
    }
    else if (key == GLFW_KEY_C && action == GLFW_PRESS) {
        clutch_mode = !clutch_mode;
    }
    else if (key == GLFW_KEY_P && action == GLFW_PRESS) {
        // TODO: Enable PiP
    }
}


// -- OpenGL and Dear ImGui --
void Image::generateEmptyTexture(uint tex_num)
{
    glGenTextures(1, &id);
    glActiveTexture(GL_TEXTURE0 + tex_num);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glActiveTexture(GL_TEXTURE0);
}

void App::updateOutputImage()
{    
    const std::vector<uchar> &buffer(layouts.getDisplays().getDataVectorForRole(DisplayRole_Primary));

    glActiveTexture(0);
    glBindTexture(GL_TEXTURE_2D, 1); // TODO: ID should be a variable
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, out_img.width, out_img.height, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)buffer.data());
}

// void App::updatePipImage()
// {
//     DDisplay cur_disp = disp_info.at(pip_display);
//     uint disp_size = cur_disp.image.size;
//
//     if (disp_size != out_img.size) {
//         // TODO: Figure out how to deal with different sizes
//     }
//
//     cv::Mat flip_mat = cv::Mat(cur_disp.image.width, cur_disp.image.height, CV_8UC3, cur_disp.image.data.data());
//     cv::flip(flip_mat, flip_mat, 0);
//
//     pip_img.resize(cur_disp.image.width, cur_disp.image.height, cur_disp.image.channels);
//     pip_img.copy_data(flip_mat);
//     glActiveTexture(2);
//     glBindTexture(GL_TEXTURE_2D, pip_img.id);
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pip_img.width, pip_img.height, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)pip_img.data.data());
// }

MMesh generateSquare()
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

    MMesh mesh;

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

void App::buildMenu(std::string title, void (App::*build_func)(void), ImGuiWindowFlags window_flags)
{

    if (!ImGui::Begin(title.c_str(), (bool *)NULL, window_flags)) {
        ImGui::End();
        return;    
    }

    ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.35f);
    (this->*build_func)();
    ImGui::PopItemWidth();
    ImGui::End();
}

void App::buildLayoutsMenu()
{
    std::shared_ptr<Layout> active_layout = layouts.getActiveLayout();

    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu(active_layout->getLayoutName().c_str())) {
            std::vector<std::string> layout_names = layouts.getLayoutList();
            bool selected;
            bool inactivate = false;

            selected = layouts.isLayoutActive(LayoutType::NONE);
            ImGui::MenuItem("Inactivate", NULL, &selected);
            if (selected)
            {
                layouts.activateLayout(LayoutType::NONE);
            }

            for (int i = 0; i < layout_names.size(); i++) {
                LayoutType layout_type = layouts.intToLayoutType(i);
                
                if (layouts.isLayoutExcluded(layout_type)) {
                    continue;
                }

                selected = layouts.isLayoutActive(layout_type);
                ImGui::MenuItem(layout_names[i].c_str(), NULL, &selected);

                if (selected) {
                    layouts.activateLayout(layout_type);
                }
            }

            ImGui::EndMenu();
        }

        ImGui::EndMenuBar();
    }

    ImGui::Text("Parameters for %s:", active_layout->getLayoutName().c_str());
    ImGui::Spacing();
    ImGui::Spacing();

    active_layout->displayLayoutParams();
}

// void App::buildDisplaySelectors()
// {
//     const char *disp_preview = disp_info.at(active_display).name.c_str(); 
//     if (ImGui::BeginCombo("Primary", disp_preview, ImGuiComboFlags_None))
//     {
//         for (int n = 0; n < disp_info.size(); n++)
//         {
//             const char *cam_name = disp_info.at(n).name.c_str();
//             const bool is_selected = (active_display == n);
//             if (ImGui::Selectable(cam_name, is_selected)) {
//                 active_display = n;
//             }

//             // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
//             if (is_selected) {
//                 ImGui::SetItemDefaultFocus();
//             }
//         }

//         ImGui::EndCombo();
//     }

//     // mismatchDisplays(active_display, pip_display, disp_info.size());

//     if (disp_info.size() > 1) {
//         ImGui::Checkbox("Pic-in-Pic Enabled", &pip_enabled);
//         if (pip_enabled) {
//             const char *pip_preview = pip_preview = disp_info.at(pip_display).name.c_str(); 
//             if (ImGui::BeginCombo("Pic-in-Pic", pip_preview, ImGuiComboFlags_None))
//             {
//                 for (int n = 0; n < disp_info.size(); n++)
//                 {
//                     // Skip currently active main display
//                     if (active_display == n) {
//                         continue;
//                     }

//                     const char *cam_name = disp_info.at(n).name.c_str();
//                     const bool is_selected = (pip_display == n);
//                     if (ImGui::Selectable(cam_name, is_selected)) {
//                         pip_display = n;
//                     }

//                     // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
//                     if (is_selected) {
//                         ImGui::SetItemDefaultFocus();
//                     }
//                 }

//                 ImGui::EndCombo();
//             }
//         }
//     }
// }

void App::buildPiPWindow()
{
    ImTextureID img_id = (ImTextureID)(pip_img.id);
    ImGui::Image(img_id, ImVec2(app_params.pip_width, app_params.pip_height));
}


// -- ROS Callbacks --

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

    // TEST
    // if (layouts.getDisplays().getPrimaryDisplayIx() == index) {
        layouts.getDisplays().fillBufferForDisplayIx(index, cur_img->image);
    // }

    std::cout << std::endl;

    // TODO: This should probably call a function to check whether a fill is needed
    // and perform the fill, rather than handling it manually

    // cv::Mat image = cur_img->image;
    // int data_size = image.cols * image.rows * image.channels();
    // if (data_size > disp_info.at(index).image.size) {
    //     disp_info.at(index).image.resize(image.cols, image.rows, image.channels());
    // }
    // disp_info.at(index).image.copy_data(image);
}


int App::run(int argc, char *argv[])
{
    // Change working directory so we can specify resources more easily
    // NOTE: This depends on the 'cwd' param of the launch file being set to "node"
    chdir("../../../src/camera_viewpoint_interface");

    if (!initialize(argc, argv)) {
        return -1;
    }

    // TODO: 
    // Create a pool with full set of displays (max 9)
    // For now, choose 1024*1024*3 bytes
    // Anytime that a display is activated, just start piping data
    // to the appropriate display slot

    // Set up display textures
    MMesh img_surface = generateSquare();
    out_img.generateEmptyTexture(1);

    uint overlay = TextureFromFile("clutch_mode_overlay.png", "resources/textures/");
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, overlay);
    
    // pip_img = Image();
    // pip_img.generateEmptyTexture(3);

    // -- Set up shaders --
    std::string base_path("resources/shaders/");

    Shader bg_shader((base_path + "bg_shader.vert").c_str(), (base_path + "bg_shader.frag").c_str());
    bg_shader.use();
    bg_shader.setInt("Texture", 1);
    bg_shader.setInt("Overlay", 2);
    // ----


    // Split robot control into its own thread to improve performance
    // std::thread robot_control(&App::handleRobotControl, this);


    ros::Rate loop_rate(app_params.loop_rate);
    while (ros::ok() && !glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // --- Build ImGui windows
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::ShowDemoWindow();
        
        ImGuiWindowFlags win_flags = 0;
        // win_flags |= ImGuiWindowFlags_NoScrollbar;
        // win_flags |= ImGuiWindowFlags_NoResize;
        // buildMenu("Displays", &App::buildDisplaySelectors, win_flags);

        win_flags = 0;
        win_flags |= ImGuiWindowFlags_NoScrollbar;
        win_flags |= ImGuiWindowFlags_NoResize;
        win_flags |= ImGuiWindowFlags_MenuBar;
        win_flags |= ImGuiWindowFlags_AlwaysAutoResize;
        ImGuiViewport* main_viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(main_viewport->GetWorkPos().x + 30, main_viewport->GetWorkPos().y + 30), ImGuiCond_Always);
        // ImGui::SetNextWindowSize(ImVec2(200, 250), ImGuiCond_Always);
        buildMenu(layouts.window_title, &App::buildLayoutsMenu, win_flags);


        // if (pip_enabled) {
        //     win_flags = 0;
        //     win_flags |= ImGuiWindowFlags_NoScrollbar;
        //     win_flags |= ImGuiWindowFlags_NoResize;
        //     win_flags |= ImGuiWindowFlags_NoCollapse;
        //     win_flags |= ImGuiWindowFlags_NoSavedSettings;
        //     ImGui::SetNextWindowSize(ImVec2(app_params.pip_width+15, app_params.pip_height+15), ImGuiCond_Once);
        //     ImGui::SetNextWindowPos(ImVec2(app_params.WINDOW_WIDTH - app_params.pip_width-120, app_params.WINDOW_HEIGHT - app_params.pip_height-100));
        //     DDisplay pip_disp = disp_info.at(pip_display);
        //     std::string disp_name = pip_disp.display_name;
        //     buildMenu(disp_name.c_str(), &App::buildPiPWindow, win_flags);
        // }
        // ---

        updateOutputImage();

        // if (pip_enabled) {
        //     updatePipImage();
        // }

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

        startTiming();
        ros::spinOnce();
        stopTiming();

        loop_rate.sleep();
    }
    // robot_control.join();
    shutdownApp();

    return 0;
}
