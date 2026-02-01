#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "webrtc_receiver.hpp"

#ifdef HAVE_VISUALIZER
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#endif

static void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "VI-SLAM PC Client" << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "Eigen version: " << EIGEN_WORLD_VERSION << "."
              << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

#ifdef HAVE_CERES
    std::cout << "Ceres Solver: Available" << std::endl;
#else
    std::cout << "Ceres Solver: Not available" << std::endl;
#endif

#ifdef HAVE_ZMQ
    std::cout << "ZeroMQ: Available" << std::endl;
#else
    std::cout << "ZeroMQ: Not available" << std::endl;
#endif

#ifdef HAVE_VISUALIZER
    std::cout << "Visualizer: Available" << std::endl;

    // Setup GLFW
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return 1;
    }

    // GL 3.3 + GLSL 330
    const char* glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "VI-SLAM PC Client Dashboard", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
#ifdef IMGUI_HAS_DOCK
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
#endif

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    std::cout << "\nDear ImGui initialized successfully" << std::endl;
    std::cout << "ImGui version: " << IMGUI_VERSION << std::endl;

    // Initialize WebRTC receiver
    std::cout << "\nInitializing WebRTC receiver..." << std::endl;
    vi_slam::WebRTCReceiver receiver;

    if (!receiver.initialize()) {
        std::cerr << "Failed to initialize WebRTC receiver" << std::endl;
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // Set up callbacks
    int frameCount = 0;
    int imuCount = 0;

    receiver.setVideoCallback([&frameCount](const cv::Mat& frame, int64_t timestamp) {
        if (++frameCount % 30 == 0) {  // Print every 30 frames
            std::cout << "Received frame: " << frame.cols << "x" << frame.rows
                      << " at " << timestamp << std::endl;
        }
    });

    receiver.setIMUCallback([&imuCount](const vi_slam::IMUSample& imu) {
        if (++imuCount % 100 == 0) {  // Print every 100 samples
            std::cout << "Received IMU: acc=(" << imu.accX << ", " << imu.accY
                      << ", " << imu.accZ << ") gyro=(" << imu.gyroX << ", "
                      << imu.gyroY << ", " << imu.gyroZ << ")" << std::endl;
        }
    });

    // Default signaling URL or from command line
    std::string signalingUrl = "ws://localhost:8080";
    if (argc > 1) {
        signalingUrl = argv[1];
    }

    bool connected = false;
    bool showDemoWindow = true;
    bool showMainWindow = true;
    ImVec4 clearColor = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    std::cout << "\nEntering main loop. Close window to exit." << std::endl;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Show Dear ImGui demo window
        if (showDemoWindow) {
            ImGui::ShowDemoWindow(&showDemoWindow);
        }

        // Main dashboard window
        if (showMainWindow) {
            ImGui::Begin("VI-SLAM PC Client Dashboard", &showMainWindow);

            ImGui::Text("Welcome to VI-SLAM PC Client!");
            ImGui::Separator();

            // Connection section
            ImGui::Text("Connection");
            ImGui::Text("Signaling URL: %s", signalingUrl.c_str());

            if (!connected) {
                if (ImGui::Button("Connect")) {
                    std::cout << "Connecting to signaling server: " << signalingUrl << std::endl;
                    if (receiver.connect(signalingUrl)) {
                        connected = true;
                        std::cout << "Connected!" << std::endl;
                    } else {
                        std::cerr << "Failed to connect" << std::endl;
                    }
                }
            } else {
                ImGui::Text("Status: Connected");
                if (ImGui::Button("Disconnect")) {
                    std::cout << "Disconnecting..." << std::endl;
                    receiver.disconnect();
                    connected = false;
                    std::cout << "Disconnected" << std::endl;
                }
            }

            ImGui::Separator();

            // Statistics section
            ImGui::Text("Statistics");
            ImGui::Text("Frames received: %d", frameCount);
            ImGui::Text("IMU samples received: %d", imuCount);
            ImGui::Text("FPS: %.1f", io.Framerate);

            ImGui::Separator();

            // Demo window toggle
            ImGui::Checkbox("Show ImGui Demo Window", &showDemoWindow);

            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clearColor.x * clearColor.w, clearColor.y * clearColor.w,
                     clearColor.z * clearColor.w, clearColor.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    std::cout << "\nCleaning up..." << std::endl;
    if (connected) {
        receiver.disconnect();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "Done." << std::endl;
    return 0;

#else
    std::cout << "Visualizer: Not available (rebuild with GLFW support)" << std::endl;
    std::cout << "\nInitializing WebRTC receiver..." << std::endl;

    // Fallback to console mode (original implementation)
    vi_slam::WebRTCReceiver receiver;

    if (!receiver.initialize()) {
        std::cerr << "Failed to initialize WebRTC receiver" << std::endl;
        return 1;
    }

    receiver.setVideoCallback([](const cv::Mat& frame, int64_t timestamp) {
        static int frameCount = 0;
        if (++frameCount % 30 == 0) {
            std::cout << "Received frame: " << frame.cols << "x" << frame.rows
                      << " at " << timestamp << std::endl;
        }
    });

    receiver.setIMUCallback([](const vi_slam::IMUSample& imu) {
        static int imuCount = 0;
        if (++imuCount % 100 == 0) {
            std::cout << "Received IMU: acc=(" << imu.accX << ", " << imu.accY
                      << ", " << imu.accZ << ") gyro=(" << imu.gyroX << ", "
                      << imu.gyroY << ", " << imu.gyroZ << ")" << std::endl;
        }
    });

    std::string signalingUrl = "ws://localhost:8080";
    if (argc > 1) {
        signalingUrl = argv[1];
    }

    std::cout << "Connecting to signaling server: " << signalingUrl << std::endl;
    if (!receiver.connect(signalingUrl)) {
        std::cerr << "Failed to connect" << std::endl;
        return 1;
    }

    std::cout << "Connected! Press Enter to stop..." << std::endl;
    std::cin.get();

    std::cout << "Disconnecting..." << std::endl;
    receiver.disconnect();

    std::cout << "Done." << std::endl;
    return 0;
#endif
}
