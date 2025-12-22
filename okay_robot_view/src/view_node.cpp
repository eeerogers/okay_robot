#include <unordered_map>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "okay_robot_common/topic.hpp"
#include "okay_robot_view/view_node.hpp"

using std::placeholders::_1;

void glfw_error_callback_(int error, const char* description)
{
    std::cerr << "GLFW error " << error << ": " << description << std::endl;
}

void spin_window(CircularBuffer<okay_robot_msgs::msg::ServoBusObservation::SharedPtr>& buffer,
    std::atomic<bool>& shutdown_flag, std::mutex& mutex)
{
    // gui setup stuff
    glfwSetErrorCallback(glfw_error_callback_);
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1200, 800, "Example", nullptr, nullptr);
    if (!window) {
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // disable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGui::StyleColorsDark();

    const char* glsl_version = "#version 130";
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;

    while (!(glfwWindowShouldClose(window) || shutdown_flag.load())) {
        glfwPollEvents();

        // start frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // make the plot fill up the window
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(display_w, display_h));

        ImGui::Begin("okay_robot", nullptr, window_flags);

        // try to plot joint positions
        if (ImPlot::BeginPlot("Joint Positions", ImVec2(-1, -1))) {
            std::lock_guard<std::mutex> lock(mutex);

            std::vector<float> times;
            std::unordered_map<int, std::vector<float>> positions;
            for (const auto& bus_obs : buffer) {
                // just skip if the full joint space is not represented for now
                if (bus_obs->observations.size() != 7)
                    continue;

                times.push_back(bus_obs->observations[0].time);
                for (const auto& obs : bus_obs->observations) {
                    if (positions.find(obs.id) != positions.end()) {
                        positions[obs.id].push_back(obs.position);
                    } else {
                        positions[obs.id] = std::vector<float>({ obs.position });
                    }
                }
            }

            ImPlot::SetupAxes("time", "position (rad)");
            ImPlot::SetupAxesLimits(times.front(), times.back(), 0.0f, 2 * M_PIf, ImGuiCond_Always);
            for (auto& plot : positions) {
                ImPlot::PlotLine(("j" + std::to_string(plot.first)).c_str(), times.data(),
                    plot.second.data(), times.size());
            }
            ImPlot::EndPlot();
        }

        ImGui::End();
        ImGui::PopStyleVar();

        // render
        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    shutdown_flag.store(true);
}

ViewNode::ViewNode()
    : Node("servo_bus_node")
{
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->poll_freq_));
    this->timer_ = this->create_wall_timer(duration, std::bind(&ViewNode::timer_callback_, this));

    this->servo_bus_observation_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::ServoBusObservation>(
            TOPIC_SERVO_BUS_OBSERVATION, 10,
            std::bind(&ViewNode::servo_bus_observation_subscriber_callback_, this, _1));

    this->gui_shutdown_flag_.store(false);
    auto gui_thread = std::bind(spin_window, std::ref(this->observations_),
        std::ref(this->gui_shutdown_flag_), std::ref(this->mutex_));
    this->gui_thread_ = std::thread(gui_thread);
}

ViewNode::~ViewNode()
{
    this->gui_shutdown_flag_.store(true);
    this->gui_thread_.join();
}

void ViewNode::timer_callback_()
{
    if (this->gui_shutdown_flag_.load()) {
        rclcpp::shutdown();
    }
}

void ViewNode::servo_bus_observation_subscriber_callback_(
    const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->observations_.push_back(msg);
}