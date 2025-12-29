#include <unordered_map>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "okay_robot_common/topic.hpp"
#include "okay_robot_view/time_series_data.hpp"
#include "okay_robot_view/view_node.hpp"

using std::placeholders::_1;

void glfw_error_callback_(int error, const char* description)
{
    std::cerr << "GLFW error " << error << ": " << description << std::endl;
}

void line_plot(TimeSeriesData time_data)
{
    // try to plot joint positions
    if (ImPlot::BeginPlot("Joint Positions", ImVec2(-1, -1))) {

        ImPlot::SetupAxes("time", "position (rad)");
        if (time_data.time.size() > 0)
            ImPlot::SetupAxesLimits(
                time_data.time.front(), time_data.time.back(), 0.0f, 2 * M_PIf, ImGuiCond_Always);
        for (auto& position_data : time_data.positions) {
            ImPlot::PlotLine(("j" + std::to_string(position_data.first)).c_str(),
                time_data.time.data(), position_data.second.data(), time_data.time.size());
        }
        ImPlot::EndPlot();
    }
}

void table_plots(TimeSeriesData time_data)
{
    if (ImGui::BeginTable("Joint Positions", 3, 0, ImVec2(-1, -1))) {
        ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_WidthFixed, 75.0f);
        ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthFixed, 75.0f);
        ImGui::TableSetupColumn("Plot");
        ImGui::TableHeadersRow();

        for (int i = 1; i <= 7; i++) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("j%d", i);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f rad", time_data.positions[i].front());
            ImGui::TableSetColumnIndex(2);
            ImGui::PushID(i);

            if (ImPlot::BeginPlot("##plot", ImVec2(-1, 104))) {

                ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoTickLabels);
                if (time_data.time.size() > 0)
                    ImPlot::SetupAxesLimits(time_data.time.front(), time_data.time.back(), 0.0f,
                        2 * M_PIf, ImGuiCond_Always);
                ImPlot::PlotLine("##plot", time_data.time.data(), time_data.positions[i].data(),
                    time_data.time.size());
                ImPlot::EndPlot();
            }

            ImGui::PopID();
        }

        ImGui::EndTable();
    }
}

void table_data(okay_robot_msgs::msg::ServoBusObservation::SharedPtr& observation)
{
    if (ImGui::BeginTable("Servo Data", 7, 0, ImVec2(-1, -1))) {
        ImGui::TableSetupColumn("Joint");
        ImGui::TableSetupColumn("Position");
        ImGui::TableSetupColumn("Speed");
        ImGui::TableSetupColumn("Load");
        ImGui::TableSetupColumn("Voltage");
        ImGui::TableSetupColumn("Temp");
        ImGui::TableSetupColumn("Current");
        ImGui::TableHeadersRow();

        for (auto& obs : observation->observations) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("j%d", obs.id);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f rad", obs.position);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f rad/s", obs.speed);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%.3f%%", obs.load);
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%.3fV", obs.voltage);
            ImGui::TableSetColumnIndex(5);
            ImGui::Text("%.3fC", obs.temperature);
            ImGui::TableSetColumnIndex(6);
            ImGui::Text("%.3fmA", obs.current * 1000.0f);
        }

        ImGui::EndTable();
    }
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

    static ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;

    while (!(glfwWindowShouldClose(window) || shutdown_flag.load())) {
        glfwPollEvents();

        // start frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // make the plot fill up the window
        static int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(display_w, display_h));

        ImGui::Begin("okay_robot", nullptr, window_flags);

        std::lock_guard<std::mutex> lock(mutex);
        std::vector<std::shared_ptr<okay_robot_msgs::msg::ServoBusObservation>> data_vector
            = buffer.to_vector();
        TimeSeriesData time_data(buffer.to_vector());

        if (ImGui::BeginTabBar("ImPlotDemoTabs")) {
            if (ImGui::BeginTabItem("All Data")) {
                table_data(data_vector.front());
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Positions Plot")) {
                line_plot(time_data);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Positions Plot Table")) {
                table_plots(time_data);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
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
    // only add observation if it has all 7 servos (for now)
    if (msg->observations.size() == 7) {
        std::lock_guard<std::mutex> lock(this->mutex_);
        this->observations_.push_back(msg);
    }
}