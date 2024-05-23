#include <memory>
#include <atomic>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/terminal.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/component_base.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/component/loop.hpp>
#include <joint_trajectory_tui_node_parameters.hpp>

namespace ros2_viz_tui
{
class JointTrajectoryTuiNode : public rclcpp::Node
{
public:
  explicit JointTrajectoryTuiNode(const rclcpp::NodeOptions &);
  ~JointTrajectoryTuiNode();

private:
  std::atomic_bool m_request_refresh_tui;
  std::vector<int> m_joint_angular_position;

  std::unique_ptr<joint_trajectory_tui_node::ParamListener> m_param_listener;
  std::unique_ptr<joint_trajectory_tui_node::Params> m_params;

  rclcpp::TimerBase::SharedPtr m_refresh_tui_timer;

  std::unique_ptr<ftxui::Loop> m_loop;
  ftxui::ScreenInteractive m_screen;
  ftxui::Component m_renderer;
  ftxui::Component m_container;
  ftxui::Components m_sliders;

  void refreshTuiTimerCallback();
};

JointTrajectoryTuiNode::JointTrajectoryTuiNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("joint_trajectory_tui", options),
  m_screen(ftxui::ScreenInteractive::TerminalOutput())
{
  m_param_listener = std::make_unique<joint_trajectory_tui_node::ParamListener>(
    this->get_node_parameters_interface());
  m_params = std::make_unique<joint_trajectory_tui_node::Params>(
    m_param_listener->get_params());

  m_joint_angular_position.resize(3);

  m_sliders.push_back(
    ftxui::Slider("Slider :", &m_joint_angular_position[0], 0, 128, 1));
  m_sliders.push_back(
    ftxui::Slider("Slider :", &m_joint_angular_position[0], 0, 128, 1));

  m_container = ftxui::Container::Vertical(m_sliders);

  m_renderer = ftxui::Renderer(
    m_container,
    [&]() {
      ftxui::Elements joint_renders;
      for (auto && s : m_sliders) {
        if (joint_renders.empty()) {
          joint_renders.push_back(ftxui::separator());
        }
        joint_renders.push_back(s->Render());
      }
      return ftxui::hbox(
      {
        ftxui::vbox(joint_renders) | ftxui::xflex,
      }) | ftxui::flex | ftxui::border |
      ftxui::size(ftxui::WIDTH, ftxui::LESS_THAN, 80);
    }
  );
  m_renderer |= ftxui::CatchEvent(
    [](ftxui::Event) -> bool {
      std::this_thread::sleep_for(std::chrono::milliseconds(0));
      return false;
    }
  );
  m_loop = std::make_unique<ftxui::Loop>(&m_screen, m_renderer);

  const unsigned int refresh_tui_duration_milliseconds = 1e3 / m_params->refresh_frequency;
  m_refresh_tui_timer = this->create_wall_timer(
    std::chrono::milliseconds(refresh_tui_duration_milliseconds),
    std::bind(
      &JointTrajectoryTuiNode::refreshTuiTimerCallback,
      this));
}

JointTrajectoryTuiNode::~JointTrajectoryTuiNode()
{
  m_loop.reset(nullptr);
  m_screen.Exit();
}

void JointTrajectoryTuiNode::refreshTuiTimerCallback()
{
  for (auto && a : m_joint_angular_position) {
    if (1 > a) {
      continue;
    }
    a--;
    m_request_refresh_tui.store(true);
  }
  if (m_loop->HasQuitted()) {
    rclcpp::shutdown();
    return;
  }
  if (m_request_refresh_tui.load()) {
    m_screen.Post(ftxui::Event::Custom);
    m_request_refresh_tui.store(false);
  }
  m_loop->RunOnce();
}
}  // namespace ros2_viz_tui

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_viz_tui::JointTrajectoryTuiNode)
