// MIT License
//
// Copyright (c) 2024 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <urdf_model/model.h>
#include <urdf_world/world.h>
#include <urdf_parser/urdf_parser.h>

#include <memory>
#include <atomic>
#include <string>
#include <deque>
#include <vector>
#include <chrono>
#include <thread>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/terminal.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/component_base.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/component/loop.hpp>
#include <ftxui/component/mouse.hpp>
#include <joint_trajectory_tui_node_parameters.hpp>

namespace ros2_tviz
{
class JointTrajectoryTuiNode : public rclcpp::Node
{
public:
  JointTrajectoryTuiNode() = delete;
  explicit JointTrajectoryTuiNode(const rclcpp::NodeOptions &);
  JointTrajectoryTuiNode(const JointTrajectoryTuiNode &) = delete;
  JointTrajectoryTuiNode(JointTrajectoryTuiNode &&) = delete;
  ~JointTrajectoryTuiNode();

  JointTrajectoryTuiNode & operator=(const JointTrajectoryTuiNode &) = delete;
  JointTrajectoryTuiNode & operator=(JointTrajectoryTuiNode &&) = delete;

private:
  std::vector<std::string> proc_joint_names_;

  std::vector<float> joint_angular_positions_;
  std::vector<float> joint_angular_velocities_;
  std::vector<float> joint_angular_effort_;

  std::vector<float> dest_joint_angular_positions_;
  std::vector<float> dest_joint_angular_velocities_;
  std::vector<float> dest_joint_angular_effort_;

  std_msgs::msg::String::ConstSharedPtr last_robot_description_;
  sensor_msgs::msg::JointState::ConstSharedPtr last_joint_states_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;

  rclcpp::TimerBase::SharedPtr refresh_tui_timer_;

  std::unique_ptr<joint_trajectory_tui_node::ParamListener> param_listener_;
  std::unique_ptr<joint_trajectory_tui_node::Params> params_;

  std::unique_ptr<ftxui::Loop> loop_;
  ftxui::ScreenInteractive screen_;
  ftxui::Component renderer_;
  ftxui::Components position_sliders_;
  ftxui::Components velocity_sliders_;
  ftxui::Components effort_sliders_;

  urdf::ModelInterfaceSharedPtr model_interface_;

  void subscribeRobotDesctiptionCallback(const std_msgs::msg::String::ConstSharedPtr &);
  void subscribeJointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr &);

  void publishJointTrajectoryFromDestState();

  void resetTui();
  void initializeTui(const std::string & urdf_str);
  void refreshTuiTimerCallback();
};

JointTrajectoryTuiNode::JointTrajectoryTuiNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("joint_trajectory_tui", options),
  proc_joint_names_(),
  joint_angular_positions_(),
  joint_angular_velocities_(),
  joint_angular_effort_(),
  dest_joint_angular_positions_(),
  dest_joint_angular_velocities_(),
  dest_joint_angular_effort_(),
  last_robot_description_(nullptr),
  last_joint_states_(nullptr),
  robot_description_subscriber_(nullptr),
  joint_state_subscriber_(nullptr),
  joint_trajectory_publisher_(nullptr),
  refresh_tui_timer_(nullptr),
  param_listener_(nullptr),
  params_(nullptr),
  loop_(nullptr),
  screen_(ftxui::ScreenInteractive::TerminalOutput())
{
  param_listener_ = std::make_unique<joint_trajectory_tui_node::ParamListener>(
    this->get_node_parameters_interface());
  params_ = std::make_unique<joint_trajectory_tui_node::Params>(
    param_listener_->get_params());

  joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory",
    3);

  robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "robot_description",
    rclcpp::QoS(5).transient_local(),
    std::bind(
      &JointTrajectoryTuiNode::subscribeRobotDesctiptionCallback,
      this,
      std::placeholders::_1));

  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    2,
    std::bind(
      &JointTrajectoryTuiNode::subscribeJointStateCallback,
      this,
      std::placeholders::_1));

  const unsigned int refresh_tui_duration_milliseconds = 1e3 / params_->refresh_frequency;
  refresh_tui_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(refresh_tui_duration_milliseconds),
    std::bind(
      &JointTrajectoryTuiNode::refreshTuiTimerCallback,
      this));
}

JointTrajectoryTuiNode::~JointTrajectoryTuiNode()
{
  if (loop_) {
    loop_.reset(nullptr);
  }
  screen_.Exit();
}

void JointTrajectoryTuiNode::subscribeRobotDesctiptionCallback(
  const std_msgs::msg::String::ConstSharedPtr & msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Ignore empty robot description");
    return;
  }
  last_robot_description_ = msg;
  if (last_joint_states_) {
    initializeTui(msg->data);
  }
}

void JointTrajectoryTuiNode::subscribeJointStateCallback(
  const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  if (msg->name.empty()) {
    RCLCPP_WARN(this->get_logger(), "Ignore empty joint state");
    return;
  }
  last_joint_states_ = msg;

  if (last_robot_description_) {
    initializeTui(last_robot_description_->data);
  }
}

void JointTrajectoryTuiNode::publishJointTrajectoryFromDestState()
{
  if (dest_joint_angular_positions_.empty()) {
    return;
  } else if (dest_joint_angular_velocities_.empty()) {
    return;
  } else if (dest_joint_angular_effort_.empty()) {
    return;
  }
  if (proc_joint_names_.empty()) {
    return;
  }
  auto msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();

  msg->header.stamp = this->get_clock()->now();
  msg->joint_names.resize(proc_joint_names_.size());
  std::copy(proc_joint_names_.cbegin(), proc_joint_names_.cend(), msg->joint_names.begin());

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(dest_joint_angular_positions_.size());
  point.velocities.resize(dest_joint_angular_velocities_.size());
  point.accelerations.push_back(0.0);
  point.effort.resize(dest_joint_angular_effort_.size());

  std::copy(
    dest_joint_angular_positions_.cbegin(),
    dest_joint_angular_positions_.cend(),
    point.positions.begin());
  std::copy(
    dest_joint_angular_velocities_.cbegin(),
    dest_joint_angular_velocities_.cend(),
    point.velocities.begin());
  std::copy(
    dest_joint_angular_effort_.cbegin(),
    dest_joint_angular_effort_.cend(),
    point.effort.begin());
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 1;
  msg->points.push_back(point);

  joint_trajectory_publisher_->publish(std::move(msg));
}

void JointTrajectoryTuiNode::resetTui()
{
  if (model_interface_) {
    model_interface_.reset();
  }
  if (loop_) {
    loop_.reset();
  }
  if (renderer_) {
    renderer_.reset();
  }
  proc_joint_names_.clear();
  position_sliders_.clear();
  velocity_sliders_.clear();
  effort_sliders_.clear();
}

void JointTrajectoryTuiNode::initializeTui(const std::string & urdf_str)
{
  static bool first_call = true;

  if (!first_call) {
    return;
  } else {
    first_call = false;
    RCLCPP_INFO(this->get_logger(), "Initialize TUI");
  }
  resetTui();
  model_interface_ = urdf::parseURDF(urdf_str);

  for (const auto & [link_name, link] : model_interface_->links_) {
    if (!link) {
      continue;
    } else if (!link->parent_joint) {
      continue;
    } else if (link->parent_joint->type == urdf::Joint::FIXED) {
      continue;
    }
    const auto joint_name = link->parent_joint->name;
    proc_joint_names_.push_back(joint_name);
  }
  {
    unsigned int slider_index = 0;
    for (const auto & name : proc_joint_names_) {
      const auto joint = model_interface_->getJoint(name);
      if (joint->limits == nullptr) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Empty " << name << " limits");
        continue;
      }
      const auto itr = std::find(
        last_joint_states_->name.cbegin(), last_joint_states_->name.cend(), name);
      if (itr == last_joint_states_->name.cend()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Not found " << name << " from joint state");
        continue;
      }
      joint_angular_positions_.push_back(0.0F);
      joint_angular_velocities_.push_back(0.0F);
      joint_angular_effort_.push_back(0.0F);
      dest_joint_angular_positions_.push_back(0.0F);
      dest_joint_angular_velocities_.push_back(0.0F);
      dest_joint_angular_effort_.push_back(0.0F);
      position_sliders_.push_back(
        ftxui::Slider(
          " dest position: ",
          &dest_joint_angular_positions_[slider_index],
          static_cast<float>(joint->limits->lower),
          static_cast<float>(joint->limits->upper),
          static_cast<float>(params_->move_joint_min_step.position)
        )
      );
      velocity_sliders_.push_back(
        ftxui::Slider(
          " dest velocity: ",
          &dest_joint_angular_velocities_[slider_index],
          static_cast<float>(-joint->limits->velocity),
          static_cast<float>(joint->limits->velocity),
          static_cast<float>(params_->move_joint_min_step.velocity)
        )
      );
      effort_sliders_.push_back(
        ftxui::Slider(
          " dest effort: ",
          &dest_joint_angular_effort_[slider_index],
          static_cast<float>(-joint->limits->effort),
          static_cast<float>(joint->limits->effort),
          static_cast<float>(params_->move_joint_min_step.effort)
        )
      );
      slider_index++;
    }
  }
  ftxui::Components sliders;

  for (unsigned int i = 0; i < position_sliders_.size(); ++i) {
    sliders.push_back(position_sliders_[i]);
    sliders.push_back(velocity_sliders_[i]);
    sliders.push_back(effort_sliders_[i]);
  }
  auto container = ftxui::Container::Vertical(std::move(sliders));
  renderer_ = ftxui::Renderer(
    container,
    [&]() {
      ftxui::Elements joint_slide_renders;
      unsigned int slider_index = 0;

      for (const auto & name : proc_joint_names_) {
        const auto joint = model_interface_->getJoint(name);

        if (joint == nullptr) {
          RCLCPP_WARN_STREAM(this->get_logger(), "Empty config of " << name << " joint");
          continue;
        } else if (joint->limits == nullptr) {
          RCLCPP_WARN_STREAM(this->get_logger(), "Empty " << name << " limits");
          continue;
        }
        const auto itr = std::find(
          last_joint_states_->name.cbegin(), last_joint_states_->name.cend(), name);

        if (itr == last_joint_states_->name.cend()) {
          RCLCPP_WARN_STREAM(this->get_logger(), "Not found " << name << " from joint state");
          continue;
        }
        const auto state_index = std::distance(last_joint_states_->name.cbegin(), itr);
        // TODO(Naoki Takahashi) non safe access
        joint_angular_positions_[slider_index] = last_joint_states_->position[state_index];
        joint_angular_velocities_[slider_index] = last_joint_states_->velocity[state_index];
        joint_angular_effort_[slider_index] = last_joint_states_->effort[state_index];

        const float move_range = static_cast<float>(joint->limits->upper - joint->limits->lower);
        const float velocity_range = static_cast<float>(2.0 * joint->limits->velocity);
        const float effort_range = static_cast<float>(2.0 * joint->limits->effort);
        const float position_rate = 0.5F + joint_angular_positions_[slider_index] / move_range;
        const float velocity_rate = 0.5F + joint_angular_velocities_[slider_index] / velocity_range;
        const float effort_rate = 0.5F + joint_angular_effort_[slider_index] / effort_range;

        joint_slide_renders.push_back(
          ftxui::vbox(
          {
            ftxui::hbox({
              ftxui::text(name + ": "),
              ftxui::text("position: " + std::to_string(joint_angular_positions_[slider_index]))
                | ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 24),
              ftxui::text("velocity: " + std::to_string(joint_angular_velocities_[slider_index]))
                | ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 24),
              ftxui::text("effort: " + std::to_string(joint_angular_effort_[slider_index]))
                | ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 24),
            }),
            ftxui::hbox({
              ftxui::text("state position: "),
              ftxui::gauge(position_rate),
            }),
            position_sliders_[slider_index]->Render(),
            ftxui::hbox({
              ftxui::text("state velocity: "),
              ftxui::gauge(velocity_rate),
            }),
            velocity_sliders_[slider_index]->Render(),
            ftxui::hbox({
              ftxui::text("state effort: "),
              ftxui::gauge(effort_rate),
            }),
            effort_sliders_[slider_index]->Render(),
          }));
        slider_index++;
      }
      return ftxui::window(
        ftxui::text(this->get_name()),
        ftxui::vbox(std::move(joint_slide_renders)) | ftxui::xflex
      );
    }
  );
  renderer_ |= ftxui::CatchEvent(
    [&](ftxui::Event event) -> bool {
      if (event.is_character()) {
        if (event.character() == "q") {
          rclcpp::shutdown();
          return true;
        }
      }
      return false;
    }
  );
  loop_ = std::make_unique<ftxui::Loop>(&screen_, renderer_);
}

void JointTrajectoryTuiNode::refreshTuiTimerCallback()
{
  if (!loop_) {
    return;
  }
  if (loop_->HasQuitted()) {
    rclcpp::shutdown();
    return;
  }
  screen_.Post(ftxui::Event::Custom);
  loop_->RunOnce();
  publishJointTrajectoryFromDestState();
  std::this_thread::sleep_for(std::chrono::milliseconds(0));
}
}  // namespace ros2_tviz

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_tviz::JointTrajectoryTuiNode)
