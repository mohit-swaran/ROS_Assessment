#include "bt_node.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir =
    ament_index_cpp::get_package_share_directory("cafe_robot_bt") + "/bt_xml";

BTnode::BTnode(const std::string &nodeName) : Node(nodeName)
{
  this->declare_parameter("location_file","none");

  RCLCPP_INFO(get_logger(), "Init done");
}

void BTnode::setup()
{
  RCLCPP_INFO(get_logger(), "Setting up");
  create_behavior_tree();
  RCLCPP_INFO(get_logger(), "BT created");

  const auto timer_period = 500ms;
  timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&BTnode::update_behavior_tree, this));

  rclcpp::spin(shared_from_this());
  rclcpp::shutdown();
}

void BTnode::create_behavior_tree()
{
  BT::BehaviorTreeFactory factory;

  // register bt node

  BT::NodeBuilder builder =
      [=](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<GoToPose>(name, config, shared_from_this());
  };

  factory.registerBuilder<GoToPose>("GoToPose", builder);

  RCLCPP_INFO(get_logger(), bt_xml_dir.c_str());

  tree_ = factory.createTreeFromFile(bt_xml_dir + "/bt.xml");
  RCLCPP_INFO(get_logger(), "3");
}

void BTnode::update_behavior_tree()
{
  BT::NodeStatus tree_status = tree_.tickRoot();

  if (tree_status == BT::NodeStatus::RUNNING)
  {
    return;
  }
  else if (tree_status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Finished Navigation");
  }
  else if (tree_status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation Failed");
    timer_->cancel();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BTnode>("bt_node");
  node->setup();

  return 0;
}