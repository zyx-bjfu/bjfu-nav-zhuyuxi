#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <functional>

#include <chrono>
#include <memory>

class RefereeMockNode : public rclcpp::Node
{
public:
  RefereeMockNode();

private:
  // 定时器回调：周期发布阶段、血量、弹量
  void publish_status();

  // 服务回调：回血
  void supply_hp_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 三个话题发布者
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stage_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hp_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ammo_pub_;

  // 服务端 + 定时器
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr supply_hp_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 状态变量
  int stage_;
  int hp_;
  int ammo_;
  const int hp_max_ = 200;
};

RefereeMockNode::RefereeMockNode()
: Node("referee_Node"), stage_(1), hp_(100), ammo_(200)
{
  // 三个话题发布者
  stage_pub_ = this->create_publisher<std_msgs::msg::Int32>("/game_stage", 10);
  hp_pub_ = this->create_publisher<std_msgs::msg::Int32>("/hp", 10);
  ammo_pub_ = this->create_publisher<std_msgs::msg::Int32>("/ammo", 10);

  // 回血服务端
  supply_hp_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/supply_hp",
    std::bind(&RefereeMockNode::supply_hp_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  // 定时器：每 500ms 发布一次状态
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RefereeMockNode::publish_status, this));

  RCLCPP_INFO(this->get_logger(), "RefereeMockNode 启动了");
}
void RefereeMockNode::publish_status()
{
  std_msgs::msg::Int32 stage_msg;
  stage_msg.data = stage_;
  stage_pub_->publish(stage_msg);

  std_msgs::msg::Int32 hp_msg;
  hp_msg.data = hp_;
  hp_pub_->publish(hp_msg);

  std_msgs::msg::Int32 ammo_msg;
  ammo_msg.data = ammo_;
  ammo_pub_->publish(ammo_msg);
}
void RefereeMockNode::supply_hp_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Trigger 请求没有字段，避免未使用警告

  hp_ += 50;
  if (hp_ > hp_max_) {
    hp_ = hp_max_;
  }

  response->success = true;
  response->message = "hp restored to " + std::to_string(hp_);

  RCLCPP_INFO(this->get_logger(), "回血成功，当前血量：%d", hp_);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RefereeMockNode>());
  rclcpp::shutdown();
  return 0;
}

