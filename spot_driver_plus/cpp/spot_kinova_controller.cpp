#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>

#include <spot_msgs/srv/set_locomotion.hpp>

const std::string JOY_TOPIC = "/joy";
const std::string SPOT_TWIST_TOPIC = "/cmd_vel";
const std::string SPOT_JOY_TOPIC = "/joy_spot";
const std::string SPOT_POSE_TOPIC = "/body_pose";
const std::string KINOVA_TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string KINOVA_JOY_TOPIC = "/joy_kinova";

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum AxisBluetooth
{
  LEFT_STICK_X_BT = 0,
  LEFT_STICK_Y_BT = 1,
  LEFT_TRIGGER_BT = 5,
  RIGHT_STICK_X_BT = 2,
  RIGHT_STICK_Y_BT = 3,
  RIGHT_TRIGGER_BT = 4,
  D_PAD_X_BT = 6,
  D_PAD_Y_BT = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  XBOX = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10,
  HOME = 11
};
enum ButtonBluetooth
{
  A_BT = 0,
  B_BT = 1,
  X_BT = 3,
  Y_BT = 4,
  LEFT_BUMPER_BT = 6,
  RIGHT_BUMPER_BT = 7,
  CHANGE_VIEW_BT = 10,
  MENU_BT = 11,
  XBOX_BT = 8,
  LEFT_STICK_CLICK_BT = 13,
  RIGHT_STICK_CLICK_BT = 14,
  HOME_BT = 15
};
enum Gait
{
  AUTO = 1,
  TROT = 2,
  CRAWL = 4,
  AMBLE = 5,
  JOG = 7,
  HOP = 8
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
const std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };

Gait getNextGait(Gait currentGait)
{
  switch (currentGait)
  {
    case AUTO: return TROT;
    case TROT: return CRAWL;
    case CRAWL: return AMBLE;
    case AMBLE: return JOG;
    case JOG: return HOP;
    case HOP: return AUTO;
    default: return AUTO; // Handle invalid or unknown values
  }
}

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmdKinova(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::shared_ptr<geometry_msgs::msg::TwistStamped>& twist)
{
  // Ignore if buttons are pressed
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    return false;
  }

  // The bread and butter: map buttons to twist commands
  twist->twist.linear.x = axes[RIGHT_STICK_Y];
  twist->twist.linear.y = axes[RIGHT_STICK_X];

  double lin_z_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_z_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.z = (lin_z_right + lin_z_left);

  twist->twist.angular.y = - 1.5 * axes[LEFT_STICK_Y];
  twist->twist.angular.x = 1.5 * axes[LEFT_STICK_X];
  
  double roll_positive = buttons[RIGHT_BUMPER];
  double roll_negative = - 1 * (buttons[LEFT_BUMPER]);
  twist->twist.angular.z = 1.5 * (roll_positive + roll_negative);
  
  return true;
}

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A Twist message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmdSpot(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::shared_ptr<geometry_msgs::msg::Twist>& twist, const float& spot_speed_factor,
                     const Gait& MODE)
{
  // Ignore if buttons are pressed
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    return false;
  }

  // Use our variable speed factor only in AUTO Locomotion Mode
  float speed = MODE == AUTO ? spot_speed_factor : 0.4;

  // The bread and butter: map buttons to twist commands
  twist->linear.x = speed * axes[LEFT_STICK_Y];
  twist->linear.y = speed * axes[LEFT_STICK_X];
  twist->angular.z = axes[RIGHT_STICK_X];

  return true;
}

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param pose A Pose message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToPoseSpot(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::shared_ptr<geometry_msgs::msg::Pose>& pose, const float& height)
{
  // Ignore if buttons are pressed
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    return false;
  }

  pose->position.z = height;

  tf2::Quaternion q;
  q.setRPY(-axes[LEFT_STICK_X], axes[LEFT_STICK_Y], axes[RIGHT_STICK_X]);
  // The bread and butter: pose buttons to twist commands
  pose->orientation.x = q.x();
  pose->orientation.y = q.y();
  pose->orientation.z = q.z();
  pose->orientation.w = q.w();
  
  return true;
}

void resetBodyPose(float& height, float& roll, float& pitch, float& yaw)
{
  height = 0.0;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
}

// For when the controller is connected with bluetooth
sensor_msgs::msg::Joy bluetooth_remapped(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
{
  sensor_msgs::msg::Joy bt_msg;

  bt_msg.header = msg->header;

  bt_msg.axes.push_back(msg->axes[LEFT_STICK_X_BT]);
  bt_msg.axes.push_back(msg->axes[LEFT_STICK_Y_BT]);
  bt_msg.axes.push_back(msg->axes[LEFT_TRIGGER_BT]);
  bt_msg.axes.push_back(msg->axes[RIGHT_STICK_X_BT]);
  bt_msg.axes.push_back(msg->axes[RIGHT_STICK_Y_BT]);
  bt_msg.axes.push_back(msg->axes[RIGHT_TRIGGER_BT]);
  bt_msg.axes.push_back(msg->axes[D_PAD_X_BT]);
  bt_msg.axes.push_back(msg->axes[D_PAD_Y_BT]);

  bt_msg.buttons.push_back(msg->buttons[A_BT]);
  bt_msg.buttons.push_back(msg->buttons[B_BT]);
  bt_msg.buttons.push_back(msg->buttons[X_BT]);
  bt_msg.buttons.push_back(msg->buttons[Y_BT]);
  bt_msg.buttons.push_back(msg->buttons[LEFT_BUMPER_BT]);
  bt_msg.buttons.push_back(msg->buttons[RIGHT_BUMPER_BT]);
  bt_msg.buttons.push_back(msg->buttons[CHANGE_VIEW_BT]);
  bt_msg.buttons.push_back(msg->buttons[MENU_BT]);
  bt_msg.buttons.push_back(msg->buttons[XBOX_BT]);
  bt_msg.buttons.push_back(msg->buttons[LEFT_STICK_CLICK_BT]);
  bt_msg.buttons.push_back(msg->buttons[RIGHT_STICK_CLICK_BT]);
  bt_msg.buttons.push_back(msg->buttons[HOME_BT]);

  return bt_msg;
}

class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("spot_controller", options)
  {
    // Setup pub/sub
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(JOY_TOPIC, 1,
                                                          [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
                                                            return joyCB(msg);
                                                          });

    pose_pub_spot_ = create_publisher<geometry_msgs::msg::Pose>(SPOT_POSE_TOPIC, 1);

    joy_pub_spot_ = create_publisher<sensor_msgs::msg::Joy>(SPOT_JOY_TOPIC, 1);
    joy_pub_kinova_ = create_publisher<sensor_msgs::msg::Joy>(KINOVA_JOY_TOPIC, 1);

    twist_pub_spot_ = create_publisher<geometry_msgs::msg::Twist>(SPOT_TWIST_TOPIC, 1);
    twist_pub_kinova_ = create_publisher<geometry_msgs::msg::TwistStamped>(KINOVA_TWIST_TOPIC, 1);

    spot_stand = create_client<std_srvs::srv::Trigger>("/stand");
    spot_sit = create_client<std_srvs::srv::Trigger>("/sit");

    spot_gait = create_client<spot_msgs::srv::SetLocomotion>("/locomotion_mode");
  }

  void gait(const Gait& MODE)
  {
    if (!spot_gait->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "locomotion service not available");
      return;
    }
    auto request = std::make_shared<spot_msgs::srv::SetLocomotion::Request>();
    request.get()->locomotion_mode = static_cast<int>(MODE);

    auto future_result = spot_gait->async_send_request(request);
  }

  void stand()
  {
    resetBodyPose(height, roll, pitch, yaw);
    locomotion_mode = AUTO;
    gait(locomotion_mode);
    is_sitting = false;
    spot_speed_factor = 0.5;

    auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
    pose_msg->position.z = height;
    pose_pub_spot_->publish(*pose_msg);

    if (!spot_stand->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "stand service not available");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result = spot_stand->async_send_request(request);
  }

  void sit()
  {
    is_sitting = true;
    if (!spot_sit->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "sit service not available");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result = spot_sit->async_send_request(request);
  }
    
  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg)
  {  
    sensor_msgs::msg::Joy msg;

    if (joy_msg->buttons.size() == 16)
    { 
      msg = bluetooth_remapped(joy_msg);
    }
    else {
      msg = *joy_msg;
    }

    auto buttons = msg.buttons;

    if (buttons[CHANGE_VIEW] && !control_spot)
    {
      control_spot = true;
    }
    else if (buttons[MENU] && control_spot)
    {
      control_spot = false;
    }

    if (msg.axes[LEFT_TRIGGER] < 0 && msg.axes[RIGHT_TRIGGER] < 0 && !cutoff_controller_flag)
    {
      cutoff_controller_flag = true;
      cutoff_controller = !cutoff_controller;
    }
    else if (msg.axes[LEFT_TRIGGER] == 1 && msg.axes[RIGHT_TRIGGER] == 1)
    {
      cutoff_controller_flag = false;
    }
    if (cutoff_controller) return;

    if (control_spot)
    {
      joy_pub_spot_->publish(msg);

      for (long unsigned int i = 0; i < buttons.size(); ++i)
      {
        if (buttons[i] == 1)
        {
          if (pressed_buttons.count(i) != 0)
          {
            buttons[i] = 0; // Set buttons[i] to 0 if already pressed
          } else
          {
            pressed_buttons.insert(i); // Add pressed button to set
          }
        }
        else if (buttons[i] == 0)
        {
          pressed_buttons.erase(i); // Remove released button from set
        }
      }

      auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
      auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();

      if (!stance)
      {
        if (convertJoyToCmdSpot(msg.axes, buttons, twist_msg, spot_speed_factor, locomotion_mode))
        {
          if (!is_sitting)
          {
            twist_pub_spot_->publish(*twist_msg);
          }
        }
      }
      else
      {
        if (convertJoyToPoseSpot(msg.axes, buttons, pose_msg, height))
        {
          pose_pub_spot_->publish(*pose_msg);
          if (!is_sitting) {
            twist_pub_spot_->publish(*twist_msg);
          }
        }
      }

      if (buttons[Y])
      {
        stand();
      }
      else if (buttons[X])
      {
        sit();
      }
      else if (buttons[B])
      {
        locomotion_mode = msg.axes[RIGHT_TRIGGER] == -1 ? getNextGait(locomotion_mode) : AUTO;
        gait(locomotion_mode);
      }
      else if (buttons[A])
      {
        if (stance) stance = false;
        else if (!stance) stance = true;
      }
      else if (buttons[LEFT_BUMPER])
      {
        spot_speed_factor = std::max(spot_speed_factor - 0.125, 0.2);
      }
      else if (buttons[RIGHT_BUMPER])
      {
        spot_speed_factor = std::min(spot_speed_factor + 0.125, 1.0);
      }
      else if (msg.axes[D_PAD_Y] != 0)
      {
        height = msg.axes[D_PAD_Y] * 0.2;
        if (!stance) {
          pose_msg->position.z = height;
          pose_pub_spot_->publish(*pose_msg);
        }
      }

    }
    else
    {
      joy_pub_kinova_->publish(msg);
      auto twist_stamped_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();

      if (convertJoyToCmdKinova(msg.axes, buttons, twist_stamped_msg))
      {
        twist_stamped_msg->header.stamp = now();
        twist_pub_kinova_->publish(*twist_stamped_msg);
      }
    }
  }

private:
  bool cutoff_controller = true;
  bool cutoff_controller_flag = false;
  bool control_spot = true;
  bool stance = false;
  bool is_sitting = true;
  float height, roll, pitch, yaw;
  float spot_speed_factor = 0.5;
  Gait locomotion_mode = AUTO;

  std::set<int> pressed_buttons; // Store pressed buttons

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_spot_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_spot_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_spot_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_kinova_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_kinova_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr spot_stand, spot_sit;
  rclcpp::Client<spot_msgs::srv::SetLocomotion>::SharedPtr spot_gait;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<JoyToServoPub>(options);
  rclcpp::spin(node);
  return 0;
}
