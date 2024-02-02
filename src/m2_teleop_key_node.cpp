#include <sstream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

class M2Teleop : public rclcpp::Node 
{
  public:

  M2Teleop() : Node("m2_teleop_key_node")
  {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&M2Teleop::timer_callback, this));

    // keytimer_ = this->create_wall_timer(
    //     500ms, std::bind(&M2Teleop::keyLoop, this));


        
  }



  void timer_callback()
  {
    //publish_mutex_.lock();
    {
      if ((current_linear_x < 1.0 / smoother_increments) &&
          (current_linear_x > -1.0 / smoother_increments))
      {
        current_linear_x = 0.0;
      }
      else if (current_linear_x > 0.0)
      {
        current_linear_x -= 1.0 / smoother_increments;
      }
      else
      {
        current_linear_x += 1.0 / smoother_increments;
      }
      if ((current_linear_y < 1.0 / smoother_increments) &&
          (current_linear_y > -1.0 / smoother_increments))
      {
        current_linear_y = 0.0;
      }
      else if (current_linear_y > 0.0)
      {
        current_linear_y -= 1.0 / smoother_increments;
      }
      else
      {
        current_linear_y += 1.0 / smoother_increments;
      }
      if ((current_angular < 1.0 / smoother_increments) &&
          (current_angular > -1.0 / smoother_increments))
      {
        current_angular = 0.0;
      }
      else if (current_angular > 0.0)
      {
        current_angular -= 1.0 / smoother_increments;
      }
      else
      {
        current_angular += 1.0 / smoother_increments;
      }
    }
    publish3(current_angular, current_linear_x, current_linear_y);
  }

void keyLoop()
  {
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the roboter like differntial drive.");
    puts("Use asdqwe keys to move the roboter like omnidirection drive.");
puts("DEBUG ");
    while (rclcpp::ok())
    {
      printf("value: 0x%02X", c);
      // get the next event from the keyboard
      if (read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }
      // https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html
      fprintf(stderr, "value: 0x%02X", c);
      RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X", c);
      switch (c)
      {
      case KEYCODE_LEFT:
      case KEYCODE_Q:

        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        current_angular += 1.0 / smoother_increments;
        if (current_angular > 1.0)
          current_angular = 1.0;
         fprintf (stderr, "LEFT = %f", current_angular);
        break;
      case KEYCODE_RIGHT:
      case KEYCODE_E:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        current_angular -= 1.0 / smoother_increments;
        if (current_angular < -1.0)
          current_angular = -1.0;
        break;
      case KEYCODE_UP:
      case KEYCODE_W:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        current_linear_x += 1.0 / smoother_increments;
        if (current_linear_x > 1.0)
          current_linear_x = 1.0;
        break;
      case KEYCODE_DOWN:
      case KEYCODE_S:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        current_linear_x -= 1.0 / smoother_increments;
        if (current_linear_x < -1.0)
          current_linear_x = -1.0;
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(this->get_logger(), "STEP RIGHT");
        current_linear_y += 1.0 / smoother_increments;
        if (current_linear_y > 1.0)
          current_linear_y = 1.0;
        break;
      case KEYCODE_A:
        RCLCPP_DEBUG(this->get_logger(), "STEP LEFT");
        current_linear_y -= 1.0 / smoother_increments;
        if (current_linear_y < -1.0)
          current_linear_y = -1.0;
        break;
      }
      // publish_mutex_.lock();
    }
    return;
  }


  static void  quit(int)
  {
    fprintf (stderr, "quit");
    //tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    exit(0);
  }
  
  int kfd = 0 ;
  struct  termios cooked, raw;

private:

  volatile double linear_ = 0.0, angular_ = 0.0;
  volatile double l_scale_ = 1.0, a_scale_ = 1.0;

  volatile double smoother_increments = 100.0;
  volatile double current_linear_x = 0.0; // current value. range -1 .. 1
  volatile double current_linear_y = 0.0; // current value. range -1 .. 1
  volatile double current_angular = 0.0;  // curent value. range -1 .. 1

  // std::mutex publish_mutex_;

  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::TimerBase::SharedPtr keytimer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  void publish3(double angular, double linearx, double lineary)
  {
    auto vel = geometry_msgs::msg::Twist();
    vel.angular.z = a_scale_ * angular;
    vel.linear.x = l_scale_ * linearx;
    vel.linear.y = l_scale_ * lineary;

  fprintf (stderr, "vel = %f, %f, %f, %f, %f", angular, linearx, lineary, a_scale_, l_scale_);
  //  fprintf (stderr, "vel = %f, %f, %f", vel.linear.x, vel.linear.y, vel.angular.z);
    vel_pub_->publish(vel);
    return;
  }


};





int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

    M2Teleop m2_teleop;
    signal(SIGINT, M2Teleop::quit);
    std::thread my_thread(std::bind(&M2Teleop::keyLoop, &m2_teleop));


    rclcpp::spin(std::make_shared<M2Teleop>());
    rclcpp::shutdown();

  //auto node = rclcpp::Node::make_shared("talker");
//   auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", 1000);
//   rclcpp::Rate loop_rate(10);
//   int count = 0;
// //  std_msgs::String msg;
//   std_msgs::msg::String msg;
// //  while (ros::ok())
//   while (rclcpp::ok())
//   {
//     std::stringstream ss;
//     ss << "hello world " << count++;
//     msg.data = ss.str();
//     RCLCPP_INFO(node->get_logger(), "%s\n", msg.data.c_str());
//     chatter_pub->publish(msg);
//     rclcpp::spin_some(node);
//     loop_rate.sleep();
//   }
  return 0;
}