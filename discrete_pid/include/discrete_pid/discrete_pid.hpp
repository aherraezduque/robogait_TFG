#ifndef DISCRETE_PID_HPP
#define DISCRETE_PID_HPP


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/float64.hpp"

class Discrete_PID : public rclcpp::Node {

public:
    Discrete_PID();

private:


    // Params regulador
    double Kp_ = 0.0;
    double Ki_ = 0.0;
    double Kd_ = 0.0;
    double T_ = 0.001;
    double setpoint_ = 2.0;

    // Limits lin vel
    double v_min_;
    double v_max_;

    // Coefientes
    double b0_, b1_, b2_;

    // Estados 
    double e_k1_ = 0.0, e_k2_ = 0.0;
    double u_k1_ = 0.0;

    // Ultimo cmd_vel de nav2 (para la orientacion)
    geometry_msgs::msg::Twist last_nav2_cmd_;


    // Topics
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr distance_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;


    // Callbacks 
    void distanceCallback(const example_interfaces::msg::Float64::SharedPtr msg);
    void nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg);



};

#endif