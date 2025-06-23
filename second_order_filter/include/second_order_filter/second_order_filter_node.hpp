#ifndef SECOND_ORDER_FILTER_NODE__SECOND_ORDER_FILTER_NODE_HPP_
#define SECOND_ORDER_FILTER_NODE__SECOND_ORDER_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class SecondOrderFilterNode : public rclcpp::Node
{
public:
    SecondOrderFilterNode();

private:

    // Parametros de la simulacion
    double omega_n_;
    double zeta_;
    double kp_;
    double tp1_, tp2_;
    double tp3_;

    double lin_delay_;
    double ang_delay_;

    int lin_delay_steps_;
    int ang_delay_steps_;

    bool lin_delay_cntdwn_enbl_;
    bool ang_delay_cntdwn_enbl_;

    // Estado lineal
    double cmd_vel_input_lin_;
    double cmd_vel_input_n_lin_;
    double x_lin_;
    double x_dot_lin_;
    double x_ddot_lin_;
    // Estado angular
    double cmd_vel_input_ang_;
    double cmd_vel_input_n_ang_;
    double x_ang_;
    double x_dot_ang_;
    double x_ddot_ang_;


    // Recursos de comunicacion
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr filtered_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Metodos
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update();

};

#endif // SECOND_ORDER_FILTER_NODE__SECOND_ORDER_FILTER_NODE_HPP_