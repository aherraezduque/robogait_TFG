#include "discrete_pid/discrete_pid.hpp"
#include <algorithm>

Discrete_PID::Discrete_PID() : Node("discrete_pid_node")
{
    // Valor por defecto parametros
    this->declare_parameter<double>("Kp", 1.1);
    this->declare_parameter<double>("Ki", 0.0);
    this->declare_parameter<double>("Kd", 0.0);
    this->declare_parameter<double>("T", 0.001);
    this->declare_parameter<double>("setpoint", 1.5);
    this->declare_parameter<double>("v_min", -3.0);
    this->declare_parameter<double>("v_max", 3.0);

    // Cargar valor de los parametros
    Kp_ = this->get_parameter("Kp").as_double();
    Ki_ = this->get_parameter("Ki").as_double();
    Kd_ = this->get_parameter("Kd").as_double();
    T_ = this->get_parameter("T").as_double();
    setpoint_ = this->get_parameter("setpoint").as_double();
    v_min_ = this->get_parameter("v_min").as_double();
    v_max_ = this->get_parameter("v_max").as_double();


    // Calcular coef
    b0_ = Kp_ + Ki_ * (T_ * 0.5) + (Kd_ / T_);

    b1_ = -Kp_ + Ki_ * (T_ * 0.5) - (2.0 * Kd_ / T_);

    b2_ = Kd_ / T_;

    // Subscripciones
    distance_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/person_distance", 10,
        std::bind(&Discrete_PID::distanceCallback, this, std::placeholders::_1));

    nav2_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel/nav2", 10,
        std::bind(&Discrete_PID::nav2Callback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/pid", 10);




    RCLCPP_INFO(this->get_logger(),
        "Discrete_PID node Kp=%.3f Ki=%.3f Kd=%.3f T=%.5f setpoint=%.3f v_min=%.3f v_max=%.3f",
        Kp_, Ki_, Kd_, T_, setpoint_, v_min_, v_max_);
}

void Discrete_PID::nav2Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_nav2_cmd_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Recibido de NAV2");
}


void Discrete_PID::distanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    if (std::isnan(msg->data) || std::isinf(msg->data)) {
        //if (std::isnan(msg->data) || std::isinf(msg->data) || (msg->data > 5.0) || (msg->data < 0.0)) {
        RCLCPP_WARN(this->get_logger(), "Distancia inválida recibida, ignorando");
        return;
    }
    double e_k = setpoint_ - msg->data;

    double u_k = u_k1_ + b0_ * e_k + b1_ * e_k1_ + b2_ * e_k2_;

    // Saturar velocidad
    u_k = std::clamp(u_k, v_min_, v_max_);

    // Publicar cmd_vel/pid (lin_vel->PID, ang_vel->NAV2)
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = u_k;
    cmd_vel.angular.z = last_nav2_cmd_.angular.z;

    cmd_vel_pub_->publish(cmd_vel);

    RCLCPP_INFO(this->get_logger(),
        "Publicando cmd_vel/pid: linear.x=%.3f, angular.z=%.3f (e_k=%.3f, setpoint=%.3f, medida=%.3f)",
        cmd_vel.linear.x, cmd_vel.angular.z, e_k, setpoint_, msg->data);

    //Actualizar estados
    e_k2_ = e_k1_;
    e_k1_ = e_k;
    u_k1_ = u_k;
}








int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Discrete_PID>());
    rclcpp::shutdown();
    return 0;
}