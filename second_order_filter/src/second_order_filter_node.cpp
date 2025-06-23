#include "second_order_filter/second_order_filter_node.hpp"


SecondOrderFilterNode::SecondOrderFilterNode() : Node("second_order_filter_node")
{
    this->declare_parameter("omega_n", 0.0);
    this->declare_parameter("zeta", 0.0);
    this->declare_parameter("kp", 0.8887);
    this->declare_parameter("tp1", 0.5617);
    this->declare_parameter("tp2", 0.56189);
    this->declare_parameter("tp3", 0.0);
    this->declare_parameter("lin_delay", 0.0);
    this->declare_parameter("ang_delay", 0.0);

    omega_n_ = this->get_parameter("omega_n").as_double();
    zeta_ = this->get_parameter("zeta").as_double();
    kp_ = this->get_parameter("kp").as_double();
    tp1_ = this->get_parameter("tp1").as_double();
    tp2_ = this->get_parameter("tp2").as_double();
    tp3_ = this->get_parameter("tp3").as_double();
    lin_delay_ = this->get_parameter("lin_delay").as_double();
    ang_delay_ = this->get_parameter("ang_delay").as_double();

    x_lin_ = 0.0;
    x_dot_lin_ = 0.0;
    x_ddot_lin_ = 0.0;

    x_ang_ = 0.0;
    x_dot_ang_ = 0.0;
    x_ddot_ang_ = 0.0;

    lin_delay_steps_ = static_cast<int>(lin_delay_ / 0.01);  //cambiar 0.01 por parametro de dt
    ang_delay_steps_ = static_cast<int>(ang_delay_ / 0.01);  //cambiar 0.01 por parametro de dt

    lin_delay_cntdwn_enbl_ = false;
    ang_delay_cntdwn_enbl_ = false;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
        std::bind(&SecondOrderFilterNode::cmdVelCallback, this, std::placeholders::_1));

    filtered_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", 10);

    /* update_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
        std::bind(&SecondOrderFilterNode::update, this)); */

        // PARA TRAPEZOIDAL SE HA MODIFICA EL dt = 0.001;
    update_timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
        std::bind(&SecondOrderFilterNode::update, this));

    RCLCPP_INFO(this->get_logger(), "Second Order Filter Node iniciado: lin_delay_steps: %d, ang_delay_steps: %d", lin_delay_steps_, ang_delay_steps_);
}

void SecondOrderFilterNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_input_lin_ = msg->linear.x;
    cmd_vel_input_ang_ = msg->angular.z;
}

// P2 y P2D
/* void SecondOrderFilterNode::update()
{
    double dt = 0.01;

    // Dinámica lineal
    double x_ddot_lin = (kp_ * cmd_vel_input_lin_ - x_lin_ - (tp1_ + tp2_) * x_dot_lin_) / (tp1_ * tp2_);
    x_dot_lin_ += x_ddot_lin * dt;
    x_lin_ += x_dot_lin_ * dt;

    // Dinámica angular
    double x_ddot_ang = (kp_ * cmd_vel_input_ang_ - x_ang_ - (tp1_ + tp2_) * x_dot_ang_) / (tp1_ * tp2_);
    x_dot_ang_ += x_ddot_ang * dt;
    x_ang_ += x_dot_ang_ * dt;

    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = x_lin_;
    msg_out.angular.z = x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
} */




/////////////////////////////////////////
/////  REVISAR LO DE KP AL FINAL  
/////////////////////////////////////////

// P2U y P2DU
/* void SecondOrderFilterNode::update()
{
    double dt = 0.01;

    if (!(cmd_vel_input_lin_ == 0.0)) {
        lin_delay_cntdwn_enbl_ = true;
        RCLCPP_INFO(this->get_logger(), "LINEAR DELAY ENABLE");
    }

    if (!(cmd_vel_input_ang_ == 0.0)) {
        ang_delay_cntdwn_enbl_ = true;
        RCLCPP_INFO(this->get_logger(), "ANGULAR DELAY ENABLE");
    }

    // Simular el retardo
    if (lin_delay_cntdwn_enbl_ && lin_delay_steps_) {
        cmd_vel_input_lin_ = 0.0;
        lin_delay_steps_--;
    }

    if (ang_delay_cntdwn_enbl_ && ang_delay_steps_) {
        cmd_vel_input_ang_ = 0.0;
        ang_delay_steps_--;
    }

    // Dinámica lineal
    double x_ddot_lin = omega_n_ * omega_n_ * (cmd_vel_input_lin_ - x_lin_) - 2.0 * zeta_ * omega_n_ * x_dot_lin_;
    x_dot_lin_ += x_ddot_lin * dt;
    x_lin_ += x_dot_lin_ * dt;

    // Dinámica angular
    double x_ddot_ang = omega_n_ * omega_n_ * (cmd_vel_input_ang_ - x_ang_) - 2.0 * zeta_ * omega_n_ * x_dot_ang_;
    x_dot_ang_ += x_ddot_ang * dt;
    x_ang_ += x_dot_ang_ * dt;

    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = kp_ * x_lin_;
    msg_out.angular.z = kp_ * x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
} */


/////////////////////////////////
// P2U y P2DU KP al principio
/////////////////////////////////
/* void SecondOrderFilterNode::update()
{
    double dt = 0.01;

    // Dinámica lineal
    double x_ddot_lin = omega_n_ * omega_n_ * (kp_ * cmd_vel_input_lin_ - x_lin_) - 2.0 * zeta_ * omega_n_ * x_dot_lin_;
    x_dot_lin_ += x_ddot_lin * dt;
    x_lin_ += x_dot_lin_ * dt;

    // Dinámica angular
    double x_ddot_ang = omega_n_ * omega_n_ * (kp_ * cmd_vel_input_ang_ - x_ang_) - 2.0 * zeta_ * omega_n_ * x_dot_ang_;
    x_dot_ang_ += x_ddot_ang * dt;
    x_ang_ += x_dot_ang_ * dt;

    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = x_lin_;
    msg_out.angular.z = x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
} */


// P3DU y P3U
/* void SecondOrderFilterNode::update()
{
    double dt = 0.01;
    double tw = 1 / omega_n_;

    // Calculo de coeficientes del denominador ---
    double a3 = tw * tw * tp3_;
    double a2 = 2.0 * zeta_ * tw * tp3_ + tw * tw;
    double a1 = 2.0 * zeta_ * tw + tp3_;
    double a0 = 1.0;

    if (!(cmd_vel_input_lin_ == 0.0)) {
        lin_delay_cntdwn_enbl_ = true;
        RCLCPP_INFO(this->get_logger(), "LINEAR DELAY ENABLE: STEPS = %d ", lin_delay_steps_);
    }

    if (!(cmd_vel_input_ang_ == 0.0)) {
        ang_delay_cntdwn_enbl_ = true;
        RCLCPP_INFO(this->get_logger(), "ANGULAR DELAY ENABLE");
    }

    // Simular el retardo
    if (lin_delay_steps_) {
        cmd_vel_input_lin_ = 0.0;
        lin_delay_steps_--;
    }

    if (ang_delay_steps_) {
        cmd_vel_input_ang_ = 0.0;
        ang_delay_steps_--;
    }

    // Dinamica lineal
    double x_dddot_lin = (kp_ * cmd_vel_input_lin_ - a0 * x_lin_ - a1 * x_dot_lin_ - a2 * x_ddot_lin_) / a3;

    x_ddot_lin_ += x_dddot_lin * dt;
    x_dot_lin_ += x_ddot_lin_ * dt;
    x_lin_ += x_dot_lin_ * dt;

    // Dinamica angular
    double x_dddot_ang = (kp_ * cmd_vel_input_ang_ - a0 * x_ang_ - a1 * x_dot_ang_ - a2 * x_ddot_ang_) / a3;

    x_ddot_ang_ += x_dddot_ang * dt;
    x_dot_ang_ += x_ddot_ang_ * dt;
    x_ang_ += x_dot_ang_ * dt;

    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = x_lin_;
    msg_out.angular.z = x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
} */


//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
///// USANDO xdot_prev para el calculo de x
/////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////
// P2U y P2DU KP al principio
/////////////////////////////////
/* void SecondOrderFilterNode::update()
{
    double dt = 0.01;
    double x_dot_lin_prev = x_dot_lin_;
    double x_dot_ang_prev = x_dot_ang_;

    // Dinámica lineal
    x_ddot_lin_ = omega_n_ * omega_n_ * (kp_ * cmd_vel_input_lin_ - x_lin_) - 2.0 * zeta_ * omega_n_ * x_dot_lin_;
    x_dot_lin_ += x_ddot_lin_ * dt;
    x_lin_ += x_dot_lin_prev * dt;

    // Dinámica angular
    x_ddot_ang_ = omega_n_ * omega_n_ * (kp_ * cmd_vel_input_ang_ - x_ang_) - 2.0 * zeta_ * omega_n_ * x_dot_ang_;
    x_dot_ang_ += x_ddot_ang_ * dt;
    x_ang_ += x_dot_ang_prev * dt;

    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = x_lin_;
    msg_out.angular.z = x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
} */

/////////////////////////////////
// P2U y P2DU KP al final
/////////////////////////////////
// P2U y P2DU
/* void SecondOrderFilterNode::update()
{
    double dt = 0.01;
    double x_dot_lin_prev = x_dot_lin_;
    double x_dot_ang_prev = x_dot_ang_;

    // Dinámica lineal
    x_ddot_lin_ = omega_n_ * omega_n_ * (cmd_vel_input_lin_ - x_lin_) - 2.0 * zeta_ * omega_n_ * x_dot_lin_;
    x_dot_lin_ += x_ddot_lin_ * dt;
    x_lin_ += x_dot_lin_prev * dt;

    // Dinámica angular
    x_ddot_ang_ = omega_n_ * omega_n_ * (cmd_vel_input_ang_ - x_ang_) - 2.0 * zeta_ * omega_n_ * x_dot_ang_;
    x_dot_ang_ += x_ddot_ang_ * dt;
    x_ang_ += x_dot_ang_prev * dt;

    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = kp_ * x_lin_;
    msg_out.angular.z = kp_ * x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
} */


// P2 y P2D
/* void SecondOrderFilterNode::update()
{
    double dt = 0.01;
    double x_dot_lin_prev = x_dot_lin_;
    double x_dot_ang_prev = x_dot_ang_;

    // Dinámica lineal
    x_ddot_lin_ = (kp_ * cmd_vel_input_lin_ - x_lin_ - (tp1_ + tp2_) * x_dot_lin_) / (tp1_ * tp2_);
    x_dot_lin_ += x_ddot_lin_ * dt;
    x_lin_ += x_dot_lin_prev * dt;

    // Dinámica angular
    x_ddot_ang_ = (kp_ * cmd_vel_input_ang_ - x_ang_ - (tp1_ + tp2_) * x_dot_ang_) / (tp1_ * tp2_);
    x_dot_ang_ += x_ddot_ang_ * dt;
    x_ang_ += x_dot_ang_prev * dt;

    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = x_lin_;
    msg_out.angular.z = x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
} */


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//  TRAPECIO
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
// P2 y P2D
void SecondOrderFilterNode::update()
{
    double dt = 0.001;
    double x2_n_lin = x_dot_lin_, x1_n_lin = x_lin_;
    double x2_n_ang = x_dot_ang_, x1_n_ang = x_ang_;

    double a = (tp1_ + tp2_) / (tp1_ * tp2_);
    double b = 1 / (tp1_ * tp2_);
    double c = kp_ / (tp1_ * tp2_);


    // Dinámica lineal
    double x1_nmas1_lin = (1 / (1 + (a * dt / 2) + ((b * dt * dt) / (2 * 2)))) *
        ((x2_n_lin * dt) +
            (x1_n_lin * (1 - (b * dt * dt / (2 * 2)) + (a * dt / 2))) +
            ((dt * dt * c * (cmd_vel_input_n_lin_ + cmd_vel_input_lin_)) / (2 * 2)));

    double x2_nmas1_lin = (2 / dt) * (x1_nmas1_lin - x1_n_lin) - x2_n_lin;

    // Dinámica angular
    double x1_nmas1_ang = (1 / (1 + (a * dt / 2) + ((b * dt * dt) / (2 * 2)))) *
        ((x2_n_ang * dt) +
            (x1_n_ang * (1 - (b * dt * dt / (2 * 2)) + (a * dt / 2))) +
            ((dt * dt * c * (cmd_vel_input_n_ang_ + cmd_vel_input_ang_)) / (2 * 2)));

    double x2_nmas1_ang = (2 / dt) * (x1_nmas1_ang - x1_n_ang) - x2_n_ang;


    // Actualizacion de valores Dinamica Lineal
    x_dot_lin_ = x2_nmas1_lin;
    x_lin_ = x1_nmas1_lin;
    cmd_vel_input_n_lin_ = cmd_vel_input_lin_;

    //Actualizacion de valores Dinamica Angular
    x_dot_ang_ = x2_nmas1_ang;
    x_ang_ = x1_nmas1_ang;
    cmd_vel_input_n_ang_ = cmd_vel_input_ang_;



    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = x_lin_;
    msg_out.angular.z = x_ang_;
    filtered_pub_->publish(msg_out);
    RCLCPP_INFO(this->get_logger(), "Publicando %lf", msg_out.linear.x);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SecondOrderFilterNode>());
    rclcpp::shutdown();
    return 0;
}