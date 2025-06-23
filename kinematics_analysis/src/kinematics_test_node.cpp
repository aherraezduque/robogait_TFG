#include "kinematics_analysis/kinematics_test_node.hpp"


KinematicsTestNode::KinematicsTestNode() : Node("kinematics_test_node")
{
    declareParameters();
    initializePublishersAndSubscribers();
    initializeDataFile();

    // Inicializar mensajes por defecto
    last_odom_msg_.header.stamp.sec = 0;
    last_odom_msg_.header.stamp.nanosec = 0;
    last_odom_msg_.pose.pose.position.x = 0.0;
    last_odom_msg_.pose.pose.position.y = 0.0;
    last_odom_msg_.pose.pose.position.z = 0.0;
    last_odom_msg_.pose.pose.orientation.w = 1.0;
    last_odom_msg_.pose.pose.orientation.x = 0.0;
    last_odom_msg_.pose.pose.orientation.y = 0.0;
    last_odom_msg_.pose.pose.orientation.z = 0.0;
    last_odom_msg_.twist.twist.linear.x = 0.0;
    last_odom_msg_.twist.twist.linear.y = 0.0;
    last_odom_msg_.twist.twist.linear.z = 0.0;
    last_odom_msg_.twist.twist.angular.x = 0.0;
    last_odom_msg_.twist.twist.angular.y = 0.0;
    last_odom_msg_.twist.twist.angular.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Kinematics Test Node iniciado. Comenzando prueba en %lf segundos...", delay_start_duration_);


    start_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&KinematicsTestNode::publishStartVel, this));

    // Iniciar el timer de delay start
    start_test_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(std::lround(delay_start_duration_ * 1000))),
        std::bind(&KinematicsTestNode::startTest, this));
}

void KinematicsTestNode::declareParameters()
{
    // Declarar parametros
    this->declare_parameter("start_velocity", 0.1);
    this->declare_parameter("test_velocity", 1.0);
    this->declare_parameter("delay_start_duration", 1.0);
    this->declare_parameter("cmd_test_duration", 7.0);
    this->declare_parameter("finish_test_duration", 7.0);
    this->declare_parameter("output_file_path", "velocity_test_results.csv");

    start_velocity_ = this->get_parameter("start_velocity").as_double();
    test_velocity_ = this->get_parameter("test_velocity").as_double();
    delay_start_duration_ = this->get_parameter("delay_start_duration").as_double();
    cmd_test_duration_ = this->get_parameter("cmd_test_duration").as_double();
    finish_test_duration_ = this->get_parameter("finish_test_duration").as_double();
    output_file_path_ = this->get_parameter("output_file_path").as_string();

    RCLCPP_INFO(this->get_logger(),
        "Configuracion: Velocidad=%lf m/s, Duracion=%lf s, Start Delay=%lf s, Tiempo para stop=%lf s",
        test_velocity_, cmd_test_duration_, delay_start_duration_, finish_test_duration_);
}

void KinematicsTestNode::initializePublishersAndSubscribers()
{
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
        std::bind(&KinematicsTestNode::cmdvelCallback, this, std::placeholders::_1));

    /* odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/wheels", 10,
        std::bind(&KinematicsTestNode::odomCallback, this, std::placeholders::_1)); */
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/diff_drive_controller/odom", 10,
        std::bind(&KinematicsTestNode::odomCallback, this, std::placeholders::_1));
}

void KinematicsTestNode::initializeDataFile()
{
    data_file_.open(output_file_path_, std::ofstream::out | std::ofstream::app);
    if (!data_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo: %s", output_file_path_.c_str());
        return;
    }

    // Si se pudo abrir, se escribe la cabecera del CSV
    data_file_ << "timestamp,"
        << "pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw,"
        << "vel_linear_x,vel_linear_y,vel_linear_z,vel_angular_x,vel_angular_y,vel_angular_z,"
        << "cmd_vel_linear_x,cmd_vel_linear_y,cmd_vel_linear_z,"
        << "cmd_vel_angular_x,cmd_vel_angular_y,cmd_vel_angular_z\n";

    RCLCPP_INFO(this->get_logger(), "Archivo de datos inicializado: %s", output_file_path_.c_str());
    test_started_ = true;
}

void KinematicsTestNode::startTest()
{
    // Cancela timer de start delay / inicio
    start_test_timer_->cancel();
    start_vel_timer_->cancel();


    cmd_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&KinematicsTestNode::publishCmdVel, this));

    RCLCPP_INFO(this->get_logger(), "Test iniciado: Enviando Twist.linear.x =  %f m/s", test_velocity_);

    // Programa el tiempo durante el que se manda la velocidad test_velocity
    stop_test_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(std::lround(cmd_test_duration_ * 1000))),
        std::bind(&KinematicsTestNode::stopRobot, this));
}

void KinematicsTestNode::stopRobot()
{
    // Cancelar timer de parada
    stop_test_timer_->cancel();

    // Cancelar timer de cmd_vel publisher
    cmd_vel_timer_->cancel();

    // Configura el timer stop_vel_timer para que publique stop durante el 
    stop_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&KinematicsTestNode::publishStopVel, this));

    RCLCPP_INFO(this->get_logger(), "Enviando comando de parada");

    // Programa el tiempo durante el que se siguen capturando datos tras mandar stop al robot
    finish_test_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(std::lround(finish_test_duration_ * 1000))),
        std::bind(&KinematicsTestNode::finishTest, this));
}

void KinematicsTestNode::finishTest()
{
    // Cancelar el timer de finalizacion del test
    finish_test_timer_->cancel();

    // Cancelar timer de cmd_vel publisher
    stop_vel_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Test finalizado");

    // Finaliza el test
    processTest();
}

void KinematicsTestNode::publishStartVel()
{
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = start_velocity_;
    twist_msg->angular.z = 0.0;

    cmd_vel_publisher_->publish(std::move(twist_msg));
}

void KinematicsTestNode::publishCmdVel()
{
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = test_velocity_;
    twist_msg->angular.z = 0.0;

    cmd_vel_publisher_->publish(std::move(twist_msg));
}

void KinematicsTestNode::publishStopVel()
{
    // Enviar stop al robot
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = 0.0;
    twist_msg->angular.z = 0.0;

    cmd_vel_publisher_->publish(std::move(twist_msg));
}

void KinematicsTestNode::processTest()
{
    RCLCPP_INFO(this->get_logger(), "Cerrando recursos de escritura...");

    // Cierra el archivo CSV
    if (data_file_.is_open()) {
        data_file_.close();
        RCLCPP_INFO(this->get_logger(), "Archivo CSV cerrado");
    }

    RCLCPP_INFO(this->get_logger(), "Los resultados se han guardado en: %s", output_file_path_.c_str());
}

void KinematicsTestNode::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Si el test no ha comenzado, no procesar datos
    if (!test_started_) {
        return;
    }

    last_cmd_vel_msg_ = *msg;
}

void KinematicsTestNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Si el test no ha comenzado, no procesar datos
    if (!test_started_) {
        return;
    }

    last_odom_msg_ = *msg;

    // Escribe los datos al archivo CSV
    writeDataToFile();
}

void KinematicsTestNode::writeDataToFile()
{
    // Si no se ha abierto el CSV o no ha comenzado la captura en el test
    if (!data_file_.is_open() || !test_started_) {
        return;
    }

    // Proteger el acceso a last_odom_msg_ durante la escritura
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Usa la marca de tiempo del mensaje de odometría
    rclcpp::Time timestamp(last_odom_msg_.header.stamp);

    // Escribe los datos en el archivo CSV
    data_file_ << timestamp.nanoseconds() << ","
        << last_odom_msg_.pose.pose.position.x << ","
        << last_odom_msg_.pose.pose.position.y << ","
        << last_odom_msg_.pose.pose.position.z << ","
        << last_odom_msg_.pose.pose.orientation.x << ","
        << last_odom_msg_.pose.pose.orientation.y << ","
        << last_odom_msg_.pose.pose.orientation.z << ","
        << last_odom_msg_.pose.pose.orientation.w << ","
        << last_odom_msg_.twist.twist.linear.x << ","
        << last_odom_msg_.twist.twist.linear.y << ","
        << last_odom_msg_.twist.twist.linear.z << ","
        << last_odom_msg_.twist.twist.angular.x << ","
        << last_odom_msg_.twist.twist.angular.y << ","
        << last_odom_msg_.twist.twist.angular.z << ","
        << last_cmd_vel_msg_.linear.x << ","
        << last_cmd_vel_msg_.linear.y << ","
        << last_cmd_vel_msg_.linear.z << ","
        << last_cmd_vel_msg_.angular.x << ","
        << last_cmd_vel_msg_.angular.y << ","
        << last_cmd_vel_msg_.angular.z << "\n";
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsTestNode>());
    rclcpp::shutdown();
    return 0;
}