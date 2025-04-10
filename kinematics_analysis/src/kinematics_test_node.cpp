#include "kinematics_analysis/kinematics_test_node.hpp"


KinematicsTestNode::KinematicsTestNode() : Node("kinematics_test_node")
{
    declareParameters();
    initializePublishersAndSubscribers();
    initializeDataFile();

    if (use_rosbag_) {
        initializeBagWriter();
    }

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

    last_imu_msg_.header.stamp.sec = 0;
    last_imu_msg_.header.stamp.nanosec = 0;
    last_imu_msg_.linear_acceleration.x = 0.0;
    last_imu_msg_.linear_acceleration.y = 0.0;
    last_imu_msg_.linear_acceleration.z = 0.0;
    last_imu_msg_.angular_velocity.x = 0.0;
    last_imu_msg_.angular_velocity.y = 0.0;
    last_imu_msg_.angular_velocity.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Kinematics Test Node iniciado. Comenzando prueba en %lf segundos...", delay_start_duration_);

    // Iniciar el time de delay start
    start_test_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(std::lround(delay_start_duration_ * 1000))), std::bind(&KinematicsTestNode::startTest, this));
}

void KinematicsTestNode::declareParameters()
{
    // Declarar parametros
    this->declare_parameter("test_velocity", 1.0);
    this->declare_parameter("delay_start_duration", 7.0);
    this->declare_parameter("cmd_test_duration", 2.0);
    this->declare_parameter("finish_test_duration", 2.0);
    this->declare_parameter("output_file_path", "velocity_test_results.csv");
    this->declare_parameter("use_rosbag", true);
    this->declare_parameter("rosbag_path", "velocity_test.bag");

    test_velocity_ = this->get_parameter("test_velocity").as_double();
    delay_start_duration_ = this->get_parameter("delay_start_duration").as_double();
    cmd_test_duration_ = this->get_parameter("cmd_test_duration").as_double();
    finish_test_duration_ = this->get_parameter("finish_test_duration").as_double();
    output_file_path_ = this->get_parameter("output_file_path").as_string();
    use_rosbag_ = this->get_parameter("use_rosbag").as_bool();
    rosbag_path_ = this->get_parameter("rosbag_path").as_string();

    RCLCPP_INFO(this->get_logger(),
        "Configuracion: Velocidad=%lf m/s, Duracion=%lf s, Start Delay=%lf s, Tiempo para stop=%lf s",
        test_velocity_, cmd_test_duration_, delay_start_duration_, finish_test_duration_);
}

void KinematicsTestNode::initializePublishersAndSubscribers()
{
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/wheels", 10,
        std::bind(&KinematicsTestNode::odomCallback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
        std::bind(&KinematicsTestNode::imuCallback, this, std::placeholders::_1));
}

void KinematicsTestNode::initializeDataFile()
{
    data_file_.open(output_file_path_, std::ofstream::out | std::ofstream::app);
    if (!data_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo: %s", output_file_path_.c_str());
        return;
    }

    // Si se pudo abrir, se escribe la cabecera del CSV
    data_file_ << "timestamp,time_from_start,"
        << "pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw,"
        << "vel_linear_x,vel_linear_y,vel_linear_z,vel_angular_x,vel_angular_y,vel_angular_z,"
        << "accel_linear_x,accel_linear_y,accel_linear_z,accel_angular_x,accel_angular_y,accel_angular_z\n";

    RCLCPP_INFO(this->get_logger(), "Archivo de datos inicializado: %s", output_file_path_.c_str());
}

void KinematicsTestNode::initializeBagWriter()
{
    try {
        bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
        bag_writer_->open(rosbag_path_);
        RCLCPP_INFO(this->get_logger(), "ROS Bag inicializado en: %s", rosbag_path_.c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error al inicializar ROS Bag: %s", e.what());
        // Si ha habido problemas no se usa
        use_rosbag_ = false;
    }

    if (use_rosbag_) {

        odom_bag_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/wheels", 10,
            std::bind(&KinematicsTestNode::odomBagCallback, this, std::placeholders::_1));

        imu_bag_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
            std::bind(&KinematicsTestNode::imuBagCallback, this, std::placeholders::_1));
    }
}

void KinematicsTestNode::startTest()
{
    // Cancela timer de start delay / inicio
    start_test_timer_->cancel();

    // Captura el tiempo de inicio del test
    test_start_reference_time_ = this->now();
    test_started_ = true;

    cmd_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&KinematicsTestNode::publishCmdVel, this));

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
        std::chrono::milliseconds(20), std::bind(&KinematicsTestNode::publishStopVel, this));

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

    // Cierra el ROS Bag si está en uso y apuntando correctamente a un objeto
    if (use_rosbag_ && bag_writer_) {
        //mirar el close, no está implementado
        bag_writer_.reset();
        RCLCPP_INFO(this->get_logger(), "ROS Bag cerrado");
    }

    RCLCPP_INFO(this->get_logger(), "Los resultados se han guardado en: %s", output_file_path_.c_str());
}


void KinematicsTestNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Si el test no ha comenzado, no procesar datos
    if (!test_started_) {
        return;
    }

    // Actualiza el último mensaje de odometría
    last_odom_msg_ = *msg;

    // Escribe los datos al archivo CSV
    writeDataToFile();
}

void KinematicsTestNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Si el test aún no ha comenzado, no procesar datos
    if (!test_started_) {
        return;
    }

    // Actualiza el último mensaje de IMU
    last_imu_msg_ = *msg;

    // Escribe los datos al archivo CSV
    writeDataToFile();
}

void KinematicsTestNode::odomBagCallback(std::shared_ptr < rclcpp::SerializedMessage> msg) const
{
    rclcpp::Time time_stamp = this->now();

    bag_writer_->write(msg, "/odometry/wheels", "nav_msgs/msg/Odometry", time_stamp);

}

void KinematicsTestNode::imuBagCallback(std::shared_ptr < rclcpp::SerializedMessage> msg) const
{
    rclcpp::Time time_stamp = this->now();

    bag_writer_->write(msg, "/imu/data", "sensor_msgs/msg/Imu", time_stamp);
}

void KinematicsTestNode::writeDataToFile()
{
    // Si no se ha abierto el CSV o no ha comenzado la captura en el test
    if (!data_file_.is_open() || !test_started_) {
        return;
    }

    // Usa la marca de tiempo más reciente (ya sea de odom o IMU)
    rclcpp::Time timestamp;

    rclcpp::Time odom_time(last_odom_msg_.header.stamp);
    rclcpp::Time imu_time(last_imu_msg_.header.stamp);
    timestamp = (odom_time > imu_time) ? odom_time : imu_time;

    // Calcula el tiempo desde el inicio del test, para tiempo relativo no absoluto
    auto time_from_start = (timestamp - test_start_reference_time_).nanoseconds();

    // Escribe los datos en el archivo CSV
    data_file_ << timestamp.nanoseconds() << ","
        << time_from_start << ","
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
        << last_imu_msg_.linear_acceleration.x << ","
        << last_imu_msg_.linear_acceleration.y << ","
        << last_imu_msg_.linear_acceleration.z << ","
        << last_imu_msg_.angular_velocity.x << ","
        << last_imu_msg_.angular_velocity.y << ","
        << last_imu_msg_.angular_velocity.z << "\n";
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsTestNode>());
    rclcpp::shutdown();
    return 0;
}