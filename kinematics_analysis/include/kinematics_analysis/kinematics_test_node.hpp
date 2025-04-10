#ifndef KINEMATICS_TEST_NODE_HPP
#define KINEMATICS_TEST_NODE_HPP

// ROS2
#include <rclcpp/rclcpp.hpp>

// MSG INTERFACES
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

// std
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

// ROSBAG
#include <rosbag2_cpp/writer.hpp>


class KinematicsTestNode : public rclcpp::Node
{
public:
    KinematicsTestNode();

private:
    ////////////////////////////////////
    //  Parametros
    ///////////////////////////////////

    // cmd_vel para el test
    double test_velocity_;
    // tiempo de espera previo al inicio del test
    double delay_start_duration_;
    // duracion de cmd_vel/test
    double cmd_test_duration_;
    // tiempo durante el que sigue midiendo tras stop cmd_vel
    double finish_test_duration_;
    // path del archivo donde se guardan las mediciones
    std::string output_file_path_;

    // habilitar captura en rosbag
    bool use_rosbag_;
    // path rosbag
    std::string rosbag_path_;

    ////////////////////////////////////
    // Publishers y Subscribers
    ////////////////////////////////////

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_bag_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_bag_subscriber_;

    ////////////////////////////////////
    // Timers
    ////////////////////////////////////

    // Al alcanzar start_delay se inicia el test, se envia test_velocity cmd_vel
    rclcpp::TimerBase::SharedPtr start_test_timer_;
    // Al alcanzar cmd_test_duration se lanza stop cmd_vel
    rclcpp::TimerBase::SharedPtr stop_test_timer_;
    // Al alcanzar finish_test_duration se dejan de capturar datos, el test termina
    rclcpp::TimerBase::SharedPtr finish_test_timer_;
    // Timer usado para la publicacion de test_velocity en cmd_vel
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
    // Timer usado para la publicacion de stop en cmd_vel
    rclcpp::TimerBase::SharedPtr stop_vel_timer_;

    ////////////////////////////////////
    // Flags
    ////////////////////////////////////

    // Controlar la captura de datos
    bool test_started_ = false;
    // Tiempo en el que comenzó la captura de datos, para referencias relativas de tiempo
    rclcpp::Time test_start_reference_time_;

    ////////////////////////////////////
    // Ultimo msgs recibidos
    ////////////////////////////////////
    nav_msgs::msg::Odometry last_odom_msg_;
    sensor_msgs::msg::Imu last_imu_msg_;

    ////////////////////////////////////
    // Recursos de almacenaje de datos
    ////////////////////////////////////

    // Archivo CSV para guardar datos en tiempo real
    std::ofstream data_file_;
    // ROS Bag writer
    std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;

    ////////////////////////////////////
    // Metodos
    ////////////////////////////////////

    void declareParameters();
    void initializePublishersAndSubscribers();
    void initializeDataFile();
    void initializeBagWriter();

    void startTest();
    void stopRobot();
    void finishTest();

    void publishCmdVel();
    void publishStopVel();

    void processTest();

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomBagCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) const;
    void imuBagCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) const;
    void writeDataToFile();
};

#endif // KINEMATICS_TEST_NODE_HPP