#include "autolabor_canbus_driver/pm1_driver.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace autolabor_driver {


    Pm1Driver::Pm1Driver() : first_send_odom_flag_(true), sample_size_(7), threshold_(1e-7) {

    }

    Pm1Driver::~Pm1Driver() {

    }

    void Pm1Driver::ask_encoder(const ros::TimerEvent &) {
        autolabor_canbus_driver::CanBusService srv;

        autolabor_canbus_driver::CanBusMessage ask_ecu_encoder;
        ask_ecu_encoder.node_type = CANBUS_NODETYPE_ECU;
        ask_ecu_encoder.node_seq = CANBUS_NODESEQ_BROADCAST;
        ask_ecu_encoder.msg_type = CANBUS_MESSAGETYPE_ECU_CURRENTENCODER;
        srv.request.requests.push_back(ask_ecu_encoder);

        autolabor_canbus_driver::CanBusMessage ask_tcu_encoder;
        ask_tcu_encoder.node_type = CANBUS_NODETYPE_TCU;
        ask_tcu_encoder.node_seq = CANBUS_NODESEQ_BROADCAST;
        ask_tcu_encoder.msg_type = CANBUS_MESSAGETYPE_TCU_CURRENTANGLE;
        srv.request.requests.push_back(ask_tcu_encoder);

        canbus_client_.call(srv);
    }

    void Pm1Driver::send_odom() {
        ros::Time now = ros::Time::now();
        if ((now - ecu_left_time_).sec < sync_timeout_ && (now - ecu_right_time_).sec < sync_timeout_) {
            if (first_send_odom_flag_) {
                ecu_left_last_encoder_ = ecu_left_current_encoder_;
                ecu_right_last_encoder_ = ecu_right_current_encoder_;
                last_send_odom_time_ = now;
                accumulation_x_ = 0.0;
                accumulation_y_ = 0.0;
                accumulation_yaw_ = 0.0;
                first_send_odom_flag_ = false;
            } else {
                double delta_time = (now - last_send_odom_time_).sec;
                last_send_odom_time_ = now;

                int delta_left = calculate_delta(ecu_left_last_encoder_, ecu_left_current_encoder_);
                int delta_right = calculate_delta(ecu_right_last_encoder_, ecu_right_current_encoder_);

                double delta_dis = (delta_left + delta_right) / speed_coefficient_ / 2.0;
                double delta_theta = (delta_right - delta_left) / speed_coefficient_ / wheel_spacing_;

                double deltaX, deltaY;
                if (delta_theta == 0) {
                    deltaX = delta_dis;
                    deltaY = 0.0;
                } else {
                    deltaX = delta_dis * (sin(delta_theta) / delta_theta);
                    deltaY = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
                }

                accumulation_x_ += (cos(accumulation_yaw_) * deltaX - sin(accumulation_yaw_) * deltaY);
                accumulation_y_ += (sin(accumulation_yaw_) * deltaX + cos(accumulation_yaw_) * deltaY);
                accumulation_yaw_ += delta_theta;

                geometry_msgs::TransformStamped transform_stamped;
                transform_stamped.header.stamp = now;
                transform_stamped.header.frame_id = odom_frame_;
                transform_stamped.child_frame_id = base_frame_;
                transform_stamped.transform.translation.x = accumulation_x_;
                transform_stamped.transform.translation.y = accumulation_y_;
                transform_stamped.transform.translation.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, accumulation_yaw_);
                transform_stamped.transform.rotation.x = q.x();
                transform_stamped.transform.rotation.y = q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();

                br_.sendTransform(transform_stamped);

                nav_msgs::Odometry odom_msg;
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.child_frame_id = base_frame_;
                odom_msg.header.stamp = now;
                odom_msg.pose.pose.position.x = accumulation_x_;
                odom_msg.pose.pose.position.y = accumulation_y_;
                odom_msg.pose.pose.position.z = 0;
                odom_msg.pose.pose.orientation.x = q.getX();
                odom_msg.pose.pose.orientation.y = q.getY();
                odom_msg.pose.pose.orientation.z = q.getZ();
                odom_msg.pose.pose.orientation.w = q.getW();
                odom_msg.twist.twist.linear.x = delta_dis / delta_time;
                odom_msg.twist.twist.linear.y = 0;
                odom_msg.twist.twist.angular.z = delta_theta / delta_time;

                odom_pub_.publish(odom_msg);
            }
        }
    }

    void Pm1Driver::send_wheel_angle(double wheel_angle) {
        std_msgs::Float64 wheel_angle_msg;
        wheel_angle_msg.data = wheel_angle;
        wheel_angle_pub_.publish(wheel_angle_msg);
    }

    double Pm1Driver::calculate_target_angle(double x, double z) {
        if (x == 0) {
            return z >= 0 ? M_PI_2 : -M_PI_2;
        } else if (z == 0) {
            return 0.0;
        } else {
            return atan(shaft_spacing_ / (x / z));
        }
    }

    // 已经排除了线速度和角速度为0的情况
    void Pm1Driver::optimize_speed(double v, double omega, double angle, double &optimize_v, double &optimize_omega) {
        double theta;
        spin_to_theta(angle, theta);
        double rho_from = -1.0, rho_to = 1.0;
        double rho_diff = fabs(rho_to - rho_from);

        double best_rho = 0;

        while (rho_diff > threshold_) {
            int best_index = -1;
            double best_value = DBL_MAX;
            for (int i = 0; i < sample_size_; i++) {
                double rho = interpolation(rho_from, rho_to, sample_size_, i);
                double v_limit, omega_limit;
                rhotheta_to_vomega(rho, theta, v_limit, omega_limit);
                double value = optimize_function(v, omega, v_limit, omega_limit);
                if (value < best_value) {
                    best_index = i;
                    best_value = value;
                }
            }

            rho_diff /= sample_size_;
            if (best_index == 0) {
                best_rho = rho_from;
                rho_to = rho_from + rho_diff;
            } else if (best_index == sample_size_ - 1) {
                best_rho = rho_to;
                rho_from = rho_to - rho_diff;
            } else {
                best_rho = interpolation(rho_from, rho_to, sample_size_, best_index);
                rho_from = best_rho - rho_diff;
                rho_to = best_rho + rho_diff;
            }
        }
        rhotheta_to_vomega(best_rho, theta, optimize_v, optimize_omega);
    }

    void Pm1Driver::driver_car(double left, double right, double angle) {
        autolabor_canbus_driver::CanBusService srv;

        autolabor_canbus_driver::CanBusMessage ecu_left_target_speed;
        ecu_left_target_speed.node_type = CANBUS_NODETYPE_ECU;
        ecu_left_target_speed.node_seq = static_cast<unsigned char>(ecu_left_id_);
        ecu_left_target_speed.msg_type = CANBUS_MESSAGETYPE_ECU_TARGETSPEED;
        int_to_uint_vector(static_cast<int32_t>(lround(left * speed_coefficient_)), ecu_left_target_speed.payload);
        srv.request.requests.push_back(ecu_left_target_speed);

        autolabor_canbus_driver::CanBusMessage ecu_right_target_speed;
        ecu_right_target_speed.node_type = CANBUS_NODETYPE_ECU;
        ecu_right_target_speed.node_seq = static_cast<unsigned char>(ecu_right_id_);
        ecu_right_target_speed.msg_type = CANBUS_MESSAGETYPE_ECU_TARGETSPEED;
        int_to_uint_vector(static_cast<int32_t>(lround(right * speed_coefficient_)), ecu_right_target_speed.payload);
        srv.request.requests.push_back(ecu_right_target_speed);

        autolabor_canbus_driver::CanBusMessage tcu_target_angle;
        tcu_target_angle.node_type = CANBUS_NODETYPE_TCU;
        tcu_target_angle.node_seq = static_cast<unsigned char>(tcu_id_);
        tcu_target_angle.msg_type = CANBUS_MESSAGETYPE_TCU_TARGETANGLE;
        short_to_uint_vector(static_cast<int16_t>(lround(angle * spin_coefficient_)), tcu_target_angle.payload);
        srv.request.requests.push_back(tcu_target_angle);

        canbus_client_.call(srv);
    }

    void Pm1Driver::handle_canbus_msg(const autolabor_canbus_driver::CanBusMessage::ConstPtr &msg) {
        if ((msg->node_type == CANBUS_NODETYPE_ECU) && (msg->msg_type == CANBUS_MESSAGETYPE_ECU_CURRENTENCODER) && (!msg->payload.empty())) {
            if (msg->node_seq == ecu_left_id_) {
                ecu_left_time_ = ros::Time::now();
                ecu_left_current_encoder_ = uint_vector_to_int(msg->payload);
            } else if (msg->node_seq == ecu_right_id_) {
                ecu_right_time_ = ros::Time::now();
                ecu_right_current_encoder_ = uint_vector_to_int(msg->payload);
            }
            send_odom();
        } else if ((msg->node_type == CANBUS_NODETYPE_TCU) && (msg->msg_type == CANBUS_MESSAGETYPE_TCU_CURRENTANGLE) && (!msg->payload.empty())) {
            int16_t wheel_spin_encoder = uint_vector_to_short(msg->payload);
            double wheel_angle = wheel_spin_encoder / spin_coefficient_;
            send_wheel_angle(wheel_angle);

            if ((twist_cache_.linear.x != 0 || twist_cache_.angular.z != 0) && (ros::Time::now() - last_twist_time_).sec < twist_timeout_) {
                double optimize_v, optimize_omega; // 速度
                optimize_speed(twist_cache_.linear.x, twist_cache_.angular.z, wheel_angle, optimize_v, optimize_omega);
                double target_angle = calculate_target_angle(twist_cache_.linear.x, twist_cache_.angular.z);
                std::cout << "target_v : " << twist_cache_.linear.x << " target_omega : " << twist_cache_.angular.z << " target_angle : " << target_angle
                          << " optimize_v : " << optimize_v << " optimize_omega : " << optimize_omega << " current_angle : " << wheel_angle << std::endl;
                driver_car(optimize_v - optimize_omega * wheel_spacing_ / 2, optimize_v + optimize_omega * wheel_spacing_ / 2, target_angle);
            } else {
                driver_car(0.0, 0.0, wheel_angle);
            }
        }
    }

    void Pm1Driver::handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg) {
        last_twist_time_ = ros::Time::now();
        twist_cache_ = *msg;
    }

    void Pm1Driver::run() {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
        private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));

        private_node.param<int>("ecu_left_id", ecu_left_id_, 0);
        private_node.param<int>("ecu_right_id", ecu_right_id_, 1);
        private_node.param<int>("tcu_id", tcu_id_, 0);
        private_node.param<int>("rate", rate_, 10);

        private_node.param<double>("reduction_ratio", reduction_ratio_, 20.0);
        private_node.param<double>("encoder_resolution", encoder_resolution_, 1600.0);
        private_node.param<double>("wheel_diameter", wheel_diameter_, 0.211);
        private_node.param<double>("wheel_spacing", wheel_spacing_, 0.412);
        private_node.param<double>("shaft_spacing", shaft_spacing_, 0.324);
        private_node.param<double>("max_speed", max_speed_, 2.0);

        private_node.param<double>("twist_timeout", twist_timeout_, 1.0);

        private_node.param<double>("path_weight", path_weight_, 1.5);
        private_node.param<double>("endpoint_weight", endpoint_weight_, 1.0);
        private_node.param<double>("angle_weight", angle_weight_, 0.5);

        sync_timeout_ = 0.5 / rate_;
        speed_coefficient_ = reduction_ratio_ * encoder_resolution_ / M_PI / wheel_diameter_;
        spin_coefficient_ = 8190.0 / M_PI;
        max_motion_encoder_ = max_speed_ * speed_coefficient_;

        canbus_msg_subscriber_ = node.subscribe("/canbus_msg", 100, &Pm1Driver::handle_canbus_msg, this);
        twist_subscriber_ = node.subscribe("/cmd_vel", 10, &Pm1Driver::handle_twist_msg, this);
        canbus_client_ = node.serviceClient<autolabor_canbus_driver::CanBusService>("canbus_server");

        odom_pub_ = node.advertise<nav_msgs::Odometry>("/odom", 10);
        wheel_angle_pub_ = node.advertise<std_msgs::Float64>("/wheel_angle", 10);

        ros::Timer ask_encoder_timer = node.createTimer(ros::Duration(1.0 / rate_), &Pm1Driver::ask_encoder, this);

        ros::spin();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pm1_driver");
    autolabor_driver::Pm1Driver driver;
    driver.run();
    return 0;
}