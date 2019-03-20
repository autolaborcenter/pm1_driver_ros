#include "autolabor_canbus_driver/pm1_driver.h"
#include "utilities/big_endian_transform.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace autolabor_driver {


    Pm1Driver::Pm1Driver() : first_send_odom_flag_(true) {

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
        if ((now - ecu_left_time_).toSec() < sync_timeout_ && (now - ecu_right_time_).toSec() < sync_timeout_) {
            if (first_send_odom_flag_) {
                ecu_left_last_encoder_ = ecu_left_current_encoder_;
                ecu_right_last_encoder_ = ecu_right_current_encoder_;
                last_send_odom_time_ = now;
                accumulation_x_ = 0.0;
                accumulation_y_ = 0.0;
                accumulation_yaw_ = 0.0;
                smooth_speed_ = 0.0;
                first_send_odom_flag_ = false;
            } else {
                double delta_time = (now - last_send_odom_time_).toSec();

                int delta_left = calculate_delta(ecu_left_last_encoder_, ecu_left_current_encoder_);
                int delta_right = calculate_delta(ecu_right_last_encoder_, ecu_right_current_encoder_);

                last_send_odom_time_ = now;
                ecu_left_last_encoder_ = ecu_left_current_encoder_;
                ecu_right_last_encoder_ = ecu_right_current_encoder_;

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
            return z >= 0 ? -M_PI_2 : M_PI_2;
        } else if (z == 0) {
            return 0.0;
        } else {
            return -atan(shaft_spacing_ / (x / z));
        }
    }

    wheels Pm1Driver::optimize_speed(double v, double omega, double angle) {
        struct velocity input_velocity = {static_cast<float>(v), static_cast<float>(omega)};
        auto physical = velocity_to_physical(&input_velocity, &user_config);
        double diff_angle = fabs(angle - physical.rudder);
        double opt_speed = diff_angle > optimize_limit_ ? 0 : (1 - diff_angle / optimize_limit_) * physical.speed;
        smooth_speed_ = opt_speed > smooth_speed_ ? fmin(opt_speed, smooth_speed_ + smooth_coefficient_) : fmax(opt_speed, smooth_speed_ - smooth_coefficient_);
        struct physical output_physical = {static_cast<float>(smooth_speed_), static_cast<float>(angle)};
        return physical_to_wheels(&output_physical, &user_config);
    }

    void Pm1Driver::driver_car(double left, double right, double angle) {
        autolabor_canbus_driver::CanBusService srv;

        autolabor_canbus_driver::CanBusMessage ecu_left_target_speed;
        ecu_left_target_speed.node_type = CANBUS_NODETYPE_ECU;
        ecu_left_target_speed.node_seq = static_cast<unsigned char>(ecu_left_id_);
        ecu_left_target_speed.msg_type = CANBUS_MESSAGETYPE_ECU_TARGETSPEED;
        ecu_left_target_speed.payload = autolabor::pack(static_cast<int32_t>(lround(left * speed_coefficient_)));
        srv.request.requests.push_back(ecu_left_target_speed);

        autolabor_canbus_driver::CanBusMessage ecu_right_target_speed;
        ecu_right_target_speed.node_type = CANBUS_NODETYPE_ECU;
        ecu_right_target_speed.node_seq = static_cast<unsigned char>(ecu_right_id_);
        ecu_right_target_speed.msg_type = CANBUS_MESSAGETYPE_ECU_TARGETSPEED;
        ecu_right_target_speed.payload = autolabor::pack(static_cast<int32_t>(lround(right * speed_coefficient_)));
        srv.request.requests.push_back(ecu_right_target_speed);

        autolabor_canbus_driver::CanBusMessage tcu_target_angle;
        tcu_target_angle.node_type = CANBUS_NODETYPE_TCU;
        tcu_target_angle.node_seq = static_cast<unsigned char>(tcu_id_);
        tcu_target_angle.msg_type = CANBUS_MESSAGETYPE_TCU_TARGETANGLE;
        tcu_target_angle.payload = autolabor::pack(static_cast<int16_t>(lround(angle * spin_coefficient_)));
        srv.request.requests.push_back(tcu_target_angle);

        canbus_client_.call(srv);
    }

    void Pm1Driver::handle_canbus_msg(const autolabor_canbus_driver::CanBusMessage::ConstPtr &msg) {
        if (msg->payload.empty()) {
            ROS_DEBUG_STREAM("RECEIVE_CANBUS_MESSAGE: " << " NODE_TYPE:" << (int) msg->node_type << ","
                                                        << " NODE_SEQ:" << (int) msg->node_seq << ","
                                                        << " MSG_TYPE:" << (int) msg->msg_type);
        } else {
            ROS_DEBUG_STREAM("RECEIVE_CANBUS_MESSAGE: " << " NODE_TYPE:" << (int) msg->node_type << ","
                                                        << " NODE_SEQ:" << (int) msg->node_seq << ","
                                                        << " MSG_TYPE:" << (int) msg->msg_type << ","
                                                        << " MSG_PAYLOAD:" << vector_to_string(msg->payload));
        }

        if ((msg->node_type == CANBUS_NODETYPE_ECU) && (msg->msg_type == CANBUS_MESSAGETYPE_ECU_CURRENTENCODER) && (!msg->payload.empty())) {
            if (msg->node_seq == ecu_left_id_) {
                ecu_left_time_ = ros::Time::now();
                ecu_left_current_encoder_ = autolabor::build<uint32_t>(msg->payload.data());
            } else if (msg->node_seq == ecu_right_id_) {
                ecu_right_time_ = ros::Time::now();
                ecu_right_current_encoder_ = autolabor::build<uint32_t>(msg->payload.data());
            }
            send_odom();
        } else if ((msg->node_type == CANBUS_NODETYPE_TCU) && (msg->msg_type == CANBUS_MESSAGETYPE_TCU_CURRENTANGLE) && (!msg->payload.empty())) {
            int16_t wheel_spin_encoder = autolabor::build<int16_t>(msg->payload.data());
            double wheel_angle = wheel_spin_encoder / spin_coefficient_;
            send_wheel_angle(wheel_angle);

            if ((twist_cache_.linear.x != 0 || twist_cache_.angular.z != 0) && (ros::Time::now() - last_twist_time_).sec < twist_timeout_) {
                struct wheels opt_wheels = optimize_speed(twist_cache_.linear.x, twist_cache_.angular.z, wheel_angle);
                double target_angle = calculate_target_angle(twist_cache_.linear.x, twist_cache_.angular.z);
                driver_car(opt_wheels.left, opt_wheels.right, target_angle);
                ROS_DEBUG_STREAM(
                    "TARGET_SPEED:  target_v ->   " << std::setw(5) << twist_cache_.linear.x << ", target_omega ->   " << twist_cache_.angular.z << ", target_angle  -> "
                                                    << target_angle);
            } else {
                smooth_speed_ = 0 > smooth_speed_ ? fmin(0, smooth_speed_ + smooth_coefficient_) : fmax(0, smooth_speed_ - smooth_coefficient_);
                struct physical physical_smooth = {static_cast<float>(smooth_speed_), static_cast<float>(wheel_angle)};
                struct wheels opt_wheels = physical_to_wheels(&physical_smooth, &user_config);
                driver_car(opt_wheels.left, opt_wheels.right, wheel_angle);
            }
        }
    }

    void Pm1Driver::handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg) {
        last_twist_time_ = ros::Time::now();
        twist_cache_ = *msg;
    }

    bool Pm1Driver::reset_odom(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        first_send_odom_flag_ = true;
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
        private_node.param<double>("wheel_diameter", wheel_diameter_, 0.20);
        private_node.param<double>("wheel_spacing", wheel_spacing_, 0.4137);
        private_node.param<double>("shaft_spacing", shaft_spacing_, 0.317);
        private_node.param<double>("max_speed", max_speed_, 2.0);

        private_node.param<double>("twist_timeout", twist_timeout_, 1.0);

        private_node.param<double>("optimize_limit", optimize_limit_, M_PI / 4);
        private_node.param<double>("smooth_coefficient", smooth_coefficient_, 0.1);

        user_config.width = static_cast<float>(wheel_spacing_);
        user_config.length = static_cast<float>(shaft_spacing_);
        user_config.radius = static_cast<float>(wheel_diameter_);
        user_config.max_wheel_speed = static_cast<float>(max_speed_);


        sync_timeout_ = 0.5 / rate_;
        speed_coefficient_ = reduction_ratio_ * encoder_resolution_ / M_PI / wheel_diameter_;
        spin_coefficient_ = 8190.0 / M_PI;
        max_motion_encoder_ = max_speed_ * speed_coefficient_;

        canbus_msg_subscriber_ = node.subscribe("/canbus_msg", 100, &Pm1Driver::handle_canbus_msg, this);
        twist_subscriber_ = node.subscribe("/cmd_vel", 10, &Pm1Driver::handle_twist_msg, this);
        canbus_client_ = node.serviceClient<autolabor_canbus_driver::CanBusService>("canbus_server");

        odom_pub_ = node.advertise<nav_msgs::Odometry>("/odom", 10);
        wheel_angle_pub_ = node.advertise<std_msgs::Float64>("/wheel_angle", 10);

        reset_odom_service_ = node.advertiseService("reset_odom", &Pm1Driver::reset_odom, this);

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