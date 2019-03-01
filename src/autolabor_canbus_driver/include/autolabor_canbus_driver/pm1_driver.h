#ifndef AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H
#define AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H

#include <climits>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_broadcaster.h>

#include "autolabor_canbus_driver/CanBusMessage.h"
#include "autolabor_canbus_driver/CanBusService.h"

const int CANBUS_NODETYPE_ECU = 0x11;
const int CANBUS_MESSAGETYPE_ECU_TARGETSPEED = 0x01;
const int CANBUS_MESSAGETYPE_ECU_CURRENTENCODER = 0x06;

const int CANBUS_NODETYPE_TCU = 0x12;
const int CANBUS_MESSAGETYPE_TCU_TARGETANGLE = 0x01;
const int CANBUS_MESSAGETYPE_TCU_CURRENTANGLE = 0x03;

const int CANBUS_NODESEQ_BROADCAST = 0x0F;

namespace autolabor_driver {

    struct Point {
        double x;
        double y;
        double yaw;
    };


    class Pm1Driver {
    public:
        Pm1Driver();

        ~Pm1Driver() = default;

        void run();

    private:
        void ask_encoder(const ros::TimerEvent &);

        void handle_canbus_msg(const autolabor_canbus_driver::CanBusMessage::ConstPtr &msg);

        void handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg);

        void send_odom();

        void send_wheel_angle(double wheel_angle);

        double calculate_target_angle(double x, double z);

        void optimize_speed(double x, double z, double angle, double &left, double &right);

        void driver_car(double left, double right, double angle);

        inline int calculate_delta(long last, long current) {
            long delta = current > last
                         ?(current - last) < INT32_MAX ? (current - last) : (current - last - UINT32_MAX)
                         :(last - current) < INT32_MAX ? (current - last) : (current - last + UINT32_MAX);
            return static_cast<int>(delta);
        }

        inline void spin_to_theta(double spin, double &theta) {
            if (spin == 0) {
                theta = M_PI_4;
            } else if (spin == M_PI_2 || spin == -M_PI_2) {
                theta = -M_PI_4;
            } else {
                double c = 2 * shaft_spacing_ / (tan(spin) * wheel_spacing_);
                if (c == -1) {
                    theta = 0;
                } else {
                    theta = atan((c - 1) / (c + 1));
                }
            }
        }

        inline void rhotheta_to_vomega(double rho, double theta, double &v, double &omega) {
            double left_speed = 0, right_speed = 0;
            if (fabs(theta) < M_PI_4) {
                left_speed = rho * max_speed_;
                right_speed = tan(theta) * left_speed;
            } else {
                right_speed = rho * max_speed_;
                left_speed = tan(M_PI_2 - theta) * right_speed;
            }
            v = (left_speed + right_speed) / 2;
            omega = (left_speed - right_speed) / wheel_spacing_;
        }

        inline void vomega_to_xyyaw(double v, double omega, double &x, double &y, double &yaw) {
            if (v == 0 && omega == 0) {
                x = 0.0;
                y = 0.0;
                yaw = 0.0;
            } else if (v == 0) {
                x = 0.0;
                y = 0.0;
                yaw = omega / rate_;
            } else if (omega == 0) {
                x = v / rate_;
                y = 0;
                yaw = 0.0;
            } else {
                double r = v / omega;
                yaw = omega / rate_;
                x = r * sin(yaw);
                y = r * (1 - cos(yaw));
            }
        }

        inline double optimize_function(double v_target, double omega_target, double v_limit, double omega_limit) {
            double x_target, y_target, yaw_target;
            vomega_to_xyyaw(v_target, omega_target, x_target, y_target, yaw_target);
            double x_limit, y_limit, yaw_limit;
            vomega_to_xyyaw(v_limit, omega_limit, x_limit, y_limit, yaw_limit);

            double distance_from_endpoint = sqrt(pow(x_target - x_limit, 2) + pow(y_target - y_limit, 2));
            double distance_from_yaw = fabs(yaw_target - yaw_limit);
            double distance_from_targetpath;
            if (y_target == 0) { // target
                distance_from_targetpath = fabs(y_limit);
            } else {
                double r_target = v_target / omega_target;
                distance_from_targetpath = fabs(sqrt(pow(x_limit, 2.0) + pow(y_limit - r_target, 2.0)) - fabs(r_target));
            }
            return path_weight_ * distance_from_targetpath + endpoint_weight_ * distance_from_endpoint + angle_weight_ * distance_from_yaw;
        }

        inline double interpolation(double from, double to, int number, int index) {
            return from + index * (to - from) / (number - 1);
        }

        inline std::string vector_to_string(std::vector<uint8_t> v) {
            std::stringstream ss;
            for (int i = 0; i < v.size(); i++)
                ss << std::hex << std::setw(2) << std::setfill('0') << (int) v[i] << " ";
            return ss.str();
        }

    private:
        ros::Time last_twist_time_;
        geometry_msgs::Twist twist_cache_;
        double twist_timeout_;

        std::string odom_frame_, base_frame_;
        int ecu_left_id_, ecu_right_id_, tcu_id_;
        int rate_;
        double sync_timeout_;

        double reduction_ratio_, encoder_resolution_, wheel_diameter_, wheel_spacing_, shaft_spacing_, max_speed_;
        double speed_coefficient_, max_motion_encoder_, spin_coefficient_;

        ros::Time ecu_left_time_, ecu_right_time_;
        long ecu_left_last_encoder_, ecu_right_last_encoder_;
        long ecu_left_current_encoder_, ecu_right_current_encoder_;

        bool first_send_odom_flag_;
        ros::Time last_send_odom_time_;
        double accumulation_x_, accumulation_y_, accumulation_yaw_;

        double path_weight_, endpoint_weight_, angle_weight_;
        int sample_size_;
        double threshold_;


        tf2_ros::TransformBroadcaster br_;

        ros::Subscriber twist_subscriber_;
        ros::Subscriber canbus_msg_subscriber_;
        ros::ServiceClient canbus_client_;

        ros::Publisher odom_pub_;
        ros::Publisher wheel_angle_pub_;

    };

}


#endif //AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H
