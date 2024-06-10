#ifndef ROS_CONTROL__UR_GRIPPER_HW_INTERFACE_H
#define ROS_CONTROL__UR_GRIPPER_HW_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "std_msgs/Float32.h"


using namespace hardware_interface;


namespace ur_gripper_hw_interface
{

    class UrGripperHwInterface: public hardware_interface::RobotHW 
    {
        public:

            UrGripperHwInterface(ros::NodeHandle& nh);
            ~UrGripperHwInterface();
            void init();
            void update(const ros::TimerEvent& e);
            void read();
            void write();


        protected:

            ros::NodeHandle nh_;
            ros::Timer non_realtime_loop_;
            ros::Duration control_period_;
            ros::Duration elapsed_time_;
            PositionJointInterface positionJointInterface;
            VelocityJointInterface velocityJointInterface;
            EffortJointInterface effortJointInterface;
            double loop_hz_;
            boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

            // Interfaces
            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::PositionJointInterface position_joint_interface_;
            hardware_interface::VelocityJointInterface velocity_joint_interface_;
            hardware_interface::EffortJointInterface effort_joint_interface_;

            // Shared memory
            int num_joints_;
            int joint_mode_; // position, velocity, or effort
            std::vector<std::string> joint_names_;
            std::vector<int> joint_types_;
            std::vector<double> joint_position_;
            std::vector<double> joint_velocity_;
            std::vector<double> joint_effort_;
            std::vector<double> joint_position_command_;
            std::vector<double> joint_velocity_command_;
            std::vector<double> joint_effort_command_;

        
        private:
            
            std_msgs::Float32 left_finger_state;
            std_msgs::Float32 left_finger_setpoint;
            
            ros::Subscriber read_hardware;
            ros::Publisher write_hardware;

            bool verbose_;
            std::string gripper_model_;
            double min_pos_;
            double max_pos_;

            double last_joint_position_command_;

            void readGripperCB(const std_msgs::Float32::ConstPtr& msg);

            double read_3fg15();
            void write_3fg15(double target);

            double read_2fg7();
            void write_2fg7(double target);
            
            double read_hand_e();
            void write_hand_e(double target);

            double read_rg2();
            void write_rg2(double target);

            double read_rg6();
            void write_rg6(double target);

            double read_vgc10();
            void write_vgc10(double target);

            double read_2f_85();
            void write_2f_85(double target);

            double read_2f_140();
            void write_2f_140(double target);

            double map(double value, double from1, double to1, double from2, double to2);
            bool command_inside_limits(double value);

    };

}

#endif

