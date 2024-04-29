#include <sstream>
#include <ur_gripper_hw_interface/ur_gripper_hw_interface.h>


using namespace hardware_interface;


namespace ur_gripper_hw_interface
{

    UrGripperHwInterface::UrGripperHwInterface(ros::NodeHandle& nh) : nh_(nh) {

        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &UrGripperHwInterface::update, this);

        read_hardware = nh_.subscribe("gripper_controller/get_pos", 1, &UrGripperHwInterface::readGripperCB, this);
        write_hardware = nh_.advertise<std_msgs::Float32>("gripper_controller/set_pos", 1000);

    }

    UrGripperHwInterface::~UrGripperHwInterface() {

    }

    void UrGripperHwInterface::init() {

        // Get joint names
        nh_.getParam("hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        last_joint_position_command_ = 0;

        verbose_ = false;
        nh_.getParam("hardware_interface/verbose", verbose_);
        gripper_model_="";
        nh_.getParam("hardware_interface/gripper_model", gripper_model_);
        min_pos_= 0;
        nh_.getParam("hardware_interface/min_pos", min_pos_);
        max_pos_= 0;
        nh_.getParam("hardware_interface/max_pos", max_pos_);


        ROS_INFO("Loading gripper model [%s] with limits from %f to %f", gripper_model_.c_str(), min_pos_, max_pos_);

        if (gripper_model_.empty()){
            
            ROS_ERROR("Parameter 'gripper_model' not defined in hardware interface configuration");
        }

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {


            joint_position_[i] = min_pos_;
            joint_velocity_[i] = 0;
            joint_effort_[i] = 0;

             // Create joint state interface: used for read()
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            joint_position_command_[i] = min_pos_;

            // Create position joint interface: used for write()
            JointHandle jointPositiontHandle(jointStateHandle, &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositiontHandle);


        }

        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);

    }


    void UrGripperHwInterface::update(const ros::TimerEvent& e) {

        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write();
    
    }


    void UrGripperHwInterface::readGripperCB(const std_msgs::Float32::ConstPtr& msg){
        
        left_finger_state.data = msg->data; // ths is used in read() method
    
    }


    void UrGripperHwInterface::read() {
        
        if(gripper_model_ == "3fg15"){
            joint_position_[0] = read_3fg15();
        }
        else if(gripper_model_ == "2fg7"){
            joint_position_[0] = read_2fg7();
        }
        else if(gripper_model_ == "hand_e"){
            joint_position_[0] = read_hand_e();
        }
        else if(gripper_model_ == "rg2"){
            joint_position_[0] = read_rg2();
        }
        else if(gripper_model_ == "rg6"){
            joint_position_[0] = read_rg6();
        }
        else if(gripper_model_ == "vgc10"){
            joint_position_[0] = read_vgc10();
        }
        else if(gripper_model_ == "2f_85"){
            joint_position_[0] = read_2f_85();
        }
        else if(gripper_model_ == "2f_140"){
            joint_position_[0] = read_2f_140();
        }
        else{
            ROS_ERROR_THROTTLE(1, "Gripper with name '%s' is not implemented", gripper_model_.c_str());  
        }

        if (verbose_){
            ROS_INFO_THROTTLE(0.5, "UR receives   %f", left_finger_state.data);
            ROS_INFO_THROTTLE(0.5, "ROS receives  %f", joint_position_[0]);
        }

    }

    void UrGripperHwInterface::write() {

        if (joint_position_command_[0] != last_joint_position_command_){

            last_joint_position_command_ = joint_position_command_[0];

            if (command_inside_limits(joint_position_command_[0]) == true){

                if(gripper_model_ == "3fg15"){
                    write_3fg15(joint_position_command_[0]);
                }
                else if(gripper_model_ == "2fg7"){
                    write_2fg7(joint_position_command_[0]);
                }
                else if(gripper_model_ == "hand_e"){
                    write_hand_e(joint_position_command_[0]);
                }
                else if(gripper_model_ == "rg2"){
                    write_rg2(joint_position_command_[0]);
                }
                else if(gripper_model_ == "rg6"){
                    write_rg6(joint_position_command_[0]);
                }
                else if(gripper_model_ == "vgc10"){
                    write_vgc10(joint_position_command_[0]);
                }
                else if(gripper_model_ == "2f_85"){
                    write_2f_85(joint_position_command_[0]);
                }
                else if(gripper_model_ == "2f_140"){
                    write_2f_140(joint_position_command_[0]);
                }
                else{
                    ROS_ERROR_THROTTLE(1, "Gripper with name '%s' is not implemented", gripper_model_.c_str());  
                }

                if (verbose_){
                    ROS_WARN_THROTTLE(0.5, "ROS sends  %f", joint_position_command_[0]);
                    ROS_WARN_THROTTLE(0.5, "UR sends   %f", left_finger_setpoint.data);
                }
            }
            else{
                ROS_WARN_THROTTLE(0.5, "Desired command is out of my limits: min_pos < desired < max_pos, %f < %f < %f",
                                        min_pos_, joint_position_command_[0], max_pos_);
            }

        }

    }

    // ------------------------ COMMON METHODS ------------------------ //

    double UrGripperHwInterface::map(double value, double from1, double to1, double from2, double to2) {

        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }

    
    bool UrGripperHwInterface::command_inside_limits(double value){

        bool is_inside = false;

        if (value >= min_pos_ && value <= max_pos_){
            is_inside = true;
        }

        return is_inside;
    }

    // ---------------------  ON ROBOT 3FG15 GRIPPER --------------------- //

    void UrGripperHwInterface::write_3fg15(double target){

        // No conversion needed
        left_finger_setpoint.data = target;
        write_hardware.publish(left_finger_setpoint); 
    }   

    double UrGripperHwInterface::read_3fg15(){

        // Conversion from gripper diameter in mm to finger rotation in rad
        double x = left_finger_state.data;
        double in_min = 21.7;
        double in_max = 135;
        double out_min = 0;
        double out_max = 2.86775049;

        double state = 2.86775049 - map(x, in_min, in_max, out_min, out_max);

        return state;
    }   

    // ---------------------  ON ROBOT 2FG7 GRIPPER --------------------- //

    void UrGripperHwInterface::write_2fg7(double target){

        // left_finger_setpoint is written in meters, but left_finger_setpoint works in mm
        left_finger_setpoint.data = target*1000;
        write_hardware.publish(left_finger_setpoint); 
    }   

    double UrGripperHwInterface::read_2fg7(){

        // Conversion from polyscope gripper units to ros gripper units
        double x = left_finger_state.data;
        double in_min = 33;
        double in_max = 85;
        double out_min = 0.0;
        double out_max = 0.019;

        double state = map(x, in_min, in_max, out_min, out_max);

        return state;
    }   


    // ---------------------  ON ROBOT HAND E GRIPPER --------------------- //

    void UrGripperHwInterface::write_hand_e(double target){

        // left_finger_setpoint is written in meters, but left_finger_setpoint works in mm
        left_finger_setpoint.data = target*1000.0;
        // ROS gives the setpoint for the distance between the left finger and the center 
        // of the gripper, but for the driver we need the distance between the two fingers
        left_finger_setpoint.data = left_finger_setpoint.data*2.0;
        write_hardware.publish(left_finger_setpoint); 

    }

    double UrGripperHwInterface::read_hand_e(){

        // left_finger_state is read in mm, but joint_position works in meters
        double state = left_finger_state.data/1000.0; 
        // the driver returns the distance between the two fingers, but we need the distance
        // between one finger and the center of the gripper
        state = state/2.0; 
        
        return state;
    }

    // ---------------------  ON ROBOT RG2 GRIPPER --------------------- //

    void UrGripperHwInterface::write_rg2(double target){

        // No conversion needed
        left_finger_setpoint.data = target*1000;
        write_hardware.publish(left_finger_setpoint); 

    }

    double UrGripperHwInterface::read_rg2(){

        // Conversion from gripper diameter in mm to finger rotation in rad
        double x = left_finger_state.data;
        double in_min = 0.1;
        double in_max = 100;
        double out_min = -0.85;
        double out_max = 0.47;

        double state = map(x, in_min, in_max, out_min, out_max);

        return state;
    }   



    // ---------------------  ON ROBOT RG6 GRIPPER --------------------- //

    void UrGripperHwInterface::write_rg6(double target){

        // No conversion needed
        left_finger_setpoint.data = target*1000;
        write_hardware.publish(left_finger_setpoint); 

    }

    double UrGripperHwInterface::read_rg6(){

        // Conversion from gripper diameter in mm to finger rotation in rad
        double x = left_finger_state.data;
        double in_min = 0.1;
        double in_max = 150;
        double out_min = -0.85;
        double out_max = 0.47;

        double state = map(x, in_min, in_max, out_min, out_max);

        return state;
    }   

    // ---------------------  ON ROBOT VGC10 GRIPPER --------------------- //

    void UrGripperHwInterface::write_vgc10(double target){

        // No conversion needed
        left_finger_setpoint.data = target;
        write_hardware.publish(left_finger_setpoint); 

    }

    double UrGripperHwInterface::read_vgc10(){

        // It is a fixed gripper
        double state = 0;

        return state;
    }   

    // ---------------------  ROBOTIQ 2F_85 GRIPPER --------------------- //

    void UrGripperHwInterface::write_2f_85(double target){

        // From meters to mm
        left_finger_setpoint.data = target*1000;
        write_hardware.publish(left_finger_setpoint);

    }

    double UrGripperHwInterface::read_2f_85(){

        // Conversion from gripper diameter in mm to finger rotation in rad
        double x = left_finger_state.data;
        double in_min = 0;
        double in_max = 85;
        double out_min = 0;
        double out_max = 0.9;

        double state = out_max-(x*(out_max/in_max));

        return state;
    }

    // ---------------------  ROBOTIQ 2F_140 GRIPPER --------------------- //

    void UrGripperHwInterface::write_2f_140(double target){

        // From meters to mm
        left_finger_setpoint.data = target*1000;
        write_hardware.publish(left_finger_setpoint);

    }

    double UrGripperHwInterface::read_2f_140(){

        // Conversion from gripper diameter in mm to finger rotation in rad
        double x = left_finger_state.data;
        double in_min = 0;
        double in_max = 140;
        double out_min = 0;
        double out_max = 0.74;

        double state = out_max-(x*(out_max/in_max));

        return state;
    }

}

