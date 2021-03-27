#include <effort_controllers/joint_velocity_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace effort_controllers {
    JointVelocityController::JointVelocityController()
            :v1(0),v2(0),v3(0), v4(0),  loop_count_(0){}

    JointVelocityController::~JointVelocityController() {
        sub_command_.shutdown();
    }
    bool JointVelocityController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
        control_toolbox::Pid pid;
        pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
        std::string  front_left_wheel_joint,back_left_wheel_joint, front_right_wheel_joint, back_right_wheel_joint;
        if (!n.getParam("joint", front_left_wheel_joint)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }
        if (!n.getParam("joint", back_left_wheel_joint)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }
        if (!n.getParam("joint", front_right_wheel_joint)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }
        if (!n.getParam("joint", back_right_wheel_joint)) {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }

        back_left_joint_ = robot->getHandle(back_left_wheel_joint);
        front_right_joint_ = robot->getHandle(front_right_wheel_joint);
        back_right_joint_ = robot->getHandle(back_right_wheel_joint);
        front_left_joint_=robot->getHandle(front_left_wheel_joint);

        if (!pid.init(ros::NodeHandle(n, "pid")))
            return false;
        controller_state_publisher_.reset(
                new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                        (n, "state", 1));
        sub_command_ = n.subscribe<std_msgs::Float64>("/controller/front_left_velocity_controller/command", 1, &JointVelocityController::setCommandFL, this);
        sub_command_ = n.subscribe<std_msgs::Float64>("/controller/front_right_velocity_controller/command", 1, &JointVelocityController::setCommandFR,this);
        sub_command_ = n.subscribe<std_msgs::Float64>("/controller/back_right_velocity_controller/command", 1, &JointVelocityController::setCommandBR,this);
        sub_command_ = n.subscribe<std_msgs::Float64>("/controller/back_left_velocity_controller/command", 1, &JointVelocityController::setCommandBL,this);
        return true;
    }

    void
    JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup) {
        pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
    }
    void JointVelocityController::starting(const ros::Time &time) {
        v1 = 0.0;
        v2 = 0.0;
        v3 = 0.0;
        v4 = 0.0;
        pid_controller_.reset();
    }
    void JointVelocityController::update(const ros::Time &time, const ros::Duration &period){
        double error_fl = v1 - front_left_joint_.getVelocity();
        double commanded_effort_fl = pid_controller_.computeCommand(error_fl, period);
        double error_fr = v2 - front_right_joint_.getVelocity();
        double commanded_effort_fr = pid_controller_.computeCommand(error_fr, period);
        double error_br = v3 - back_right_joint_.getVelocity();
        double commanded_effort_br = pid_controller_.computeCommand(error_br, period);
        double error_bl = v4 - back_left_joint_.getVelocity();
        double commanded_effort_bl = pid_controller_.computeCommand(error_bl, period);

        //set by pid_controller
        front_left_joint_.setCommand(commanded_effort_fl);
        front_right_joint_.setCommand(commanded_effort_fr);
        back_left_joint_.setCommand(commanded_effort_bl);
        back_right_joint_.setCommand(commanded_effort_br);


        if (loop_count_ % 10 == 0) {
            if (controller_state_publisher_ && controller_state_publisher_->trylock()){
                controller_state_publisher_->msg_.header.stamp = time;
                controller_state_publisher_->msg_.set_point = v1;
                controller_state_publisher_->msg_.process_value = front_left_joint_.getVelocity();
                controller_state_publisher_->msg_.error = error_fl;
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = commanded_effort_fl;

                double dummy;
                bool antiwindup;
                getGains(controller_state_publisher_->msg_.p,
                         controller_state_publisher_->msg_.i,
                         controller_state_publisher_->msg_.d,
                         controller_state_publisher_->msg_.i_clamp,
                         dummy,
                         antiwindup);
                controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
                controller_state_publisher_->unlockAndPublish();
            }
        }
        loop_count_++;

    }
    void JointVelocityController::setCommandFL(const std_msgs::Float64ConstPtr &msg) {
        v1 = msg->data;
    }
    void JointVelocityController::setCommandFR(const std_msgs::Float64ConstPtr &msg){
        v2 = msg->data;
    }
    void JointVelocityController::setCommandBL(const std_msgs::Float64ConstPtr &msg){
        v4 = msg->data;
    }
    void JointVelocityController::setCommandBR(const std_msgs::Float64ConstPtr &msg){
        v3 = msg->data;
    }

}//namespace


PLUGINLIB_EXPORT_CLASS(effort_controllers::JointVelocityController, controller_interface::ControllerBase)