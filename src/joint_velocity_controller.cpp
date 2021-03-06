#include <effort_controllers/joint_velocity_controller.h>
#include <pluginlib/class_list_macros.hpp>
namespace effort_controllers {
    JointVelocityController::JointVelocityController()
            : command_(0), loop_count_(0) {}

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
        front_left_joint_ = robot->getHandle(front_left_wheel_joint);
        back_left_joint_ = robot->getHandle(back_left_wheel_joint);
        front_right_joint_ = robot->getHandle(front_right_wheel_joint);
        back_right_joint_ = robot->getHandle(back_right_wheel_joint);
        if (!pid.init(ros::NodeHandle(n, "pid")))
            return false;
        controller_state_publisher_.reset(
                new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
                        (n, "state", 1));
        sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointVelocityController::setCommandCB, this);
        return true;
    }

    void
    JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup) {
        pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
    }
    void JointVelocityController::starting(const ros::Time &time) {
        command_ = 0.0;
        pid_controller_.reset();
    }
    void JointVelocityController::update(const ros::Time &time, const ros::Duration &period){
        double error = command_ - joint_.getVelocity();
        double commanded_effort = pid_controller_.computeCommand(error, period);
        joint_.setCommand(commanded_effort);
        if (loop_count_ % 10 == 0) {
            if (controller_state_publisher_ && controller_state_publisher_->trylock()){
                controller_state_publisher_->msg_.header.stamp = time;
                controller_state_publisher_->msg_.set_point = command_;
                controller_state_publisher_->msg_.process_value = joint_.getVelocity();
                controller_state_publisher_->msg_.error = error;
                controller_state_publisher_->msg_.time_step = period.toSec();
                controller_state_publisher_->msg_.command = commanded_effort;
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
    void JointVelocityController::setCommandCB(const std_msgs::Float64ConstPtr &msg) {
        command_ = msg->data;
    }

}//namespace
PLUGINLIB_EXPORT_CLASS(effort_controllers::JointVelocityController, controller_interface::ControllerBase)
