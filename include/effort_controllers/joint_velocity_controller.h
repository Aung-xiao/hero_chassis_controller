
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

namespace effort_controllers
{

    class JointVelocityController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:

        JointVelocityController();
        ~JointVelocityController();

        bool init(hardware_interface::EffortJointInterface *robot, const std::string &joint_name, const control_toolbox::Pid &pid);

        /** \brief The init function is called to initialize the controller from a
         * non-realtime thread with a pointer to the hardware interface, itself,
         * instead of a pointer to a RobotHW.
         *
         * \param robot The specific hardware interface used by this controller.
         *
         * \param n A NodeHandle in the namespace from which the controller
         * should read its configuration, and where it should set up its ROS
         * interface.
         *
         * \returns True if initialization was successful and the controller
         * is ready to be started.
         */
        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        /*!
         * \brief Give set velocity of the joint for next update: revolute (angle) and prismatic (velocity)
         *
         * \param double pos Velocity command to issue
         */
        void setCommand(double cmd);

        /*!
         * \brief Get latest velocity command to the joint: revolute (angle) and prismatic (velocity).
         */
        void getCommand(double & cmd);

        /** \brief This is called from within the realtime thread just before the
         * first call to \ref update
         *
         * \param time The current time
         */
        void starting(const ros::Time& time);

        /*!
         * \brief Issues commands to the joint. Should be called at regular intervals
         */
        void update(const ros::Time& time, const ros::Duration& period);

        /**
         * \brief Get the PID parameters
         */
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

        /**
         * \brief Get the PID parameters
         */
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

        /**
         * \brief Print debug info to console
         */
        void printDebug();

        /**
         * \brief Set the PID parameters
         */
        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

        /**
         * \brief Get the name of the joint this controller uses
         */
        std::string getJointName();

        hardware_interface::JointHandle joint_;
        double command_;                                /**< Last commanded velocity. */
        hardware_interface::JointHandle front_left_joint_,back_left_joint_,front_right_joint_,back_right_joint_;
    private:
        int loop_count_;
        control_toolbox::Pid pid_controller_;           /**< Internal PID controller. */

        std::unique_ptr<
                realtime_tools::RealtimePublisher<
                        control_msgs::JointControllerState> > controller_state_publisher_ ;

        ros::Subscriber sub_command_;
        /**
         * \brief Callback from /command subscriber for setpoint
         */
        void setCommandCB(const std_msgs::Float64ConstPtr& msg);

    };

} // namespace

