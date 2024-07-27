#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <string>
#include <common/Quadruped.h>

class IOROS : public IOInterface
{
    public:
        IOROS(Quadruped quad);
        ~IOROS();
        void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

    private:
        void sendCmd(const LowlevelCmd *cmd);
        void recvState(LowlevelState *state);
        ros::NodeHandle _nm;
        ros::Subscriber _servo_sub[12], _imu_sub, _state_sub, _foot_force_sub[4], _manipulation_force_sub, _object_sub[2];
        ros::Publisher _servo_pub[12], _pose_pub;
        unitree_legged_msgs::LowCmd _lowCmd;
        unitree_legged_msgs::LowState _lowState;
        std::string _robot_name;
        float _camera_link_length;
        HighlevelCmd Highcmd;
        Quadruped _quad;
        
        ros::Time _currentTime;
        std::string tf_prefix;
        geometry_msgs::TransformStamped _trunkTF;
        tf::TransformBroadcaster _trunkTF_broadcaster;
        Eigen::Matrix3d rotmat;
        Eigen::Vector3d cmd_body;
        double omega_z;

        geometry_msgs::PoseWithCovarianceStamped _poseMsg;

        void initRecv(); // initialize subscribers
        void initSend(); // initialize publishers
    
        void imuCallback(const sensor_msgs::Imu & msg);
        void StateCallback(const gazebo_msgs::ModelStates & msg);
        // add cheater callback from gazebo later

        void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
        void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
        void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
        void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
        void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
        void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
        void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
        void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
        void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);

        void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
        void RLfootCallback(const geometry_msgs::WrenchStamped& msg);

        void ManiForceCallback(const geometry_msgs::Wrench& msg);
        void cmdvelCallback(const geometry_msgs::Twist& msg);
        void poseCallback(const geometry_msgs::Pose& msg);
        void timerCallback(const ros::TimerEvent&); 
        bool msg_received =false;
        bool planner_running = false;
        ros::Timer timer;

        boost::array<double, 36> _odom_pose_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0,
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
        boost::array<double, 36> _odom_twist_covariance = {1e-9, 0, 0, 0, 0, 0, 
                                        0, 1e-3, 1e-9, 0, 0, 0, 
                                        0, 0, 1e6, 0, 0, 0, 
                                        0, 0, 0, 1e6, 0, 0, 
                                        0, 0, 0, 0, 1e6, 0, 
                                        0, 0, 0, 0, 0, 1e-9};
};

#endif
