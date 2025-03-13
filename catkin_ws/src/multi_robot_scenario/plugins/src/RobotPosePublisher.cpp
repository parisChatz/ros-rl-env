#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>

namespace gazebo
{
  class RobotPosePublisher : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle* nh;
    ros::Publisher pose_pub;

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      model = _parent;
      nh = new ros::NodeHandle();

    //   if (_sdf->HasElement("update_rate"))
    //     {
    //         double update_rate = _sdf->Get<double>("update_rate");
    //     }
      pose_pub = nh->advertise<geometry_msgs::Pose>("/robot_pose", 100);

      updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RobotPosePublisher::OnUpdate, this));
    }

    void OnUpdate()
    {
      geometry_msgs::Pose pose_msg;
      pose_msg.position.x = model->WorldPose().Pos().X();
      pose_msg.position.y = model->WorldPose().Pos().Y();
      pose_msg.position.z = model->WorldPose().Pos().Z();
      pose_msg.orientation.x = model->WorldPose().Rot().X();
      pose_msg.orientation.y = model->WorldPose().Rot().Y();
      pose_msg.orientation.z = model->WorldPose().Rot().Z();
      pose_msg.orientation.w = model->WorldPose().Rot().W();

      pose_pub.publish(pose_msg);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(RobotPosePublisher)
}
