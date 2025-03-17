#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>


namespace gazebo
{
  class RobotPosePublisher : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle* nh;
    ros::Publisher pose_pub;
    ros::Publisher twist_pub;

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
      twist_pub = nh->advertise<geometry_msgs::Twist>("/robot_twist", 100);

      updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RobotPosePublisher::OnUpdate, this));
    }

    void OnUpdate()
    {
      gazebo_msgs::ModelState state_msg;
      geometry_msgs::Pose pose_msg;
      geometry_msgs::Twist twist_msg;

      pose_msg.position.x = model->WorldPose().Pos().X();
      pose_msg.position.y = model->WorldPose().Pos().Y();
      pose_msg.position.z = model->WorldPose().Pos().Z();
      pose_msg.orientation.x = model->WorldPose().Rot().X();
      pose_msg.orientation.y = model->WorldPose().Rot().Y();
      pose_msg.orientation.z = model->WorldPose().Rot().Z();
      pose_msg.orientation.w = model->WorldPose().Rot().W();

      twist_msg.linear.x = model->WorldLinearVel()[0];
      twist_msg.linear.y = model->WorldLinearVel()[1];
      twist_msg.linear.z = model->WorldLinearVel()[2];
      twist_msg.angular.x = model->WorldAngularVel()[0];
      twist_msg.angular.y = model->WorldAngularVel()[1];
      twist_msg.angular.z = model->WorldAngularVel()[2];



      // twist_mgs.linear.x = model->WorldPose().Twist().linear().X()
      // twist_mgs.linear.x = model->WorldPose().Twist().linear().Y()
      // twist_mgs.linear.x = model->WorldPose().Twist().linear().Z()
      // twist_mgs.linear.x = model->WorldPose().Twist().linear().X()
      // twist_mgs.linear.x = model->WorldPose().Twist().linear().X()
      // twist_mgs.linear.x = model->WorldPose().Twist().linear().X()


      pose_pub.publish(pose_msg);
      twist_pub.publish(twist_msg);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(RobotPosePublisher)
}
