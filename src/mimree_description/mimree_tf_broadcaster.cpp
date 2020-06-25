#include <math.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#define PI 3.14159265

class MIMReeTfBroadcaster{

    public:
        bool first_pose_cb; 
        std::string name;
        std::string sensor_name;
        const std::vector<double> UAV_SCAN = {0.191, 0, 0.591936, 0., PI / 2., -PI / 2.};
        const std::vector<double> UAV_ALT = {0.151, 0, 0.591936, 0, PI / 2., 0};
        const std::vector<double> ASV_ALT = {5.3, -0.3, 2.08, 0, 0, 0};
        ros::NodeHandle n_;
        geometry_msgs::Point veh_position;
        geometry_msgs::Quaternion veh_orientation;
        ros::Subscriber pose_sub;
        ros::Subscriber laser_sub;
        std::vector<double> sensor_pose;
        tf::TransformBroadcaster br;
        
        MIMReeTfBroadcaster(ros::NodeHandle n, std::string name, std::string type, std::string sensor_name):
            n_(n),
            name(name),
            first_pose_cb(false),
            sensor_name(sensor_name)
        {
            pose_sub = n_.subscribe("/" + name + "/mavros/local_position/pose", 10, &MIMReeTfBroadcaster::poseCB, this);
            laser_sub = n_.subscribe("/" + name + "/" + sensor_name + "/laser_scan", 10, &MIMReeTfBroadcaster::scanCB, this);
            if ((type.compare("mimree_uav") == 0) && (sensor_name.compare("altLidar") == 0))
                sensor_pose = UAV_ALT;
            if ((type.compare("mimree_uav") == 0) && (sensor_name.compare("scanningLidar")) == 0)
                sensor_pose = UAV_SCAN;
            if (type.compare("mimree_asv") == 0)
                sensor_pose = ASV_ALT;
        }

        void poseCB(const geometry_msgs::PoseStamped::ConstPtr& pose_in)
        {
            first_pose_cb = true;
            this->veh_position = pose_in->pose.position;
            this->veh_orientation = pose_in->pose.orientation;
        }

        void scanCB (const sensor_msgs::LaserScan::ConstPtr& scan_in)
        {
            if (first_pose_cb){
                // Get sensor position and orientation
                double roll, pitch, yaw;
                tf::Quaternion quaternion(this->veh_orientation.x, this->veh_orientation.y, this->veh_orientation.z, this->veh_orientation.w);
                tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
                double x = this->veh_position.x + (
                    sensor_pose[0] * cos(yaw) + sensor_pose[1] * sin(yaw)
                );
                double y = this->veh_position.y + ( 
                    sensor_pose[0] * sin(yaw) + sensor_pose[1] * cos(yaw)
                );
                double z = this->veh_position.z + sensor_pose[2];
                tf::Quaternion veh_quat = tf::createQuaternionFromRPY(roll + sensor_pose[3], pitch + sensor_pose[4], yaw + sensor_pose[5]);
                // broadcast sensor tf 
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(x, y, z));
                transform.setRotation(veh_quat);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", this->name + "_" + this->sensor_name));
            }
        }
};

int main(int argc, char** argv)
{
  if (argc < 4){
    ROS_WARN("The node needs two arguments [vehicle_name] [vehicle_type] [sensor_name]");
    return 0;
  }
  std::string name = argv[1];
  std::string type = argv[2];
  std::string sensor_name = argv[3];
  ros::init(argc, argv, name + "_" + sensor_name + "_tf_broadcaster");
  ros::NodeHandle n;
  MIMReeTfBroadcaster lstopc(n, name, type, sensor_name);
  ros::spin();
  return 0;
}
