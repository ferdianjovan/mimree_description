#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseStamped.h"

class LaserScanToPointCloud{

public:
  bool first_pose_cb; 
  std::string name;
  ros::NodeHandle n_;
  ros::Subscriber pose_sub;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n, std::string name, std::string sensor_name) : 
    n_(n),
    name(name),
    first_pose_cb(false),
    laser_sub_(n_, "/" + name + "/" + sensor_name + "/laser_scan", 150),
    laser_notifier_(laser_sub_, listener_, "map", 150)
  {
    pose_sub = n_.subscribe("/" + name + "/mavros/local_position/pose", 20, &LaserScanToPointCloud::poseCB, this);
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(4.00));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/" + name + "/" + sensor_name + "/point_cloud", 1);
  }


  void poseCB(const geometry_msgs::PoseStamped::ConstPtr& pose_in)
  {
    first_pose_cb = true;
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    if (first_pose_cb){
        sensor_msgs::PointCloud cloud;
        try{
            projector_.transformLaserScanToPointCloud(
              "map", *scan_in, cloud, listener_);
        }
        catch (tf::TransformException& e){
            std::cout << e.what();
            return;
        }
        // Do something with cloud.
        scan_pub_.publish(cloud);
    }
  }
};

int main(int argc, char** argv)
{
  if (argc < 3){
    std::cout << "The node needs two arguments [name_of_the_vehicle] [scan_sensor_name]" << std::endl;
    return 0;
  }
  std::string name = argv[1];
  std::string sensor_name = argv[2];
  ros::init(argc, argv, name + "_" + sensor_name + "_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n, name, sensor_name);
  
  ros::spin();
  
  return 0;
}
