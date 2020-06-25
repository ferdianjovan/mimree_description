// FILE: "Wind Turbine Detector Based on Edge Detection" 
// AUTHOR: Ferdian Jovan
// SUMMARY: This program receives the LaserScan msgs and executes an edge detection algorithm
// > to search for a wind turbine or blade(s) of a wind turbine. 
// At the end publishes an estimated wind turbine position 
// > and its position relative to the sensor.
//
// NOTES: This wind turbine detector is inspired by the work described in:
// Bellotto, N. & Hu, H. Multisensor-Based Human Detection and Tracking for Mobile Service Robots 
// IEEE Trans. on Systems, Man, and Cybernetics -- Part B, 2009, 39, 167-181

// Third Party Packages
#include <list>
#include <cmath>
#include <vector>
#include <string>

// ROS Packages
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#define PI 3.14159265
#define FILTER_SIZE 3
#define FLANK_THRESHOLD 0.3

#define FLANK_U 1
#define FLANK_D -1

//Antropometric parameters
#define ANTRO_tower_length0 60.0
#define ANTRO_tower_length1 160.0
#define ANTRO_blade_thick0 3.0
#define ANTRO_blade_thick1 12.0

// Pattern Type
#define TYPE_TA 1 // Tower seen from nacelle level
#define TYPE_TB 2 // Tower seen from below nacelle
#define TYPE_SB 3 // Blade alone

using namespace std;

bool sensor_on = false;
vector < double > rec_x;
vector < double > rec_y;
sensor_msgs::LaserScan SensorMsg;
geometry_msgs::PoseStamped veh_pose;

double Dist2D( double x0, double y0, double x1, double y1 );
void PoseCB( const geometry_msgs::PoseStamped::ConstPtr& pose_in );
void LaserCallback ( const sensor_msgs::LaserScan::ConstPtr& msg );
void FindPattern( string str, string pattern, list <int> *element_found );
void LaserFilter_Mean( vector <double> *vector_r, unsigned size, double range_max );
void TowerPose( vector <double> *r_x, vector <double> *r_y, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y );
void ValidatePattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y, vector <double> laser_r, double range_max );


int main( int argc, char **argv ){

  if ( argc < 3 ){
    cout << "The node needs two arguments [name_of_the_vehicle] [scan_sensor_name]" << endl;
    return 0;
  }

  string name = argv[ 1 ];
  string sensor_name = argv[ 2 ];
  ros::init( argc, argv, name + "_" + sensor_name + "_wt_detector" );
  ros::NodeHandle n;
  ros::Publisher node_pub = n.advertise< geometry_msgs::PoseArray >( "edge_wt_detector", 2 ); // Humans in the environment

  // Subscribers
  ros::Subscriber pose_sub = n.subscribe( "/" + name + "/mavros/local_position/pose", 10, PoseCB );
  ros::Subscriber node_sub = n.subscribe( "/" + name + "/" + sensor_name + "/laser_scan", 2, LaserCallback );
  geometry_msgs::PoseArray msgx;
  tf::TransformListener listener(ros::Duration(4.0), true);
  ros::Rate loop_rate( 15 );

  int seq_counter = 0;
  
  while( ros::ok() ){
    if( sensor_on ){
      // Copying to proper PoseArray data structure
      vector < geometry_msgs::Pose > wt_poses;
      for( int K = 0; K < rec_x.size(); K++ ){
        // transform point from laser frame to map frame
	    geometry_msgs::PointStamped point_origin;
        geometry_msgs::PointStamped point_target;
        point_origin.header.seq = seq_counter;
        point_origin.header.stamp = ros::Time::now() - ros::Duration(0.5);
        point_origin.header.frame_id = SensorMsg.header.frame_id;
        point_origin.point.x = rec_x[ K ];
        point_origin.point.y = rec_y[ K ];
        point_origin.point.z = 0.0;
        try{
            listener.transformPoint("map", point_origin, point_target); 
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
            loop_rate.sleep();
        } 
        // Get veh orientation and infer wt orientation
        double roll, pitch, yaw;
        tf::Quaternion quaternion( veh_pose.pose.orientation.x, veh_pose.pose.orientation.y, veh_pose.pose.orientation.z, veh_pose.pose.orientation.w );
        tf::Matrix3x3( quaternion ).getRPY( roll, pitch, yaw );
        geometry_msgs::Quaternion wt_orien = tf::createQuaternionMsgFromRollPitchYaw( roll, pitch, yaw + PI );
        // Fill in the pose array
        geometry_msgs::Pose wt_pose;
        wt_pose.position = point_target.point;
        wt_pose.orientation = wt_orien;
	    wt_poses.push_back( wt_pose );
      }

      // Header config
      msgx.header.stamp = ros::Time::now();
      msgx.header.frame_id = "map";
      msgx.header.seq = seq_counter;
      msgx.poses = wt_poses;
      // WT poses publish
      node_pub.publish( msgx );
    }

    ros::spinOnce();
    loop_rate.sleep();
    seq_counter++;
  }

  return 0;
}


void PoseCB(const geometry_msgs::PoseStamped::ConstPtr& pose_in)
{
  sensor_on = true;
  veh_pose = *pose_in;
}


void LaserCallback ( const sensor_msgs::LaserScan::ConstPtr& msg ){

  // To get header data from sensor msg
  SensorMsg = *msg;

  // Vectors...
  rec_x.clear(); 
  rec_y.clear(); 
  
  double px, py, pr, pt;
  vector < double >  laser_x;
  vector < double >  laser_y;
  vector < double >  laser_r;
  vector < double >  laser_t;
  double range_max = msg->range_max, range_min = msg->range_min;
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    pr = msg->ranges[ i ];
    if ( pr < range_min )
        pr = range_min;
    if ( pr > range_max )
        pr = range_max;
    pt = msg->angle_min + ( i * msg->angle_increment );
    laser_r.push_back( pr );
    laser_t.push_back( pt );
  }
  
  // Filtering laser scan
  LaserFilter_Mean( &laser_r, FILTER_SIZE, range_max );
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    px = laser_r[ i ] * cos( laser_t[ i ] );
    py = laser_r[ i ] * sin( laser_t[ i ] );
    laser_x.push_back( px );
    laser_y.push_back( py );
  }
	 
  string str_aux = "";
  // Finding flanks in the laser scan...
  vector < int > laser_flank;
  laser_flank.assign( laser_r.size(), 0 );
  for( unsigned i = 1; i < laser_flank.size(); i++ ){
    if( fabs( laser_r[ i ] - laser_r[ i - 1 ] ) > FLANK_THRESHOLD )
      laser_flank[ i ] = ( ( laser_r[ i ] - laser_r[ i - 1 ] ) > 0 ) ? FLANK_U : FLANK_D;
  }
  
  vector < int > flank_id0;
  vector < int > flank_id1;
  string flank_string = "";
  int past_value = 0;
  int idx = 0;
  for( unsigned i = 1; i < laser_flank.size(); i++ ){
    if( laser_flank[ i ] != 0 ){
      if( past_value != laser_flank[ i ] ){
        flank_id0.push_back( i - 1 );
        flank_id1.push_back( i );
        flank_string += ( laser_flank[ i ] > 0 ) ? "S" : "B";
        idx++;
      }
      else
      	flank_id1[ idx - 1 ] =  i;    
    }
    past_value = laser_flank[ i ];
  }  

  // PATTERN RECOGNITION
  string TA  = "BS";
  string TB1  = "BBS";
  string TB2  = "BSB";
  string SB = "BS";
  
  list < int > Pattern_TA;
  list < int > Pattern_TB1;
  list < int > Pattern_TB2;
  list < int > Pattern_SB;
 
  FindPattern( flank_string, TA, &Pattern_TA );
  FindPattern( flank_string, TB1, &Pattern_TB1 );
  FindPattern( flank_string, TB2, &Pattern_TB2 );
  FindPattern( flank_string, SB, &Pattern_SB );

  // ANTROPOMETRIC VALIDATION (the non antropometric patterns are erased from the list)
  ValidatePattern( &Pattern_TA, TYPE_TA, flank_id0, flank_id1, laser_x, laser_y, laser_r, range_max );
  ValidatePattern( &Pattern_TB1, TYPE_TB, flank_id0, flank_id1, laser_x, laser_y, laser_r, range_max );
  ValidatePattern( &Pattern_TB2, TYPE_TB, flank_id0, flank_id1, laser_x, laser_y, laser_r, range_max );
  ValidatePattern( &Pattern_SB, TYPE_SB, flank_id0, flank_id1, laser_x, laser_y, laser_r, range_max );

  // ERASE REDUNDANT PATTERNS FROM ACCEPTED ONES (TB > TA > DB > SB)
  // Clear SB if TB or TA is detected
  if (Pattern_TB1.size() > 0 || Pattern_TB2.size() > 0 || Pattern_TA.size() > 0)
      Pattern_SB.clear();
  // Clear TA if TB "BSB" is detected
  if (Pattern_TB2.size() > 0)
      Pattern_TA.clear();

  //CENTROID PATTERN COMPUTATION & UNCERTAINTY
  rec_x.clear();
  rec_y.clear();
  
  TowerPose( &rec_x, &rec_y, Pattern_TA, TYPE_TA, flank_id0, flank_id1, laser_x, laser_y);
  TowerPose( &rec_x, &rec_y, Pattern_TB1, TYPE_TB, flank_id0, flank_id1, laser_x, laser_y);
  TowerPose( &rec_x, &rec_y, Pattern_TB2, TYPE_TB, flank_id0, flank_id1, laser_x, laser_y);
  TowerPose( &rec_x, &rec_y, Pattern_SB, TYPE_SB, flank_id0, flank_id1, laser_x, laser_y);
}

// Mean value of the 'size' adjacent values
void LaserFilter_Mean( vector < double > *vector_r, unsigned size, double range_max ){
  for( unsigned i = 0; i < ( ( *vector_r ).size() - size ); i++ ){
      double mean = 0;
      int denominator = 0;
      for( unsigned k = 0; k < size; k++  ){
        if ( ( *vector_r )[ i + k ] < range_max ){
	        mean += (*vector_r)[ i + k ];
            denominator++;
        }
      }
      if ( denominator > 0 )
        (*vector_r)[ i ] = mean / denominator;
      else
        (*vector_r)[ i ] = range_max;
  }
}

// Reports a found string pattern in a list
void FindPattern( string str, string pattern, list <int> *element_found ){
  size_t found = 0;

  while( string::npos != ( found = str.find( pattern, found ) ) ){
    (*element_found).push_back( found ); 
    found++;
  }
  
} 

// Performs the antropometric validation of the leg patterns
void ValidatePattern( list < int > *Pattern_list, int TYPE,  vector < int > flank_id0,  vector < int > flank_id1, vector < double > laser_x, vector < double > laser_y, vector < double > laser_r, double range_max){
  
  double ANTRO_height; // Antropometric values from patterns to compare with constants.
  bool SavePattern = true;
  bool cond_no_hole = true, cond_height = true;
  
  for( list<int>::iterator it = (*Pattern_list).begin(); it != (*Pattern_list).end(); it++ ){
    // Obtain antropometric values
    switch( TYPE ){
      case TYPE_TA: // BS
        ANTRO_height = Dist2D( laser_x[ flank_id0[ *it ] ], laser_y[ flank_id0[ *it ] ], laser_x[ flank_id1[ *it + 1 ] - 1 ], laser_y[ flank_id1[ *it + 1 ] - 1 ] );
        cond_no_hole = true;
        for(int i = flank_id0[ *it ] + 1; i < flank_id1[ *it + 1 ]; i++)
            if (laser_r[ i ] >= range_max){
                cond_no_hole = false;
                break;
            }
        cond_height = ( ( ANTRO_height >= ANTRO_tower_length0 ) && ( ANTRO_height <= ANTRO_tower_length1 ) );
        break;
      case TYPE_TB: // BBS or BSB
        ANTRO_height = Dist2D( laser_x[ flank_id0[ *it ] ], laser_y[ flank_id0[ *it ] ], laser_x[ flank_id1[ *it + 2 ] - 1 ], laser_y[ flank_id1[ *it + 2 ] - 1 ] );
        cond_no_hole = true;
        for(int i = flank_id0[ *it ] + 1; i < flank_id1[ *it + 2 ]; i++)
            if (laser_r[ i ] >= range_max){
                cond_no_hole = false;
                break;
            }
        cond_height = ( ( ANTRO_height >= ANTRO_tower_length0 ) && ( ANTRO_height <= ANTRO_tower_length1 ) );
        break;
      case TYPE_SB: // BS
        ANTRO_height = Dist2D( laser_x[ flank_id0[ *it ] ], laser_y[ flank_id0[ *it ] ], laser_x[ flank_id1[ *it + 1 ] - 1 ], laser_y[ flank_id1[ *it + 1 ] - 1 ] );
        cond_no_hole = true;
        for(int i = flank_id0[ *it ] + 1; i < flank_id1[ *it + 1 ]; i++)
            if (laser_r[ i ] >= range_max){
                cond_no_hole = false;
                break;
            }
        cond_height = ( ( ANTRO_height >= ANTRO_blade_thick0 ) && ( ANTRO_height <= ANTRO_blade_thick1 ) );
        break;
    }
    // Save the pattern if it matches
    SavePattern = cond_no_hole && cond_height;
    if( !SavePattern ){
      it = ( *Pattern_list ).erase( it );
      it--;
    }
  }  
}

// Euclidean distance between two coordinate points
double Dist2D( double x0, double y0, double x1, double y1 ){
  return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}

void TowerPose( vector < double > *r_x, vector < double > *r_y, list < int > Pattern_list, int TYPE,  vector < int > flank_id0,  vector < int > flank_id1, vector < double > laser_x, vector < double > laser_y ){
  
  double c_x, c_y;
  list< int >::iterator it;

  for( it = Pattern_list.begin(); it != Pattern_list.end(); it++ ){
    switch( TYPE ){
      case TYPE_SB:
      case TYPE_TA:
        c_x = laser_x[ flank_id1[ *it + 1 ] - 1 ];
        c_y = laser_y[ flank_id1[ *it + 1 ] - 1 ];
        break;
      case TYPE_TB:
        c_x = laser_x[ flank_id1[ *it + 2 ] - 1 ];
        c_y = laser_y[ flank_id1[ *it + 2 ] - 1 ];
        break;
    }
   
    ( *r_x ).push_back( c_x );
    ( *r_y ).push_back( c_y );
  }
}
