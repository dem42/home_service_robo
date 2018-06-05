#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <cmath>

const double THRESHOLD_POS = 1.0;
const double THRESHOLD_ORIENT = 0.05;

enum STAGE {
  PICKING_UP, DROPPING_OFF, DONE
};

// variables
ros::Publisher marker_pub;
STAGE current_stage;
geometry_msgs::Pose target;
geometry_msgs::Pose pickup_target;
geometry_msgs::Pose dropoff_target;

double euclidDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

double quaternionInnerProduct(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
  return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
}

double quaternionDistance(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
  double qInner = quaternionInnerProduct(q1, q2);
  return 1.0 - qInner;
}

bool closeEnough(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  double ed = euclidDistance(p1.position, p2.position);
  double qd = quaternionDistance(p1.orientation, p2.orientation);
  //ROS_INFO("[ADD_MARKERS] Is close enough? pose dist: %f, orient dist: %f.", ed, qd);
  return ed < THRESHOLD_POS && qd < THRESHOLD_ORIENT;
}

void sendMarkerAt(ros::Publisher& marker_pub, bool hide, geometry_msgs::Pose& targetPose) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = hide ? visualization_msgs::Marker::DELETE : visualization_msgs::Marker::ADD;

    // set pose
    marker.pose = targetPose;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("[ADD_MARKER] Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
  geometry_msgs::Pose robotPose = odom_msg->pose.pose;

  if (closeEnough(robotPose, target)) {

    double ed = euclidDistance(robotPose.position, target.position);
    double qd = quaternionDistance(robotPose.orientation, target.orientation);
        
    if (current_stage == PICKING_UP) {
      ROS_INFO("[ADD_MARKERS] Reached pickup. Hiding marker");
      ROS_INFO("[ADD_MARKERS] Pose dist: %f, orient dist: %f.", ed, qd);
      sendMarkerAt(marker_pub, true, pickup_target);
      current_stage = DROPPING_OFF;
      target = dropoff_target;
    }
    else if (current_stage == DROPPING_OFF) {
      ROS_INFO("[ADD_MARKERS] Reached dropoff.");
      ROS_INFO("[ADD_MARKERS] Pose dist: %f, orient dist: %f.", ed, qd);
      sendMarkerAt(marker_pub, false, dropoff_target);
      current_stage = DONE;
    }
  }
}

void initTarget(geometry_msgs::Pose& target, double x, double y) {
  target.position.x = x;
  target.position.y = y;
  target.position.z = 0.0;
  target.orientation.w = 1.0;
}

int main( int argc, char** argv )
{
  current_stage = PICKING_UP;
  initTarget(pickup_target, 3.5, 4.0);
  initTarget(dropoff_target, -3.5, 0.0);
  target = pickup_target;
  
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_subscriber = n.subscribe("/odom", 1000, odom_callback);
  
  sendMarkerAt(marker_pub, false, pickup_target);
  ROS_INFO("[ADD_MARKERS] Markers initialized at pickup zone");

  ros::Duration time_between_ros_wakeups(0.001);
  while (ros::ok()) {
    ros::spinOnce();
    time_between_ros_wakeups.sleep();
  }
  
  return 0;
}
