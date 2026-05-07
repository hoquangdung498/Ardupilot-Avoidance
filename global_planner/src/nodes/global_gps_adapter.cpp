#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/HomePosition.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/LocalCartesian.hpp>

class GlobalGpsAdapter {
public:
  GlobalGpsAdapter(ros::NodeHandle& nh) : nh_(nh), home_received_(false) {
    
    // Lắng nghe Home Position để khởi tạo hệ tọa độ gốc
    home_sub_ = nh_.subscribe("mavros/home_position/home", 1, &GlobalGpsAdapter::homeCallback, this);
    
    // Lắng nghe Waypoints từ Ardupilot (Mission Planning)
    wp_list_sub_ = nh_.subscribe("mavros/mission/waypoints", 10, &GlobalGpsAdapter::missionCallback, this);
    
    // Mở rộng thêm: Nhận trực tiếp một tọa độ GPS từ bất kỳ nguồn nào (VD: App điện thoại, script)
    gps_goal_sub_ = nh_.subscribe("global_planner/gps_goal", 1, &GlobalGpsAdapter::gpsGoalCallback, this);

    // Gửi tín hiệu đã chuyển đổi ENU tới Global Planner
    move_base_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber wp_list_sub_;
  ros::Subscriber home_sub_;
  ros::Subscriber gps_goal_sub_;
  ros::Publisher move_base_pub_;

  GeographicLib::LocalCartesian geo_converter_;
  bool home_received_;

  void homeCallback(const mavros_msgs::HomePosition& msg) {
    geo_converter_.Reset(msg.geo.latitude, msg.geo.longitude, msg.geo.altitude);
    home_received_ = true;
    ROS_INFO_ONCE("GlobalGpsAdapter: Đã nhận được Home Position, hệ tọa độ Local ENU đã được thiết lập!");
  }

  void publishEnuGoal(double lat, double lon, double alt) {
    if (!home_received_) {
        ROS_WARN_THROTTLE(2.0, "Đang chờ nhận Home Position từ Ardupilot để chuyển đổi tọa độ...");
        return;
    }

    double x_enu, y_enu, z_enu;
    geo_converter_.Forward(lat, lon, alt, x_enu, y_enu, z_enu);

    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "local_origin"; // Frame gốc của Global Planner
    
    goal_msg.pose.position.x = x_enu;
    goal_msg.pose.position.y = y_enu;
    goal_msg.pose.position.z = z_enu;
    
    // Quaternion mặc định hướng về phía trước
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.0;
    goal_msg.pose.orientation.w = 1.0;

    move_base_pub_.publish(goal_msg);
    ROS_INFO("Global Planner Goal (ENU) Update: X: %.2f, Y: %.2f, Z: %.2f", x_enu, y_enu, z_enu);
  }

  void gpsGoalCallback(const sensor_msgs::NavSatFix& msg) {
    publishEnuGoal(msg.latitude, msg.longitude, msg.altitude);
  }

  void missionCallback(const mavros_msgs::WaypointList& msg) {
    for (const auto& wp : msg.waypoints) {
      if (wp.is_current) {
        if (wp.frame == mavros_msgs::Waypoint::FRAME_GLOBAL ||
            wp.frame == mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT) {
            publishEnuGoal(wp.x_lat, wp.y_long, wp.z_alt);
        } else {
            ROS_WARN_THROTTLE(2.0, "Waypoint hiện tại không phải là định dạng GPS (GLOBAL). Bỏ qua.");
        }
        break; // Đã xử lý wp hiện tại xong
      }
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_gps_adapter");
  ros::NodeHandle nh;
  GlobalGpsAdapter adapter(nh);
  ros::spin();
  return 0;
}
