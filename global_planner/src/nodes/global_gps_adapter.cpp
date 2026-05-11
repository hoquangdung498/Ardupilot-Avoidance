#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/HomePosition.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>

class GlobalGpsAdapter {
public:
  GlobalGpsAdapter(ros::NodeHandle& nh) : nh_(nh), home_received_(false), mission_received_(false), current_wp_index_(1), has_target_(false) {
    
    // Lắng nghe Home Position để khởi tạo hệ tọa độ gốc
    home_sub_ = nh_.subscribe("mavros/home_position/home", 1, &GlobalGpsAdapter::homeCallback, this);
    
    // Lắng nghe Waypoints từ Ardupilot (Mission Planning)
    wp_list_sub_ = nh_.subscribe("mavros/mission/waypoints", 1, &GlobalGpsAdapter::missionCallback, this);
    
    // Lắng nghe vị trí hiện tại để tự động chuyển Waypoint
    local_pose_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &GlobalGpsAdapter::localPoseCallback, this);

    // Mở rộng thêm: Nhận trực tiếp một tọa độ GPS từ bất kỳ nguồn nào
    gps_goal_sub_ = nh_.subscribe("global_planner/gps_goal", 1, &GlobalGpsAdapter::gpsGoalCallback, this);

    // Gửi thẳng mục tiêu trung gian (intermediate_goal) sang Local Planner
    // nếu không muốn dùng Octomap và Global Planner
    move_base_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("intermediate_goal", 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber wp_list_sub_;
  ros::Subscriber home_sub_;
  ros::Subscriber gps_goal_sub_;
  ros::Subscriber local_pose_sub_;
  ros::Publisher move_base_pub_;

  GeographicLib::LocalCartesian geo_converter_;
  bool home_received_;
  double home_alt_;

  mavros_msgs::WaypointList mission_;
  bool mission_received_;
  int current_wp_index_;
  
  bool has_target_;
  double target_x_, target_y_, target_z_;
  ros::Time last_publish_time_;

  void homeCallback(const mavros_msgs::HomePosition& msg) {
    geo_converter_.Reset(msg.geo.latitude, msg.geo.longitude, msg.geo.altitude);
    home_alt_ = msg.geo.altitude;
    home_received_ = true;
    ROS_INFO_ONCE("GlobalGpsAdapter: Received Home Position, Local ENU coordinate system has been set!");
  }

  void publishEnuGoal(double lat, double lon, double alt) {
    if (!home_received_) {
        ROS_WARN_THROTTLE(2.0, "Waiting for Home Position to convert coordinates...");
        return;
    }

    double x_enu, y_enu, z_enu;
    geo_converter_.Forward(lat, lon, alt, x_enu, y_enu, z_enu);

    target_x_ = x_enu;
    target_y_ = y_enu;
    target_z_ = z_enu;
    has_target_ = true;

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
    last_publish_time_ = ros::Time::now();
    ROS_INFO("Waypoint %d ->  ENU Goal: X: %.2f, Y: %.2f, Z: %.2f", current_wp_index_, x_enu, y_enu, z_enu);
  }

  void updateMissionTarget() {
    if (!mission_received_ || !home_received_) return;
    if (current_wp_index_ >= mission_.waypoints.size()) {
        ROS_INFO_THROTTLE(5.0, "Finished all waypoints!");
        return;
    }

    auto wp = mission_.waypoints[current_wp_index_];
    
    // Nếu là lệnh RTL (Return To Launch, command = 20)
    if (wp.command == 20) {
        double rtl_alt;
        nh_.param("rtl_altitude", rtl_alt, 10.0);
        ROS_INFO("Waypoint is RTL. Returning to home at %.1fm.", rtl_alt);
        
        // Trả về đúng vị trí Home thay vì local_origin để tránh lệch tọa độ với ArduPilot
        target_x_ = 0.0; target_y_ = 0.0; target_z_ = rtl_alt;
        has_target_ = true;
        
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "local_origin";
        goal_msg.pose.position.x = 0.0;
        goal_msg.pose.position.y = 0.0;
        goal_msg.pose.position.z = rtl_alt;
        goal_msg.pose.orientation.w = 1.0;
        move_base_pub_.publish(goal_msg);
        last_publish_time_ = ros::Time::now();
        return;
    }

    if (wp.frame == mavros_msgs::Waypoint::FRAME_GLOBAL) {
        publishEnuGoal(wp.x_lat, wp.y_long, wp.z_alt);
    } else if (wp.frame == mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT) {
        publishEnuGoal(wp.x_lat, wp.y_long, wp.z_alt + home_alt_);
    } else {
        ROS_WARN("Current waypoint is not in GPS format. Skipping.");
        current_wp_index_++;
        updateMissionTarget();
    }
  }

  void localPoseCallback(const geometry_msgs::PoseStamped& msg) {
    if (!has_target_ || !mission_received_) return;

    double dx = msg.pose.position.x - target_x_;
    double dy = msg.pose.position.y - target_y_;
    double dz = msg.pose.position.z - target_z_;
    double dist_2d = std::sqrt(dx*dx + dy*dy);

    // Lấy thông số cấu hình bán kính chấp nhận Waypoint từ tham số ROS (nếu có), mặc định 2.5m
    double wp_radius;
    nh_.param("waypoint_acceptance_radius", wp_radius, 2.5);

    // Kiểm tra khoảng cách 2D và độ cao Z riêng biệt, sử dụng cấu hình bán kính
    if (dist_2d < wp_radius && std::abs(dz) < 1.5) {
      ROS_INFO("Arrived at waypoint %d (2D error %.2fm, alt error %.2fm). Moving to next waypoint!", current_wp_index_, dist_2d, std::abs(dz));
      current_wp_index_++;
      updateMissionTarget();
    } else {
      // Gửi lại tọa độ đích với tần số 1Hz để tránh flood mạng
      if (current_wp_index_ < mission_.waypoints.size()) {
          ros::Time now = ros::Time::now();
          if ((now - last_publish_time_).toSec() > 1.0) {
              geometry_msgs::PoseStamped goal_msg;
              goal_msg.header.stamp = now;
              goal_msg.header.frame_id = "local_origin";
              goal_msg.pose.position.x = target_x_;
              goal_msg.pose.position.y = target_y_;
              goal_msg.pose.position.z = target_z_;
              goal_msg.pose.orientation.w = 1.0;
              move_base_pub_.publish(goal_msg);
              last_publish_time_ = now;
          }
      }
    }
  }

  void gpsGoalCallback(const sensor_msgs::NavSatFix& msg) {
    publishEnuGoal(msg.latitude, msg.longitude, msg.altitude);
  }

  void missionCallback(const mavros_msgs::WaypointList& msg) {
    if (msg.waypoints.size() < 2) return;
    
    bool mission_changed = false;
    if (!mission_received_) {
        mission_changed = true;
    } else if (mission_.waypoints.size() != msg.waypoints.size()) {
        mission_changed = true;
    } else {
        // Kiểm tra xem có bất kỳ waypoint nào bị thay đổi tọa độ hoặc lệnh không
        for (size_t i = 0; i < msg.waypoints.size(); ++i) {
            if (mission_.waypoints[i].command != msg.waypoints[i].command ||
                std::abs(mission_.waypoints[i].x_lat - msg.waypoints[i].x_lat) > 0.000001 ||
                std::abs(mission_.waypoints[i].y_long - msg.waypoints[i].y_long) > 0.000001 ||
                std::abs(mission_.waypoints[i].z_alt - msg.waypoints[i].z_alt) > 0.1) {
                mission_changed = true;
                break;
            }
        }
    }

    if (mission_changed) {
        mission_ = msg;
        mission_received_ = true;
        current_wp_index_ = 1;
        ROS_INFO("Had received new mission with %lu waypoints", msg.waypoints.size());
        updateMissionTarget();
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
