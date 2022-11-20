#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"   
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <math.h>

#define MAX_L_STEER -50
#define MAX_R_STEER 50
#define STEER_NEUTRAL_ANGLE 50

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 25 // ????? ?? ??? ?
#define WayPoint_X_Tor 0.25
#define WayPoint_Y_Tor 0.25

#define V_Region_NO  2
#define V_Speed_Region_NO 3
#define W_Region_NO  1

double pos_x = 0.0;
double pos_y = 0.0;

int vision_steering_angle = 0;
int waypoint_steering_angle = 0;
int car_speed = 0;

double roll,pitch,yaw;

struct Point 
{ 
   double x; 
   double y; 
   double z;
};

struct WayPoints
{
   double x;
   double y;   
};

struct Rect_Region
{
   double top;
   double bottom;
   double left;
   double right;   
};

geometry_msgs::Pose2D my_pose;

struct Rect_Region Vision_Region[V_Region_NO];
struct Rect_Region Vision_Speed_Region[V_Region_NO];
struct Rect_Region WayPoint_Region[W_Region_NO];
struct WayPoints my_waypoints_list[WayPoints_NO];

void poseCallback(const geometry_msgs::PoseStamped& msg)
{
   my_pose.x = (double)msg.pose.position.x;
   my_pose.y = (double)msg.pose.position.y;
   
   tf2::Quaternion q(
        msg.pose.orientation.x,        msg.pose.orientation.y,
        msg.pose.orientation.z,        msg.pose.orientation.w);
        tf2::Matrix3x3 m(q);     
 
    m.getRPY(roll, pitch, yaw);
    my_pose.theta = yaw;      
}

void init_vision_region(void)
{
   Vision_Region[0].top    =  10.0;
   Vision_Region[0].bottom =  -10.0;
   Vision_Region[0].left   =  10.0; 
   Vision_Region[0].right  =  -10.0;   
   
   Vision_Region[1].top    =  0.0;
   Vision_Region[1].bottom =  0.0;
   Vision_Region[1].left   =  0.0; 
   Vision_Region[1].right  =  0.0;   
}

void init_vision_speed_region(void)
{
   Vision_Speed_Region[0].top    =  0.0;
   Vision_Speed_Region[0].bottom =  0.0;
   Vision_Speed_Region[0].left   =  0.0; 
   Vision_Speed_Region[0].right  =  0.0;
   
    Vision_Speed_Region[1].top    =  0.0;
   Vision_Speed_Region[1].bottom =  0.0;
   Vision_Speed_Region[1].left   =  0.0; 
   Vision_Speed_Region[1].right  =  0.0;      
   
   Vision_Speed_Region[2].top    =  0.0;
   Vision_Speed_Region[2].bottom =  0.0;
   Vision_Speed_Region[2].left   =  0.0; 
   Vision_Speed_Region[2].right  =  0.0;      
}

void init_waypoint_region(void)
{
   
   WayPoint_Region[0].top    =  10.0;
   WayPoint_Region[0].bottom =  -10.0;
   WayPoint_Region[0].left   =  -10.0; 
   WayPoint_Region[0].right  =  10.0;

   /*
   WayPoint_Region[0].top    =  1.0;
   WayPoint_Region[0].bottom =  0.1;
   WayPoint_Region[0].left   =  0.1; 
   WayPoint_Region[0].right  =  0.4;
   
   WayPoint_Region[1].top    =  1.6;
   WayPoint_Region[1].bottom =  1.2;
   WayPoint_Region[1].left   =  0.1; 
   WayPoint_Region[1].right  =  0.4;
   * */
}

void init_waypoint(void)
{
//    my_waypoints_list.x = [0.049, 0.626, 2.128, 3.872, 3.568,
//                      2.072, 0.36, 1.616, 2.92, 3.864,
//                      5.92, 7.58, 6.876, 5.768, 6.6,
//                      7.552, 6.06, 4.392, 4.416, 4.392,
//                      1.824, 1.204, 1.848, 6.308, 6.956]

//    my_waypoints_list.y = [1.389, 2.36, 3.354, 4.319, 4.975,
//                      5.306, 6.286, 7.294, 7.704, 6.844,
//                      7.288, 6.42, 5.5, 4.508, 3.684,
//                      2.704, 1.688, 1.752, 4.924, 1.742,
//                      1.684, 1.08, 0.404, 0.392, 1.18]

	my_waypoints_list[0].x = 1.389;   
    my_waypoints_list[0].y = 0.049;       

    my_waypoints_list[1].x = 2.428;   
    my_waypoints_list[1].y = -0.626;        
    
    my_waypoints_list[2].x = 2.68;   
    my_waypoints_list[2].y = -1.808; 
    
    my_waypoints_list[3].x = 3.453;   
    my_waypoints_list[3].y = -3.132;        
    
    my_waypoints_list[4].x = 5.277;   
    my_waypoints_list[4].y = -3.044;     

    my_waypoints_list[5].x = 5.306;   
    my_waypoints_list[5].y = -1.868;     

    my_waypoints_list[6].x = 5.365;   
    my_waypoints_list[6].y = -0.972;     

    my_waypoints_list[7].x = 7.294;   
    my_waypoints_list[7].y = -1.616;     

    my_waypoints_list[8].x = 6.906;   
    my_waypoints_list[8].y = -2.528;     

    my_waypoints_list[9].x = 6.906;   
    my_waypoints_list[9].y = -4.276;     

    my_waypoints_list[10].x = 7.286;   
    my_waypoints_list[10].y = -6.2742;     

    my_waypoints_list[11].x = 6.42;   
    my_waypoints_list[11].y = -7.58;     

    my_waypoints_list[12].x = 5.5;   
    my_waypoints_list[12].y = -6.876;     

    my_waypoints_list[13].x = 4.508;   
    my_waypoints_list[13].y = -5.768;     

    my_waypoints_list[14].x = 3.684;   
    my_waypoints_list[14].y = -6.6;     

    my_waypoints_list[15].x = 2.704;   
    my_waypoints_list[15].y = -7.552;     

    my_waypoints_list[16].x = 1.688;   
    my_waypoints_list[16].y = -6.06;     

    my_waypoints_list[17].x = 1.752;   
    my_waypoints_list[17].y = -4.392;     

    my_waypoints_list[18].x = 4.924;   
    my_waypoints_list[18].y = -4.416;     

    my_waypoints_list[19].x = 1.742;   
    my_waypoints_list[19].y = -4.392;  

    my_waypoints_list[20].x = 1.684;   
    my_waypoints_list[20].y = -1.824;  

    my_waypoints_list[21].x = 1.08;   
    my_waypoints_list[21].y = -1.204;  

    my_waypoints_list[22].x = 0.404;   
    my_waypoints_list[22].y = -1.848;  

    my_waypoints_list[23].x = 0.392;   
    my_waypoints_list[23].y = -6.308;  

    my_waypoints_list[24].x = 1.18;   
    my_waypoints_list[24].y = -6.956;   

}


int check_car_vision_region(void)
{
   int i, id_region = -1;
         
   for(i=0;i<V_Region_NO;i++)
   {   
      if(Vision_Region[i].left > Vision_Region[i].right)
      { 
      double temp;
      temp = Vision_Region[i].left;
        Vision_Region[i].left  = Vision_Region[i].right;
        Vision_Region[i].right = temp;
       }
       //printf("%d : %6.3lf %6.3lf %6.3lf %6.3lf\n", i, Vision_Region[i].left, Vision_Region[i].right, Vision_Region[i].bottom, Vision_Region[i].top);
       if(   (my_pose.y>=Vision_Region[i].left) && (my_pose.y<=Vision_Region[i].right) 
          && (my_pose.x>=Vision_Region[i].bottom) && (my_pose.x<=Vision_Region[i].top)  )
       {         
         id_region = i;         
         break;
       }
   }   
   return id_region;   
}

int check_car_vision_speed_region(void)
{
   int i, id_region = -1;
         
   for(i=0;i<V_Speed_Region_NO;i++)
   {   
        if(Vision_Speed_Region[i].left > Vision_Speed_Region[i].right)
      { 
      double temp;
      temp = Vision_Speed_Region[i].left;
        Vision_Speed_Region[i].left  = Vision_Speed_Region[i].right;
        Vision_Speed_Region[i].right = temp;
       }
       //printf("%d : %6.3lf %6.3lf %6.3lf %6.3lf\n", i, Vision_Region[i].left, Vision_Region[i].right, Vision_Region[i].bottom, Vision_Region[i].top);
       if(   (my_pose.y>=Vision_Speed_Region[i].left) && (my_pose.y<=Vision_Speed_Region[i].right) 
          && (my_pose.x>=Vision_Speed_Region[i].bottom) && (my_pose.x<=Vision_Speed_Region[i].top)  )
       {         
         id_region = i;         
         break;
       }
   }   
   return id_region;   
}



int check_car_waypoint_region(void)
{
   int i, id_region = -1;
   
   for(i=0;i<W_Region_NO;i++)
   {   
      if(WayPoint_Region[i].left > WayPoint_Region[i].right)
      { 
      double temp;
      temp = WayPoint_Region[i].left;
        WayPoint_Region[i].left  = WayPoint_Region[i].right;
        WayPoint_Region[i].right = temp;
       }
       //printf("%d : %6.3lf %6.3lf %6.3lf %6.3lf\n", i, WayPoint_Region[i].left, WayPoint_Region[i].right,  WayPoint_Region[i].bottom,  WayPoint_Region[i].top);
       if(   (my_pose.y>=WayPoint_Region[i].left) && (my_pose.y<=WayPoint_Region[i].right) 
          && (my_pose.x>=WayPoint_Region[i].bottom) && (my_pose.x<=WayPoint_Region[i].top)  )
       {         
         id_region = i;
         break;
       }
   }   
   return id_region;   
}

void VisionSteerControlCallback(const std_msgs::Int16& angle)
{
  vision_steering_angle = (int)(angle.data) ;
  
  if(vision_steering_angle >= MAX_R_STEER)  vision_steering_angle = MAX_R_STEER;
  if(vision_steering_angle <= MAX_L_STEER)  vision_steering_angle = MAX_L_STEER;  
}

void WaySteerControlCallback(const std_msgs::Int16& angle)
{
  waypoint_steering_angle = (int)(angle.data) ;
 
  if(waypoint_steering_angle >= MAX_R_STEER)  waypoint_steering_angle = MAX_R_STEER;
  if(waypoint_steering_angle <= MAX_L_STEER)  waypoint_steering_angle = MAX_L_STEER;  
}


int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "race_waypoints_navigation");

  ros::NodeHandle n;
 
  ros::Subscriber sub1 = n.subscribe("/Car_Control_cmd/W_SteerAngle_Int16",10, &WaySteerControlCallback);  
  ros::Subscriber sub2 = n.subscribe("/Car_Control_cmd/V_SteerAngle_Int16",10, &VisionSteerControlCallback);  
  ros::Subscriber sub3 = n.subscribe("/slam_out_pose",100, &poseCallback);
   
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_Int16", 10);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 10);
 
  ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("marker/node", 1);
  ros::Publisher target_pos_pub = n.advertise<geometry_msgs::Pose2D>("/pose_goal", 10);
 
  ros::Rate loop_rate(10);  // 10
  
  std_msgs::Int16 s_angle;
  std_msgs::Int16 c_speed;
  

  int count = 0;
  int mission_flag[WayPoints_NO] = {0,};
  double pos_error_x = 0.0;
  double pos_error_y = 0.0; 
  
  geometry_msgs::Pose2D pose_goal;
  
   
  Point p; std::vector<Point> vec_point; 
  
  init_waypoint();
  for(int i=0; i< WayPoints_NO ; i++)
  { 
     p.x = my_waypoints_list[i].x; 
     p.y = my_waypoints_list[i].y; 
     p.z = 0.0; 
     vec_point.push_back(p); 
  }
  
  visualization_msgs::MarkerArray marker_arr; 
  for (size_t i = 0; i < vec_point.size(); i++)
  { 
    Point o_marker = vec_point[i]; 
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "/map"; // map frame ±âÁØ 
    marker.header.stamp = ros::Time::now(); 
    marker.type = visualization_msgs::Marker::SPHERE; 
    marker.id = i; 
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.orientation.w = 1.0; 
    marker.pose.position.x = o_marker.x; // marker x position
    marker.pose.position.y = o_marker.y; // marker y position  
    // Points are green 
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; 
    marker.scale.x = 0.1; 
    marker.scale.y = 0.1; 
    marker_arr.markers.push_back(marker);
  } 
  
  int vision_id = -1;
  int vision_speed_id = -1;
  int waypoint_id = -1;
  int wp_go_id = 0;
  
  init_vision_region();
  init_waypoint_region();
  init_vision_speed_region();

  pose_goal.x = my_waypoints_list[wp_go_id].x;
  pose_goal.y = my_waypoints_list[wp_go_id].y;
  pose_goal.theta = DEG2RAD(0);
  target_pos_pub.publish(pose_goal);
     
  while (ros::ok())
  {   
     
   ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf ", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta) ); 
   vision_id = check_car_vision_region();
    vision_id = -1; 
   if(vision_id != -1)
   {
      ROS_INFO("Vision Region : %2d",vision_speed_id);   
      s_angle.data = vision_steering_angle; 
      
      vision_speed_id = check_car_vision_speed_region();   
       if(vision_speed_id == -1) 
       {
         c_speed.data = 120;    
      }
       else c_speed.data = 180;       
    }
    
    waypoint_id = check_car_waypoint_region();
    waypoint_id = 0;    
    if(waypoint_id!= -1)
    {   
       ROS_INFO("WayPoint Region : %2d",waypoint_id);
       
       pos_error_x = abs(my_pose.x - my_waypoints_list[wp_go_id].x);
       pos_error_y = abs(my_pose.y - my_waypoints_list[wp_go_id].y); 
       pose_goal.x = my_waypoints_list[wp_go_id].x;
       pose_goal.y = my_waypoints_list[wp_go_id].y;
       pose_goal.theta = DEG2RAD(0);
       target_pos_pub.publish(pose_goal);
       ROS_INFO("WayPoint-%d",wp_go_id); 
      // ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf E_x : %6.3lf  E_y : %6.3lf", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta), pos_error_x, pos_error_y);  
       if(( pos_error_x<= WayPoint_X_Tor)&&( pos_error_y <= WayPoint_Y_Tor))
        {  
          ROS_INFO("WayPoint-%d",wp_go_id); 
          wp_go_id++;
        }     
               
       s_angle.data = waypoint_steering_angle;    
       c_speed.data = 65;
       
       if(wp_go_id >= WayPoints_NO) 
       {
          c_speed.data = 0;   
          wp_go_id = WayPoints_NO;   
       }
   }   
   
   // publish topics   
   printf("steering_angle %d %d \n",s_angle.data ,c_speed.data);
   car_control_pub1.publish(s_angle);
   car_control_pub2.publish(c_speed);
   markers_pub.publish(marker_arr);
    
   loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

/*
      pos_error_x = abs(my_pose.x - my_waypoints_list[0].x);
       pos_error_y = abs(my_pose.y - my_waypoints_list[0].y); 
       pose_goal.x = my_waypoints_list[0].x;
       pose_goal.y = my_waypoints_list[0].y;
       pose_goal.theta = DEG2RAD(0);
       target_pos_pub.publish(pose_goal);
    
       ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf E_x : %6.3lf  E_y : %6.3lf", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta), pos_error_x, pos_error_y);  
       if(( pos_error_x<= WayPoint_X_Tor)&&( pos_error_y <= WayPoint_Y_Tor))
        {  
          ROS_INFO("WayPoint-1"); 
          mission_flag[0] = 1; 
       } 
      */
