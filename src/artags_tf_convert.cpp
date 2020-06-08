#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class ARtags_tf_convert {
public:
    ARtags_tf_convert ( ros::NodeHandle& nodehandle )
            : nh_( nodehandle )
    {
        camera_action_sub_ = nh_.subscribe( "artags_action", 10, &ARtags_tf_convert::CameraActionCB, this );
        accuracy_pub_ = nh_.advertise< geometry_msgs::Point > ( "artags_positioning", 10 );
        cmd_vel_pub_ = nh_.advertise< geometry_msgs::Twist > ( "cmd_vel", 10 );
    }
private:

    void CameraTFCB(const ar_track_alvar_msgs::AlvarMarkers& msg )
    {
        if(msg.markers.size() > 0)
        {
            std::cout << "loop start" << std::endl;
            geometry_msgs::Twist Vel;
            geometry_msgs::Point Position;

            float x = msg.markers[0].pose.pose.position.x;
            float y = msg.markers[0].pose.pose.position.y;
            float yaw = fabs( tf2::getYaw(msg.markers[0].pose.pose.orientation)*180/3.14 );
            float x_diff = 0.8;


            float max_vtheta = 0.3;
            float max_vtranslation = 0.15;

            if( yaw > 91 || yaw < 89 )
            {
                Vel.angular.z = -( yaw/fabs( yaw ) ) * max_vtheta;
            }
            else
            {
                if( fabs( x - x_diff ) > 0.01 )
                {
                    Vel.linear.x = ( x - x_diff ) / fabs( x - x_diff ) * max_vtranslation;
                }
                if( fabs( y ) > 0.01 )
                {
                    Vel.linear.y = -( y / fabs( y ) ) * max_vtranslation;
                }
            }

            Position.x = x - x_diff;
            Position.y = y;
            Position.z = yaw;
            std::cout << "x= " << x - x_diff << "  |  "
                      << "y= " << y << "  |  "
                      << "yaw= " << yaw << std::endl;
            std::cout << "Vel.x= " << Vel.linear.x << "  |  "
                      << "Vel.y= " << Vel.linear.y << "  |  "
                      << "Vel.th= " << Vel.angular.z << std::endl;
            std::cout << "----------------------------------------" << std::endl;

            accuracy_pub_.publish( Position );
            cmd_vel_pub_.publish( Vel );
        }
        else
        {
          std::cout << "There isn't any AR_tags " << std::endl;
        }

    }

    void CameraActionCB( const std_msgs::String& str )
    {
        if ( !( str.data == "start" ) )
        {
            std::cout << "CameraTF_convert shutdown" << std::endl;
            camera_action_sub_.shutdown();
        }

        camera_action_sub_ = nh_.subscribe( "ar_pose_marker", 10, &ARtags_tf_convert::CameraTFCB, this );
    }


    ros::NodeHandle nh_;
    ros::Subscriber camera_action_sub_;
    ros::Publisher accuracy_pub_;
    ros::Publisher cmd_vel_pub_;
};// end of class


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ARtags_tf_convert");
  ros::NodeHandle nh;
  ARtags_tf_convert  TFC(nh);
  ros::spin();
  return 0;
}
