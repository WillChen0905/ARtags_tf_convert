#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"





class ARtags_tf_convert {
public:
    ARtags_tf_convert ( ros::NodeHandle& nodehandle )
            : nh_( nodehandle )
    {
        camera_tf_sub_ = nh_.subscribe( "camera_image", 10, &ARtags_tf_convert::CameraTFCB, this );
        cmd_vel_pub_ = nh_.advertise< geometry_msgs::Twist > ( "cmd_vel", 10 );
    }
private:

    void VelocityPub( const geometry_msgs::Pose& pose )
    {
        geometry_msgs::Twist Vel;

        float x = pose.position.x;
        float y = pose.position.y;
        float yaw = tf2::getYaw(pose.orientation)*180/3.14;

        float max_vtheta = 0.3;
        float max_vtranslation = 0.15;

        if( fabs( yaw ) > 5  )
        {
            Vel.angular.z = -1 * ( max_vtheta * ( yaw / 180 ) );
        }
        else
        {
            if( fabs( x ) > 0.02 || fabs( y ) > 0.3 )
            {
                float dist_ratio_x = 0.1 - fabs( x ) / 0.1;
                float dist_ratio_y = 1.0 - fabs( y ) / 0.7;

                Vel.linear.x = -1 * ( max_vtranslation * dist_ratio_x );
                Vel.linear.y = -1 * ( max_vtranslation * dist_ratio_y );
            }
        }

        std::cout << "Vel.x= " <<　Vel.linear.x << "  |  "
                  << "Vel.y= " <<　Vel.linear.y << "  |  "
                  << "Vel.th= " <<　Vel.angular.z << std::endl;
        std::cout << "--------------------------------------------------------" << std::endl;

        cmd_vel_pub_.publish( Vel );

    }

    void CameraTFCB( const std::msgs::Strng& str )
    {
        if ( !( str.data == "start" ) )
        {
            std::cout << "CameraTF_convert shutdown" << std::endl;
            camera_tf_sub_.shutdown();
        }


        // Get TF from camera to base_link
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        bool is_Transform = true;

        do
        {
            try
            {
                transformStamped = tfBuffer.lookupTransform("base_link", "camera", ros::Time(0));
                std::cout << "Get Transform" << std::endl;
                is_Transform = false;
            }
            catch( tf2::TransformException& ex )
            {
                ROS_WARN("%s", ex.what());
            }
        }
        while( is_Transform );

        // Convert
        geometry_msgs::Pose originTag;
        geometry_msgs::Pose Tag_to_Aiv;

        tf2::doTransform( originTag, Tag_to_Aiv, transformStamped );
        std::cout << "originTag.x= " <<　originTag.position.x << "  |  "
                  << "Tag_to_Aiv.x= " <<　Tag_to_Aiv.position.x << std::endl;
        std::cout << "originTag.y= " <<　originTag.position.y << "  |  "
                  << "Tag_to_Aiv.y= " <<　Tag_to_Aiv.position.y << std::endl;
        std::cout << "originTag.yaw= " <<　tf2::getYaw(originTag.orientation) << "  |  "
                  << "Tag_to_Aiv.yaw= " <<　tf2::getYaw(Tag_to_Aiv.orientation) << std::endl;
        // std::cout << "--------------------------------------------------------" << std::endl;

        VelocityPub( Tag_to_Aiv );
    }


    ros::NodeHandle nh_;
    ros::Subscriber camera_tf_sub_;
    ros::Publisher cmd_vel_pub_;
};// end of class
