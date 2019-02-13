#ifndef ANGLE_TRANSFORMS_H
#define ANGLE_TRANSFORMS_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

using Eigen::MatrixXd;

class angle_transform_publisher
{
	
	
	public:
	float angle(int pixel, const sensor_msgs::CameraInfoConstPtr& camera_info)
	{
		Eigen::MatrixXd mat(3,3);
		mat << camera_info->P[0], camera_info->P[1], camera_info->P[2],
		       camera_info->P[4], camera_info->P[5], camera_info->P[6],
		       camera_info->P[8], camera_info->P[9], camera_info->P[10];
		
		Eigen::MatrixXd pixel_pos(3,1);
			pixel_pos << 320, (float)pixel, 1;
			
		Eigen::MatrixXd result = mat.inverse() * pixel_pos;
		
		std::cout << result << std::endl;
		std::cout << "angle = " << atan(result(1)/result(2)) << std::endl; 
    
		return  atan(result(1)/result(2));
		
	}
	bool publish_angle_transforms(std::string from_tf, std::vector<std::string> to_tf, std::vector<float> angle)
	{
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		if(to_tf.size() != angle.size())
		{
			std::cout << "number of tf strings and andgles do not match" << std::endl;
			return false; 
		}
		for(int i = 0; i<to_tf.size();i++)
		{
			transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			tf::Quaternion q;
			q.setRPY(0, angle[i], 0);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), from_tf, to_tf[i]));
		}
	}
};
#endif
