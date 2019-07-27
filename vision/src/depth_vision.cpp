//
// Created by enrico on 18/07/19.
//

#include <ros/ros.h>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include <cmath>        // std::atan2
#include <valarray>     // std::valarray, std::atan2

class DepthCamera{
protected:
    visualization_msgs::MarkerArray markers_;
    ros::Publisher pub_pcd_rot, pub_pcd_crop, pub_boxes, pub_poses, pub_pose;
    ros::Subscriber sub_pcd, sub_fv;

    ros::NodeHandle *nh_;
    ros::Rate *r;

    double x,y,z,roll,pitch,yaw;
   // double qx,qy,qz,qw;

    tf::Matrix3x3 m;
    tf::Quaternion quaternion;
    tf::TransformListener tf_listener, tf_2;
    tf::StampedTransform T_base_cam, T2;

    geometry_msgs::TransformStamped msg;

    sensor_msgs::PointCloud2 cloud, cloud_new;
    pcl::PCLPointCloud2 cloud_pcl;

//const boost::shared_ptr<const sensor_msgs::PointCloud2>
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        cloud = *msg;
    }
/*
    void fakeVision(const gazebo_msgs::ModelStates::ConstPtr &msg) {

        geometry_msgs::PoseStamped pose;

        int k = 0;
        for (int i = 0; i < msg->pose.size(); i++) {

            if (strcmp(msg->name.at(i).c_str(), obj) == 0) {

                pose.pose = msg->pose.at(i);
                //ROS_INFO_STREAM_THROTTLE(1, "coke: " << pose.pose.position.z);
                pub_poses.publish(pose);
            }

            if (strcmp(msg->name.at(i).c_str(), ee) == 0) {

                pose.pose = msg->pose.at(i);
                //ROS_INFO_STREAM_THROTTLE(1, "hand: " << pose.pose.position.z);
                pub_poses.publish(pose);
            }

        }
*/

        public:
    DepthCamera(){

        nh_=new ros::NodeHandle();
        sub_pcd = nh_->subscribe("/camera/depth_registered/points", 10, &DepthCamera::PointCloudCallback, this);

        pub_boxes = nh_->advertise<visualization_msgs::MarkerArray>("bbox", 1);
        pub_poses = nh_->advertise<geometry_msgs::PoseArray>("/bbox_poses", 1);
        pub_pose  = nh_->advertise<geometry_msgs::PoseStamped>("/bbox_pose", 1000);
        pub_pcd_rot = nh_->advertise<sensor_msgs::PointCloud2>("/point_cloud_world", 1);
        pub_pcd_crop = nh_->advertise<sensor_msgs::PointCloud2>("/point_cloud_crop", 1);

        r=new ros::Rate(2);
        while(cloud.width==0){
            r->sleep();
            ros::spinOnce();
        }

        bool got_trans=false;
        while (!got_trans){
            try{
                tf_listener.waitForTransform("world",cloud.header.frame_id,cloud.header.stamp,ros::Duration(1.0));
                tf_listener.lookupTransform("world",cloud.header.frame_id,cloud.header.stamp,T_base_cam);
                msg.transform.translation.x = T_base_cam.getOrigin().x();
                msg.transform.translation.y = T_base_cam.getOrigin().y();
                msg.transform.translation.z = T_base_cam.getOrigin().z();
                msg.transform.rotation.x = T_base_cam.getRotation().x();
                msg.transform.rotation.y = T_base_cam.getRotation().y();
                msg.transform.rotation.z = T_base_cam.getRotation().z();
                msg.transform.rotation.w = T_base_cam.getRotation().w();

                x= msg.transform.translation.x;
                y= msg.transform.translation.y;
                z= msg.transform.translation.z;

                quaternion.setValue(msg.transform.rotation.x,
                                    msg.transform.rotation.y,
                                    msg.transform.rotation.z,
                                    msg.transform.rotation.w );

                m.setRotation(quaternion);
                m.getRPY(roll,pitch,yaw);
                //tf2::doTransform(cloud, cloud_new, msg);
                //pub_pcd_rot.publish(cloud_new);
                got_trans = true;
            }
            catch (const tf2::ExtrapolationException& e){
                ROS_ERROR_STREAM(e.what());
            }
            ros::spinOnce();
        }
    }

    pcl::PCLPointCloud2 point_cloud_roi(){

        pcl_conversions::toPCL(cloud, cloud_pcl);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>),
                                            temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>),
                                            cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromPCLPointCloud2(cloud_pcl, *temp_cloud1);

        // Transform PointCloud - From camera_depth_optical_frame to world

        pcl_ros::transformPointCloud(*temp_cloud1,*temp_cloud2, T_base_cam);

        // Crop Box Filter - It allows to select the point clouds that belong to a box

        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setInputCloud(temp_cloud2);

        boxFilter.setMin(Eigen::Vector4f(-0.3, -0.38, 0.884, 1.0));   //-0.3, -0.38, 0.884, 1.0
        boxFilter.setMax(Eigen::Vector4f(0.52, 0.38, 0.98, 1.0));     //0.52, 0.38, 0.98, 1.0

        boxFilter.filter(*cloud_out);

        pcl::toPCLPointCloud2(*cloud_out, cloud_pcl);

        pub_pcd_crop.publish(cloud_pcl);


        return cloud_pcl;
    }


    std::vector<Eigen::Vector3f> BBox(const pcl::PCLPointCloud2 &cloud_in) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromPCLPointCloud2(cloud_in, *temp_cloud);

        std::vector<Eigen::Vector3f> bbox;
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(temp_cloud);
        feature_extractor.compute();

        pcl::PointXYZ min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;

        feature_extractor.getAABB(min_point_AABB, max_point_AABB);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p2(min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p3(max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p4(max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p5(min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
        Eigen::Vector3f p6(min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p7(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
        Eigen::Vector3f p8(max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

        bbox.push_back(rotational_matrix_OBB * p1 + position);
        bbox.push_back(rotational_matrix_OBB * p2 + position);
        bbox.push_back(rotational_matrix_OBB * p3 + position);
        bbox.push_back(rotational_matrix_OBB * p4 + position);
        bbox.push_back(rotational_matrix_OBB * p5 + position);
        bbox.push_back(rotational_matrix_OBB * p6 + position);
        bbox.push_back(rotational_matrix_OBB * p7 + position);
        bbox.push_back(rotational_matrix_OBB * p8 + position);

         return bbox;

     }

     visualization_msgs::MarkerArray create_marker (std::vector<Eigen::Vector3f> bbox) {

        visualization_msgs::Marker mark;
        visualization_msgs::MarkerArray marker_ar;

        std_msgs::ColorRGBA color;
            color.r = 0.8; color.g = 0.3; color.a = 0.7;

            mark.action = visualization_msgs::Marker::ADD;
            mark.id = 0;
            mark.scale.x = 0.005;
            mark.type = visualization_msgs::Marker::LINE_LIST;
            std::vector<geometry_msgs::Point> p_v;
            for (int i = 0; i < 8; i++) {
                geometry_msgs::Point p;
                p.x = bbox.at(i)[0];
                p.y = bbox.at(i)[1];
                p.z = bbox.at(i)[2];
                p_v.push_back(p);
            }
            mark.points.push_back(p_v.at(0));
            mark.points.push_back(p_v.at(1));
            mark.points.push_back(p_v.at(0));
            mark.points.push_back(p_v.at(3));
            mark.points.push_back(p_v.at(0));
            mark.points.push_back(p_v.at(4));
            mark.points.push_back(p_v.at(4));
            mark.points.push_back(p_v.at(5));
            mark.points.push_back(p_v.at(4));
            mark.points.push_back(p_v.at(7));
            mark.points.push_back(p_v.at(1));
            mark.points.push_back(p_v.at(5));

            mark.points.push_back(p_v.at(5));
            mark.points.push_back(p_v.at(6));
            mark.points.push_back(p_v.at(6));
            mark.points.push_back(p_v.at(7));
            mark.points.push_back(p_v.at(1));
            mark.points.push_back(p_v.at(2));
            mark.points.push_back(p_v.at(3));
            mark.points.push_back(p_v.at(7));
            mark.points.push_back(p_v.at(2));
            mark.points.push_back(p_v.at(3));
            mark.points.push_back(p_v.at(2));
            mark.points.push_back(p_v.at(6));

            mark.color = color;
            mark.lifetime = ros::Duration(5);

            mark.pose.orientation.w = 1.0;
            mark.header.frame_id = "world";

            marker_ar.markers.push_back(mark);

            pub_boxes.publish(marker_ar);

            return marker_ar;
        }

    void BBoxPose(const visualization_msgs::MarkerArray marker_ar, std::vector<Eigen::Vector3f> bbox,
                  const tf::StampedTransform T_base_cam) {

        visualization_msgs::Marker mark;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseArray poses_ar;
        std::vector<geometry_msgs::Vector3> dimensions;
        double ym, pm, rm;
        tf::Quaternion qm;
        dimensions.resize(marker_ar.markers.size());

        for (int i = 0; i < 8; i++) {
            pose.pose.position.x += bbox.at(i)[0];
            pose.pose.position.y += bbox.at(i)[1];
            pose.pose.position.z += bbox.at(i)[2];
        }
        pose.pose.position.x /= 8;
        pose.pose.position.y /= 8;
        pose.pose.position.z /= 8;

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;


        //tf::quaternionTFToMsg(quaternion, pose.pose.orientation);
        poses_ar.header.frame_id = "world";

        std::vector<Eigen::Vector3f> v;
        v.push_back(bbox.at(1) - bbox.at(0));
        v.push_back(bbox.at(3) - bbox.at(0));
        v.push_back(bbox.at(4) - bbox.at(0));

        Eigen::Vector3f ux, uy, uz, vx, vy, vz, cross;

       ux << 1, 0, 0; // -2.618, 0.000, -0.001;
       uy << 0, 1, 0;
       uz << 0, 0, 1;

       vx=bbox.at(1) - bbox.at(0);
       vy=bbox.at(3) - bbox.at(0);
       vz=bbox.at(4) - bbox.at(0);

        tf::Quaternion q;

        float det, dot, dot1, dot2, lenSq1, lenSq2;
        double theta;

        vz = vz/vz.norm();
        dot = ux[0]*vz[0] + uz[1]*vz[1] + ux[2]*vz[2];    //dot product
        //det = ux[0]*vz[1]*1 + vz[0]*0*ux[2] + 0*ux[1]*vz[2] - ux[2]*vz[1]*0 - vz[2]*0*ux[0] - 1*ux[1]*vz[0];
        //theta = atan2(det, dot);
        //if (theta > M_PI)        { theta -= 2 * M_PI; }
        //else if (theta <= -M_PI) { theta += 2 * M_PI; }

        double uvi, uvj, uvk;
        uvi = ux[1] * vz[2] - vz[1] * ux[2];
        uvj = vz[0] * ux[2] - ux[0] * vz[2];
        uvk = ux[0] * vz[1] - vz[0] * ux[1];

        cross << uvi, uvj, uvk;

        theta = atan2(cross.norm(), fabs(dot));

        dot1 = vx[0]*vy[0] + vx[1]*vy[1] + vx[2]*vy[2];

        q.setRPY(0,0,theta);
        //ROS_INFO("DOT: %f \n %f \n", dot, dot2);
        //ROS_INFO("Theta: %f \n" , theta);

        tf::Quaternion q2;
        tf::Matrix3x3 R;
        R.setValue( 0, 1, 0,
                   -1, 0, 0,
                    0, 0, 1);
        R.getRotation(q2);
        tf::quaternionTFToMsg(q*q2, pose.pose.orientation);

        poses_ar.poses.push_back(pose.pose);

        pub_poses.publish(poses_ar);
        pose.header.frame_id="world";
        pub_pose.publish(pose);
    }

    void run(){
        while(ros::ok()){
            ros::spinOnce();

            //sensor_msgs::PointCloud2 cld = transform();
            pcl::PCLPointCloud2 pcl_cld = point_cloud_roi();
            std::vector<Eigen::Vector3f> bbox = BBox(pcl_cld);
            visualization_msgs::MarkerArray markerArray = create_marker(bbox);
            BBoxPose(markerArray, bbox, T_base_cam);

            r->sleep();
        }
    }
};

int main (int argc, char** argv)
{

    ros::init(argc,argv,"obj_bbox");
    DepthCamera dc;
    dc.run();
}