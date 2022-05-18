//
// Created by maxima on 24/12/18.
//

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>

#include <pcl/common/angles.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>


#include <pcl/filters/extract_indices.h>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/PointIndices.h>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <shape_msgs/SolidPrimitive.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


#include "common_defs.h"
// #include "cone_visualizer.h"
#include <eigen3/Eigen/Dense>


#include <cmath>


namespace LIDAR {

    class OutlierFilter{
    public:
        class Params{
        public:
            bool z_threshold_enable = true;
            float z_threshold_min = -1.0f;
            float z_threshold_max = 2.0f;
            float min_distance = 1.5f;
            float intensity_threshold = 60.0f;
            Params(){}
        };

    protected:
        Params params;
        // Visualizer vis;
        ros::Publisher marker_pub;
    public:
        OutlierFilter(ros::Publisher marker_pub, Params params = Params()) :
                params(params),marker_pub(marker_pub){
        }

        void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

            //ROS_INFO("size: %d, point_step: %d, row_step: %d, width: %d, height: %d]", (int)msg->data.size(), (int)msg->point_step, (int)msg->row_step, (int)msg->width, (int)msg->height);
            Cloud::Ptr cloud_in(new Cloud), cloud_out(new Cloud);

            pcl::fromROSMsg(*msg, *cloud_in);

            // filter
            otlier_filter_impl(cloud_in, cloud_out);

            // display

        }

        ros::NodeHandle nh;
        ros::Publisher pubConesIntensity = nh.advertise<visualization_msgs::MarkerArray>("/cones_Intensity", 1);
        ros::Publisher pubCones = nh.advertise<visualization_msgs::MarkerArray>("/cones_location", 1);
        ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/point_processed", 1);
        ros::Publisher pubConeMarkerArray = nh.advertise<visualization_msgs::MarkerArray>("/conesSorted", 1);
        ros::Publisher pubPolyMarkerArray0 = nh.advertise<visualization_msgs::MarkerArray>("/polylane_0", 1);
        ros::Publisher pubPolyMarkerArray1 = nh.advertise<visualization_msgs::MarkerArray>("/polylane_1", 1);
        
    protected:
        void publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<Point>::Ptr in_cloud_to_publish_ptr)
        {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
            std_msgs::Header _velodyne_header;
            _velodyne_header.frame_id = "velodyne";
            _velodyne_header.stamp = ros::Time::now();
            cloud_msg.header = _velodyne_header;
            in_publisher->publish(cloud_msg);
        }

        void otlier_filter_impl(Cloud::Ptr &cloud_in, Cloud::Ptr &cloud_out) {

            { // remove distant object
                pcl::PassThrough<Point> pass_z;
                pass_z.setInputCloud(cloud_in);
                pass_z.setFilterFieldName("z");
                pass_z.setFilterLimits(params.z_threshold_min, params.z_threshold_max);
                pass_z.filter(*cloud_out);
                // y 기준 필터 적용?ㅋㅋ 몰라
            }

            pcl::ModelCoefficients::Ptr plane_coefs (new pcl::ModelCoefficients);
            { // RANSAC plane estimation
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

                // Create the segmentation object
                pcl::SACSegmentation<Point> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.01);
                // at most 15 degrees from z axis
                seg.setAxis(Eigen::Vector3f(0, 0, 1));
                seg.setEpsAngle(pcl::deg2rad(15.0));

                seg.setInputCloud(cloud_out);
                seg.segment(*inliers, *plane_coefs);
            }



            Segmentation segm(cloud_out->size());
            Cloud::Ptr cloud_cones(new Cloud);
            cloud_cones->reserve(cloud_out->size());
            { // segment the cones
                for (int n = 0; n < cloud_out->size(); n++) {
                    Point &pt = (*cloud_out)[n];
                    float height = (float) pcl::pointToPlaneDistanceSigned(pt,plane_coefs->values[0],
                                                                           plane_coefs->values[1],
                                                                           plane_coefs->values[2],
                                                                           plane_coefs->values[3]);
                    float distance = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                    float distance_xy = sqrtf(pt.x * pt.x + pt.y * pt.y);




                    // 쌍곡선
                    double a = 3.;
                    double b = 2.;
                    
                    auto &s = segm[n];
                    if (distance_xy < params.min_distance){
                        s = PointClasses::too_close;
                    }
                    else if ((height < 0.01) || (distance < 7.0 & height < 0.03))
                        s = PointClasses::ground;
                    else if (distance > 9.0 || pt.y < -sqrt(pow(b, 2)*(pow(pt.x, 2)/pow(a, 2) +1)) || pt.y > sqrt(pow(b, 2)*(pow(pt.x, 2)/pow(a, 2) +1)) || pt.x > 6.0 || pt.x < -6.0 ) {
                        s = PointClasses::too_far;
                    } else if (height > 0.85 || height < 0.2) {
                        s = PointClasses::too_high;
                    } else {
                        s = PointClasses::inlier;
                        cloud_cones->push_back(pt);
                    }
                }
            }
            publishCloud(&pubCloud, cloud_cones);

            std::vector<ConeDescriptor> cones;

            // Euclidean Clustering 수행
            SegmentConeInstances(cloud_cones, cones);


            // 6개 기준으로 정렬하므로 그 이하인 경우 정렬하지 않음
            // 주변 물체 때문에 너무 많은 cluster가 생겨서 클러스터가 자꾸 사라짐
            // 백파일 재로깅 필요

            int fittingCone = 6;
            if (cones.size() >= fittingCone ){
                // 거리 기준으로 상위 6개 sort
                sortCones(cones, 0, fittingCone);
                // y 기준으로 6개 콘에 대해 sort
                std::vector<std::vector<double>> sortedCones = sortCones(cones, 2, fittingCone);
                markCones(sortedCones);
                // 상위 3개와 하위 3개에 대해 콘끼리 중심점 연산 및 Curve Fittitng
                std::vector<double> rCoefs = curveFitting3(std::vector<std::vector<double>>(sortedCones.begin(), sortedCones.begin()+1));
                std::vector<double> lCoefs = curveFitting3(std::vector<std::vector<double>>(sortedCones.begin()+3, sortedCones.end()));
                // std::vector<double> rCoefs = curveFitting2(std::vector<std::vector<double>>(sortedCones.begin(), sortedCones.begin()+1));
                // std::vector<double> lCoefs = curveFitting2(std::vector<std::vector<double>>(sortedCones.begin()+2, sortedCones.end()));
                
                // vis Trajectory!!
                visTrajectory(rCoefs, 0.3, 15, 0);
                visTrajectory(lCoefs, 0.3, 15, 1);
                
            }
            else{
                ROS_ERROR("LACK OF CLUSTERS");
            }
            vis_cones_loc(cones);
            vis_cones_info(cones);
            // vis_trajectory(cones);
            // vis.draw(cloud_in, cloud_out, segm, plane_coefs, cones);
        }


        void SegmentConeInstances(Cloud::Ptr cloud_cones, std::vector<ConeDescriptor> &cones) {

            double cluster_tolerance;
            int min_cluster_size, max_cluster_size;
            ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.25);
            ros::param::param("ec_min_cluster_size", min_cluster_size, 1);
            ros::param::param("ec_max_cluster_size", max_cluster_size, 400);

            std::vector<pcl::PointIndices> object_indices;
            pcl::EuclideanClusterExtraction<Point> euclid;
            euclid.setInputCloud(cloud_cones);
            euclid.setClusterTolerance(cluster_tolerance);
            euclid.setMinClusterSize(min_cluster_size);
            euclid.setMaxClusterSize(max_cluster_size);
            euclid.extract(object_indices);

            pcl::ExtractIndices<Point> extract;
            extract.setInputCloud(cloud_cones);

            cones.reserve(object_indices.size());
            size_t min_size = std::numeric_limits<size_t>::max();
            size_t max_size = std::numeric_limits<size_t>::min();
            pcl::PointIndices::Ptr object_ptr(new pcl::PointIndices);
            for (auto& object: object_indices) {
                size_t cluster_size = object.indices.size();
                if (cluster_size < min_size) {
                    min_size = cluster_size;
                }
                if (cluster_size > max_size) {
                    max_size = cluster_size;
                }
                ConeDescriptor cone;
                *object_ptr = object;
                extract.setIndices(object_ptr);
                extract.filter(*cone.cloud);
                cone.calculate();
                cones.push_back(cone);
            }

            ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
                     object_indices.size(), min_size, max_size);
        }


        std::vector<std::vector<double>> sortCones(std::vector<ConeDescriptor> &cones, int axis, int num){
            /*
            axis = 0: xy distance
            axis = 1: x
            axis = 2: y
            */
            std::vector<std::vector<double>> conesBeforeSort;
            int cnt = 0;
            if (axis == 0){
                for(auto cone : cones){
                    double x = cone.mean.x;
                    double y = cone.mean.y;
                    double distance = sqrtf(pow(x, 2) + pow(y, 2));
                    std::vector<double> dis_idx_tmp;
                    dis_idx_tmp.push_back(distance);
                    dis_idx_tmp.push_back(x);
                    dis_idx_tmp.push_back(y);
                    conesBeforeSort.push_back(dis_idx_tmp);
                    cnt++;
                    if (cnt > num){
                        break;
                    }
                }  
                std::cout<<"Before sort"<<std::endl;
                std::cout<<conesBeforeSort[0][0]<<std::endl;
                sort(conesBeforeSort.begin(), conesBeforeSort.end());
                std::cout<<conesBeforeSort[0][0]<<std::endl;
                std::cout<<"After sort"<<std::endl;


            }
            else if(axis == 2){ // sort y
                for(auto cone : cones){
                    double x = cone.mean.x;
                    double y = cone.mean.y;
                    double distance = sqrtf(pow(x, 2) + pow(y, 2));
                    std::vector<double> dis_idx_tmp;
                    // dis_idx_tmp.push_back(distance);
                    dis_idx_tmp.push_back(y);
                    dis_idx_tmp.push_back(x);
                    conesBeforeSort.push_back(dis_idx_tmp);
                    cnt++;
                    if (cnt > num){
                        break;
                    }
                } 
            // implement for x!
            }

            //sort
            std::cout<<"Before sort"<<std::endl;
            std::cout<<conesBeforeSort[0][0]<<std::endl;
            sort(conesBeforeSort.begin(), conesBeforeSort.end());
            std::cout<<conesBeforeSort[0][0]<<std::endl;
            std::cout<<"After sort"<<std::endl;

            // 
            std::vector<std::vector<double>> conesSortResult;
            for (int i = 0; i <num; i++){
                double x = 0.;
                double y = 0.;
                if(axis == 0){
                    x = conesBeforeSort[i][1];
                    y = conesBeforeSort[i][2];
                }
                else if(axis == 2){
                    x = conesBeforeSort[i][1];
                    y = conesBeforeSort[i][0];
                }

                std::vector<double> xy_pos;
                xy_pos.push_back(x);
                xy_pos.push_back(y);
                conesSortResult.push_back(xy_pos);
            }
            std::cout<<"Num Of Cones: " << std::to_string(conesSortResult.size()) << std::endl;
            return conesSortResult;
        }

        std::vector<double> curveFitting3(std::vector<std::vector<double>> conePos){
            int down_size = (conePos.size());
            Eigen::MatrixXd X_Matrix(down_size, 3);
            Eigen::VectorXd y_Vector(down_size);
            Eigen::VectorXd a_Vector(3);

            // Eigen의 매트릭스에 포인트를 넣어준다.
            for (int i_point = 0; i_point < down_size; i_point++) {
                double x = conePos[i_point][0];
                double y = conePos[i_point][1];

                X_Matrix(i_point, 0) = 1;
                X_Matrix(i_point, 1) = x;
                X_Matrix(i_point, 2) = x * x;
                // X_Matrix(i_point, 3) = x * x * x;
                y_Vector(i_point) = y;
            }

            a_Vector = ((X_Matrix.transpose() * X_Matrix).inverse() * X_Matrix.transpose()) * y_Vector;
            std::vector<double> curveCoefs;
            curveCoefs.push_back(a_Vector(0));
            curveCoefs.push_back(a_Vector(1));
            curveCoefs.push_back(a_Vector(2));
            curveCoefs.push_back(0.0);//a_Vector(3));
            return curveCoefs;
        }
        std::vector<double> curveFitting2(std::vector<std::vector<double>> conePos){
            int down_size = (conePos.size());
            Eigen::MatrixXd X_Matrix(down_size, 2);
            Eigen::VectorXd y_Vector(down_size);
            Eigen::VectorXd a_Vector(2);

            // Eigen의 매트릭스에 포인트를 넣어준다.
            for (int i_point = 0; i_point < down_size; i_point++) {
                double x = conePos[i_point][0];
                double y = conePos[i_point][1];

                X_Matrix(i_point, 0) = 1;
                X_Matrix(i_point, 1) = x;
                // X_Matrix(i_point, 2) = x * x;
                // X_Matrix(i_point, 3) = x * x * x;
                y_Vector(i_point) = y;
            }

            a_Vector = ((X_Matrix.transpose() * X_Matrix).inverse() * X_Matrix.transpose()) * y_Vector;
            std::vector<double> curveCoefs;
            curveCoefs.push_back(a_Vector(0));
            curveCoefs.push_back(a_Vector(1));
            curveCoefs.push_back(0.0);//a_Vector(2));
            curveCoefs.push_back(0.0);//a_Vector(3));
            return curveCoefs;
        }

        void vis_cones_info(std::vector<ConeDescriptor> &cones){

            visualization_msgs::MarkerArray markerArray;
            int id = 0;

            // for (auto i_lane = 0; i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) 
            for(int idx= 0; idx<cones.size(); idx++){
                // if(cones[idx].radius> 0.4 || abs(cones[idx].mean.y) > 3.0 ){
                //     continue;
                // }
                double x = cones[idx].mean.x;
                double y = cones[idx].mean.y;


                visualization_msgs::Marker marker;
                // marker.header.frame_id = m_polyLanes.polyfitLanes[i_lane].frame_id;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = ros::Time::now();

                // marker.ns = m_polyLanes.polyfitLanes[i_lane].id;
                marker.ns = "vis_cones";
                marker.id = id;                
                marker.text = "Intensity: "+ std::to_string(cones[idx].mean.intensity) 
                                // + " | Valid: " + (cones[idx].valid ? "true" : "false")
                                + " | Size: " + std::to_string(cones[idx].cloud->size())
                                + " | Distance: " + std::to_string(sqrt(pow(x, 2)+pow(y, 2)))
                                // + " | x: " + std::to_string(x)
                                + " | y: " + std::to_string(y)
                                + "\nRadius: " + std::to_string(cones[idx].radius)
                                + "Stddev: " + std::to_string(cones[idx].stddev.x) + std::to_string(cones[idx].stddev.y);
                                
                // marker.type = visualization_msgs::Marker::CYLINDER;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = 0.3;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.1;
                marker.pose.orientation.w = 1.0;
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0;
                marker.scale.x = 0.13;
                marker.scale.y = 0.13;
                marker.scale.z = 0.13;
                marker.lifetime = ros::Duration(0.1);

                markerArray.markers.push_back(marker);
                id++;
            }

            pubConesIntensity.publish(markerArray);
        }

        void vis_cones_loc(std::vector<ConeDescriptor> &cones){

            visualization_msgs::MarkerArray markerArray;
            int id = 0;

            // for (auto i_lane = 0; i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) 
            for(int idx= 0; idx<cones.size(); idx++){
                // if(cones[idx].radius> 0.4 || abs(cones[idx].mean.y) > 3.0 ){
                //     continue;
                // }
                // if(cones[idx].valid == false){
                //     continue;
                // }
                double x = cones[idx].mean.x;
                double y = cones[idx].mean.y;


                visualization_msgs::Marker marker;
                // marker.header.frame_id = m_polyLanes.polyfitLanes[i_lane].frame_id;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = ros::Time::now();

                // marker.ns = m_polyLanes.polyfitLanes[i_lane].id;
                marker.ns = "vis_cones";
                marker.id = id;


                marker.type = visualization_msgs::Marker::CYLINDER;
                // marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = -0.4;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.1;
                marker.pose.orientation.w = 1.0;
                Params param;
                if (cones[idx].mean.intensity < param.intensity_threshold){
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 1.0f;
                }
                else{// (cones[idx].mean.intensity < intensity_threshold){
                    marker.color.r = 1.0f;
                    marker.color.g = 0.83f;
                    marker.color.b = 0.0f;
                }
                marker.color.a = 1.0;
                marker.scale.x = 0.15;
                marker.scale.y = 0.15;
                marker.scale.z = 0.4;
                marker.lifetime = ros::Duration(0.1);

                markerArray.markers.push_back(marker);
                id++;
            }

            pubCones.publish(markerArray);
        }

        void markCones(std::vector<std::vector<double>> vis_cluster) {

            visualization_msgs::MarkerArray markerArray;
            int id = 0;

            // for (auto i_lane = 0; i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) 
            for(int idx= 0; idx<vis_cluster.size(); idx++){
                double x = vis_cluster[idx][0];
                double y = vis_cluster[idx][1];


                visualization_msgs::Marker marker;
                // marker.header.frame_id = m_polyLanes.polyfitLanes[i_lane].frame_id;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = ros::Time::now();

                // marker.ns = m_polyLanes.polyfitLanes[i_lane].id;
                marker.ns = "vis_marker";
                marker.id = id;

                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = 0.0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.1;
                marker.pose.orientation.w = 1.0;
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0;
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;
                marker.lifetime = ros::Duration(0);

            }

            pubConeMarkerArray.publish(markerArray);
        }

        void visTrajectory(std::vector<double> coefs, double interval, double ROILength, int idx) {

            visualization_msgs::MarkerArray markerArray;
            // for (auto i_lane = 0; i_lane < m_polyLanes.polyfitLanes.size(); i_lane++) 
            double a0 = coefs.at(0);
            double a1 = coefs.at(1);
            double a2 = coefs.at(2);

            double x = -10.0;
            double y = a0;

            double distance_square = x * x + y * y;
            
            int id = 0;
            while (distance_square < ROILength * ROILength) {
                // double a0 = m_polyLanes.polyfitLanes[i_lane].a0;
                // double a1 = m_polyLanes.polyfitLanes[i_lane].a1;
                // double a2 = m_polyLanes.polyfitLanes[i_lane].a2;
                // double a3 = m_polyLanes.polyfitLanes[i_lane].a3;

                y = a0 + a1 * x + a2 * x * x; //a3 * x * x * x;
                distance_square = x * x + y * y;

                visualization_msgs::Marker marker;
                // marker.header.frame_id = m_polyLanes.polyfitLanes[i_lane].frame_id;
                marker.header.frame_id = "velodyne";
                marker.header.stamp = ros::Time::now();

                // marker.ns = m_polyLanes.polyfitLanes[i_lane].id;
                marker.ns = "polylane";
                marker.id = id;

                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = 0.0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.1;
                marker.pose.orientation.w = 1.0;
                if(idx == 0){
                    marker.color.r = 0.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.0f;
                }
                else{
                    marker.color.r = 1.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.0f;
                }
                marker.color.a = 1.0;
                marker.scale.x = 0.06;
                marker.scale.y = 0.06;
                marker.scale.z = 0.06;
                marker.lifetime = ros::Duration(0.1);

                markerArray.markers.push_back(marker);
                x += interval;
                id++;
            }
            if (idx == 0){
                pubPolyMarkerArray0.publish(markerArray);
            }
            else{
                pubPolyMarkerArray1.publish(markerArray);
            }
        }
    };
};
