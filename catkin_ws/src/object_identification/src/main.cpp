#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <cmath>
#include<fstream>
#include<iostream>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include "object_identification/register_object.h"
#include "object_identification/recognition_object.h"
#include "object_identification/training.h"
#include "object_identification/get_attribute.h"

#include "object_identification/Attribute.h"
#include "object_identification/AttributeArray.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include <std_srvs/Empty.h>


class Util {
  public:
    // sprit string 
    static std::vector < std::string > split(std::string str, char del);

    static bool checkFileExistence(const std::string & str);
    static void transformCloud(pcl::PointCloud < pcl::PointXYZRGB >::Ptr input_cloud_, geometry_msgs::TransformStamped transformStamped);
};

std::vector < std::string > Util::split(std::string str, char del)
{
    int first = 0;
    int last = str.find_first_of(del);

    std::vector < std::string > result;

    while (first < str.size()) {
        std::string subStr(str, first, last - first);

        result.push_back(subStr);

        first = last + 1;
        last = str.find_first_of(del, first);

        if (last == std::string::npos) {
            last = str.size();
        }
    }

    return result;
}

bool Util::checkFileExistence(const std::string & str)
{
    std::ifstream ifs(str.c_str());
    return ifs.is_open();
}

void Util::transformCloud(pcl::PointCloud < pcl::PointXYZRGB >::Ptr input_cloud_, geometry_msgs::TransformStamped transformStamped)
{
    // Convert the point cloud to msg for transformation
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*input_cloud_, cloud_msg);
  
    // Transformation to base_link frame
    Eigen::Matrix4f mat = tf2::transformToEigen(transformStamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(mat, cloud_msg, cloud_msg);
  
    // Convert the msg to point cloud
    pcl::fromROSMsg(cloud_msg, *input_cloud_);
}



class ObjectIdentification {
  public:
    ObjectIdentification(std::string point_topic_name, std::string rgb_topic_name,  std::string rgb_info_topic_name);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber rgb_sub_;
    ros::Subscriber cinfo_sub_;
    ros::Publisher bb_pub_;
    ros::Publisher cloud_pub_;

    ros::ServiceServer service_register;
    ros::ServiceServer service_recog;
    ros::ServiceServer service_update_threshold;
    ros::ServiceServer service_training;
    ros::ServiceClient client_attr ;

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentation_cloud_;
    pcl::PointCloud < pcl::PointXYZRGB >::Ptr remove_cloud_;
    pcl::PointCloud < pcl::PointXYZRGB >::Ptr input_cloud_data_;
    pcl::PointCloud < pcl::PointXYZRGB >::Ptr input_cloud_data_raw_;
    cv::Mat input_image;
    std_msgs::Header image_header;

    sensor_msgs::CameraInfo cinfo;
    std::multimap < std::string, std::vector < double > >colorhist_mp;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener * tf_listener_;
    
    const bool debug = false;

    const int maxval = 255;
    const double s_th = 0.2;
    const double v_th = 0.2;
    const int num_hist_bin = 26;
    Eigen::Vector4f boxFilter_min ;
    Eigen::Vector4f boxFilter_max ;
    int get_index(int R, int G, int B);

    void clastering(std::vector < pcl::PointIndices > &cluster_indices);

    struct cluster_parameter {
        double dist;
        int px_min;
        int py_min;
        int px_max;
        int py_max;
    };

    double calc_cluster_parameter(pcl::PointIndices
                                  cluster_indices,
                                  struct cluster_parameter &result);

    void calc_histgram(pcl::PointIndices cluster_indices,
                       std::vector < double >&hist);
    bool training_srv(object_identification::training::Request & req,
                  object_identification::training::Request & res);
    bool recogntion_object_srv(object_identification::
                           recognition_object::Request & req,
                           object_identification::
                           recognition_object::Response & res);
    bool update_threshold_srv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void recogntion_object(darknet_ros_msgs::BoundingBoxes &result_data, cv::Mat &recog_image);

    bool register_object_srv(object_identification::
                         register_object::Request & req,
                         object_identification::
                         register_object::Response & res);
    void CinfoCb(const sensor_msgs::CameraInfo & msg);
    void RGBCb(const sensor_msgs::ImageConstPtr & msg);
    void SegmentationCb(const sensor_msgs::PointCloud2ConstPtr & msg);
};


ObjectIdentification::ObjectIdentification(std::string point_topic_name, std::string rgb_topic_name,  std::string rgb_info_topic_name):
remove_cloud_(new pcl::PointCloud < pcl::PointXYZRGB >),
input_cloud_data_(new pcl::PointCloud < pcl::PointXYZRGB >),
input_cloud_data_raw_(new pcl::PointCloud < pcl::PointXYZRGB >)
{
    /*
    std::string point_topic_name = "/camera/depth_registered/points";
    std::string rgb_topic_name = "/camera/rgb/image_color";
    std::string rgb_info_topic_name = "/camera/rgb/camera_info";
    */
    cloud_sub_ =
        nh_.subscribe(point_topic_name, 1,
                      &ObjectIdentification::SegmentationCb, this);
    rgb_sub_ =
        nh_.subscribe(rgb_topic_name, 1,
                      &ObjectIdentification::RGBCb, this);
    cinfo_sub_ =
        nh_.subscribe(rgb_info_topic_name, 1,
                      &ObjectIdentification::CinfoCb, this);
    service_register =
        nh_.advertiseService("register_object",
                             &ObjectIdentification::register_object_srv,
                             this);
    service_recog =
        nh_.advertiseService("recognition_object",
                             &ObjectIdentification::recogntion_object_srv,
                             this);
    service_training =
        nh_.advertiseService("training",
                             &ObjectIdentification::training_srv, this);

    service_update_threshold = 
        nh_.advertiseService("update_threshold", 
                             &ObjectIdentification::update_threshold_srv, this);
    
    bb_pub_ =
       nh_.advertise < darknet_ros_msgs::BoundingBoxes >
       ("/recongnition_result", 1);

    cloud_pub_ =
       nh_.advertise <sensor_msgs::PointCloud2>
       ("/object_identification_points", 1);

    client_attr = nh_.serviceClient<object_identification::get_attribute>("/get_attribute"); 

    float x_min = 0, x_max = 1.5, y_min = -0.5, y_max = 0.5, z_min = 0.0, z_max = 0.5;
    nh_.getParam("/ork_tf_broadcaster/centroid_x_max", x_max);
    nh_.getParam("/ork_tf_broadcaster/centroid_x_min", x_min);
    nh_.getParam("/ork_tf_broadcaster/centroid_y_max", y_max);
    nh_.getParam("/ork_tf_broadcaster/centroid_y_min", y_min);
    nh_.getParam("/ork_tf_broadcaster/centroid_z_max", z_max);
    nh_.getParam("/ork_tf_broadcaster/centroid_z_min", z_min);
    boxFilter_min = Eigen::Vector4f(x_min, y_min, z_min, 1.0);
    boxFilter_max = Eigen::Vector4f(x_max, y_max, z_max, 1.0);
//    boxFilter_min = Eigen::Vector4f(-1.5, -1.5, 0, 1.0);
//    boxFilter_max = Eigen::Vector4f(1.5, 1.5, 1.0, 1.0);

    // TF listener
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
}

int ObjectIdentification::get_index(int R, int G, int B)
{
    double hue, v, s;
    int max, min;

    max = min = R;
    if (max < G)
        max = G;
    if (max < B)
        max = B;
    if (min > G)
        min = G;
    if (min > B)
        min = B;

    v = (double) max / (double) maxval;

    //s=((double)(max-min))/((double)sertch_maxval);
    s = ((double) (max - min)) / ((double) max);
    if (max == R) {
        hue = 60 * ((double) (G - B)) / ((double) (max - min)) + 0.;
    } else if (max == G) {
        hue = 60 * ((double) (B - R)) / ((double) (max - min)) + 120.;
    } else {
        hue = 60 * ((double) (R - G)) / ((double) (max - min)) + 240.;
    }

    while (hue >= 360.) {
        hue -= 360.;
    }
    while (hue < 0.) {
        hue += 360.;
    }

    int ret = 0;
    if (v < v_th) {
        ret = 0;
    } else if (s < s_th) {
        ret = 1;
    } else if (s < (s_th + (1. - s_th) / 2.)) {
        ret = 2 + (int) (hue / 30.);
    } else {
        ret = 2 + 12 + (int) (hue / 30.);
    }
    return ret;
}

void ObjectIdentification::clastering(std::vector < pcl::PointIndices >
                                         &cluster_indices)
{
    //Delete NaN data
    std::vector < int >nan_indices;
    pcl::removeNaNFromPointCloud(*input_cloud_data_,
                                 *input_cloud_data_, nan_indices);
    
    // Transform
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped transformStampedInverse;

    // Get transform between camera_frame and base_link
    try{
      // Listen ot a transform from base_link to camera frame to fix the X-Y plane parallel to the floor
      transformStamped = tf_buffer_.lookupTransform("base_link", "head_rgbd_sensor_rgb_frame",
                               ros::Time(0));
      transformStampedInverse = tf_buffer_.lookupTransform("head_rgbd_sensor_rgb_frame", "base_link",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    // Transform the point cloud frame to base_link
    Util::transformCloud(input_cloud_data_, transformStamped);

    ROS_INFO("clastering() : cloud size %d", (int) input_cloud_data_->size());

    //Delete distant observation points
    pcl::CropBox < pcl::PointXYZRGB > boxFilter;
    boxFilter.setMin(boxFilter_min);
    boxFilter.setMax(boxFilter_max);
    boxFilter.setInputCloud(input_cloud_data_);
    boxFilter.filter(*input_cloud_data_);


    ROS_INFO("clastering() : cloud size after box filter  %d", (int) input_cloud_data_->size());

    //Delete plane points
    pcl::SACSegmentation < pcl::PointXYZRGB > seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(input_cloud_data_);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients coefficients;
    seg.segment(*inliers, coefficients);

    ROS_INFO("RANSAC inlier size : %d", (int) inliers->indices.size());

    // Debug
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*input_cloud_data_, cloud_msg);
    cloud_msg.header.frame_id = "base_link";
    cloud_pub_.publish(cloud_msg);

    pcl::ExtractIndices < pcl::PointXYZRGB > extract;
    extract.setInputCloud(input_cloud_data_);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*remove_cloud_);


    //Clustering
    pcl::search::KdTree <
        pcl::PointXYZRGB >::Ptr tree(new pcl::search::KdTree <
                                     pcl::PointXYZRGB >);
    tree->setInputCloud(remove_cloud_);
    pcl::EuclideanClusterExtraction < pcl::PointXYZRGB > clustering;
    clustering.setClusterTolerance(0.05);
    clustering.setMinClusterSize(20);
    clustering.setMaxClusterSize(300000);
    clustering.setSearchMethod(tree);
    clustering.setInputCloud(remove_cloud_);
    clustering.extract(cluster_indices);

    // Transform the point cloud frame to base_link
    Util::transformCloud(remove_cloud_, transformStampedInverse);
}


double ObjectIdentification::calc_cluster_parameter(pcl::PointIndices
                                                       cluster_indices,
                                                       struct
                                                       cluster_parameter
                                                       &result)
{
    double cx = 0;
    double cy = 0;
    double cz = 0;
    double px_min = 10000;
    double py_min = 10000;
    double px_max = -1;
    double py_max = -1;
    double px = 0;
    double py = 0;
    Eigen::MatrixXd Pmat(3, 4);
    //prepare  projection matrix
    for (int i = 0; i < 3; i++) {
        Pmat(i, 3) = 0;
        for (int j = 0; j < 3; j++) {
            Pmat(i, j) = this->cinfo.K[i * 3 + j];
        }
    }

    // calc center point and bounding box at image
    for (int i = 0; i < cluster_indices.indices.size(); i++) {
        int point_idx = cluster_indices.indices[i];
        cx += remove_cloud_->points[point_idx].x;
        cy += remove_cloud_->points[point_idx].y;
        cz += remove_cloud_->points[point_idx].z;
        Eigen::Vector4d
            p(remove_cloud_->points[point_idx].x,
              remove_cloud_->points[point_idx].y,
              remove_cloud_->points[point_idx].z, 1);
        Eigen::Vector3d pd = Pmat * p;
        px = pd(0) / pd(2);
        py = pd(1) / pd(2);
        px_min = std::min(px_min, px);
        py_min = std::min(py_min, py);
        px_max = std::max(px_max, px);
        py_max = std::max(py_max, py);
    }
    cx /= cluster_indices.indices.size();
    cy /= cluster_indices.indices.size();
    cz /= cluster_indices.indices.size();
    result.dist = std::sqrt(cx * cx + cy * cy + cz * cz);
    result.px_min = (int) px_min;
    result.py_min = (int) py_min;
    result.px_max = (int) px_max;
    result.py_max = (int) py_max;
}

void ObjectIdentification::calc_histgram(pcl::
                                            PointIndices cluster_indices,
                                            std::vector < double >&hist)
{
    for (int i = 0; i < cluster_indices.indices.size(); i++) {
        int point_idx = cluster_indices.indices[i];
        int r = remove_cloud_->points[point_idx].r;
        int g = remove_cloud_->points[point_idx].g;
        int b = remove_cloud_->points[point_idx].b;
        int index = get_index(r, g, b);
        hist[index] += 1.0;
    }
    for (int i = 0; i < num_hist_bin; i++) {
        hist[i] /= (double) cluster_indices.indices.size();
    }

}

bool ObjectIdentification::
training_srv(object_identification::training::Request & req,
         object_identification::training::Request & res)
{
    colorhist_mp.clear();

    std::string pkg_path =
        ros::package::getPath("object_identification");
    std::string database_path = pkg_path + "/database/";
    DIR *dp;                    // ディレクトリへのポインタ
    dirent *entry;              // readdir() で返されるエントリーポイント
    dp = opendir(database_path.c_str());
    if (dp == NULL) {
        return true;
    }
    for (entry = readdir(dp); entry != NULL; entry = readdir(dp)) {
        if (entry->d_name[0] == '.') {
            continue;
        }
        //std::cout << entry->d_name << std::endl;
        DIR *dp_c;
        dirent *entry_c;
        std::string load_path = database_path + entry->d_name;
        dp_c = opendir(load_path.c_str());
        for (entry_c = readdir(dp_c); entry_c != NULL;
             entry_c = readdir(dp_c)) {
            //std::cout << entry_c->d_name<< std::endl;
            if (entry_c->d_name[0] != 'c') {
                continue;
            }
            std::string color_txt =
                database_path + entry->d_name + '/' + entry_c->d_name;
            std::ifstream ifs(color_txt.c_str());
            std::string read_line_data;
            getline(ifs, read_line_data);
            ifs.close();
            //std::cout << read_line_data << std::endl;
            std::vector < double >hist;
            std::vector < std::string > hist_str =
                Util::split(read_line_data, ',');
            for (int i = 0; i < num_hist_bin; i++) {
                hist.push_back(std::stof(hist_str[i]));
            }
            colorhist_mp.insert(std::pair < std::string,
                                std::vector <
                                double > >(std::string(entry->d_name),
                                           hist));
        }
    }
    //debug 
    /*
       for (auto itr = colorhist_mp.begin(); itr != colorhist_mp.end();
       itr++) {
       std::cout << itr->first << ':' << std::endl;
       for (int i = 0; i < num_hist_bin; i++) {
       std::cout << "   " << i << " : " << itr->second[i] << std::
       endl;
       }
       }
     */
    return true;
}

bool ObjectIdentification::update_threshold_srv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  // Get parameters
  float x_min = -1.5, x_max = 1.5, y_min = -1.5, y_max = 1.5, z_min = 0.0, z_max = 1.0;
  nh_.getParam("/object_identification/x_max", x_max);
  nh_.getParam("/object_identification/x_min", x_min);
  nh_.getParam("/object_identification/y_max", y_max);
  nh_.getParam("/object_identification/y_min", y_min);
  nh_.getParam("/object_identification/z_max", z_max);
  nh_.getParam("/object_identification/z_min", z_min);
  boxFilter_min = Eigen::Vector4f(x_min, y_min, z_min, 1.0);
  boxFilter_max = Eigen::Vector4f(x_max, y_max, z_max, 1.0);

  ROS_INFO("x_max : %f, x_min : %f, y_max : %f, y_min : %f, z_max : %f, z_min : %f", x_max, x_min, y_max, y_min, z_max, z_min);

  return true;
}

bool ObjectIdentification::recogntion_object_srv(object_identification::
                                                recognition_object::Request
                                                & req,
                                                object_identification::
                                                recognition_object::Response
                                                & res)
{
    darknet_ros_msgs::BoundingBoxes result_data;
    cv::Mat recog_image;
    recogntion_object(result_data, recog_image);
    res.result = result_data;
    return true;
}


void ObjectIdentification::recogntion_object(darknet_ros_msgs::BoundingBoxes &result_data, cv::Mat &recog_image)
{
    std::vector < pcl::PointIndices > cluster_indices;
    cluster_indices.clear();
    clastering(cluster_indices);
    std::cout << "Cluster size : " << cluster_indices.size() << std::endl;

    ROS_INFO("Cluster size : %d", (int) cluster_indices.size());

    //darknet_ros_msgs::BoundingBoxes result_data;

    //cv::Mat recog_image;
    input_image.copyTo(recog_image);

    for (int i = 0; i < cluster_indices.size(); i++) {
        std::vector < double >query_hist(num_hist_bin);
        for (int j = 0; j < num_hist_bin; j++) {
            query_hist[j] = 0.0;
        }
        calc_histgram(cluster_indices[i], query_hist);
        double sim = 0.0;
        std::string name = "";
        for (auto itr = colorhist_mp.begin();
             itr != colorhist_mp.end(); itr++) {
            double work_sim = 0.0;
            for (int j = 0; j < num_hist_bin; j++) {
                work_sim += std::min(query_hist[j], itr->second[j]);
            }
            //std::cout << itr->first << " " << work_sim << std::endl;
            if (work_sim > sim) {
                sim = work_sim;
                name = itr->first;
            }
        }
        if (sim <= 0.25) {
            continue;
        }
        std::cout << i << " : " << name << " " << sim << std::endl;
        struct cluster_parameter result;
        calc_cluster_parameter(cluster_indices[i], result);
        cv::rectangle(recog_image,
                      cv::Point(result.px_min, result.py_min),
                      cv::Point(result.px_max, result.py_max),
                      cv::Scalar(0, 0, 255), 3);
        std::stringstream ss;
        ss << name << " " << std::setprecision(4) << sim;
        cv::putText(recog_image, ss.str(),
                    cv::Point(result.px_min, result.py_min),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
        darknet_ros_msgs::BoundingBox result_dat;
        result_dat.Class = name;
        result_dat.probability = sim;
        result_dat.xmin = result.px_min;
        result_dat.xmax = result.px_max;
        result_dat.ymin = result.py_min;
        result_dat.ymax = result.py_max;
        result_data.bounding_boxes.push_back(result_dat);
    }
    result_data.image_header = image_header;
    result_data.header = image_header;
    //bb_pub_.publish(result_data);
    //res.result = result_data;
    if(debug){
        cv::imshow("Recognition result", recog_image);
        cv::waitKey();
        cv::destroyAllWindows();
    }

    //return true;
}

bool ObjectIdentification::register_object_srv(object_identification::
                                              register_object::
                                              Request & req,
                                              object_identification::
                                              register_object::
                                              Response & res)
{
    ROS_INFO("register_object_srv : Start");
    std::string name = req.name;
    std::string pkg_path =
        ros::package::getPath("object_identification");
    std::string database_path = pkg_path + "/database/";
    std::string savedir_path = database_path + name;
    //std::cout << savedir_path << std::endl;

    std::vector < pcl::PointIndices > cluster_indices;
    cluster_indices.clear();
    clastering(cluster_indices);
    if (cluster_indices.size() > 0) {
        int min_idx = 0;
        struct cluster_parameter result;
        calc_cluster_parameter(cluster_indices[0], result);
        for (int i = 1; i < cluster_indices.size(); i++) {
            struct cluster_parameter result_work;
            calc_cluster_parameter(cluster_indices[i], result_work);
            if (result.dist > result_work.dist) {
                result = result_work;
                min_idx = i;
            }
        }
        cv::Mat recog_image;
        input_image.copyTo(recog_image);
        cv::rectangle(recog_image,
                      cv::Point(result.px_min, result.py_min),
                      cv::Point(result.px_max, result.py_max),
                      cv::Scalar(0, 0, 255), 3);
        cv::Mat target_image = cv::Mat(input_image,
                                       cv::Rect(result.px_min,
                                                result.py_min,
                                                result.px_max -
                                                result.px_min,
                                                result.py_max -
                                                result.py_min));
        if(debug){
            cv::imshow("recog_image", recog_image);
            //cv::imshow("target_image", target_image);
            cv::waitKey();
            cv::destroyAllWindows();
        }

        std::vector < double >color_hist(num_hist_bin);
        for (int i = 0; i < num_hist_bin; i++) {
            color_hist[i] = 0;
        }
        calc_histgram(cluster_indices[min_idx], color_hist);
        std::stringstream ss_hist;
        ss_hist << color_hist[0];
        for (int i = 1; i < num_hist_bin; i++) {
            ss_hist << "," << color_hist[i];
        }

        //File save
        mkdir(database_path.c_str(), 0777);
        mkdir(savedir_path.c_str(), 0777);
        mkdir(savedir_path.c_str(), 0777);
        int count = 0;
        while (true) {
            std::stringstream ss;
            ss << savedir_path << "/" << "input_" << std::setw(4) <<
                std::setfill('0') << count << ".png";
            std::string filename = ss.str();
            if (!Util::checkFileExistence(filename)) {
                break;
            }
            count++;
        }
        std::stringstream ss_file;
        ss_file << savedir_path << "/" << "colorhistogram_" <<
            std::setw(4) << std::setfill('0') << count << ".txt";
        std::ofstream outputfile(ss_file.str().c_str());
        outputfile << ss_hist.str();
        outputfile.close();
        ss_file.str("");
        ss_file << savedir_path << "/" << "input_" << std::setw(4) <<
            std::setfill('0') << count << ".png";
        cv::imwrite(ss_file.str().c_str(), input_image);
        ss_file.str("");
        ss_file << savedir_path << "/" << "recog_" << std::setw(4) <<
            std::setfill('0') << count << ".png";
        cv::imwrite(ss_file.str().c_str(), recog_image);
        ss_file.str("");
        ss_file << savedir_path << "/" << "target_" << std::setw(4) <<
            std::setfill('0') << count << ".png";
        cv::imwrite(ss_file.str().c_str(), target_image);
        ss_file.str("");

        cv_bridge::CvImage out_msg;
        object_identification::get_attribute srv;
        target_image.copyTo(out_msg.image);
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        srv.request.image = *out_msg.toImageMsg();
        client_attr.call(srv);
        //auto attributes = srv.response.attributes.attributes;
        //std::cout << attributes[0].attribute_name << " " << attributes[0].score << std::endl;
        res.attributes = srv.response.attributes;

    }
    ROS_INFO("register_object_srv : End");
    return true;
}


void ObjectIdentification::CinfoCb(const sensor_msgs::CameraInfo & msg)
{
    this->cinfo = msg;
}

void ObjectIdentification::RGBCb(const sensor_msgs::ImageConstPtr & msg)
{
    //std::cout << "call " << __func__ << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(input_image);
    //image_header = msg->header;
}


void ObjectIdentification::
SegmentationCb(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    //std::cout << "call " << __func__ << std::endl;
    pcl::PointCloud < pcl::PointXYZRGB > cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::copyPointCloud(cloud, *input_cloud_data_);
    pcl::copyPointCloud(cloud, *input_cloud_data_raw_);
    image_header = msg->header;

    ROS_INFO(" SegmentationCb called");
    if(colorhist_mp.size()>0){
        darknet_ros_msgs::BoundingBoxes result_data;
        cv::Mat recog_image;
        recogntion_object(result_data, recog_image);
        bb_pub_.publish(result_data);

    }
    ROS_INFO(" SegmentationCb end");

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_identification");
    std::string point_topic_name ;// "/camera/depth_registered/points";
    std::string rgb_topic_name ;// "/camera/rgb/image_color";
    std::string rgb_info_topic_name ;// "/camera/rgb/camera_info";
    ros::param::get("/object_identification/point_topic", point_topic_name);
    ros::param::get("/object_identification/image_topic", rgb_topic_name);
    ros::param::get("object_identification/camerainfo_topic", rgb_info_topic_name);
    ObjectIdentification oi(point_topic_name, rgb_topic_name, rgb_info_topic_name);
    ros::Rate spin_rate(10);

    //pcl::visualization::CloudViewer viewer("Viewer");

    while (ros::ok()) {
        ros::spinOnce();
        /*
           if (!viewer.wasStopped()) {
           viewer.showCloud(plane_segmentation.GetSegmentationCloud());
           }
         */
        spin_rate.sleep();
    }
    return 0;
}
