//
// Created by tammo on 31.01.18.
//


#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/cvfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudRGBAPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudPointNormalPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointIndices::Ptr PointIndices;
typedef std::vector<pcl::PointIndices> PointIndicesVector;
typedef std::vector<pcl::PointIndices::Ptr> PointIndicesVectorPtr;

typedef pcl::PointCloud<pcl::VFHSignature308>::Ptr PointCloudVFHS308Ptr;
typedef sensor_msgs::PointCloud2 SMSGSPointCloud2;

/**
 * estimating surface normals
 * @param input
 * @return
 */
PointCloudNormalPtr estimateSurfaceNormals(PointCloudRGBAPtr input) {

    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud(input);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZRGBA>());
    ne.setSearchMethod(tree);

    PointCloudNormalPtr cloud_normals(new PointCloudNormal);

    ne.setRadiusSearch(0.03); // Use all neighbors in a sphere of radius 3cm

    ne.compute(*cloud_normals);

    return cloud_normals;
}

/**
 * estimate the Features of a pointcloud using VFHSignature308
 * @param input
 * @return
 */
std::vector<float> cvfhRecognition(PointCloudRGBAPtr input) {
    std::vector<float> result;
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the CVFH descriptors.
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

    // Estimate the normals of the object.
    normals = estimateSurfaceNormals(input);

    // New KdTree to search with.
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);

    // CVFH estimation object.
    pcl::CVFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(input);
    cvfh.setInputNormals(normals);
    cvfh.setSearchMethod(kdtree);
    cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI);
    cvfh.setCurvatureThreshold(1.0);
    cvfh.setNormalizeBins(false);

    cvfh.compute(*descriptors);

    for (size_t i = 0; i < 308; i++){
        result.push_back(descriptors->points[0].histogram[i]);
    }

    std::cout << "normals OK" << std::endl;

    return result; // to vector
}

/**
 * Color Histogram from RGBA-PointCloud
 * @param input
 * @return concatenated floats (r,g,b (in that order) from Pointcloud-Points
 */
std::vector<int> produceColorHist(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud){


    //input->resize(500); // resize for vector messages

    std::vector<int> red;
    std::vector<int> green;
    std::vector<int> blue;
    std::vector<int> result;


    for (size_t i = 0; i <  cloud->size(); i++){
        pcl::PointXYZRGBA p = cloud->points[i];

        uint32_t rgba = p.rgb;// *reinterpret_cast<int*>(&p.rgb);
        uint8_t r = (rgba >> 16) & 0x0000ff;
        uint8_t g = (rgba >> 8)  & 0x0000ff;
        uint8_t b = (rgba)       & 0x0000ff;
/*
            unsigned color = *(const float *)&p.rgba;
            unsigned  r = color & 0xff;
            unsigned  g = (color >> 8) & 0xff;
            unsigned  b = (color >> 16) & 0xff;
            printf("%ul,%ul,%ul\n", r, g, b);
*/
        red.push_back(p.r);
        green.push_back(p.g);
        blue.push_back(p.b);

        std::cout << "r " << r << std::endl;
        std::cout << "g " << g << std::endl;
        std::cout << "b " << b << std::endl;

        std::cout << "r as int " << (int) r << std::endl;
        std::cout << "g as int " << (int) g << std::endl;
        std::cout << "b as int " << (int) b << std::endl;


    }

    // concatenate red, green and blue entries
    result.insert(result.begin(),red.begin(),red.end());
    result.insert(result.end(),green.begin(),green.end());
    result.insert(result.end(),blue.begin(),blue.end());

    return result;

}


void batchPCD2histograms(std::string input) {
    std::ifstream is(input.c_str());
    std::string line;
    std::string line_trimmed;
    while (getline(is, line)) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        // sensor_msgs::PointCloud2Ptr test (new sensor_msgs::PointCloud2); // undefined reference to `ros::TimeBase<ros::Time, ros::Duration>::fromNSec(unsigned long)'
        // pcl::PCLPointCloud2Ptr input_pclpc2 (new pcl::PCLPointCloud2);
        std::vector<float> input_cvfhs_features;
        std::vector<int> input_color_features;

        // load file

        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(line, *input_cloud) != 0){ // Failed to find match for field 'rgba'.

            line.erase(line.size() - 4, 4);
            std::string normals = line + "_normals_histogram.csv";
            std::string colors = line + "_colors_histogram.csv";
            std::ofstream os_normals(normals.c_str());
            std::ofstream os_colors(colors.c_str());

            // save empty .csv

            os_normals.close();
            os_colors.close();

        } else {
            //pcl::io::savePCDFile(line + ".CONV.pcd", *input_pclpc2);

            //pcl::io::loadPCDFile<pcl::PointXYZRGBA>(line + ".CONV.pcd", *input_cloud);
            line.erase(line.size() - 4, 4);
            std::string normals = line + "_normals_histogram.csv";
            std::string colors = line + "_colors_histogram.csv";
            std::ofstream os_normals(normals.c_str());
            std::ofstream os_colors(colors.c_str());


            // estimate features
            input_cvfhs_features = cvfhRecognition(input_cloud);
            std::cout << "cvfh OK" << std::endl;

            input_color_features = produceColorHist(input_cloud);
            std::cout << "colors OK" << std::endl;


            // save features to .csv

            for (int i = 0; i < input_cvfhs_features.size(); i++) {
                if (input_cvfhs_features.size() != i) {
                    os_normals << input_cvfhs_features[i] << ", ";
                } else {
                    os_normals << input_cvfhs_features[i];

                }
            }

            for (int j = 0; j < input_color_features.size(); j++) {
                if (input_color_features.size() != j){
                    os_colors << input_color_features[j] << ", ";

                } else {
                    os_colors << input_color_features[j];

                }
            }


            os_normals.close();
            os_colors.close();
        }

    }
}

int main(int argc, char** argv){
    batchPCD2histograms(argv[1]);
    return 0;
}

/**

        std::cout << "unpacked r: " << r << std::endl;
        std::cout << "r as int: " << (int) r << std::endl;
        std::cout << "unpacked g: " << g << std::endl;
        std::cout << "g as int: " << (int) g << std::endl;
        std::cout << "unpacked b: " << b << std::endl;
        std::cout << "b as int: " << (int) b << std::endl;

        std::cout << "point r: " << point.r << std::endl;
        std::cout << "r as int: " << (int) point.r << std::endl;
        std::cout << "point g: " << point.g << std::endl;
        std::cout << "g as int: " << (int) point.g << std::endl;
        std::cout << "point b: " << point.b << std::endl;
        std::cout << "b as int: " << (int) point.b << std::endl;
        std::cout << "r in vector" << red[i] << std::endl;
        std::cout << "g in vector" << green[i] << std::endl;
        std::cout << "b in vector" << blue[i] << std::endl;

        **/