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


typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudRGBPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudPointNormalPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
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
PointCloudNormalPtr estimateSurfaceNormals(PointCloudRGBPtr input) {

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(input);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZRGB>());
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
std::vector<float> cvfhRecognition(PointCloudRGBPtr input) {
    std::vector<float> result;
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the CVFH descriptors.
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

    // Estimate the normals of the object.
    normals = estimateSurfaceNormals(input);

    // New KdTree to search with.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    // CVFH estimation object.
    pcl::CVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> cvfh;
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
std::vector<unsigned int> produceColorHist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud){

    unsigned int red[256];
    unsigned int green[256];
    unsigned int blue[256];
        std::vector<unsigned int> result;



        for (int i = 0; i <  cloud->size(); i++){
            pcl::PointXYZRGB p = cloud->points[i];
            float rgb = *reinterpret_cast<int*>(&p.rgb);
            unsigned int r = ((int)rgb >> 16) & 0x0000ff;
            unsigned int g = ((int)rgb >> 8)  & 0x0000ff;
            unsigned int b = ((int)rgb)       & 0x0000ff;

            // increase value in bin
            red[r]++;
            green[g]++;
            blue[b]++;
            std::cout << "rgb is: " << (int)p.r << ", " << (int)p.g << ", " << (int)p.b << "." << std::endl;


        }
        // concatenate red, green and blue entries
        for (int x = 0; x < 255; x++){
            result.push_back(red[x]);
        }
        for (int y = 0; y < 255; y++){
            result.push_back(green[y]);
        }
        for (int z = 0; z < 255; z++){
            result.push_back(blue[z]);
        }
    return result;

}


void batchPCD2histograms(std::string input) {
    std::ifstream is(input.c_str());
    std::string line;
    std::string line_trimmed;
    while (getline(is, line)) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        // sensor_msgs::PointCloud2Ptr test (new sensor_msgs::PointCloud2); // undefined reference to `ros::TimeBase<ros::Time, ros::Duration>::fromNSec(unsigned long)'
        // pcl::PCLPointCloud2Ptr input_pclpc2 (new pcl::PCLPointCloud2);
        std::vector<float> input_cvfhs_features;
        std::vector<unsigned int> input_color_features;

        // load file
        std::cout << "load file " << line << std::endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(line, *input_cloud) != 0){ // Failed to find match for field 'rgba'.

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

            //pcl::io::loadPCDFile<pcl::PointXYZRGB>(line + ".CONV.pcd", *input_cloud);
            std::cout << "cloud size is: " << input_cloud->size() << std::endl;
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