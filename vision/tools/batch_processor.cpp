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

PointCloudNormalPtr cloud_normals(new PointCloudNormal);

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
std::vector<uint64_t> produceColorHist(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud){
    int red[8];
    int green[8];
    int blue[8];
    std::vector<uint64_t> result;

    // initialize all array-values with 0
    for (int i = 0; i < 8;i++){
        red[i] = 0;
        green[i] = 0;
        blue[i] = 0;
    }

    for (int i = 0; i <  cloud->size(); i++){
        pcl::PointXYZRGB p = cloud->points[i];
        // increase value in bin at given index
        if (p.r < 32){
            red[0]++;

        } else if (p.r >= 32 && p.r < 64){
            red[1]++;

        } else if (p.r >= 64 && p.r < 96){
            red[2]++;
        } else if (p.r >= 96 && p.r < 128){
            red[3]++;

        } else if (p.r >= 128 && p.r < 160){
            red[4]++;

        } else if (p.r >= 160 && p.r < 192){
            red[5]++;

        } else if (p.r >= 192 && p.r < 224){
            red[6]++;

        } else if (p.r >= 224 && p.r < 256){
            red[7]++;

        }

        if (p.g < 32){
            green[0]++;

        } else if (p.g >= 32 && p.g < 64){
            green[1]++;

        } else if (p.g >= 64 && p.g < 96){
            green[2]++;
        } else if (p.g >= 96 && p.g < 128){
            green[3]++;

        } else if (p.g >= 128 && p.g < 160){
            green[4]++;

        } else if (p.g >= 160 && p.g < 192){
            green[5]++;

        } else if (p.g >= 192 && p.g < 224){
            green[6]++;

        } else if (p.g >= 224 && p.g < 256){
            green[7]++;

        }

        if (p.b < 32){
            blue[0]++;

        } else if (p.b >= 32 && p.b < 64){
            blue[1]++;

        } else if (p.b >= 64 && p.b < 96){
            blue[2]++;
        } else if (p.b >= 96 && p.b < 128){
            blue[3]++;

        } else if (p.b >= 128 && p.b < 160){
            blue[4]++;

        } else if (p.b >= 160 && p.b < 192){
            blue[5]++;

        } else if (p.b >= 192 && p.b < 224){
            blue[6]++;

        } else if (p.b >= 224 && p.b < 256){
            blue[7]++;

        }
    }

    // concatenate red, green and blue entries
    for (int r = 0; r < 8; r++){
        result.push_back(red[r]);
    }
    for (int g = 0; g < 8; g++){
        result.push_back(green[g]);
    }
    for (int b = 0; b < 8; b++){
        result.push_back(blue[b]);
    }

    return result;

}


void batchPCD2histograms(std::string input) {
    std::ifstream is(input.c_str());
    std::string line;
    std::string line_trimmed;

    while (getline(is, line)) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>), input_sampler (new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<float> input_cvfhs_features;
        std::vector<uint64_t> input_color_features;

        // load file
        std::cout << "load file " << line << std::endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(line, *input_sampler) != 0){
            //downsample partial view
            std::cout << "input before filtering size is: " << input_sampler->size() << std::endl;

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(input_sampler);
            sor.setLeafSize(0.005f, 0.005f, 0.005f); //from 0.005 (perception) zu
            sor.filter(*input_cloud);

            //prepare files
            line.erase(line.size() - 4, 4);
            std::string normals = line + "_normals_histogram.csv";
            std::string colors = line + "_colors_histogram.csv";
            std::ofstream os_normals(normals.c_str());
            std::ofstream os_colors(colors.c_str());
            //std::string normals_list = line + "_normals_only.csv";
            //std::ofstream os_normals_list(normals_list.c_str());

            // save empty .csv

            os_normals.close();
            os_colors.close();
            //os_normals_list;

        } else {
            //downsample partial view
            std::cout << "input before filtering size is: " << input_sampler->size() << std::endl;

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(input_sampler);
            sor.setLeafSize(0.005f, 0.005f, 0.005f); //from 0.005 (perception) zu
            sor.filter(*input_cloud);
            // prepare files
            std::cout << "cloud size is: " << input_cloud->size() << std::endl;
            line.erase(line.size() - 4, 4);
            std::string normals = line + "_normals_histogram.csv";
            std::string colors = line + "_colors_histogram.csv";
            std::ofstream os_normals(normals.c_str());
            std::ofstream os_colors(colors.c_str());
            //std::string normals_list = line + "_curvature.csv";
            //std::ofstream os_normals_list(normals_list.c_str());


            // estimate features

            std::cout << "estimating cvfh features..." << std::endl;
            input_cvfhs_features = cvfhRecognition(input_cloud);

            input_color_features = produceColorHist(input_cloud);
            std::cout << "creating color histogram..." << std::endl;

                // save features to .csv
                std::cout << "feature and color histogram size ok. saving file" << std::endl;

                for (int i = 0; i < input_cvfhs_features.size(); i++) {
                    if (i == input_cvfhs_features.size() - 1) {
                        os_normals << input_cvfhs_features[i];
                    } else {
                        os_normals << input_cvfhs_features[i] << ", ";


                    }
                }


            for (int j = 0; j < input_color_features.size(); j++) {
                if (j == input_color_features.size()-1){
                    os_colors << input_color_features[j];

                } else {
                    os_colors << input_color_features[j] << ", ";


                }
            }
                /**
                // curvature of normals
                int curvatures[4] = {0,0,0,0};
                 for (int k = 0; k < cloud_normals->size(); k++) {
                     pcl::Normal p = cloud_normals->points[k];
                     if (p.curvature == 0) {
                         curvatures[0]++;
                     } else if (p.curvature > 0 && p.curvature < 0.05) {
                         curvatures[2]++;
                     } else if (p.curvature > 0.05 && p.curvature <= 0.001) {
                         curvatures[3]++;
                     } else {
                         curvatures[1]++;
                     }
                 }

                for (int l = 0; l < 4; l++) {


                    if (l == 3){
                        os_normals_list << curvatures[l];

                    } else {
                        os_normals_list << curvatures[l] << ", ";


                    }
                }
                 }
                 **/
                // os_normals_list.close();
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
