
#include "perception.h"
#include "../saving/saving.h"


geometry_msgs::PointStamped centroid_stamped_perc;

std::string error_message_perc;

/** -------------------------- BEGIN OF IMPLEMENTATION ---------------------- **/
class CloudTransformer {
public:
    explicit CloudTransformer(ros::NodeHandle nh)
            : nh_(nh) {
        // Define Publishers and Subscribers here
        //pcl_sub_ = nh_.subscribe(REAL_KINECT_POINTS_FRAME, 10, &CloudTransformer::transform, this);
        //pcl_pub_ = nh_.advertise<PointCloudXYZ>("/vision_main/point_cloud_odom_combined", 1); // <sensor_msgs::PointCloud2>

        buffer_.reset(new PointCloudXYZ); // (new sensor_msgs::PointCloud2)
        buffer_->header.frame_id = "odom_combined";
    }

    PointCloudXYZPtr transform(const PointCloudXYZPtr cloud, std::string target_frame,
                               std::string source_frame) // sensor_msgs::PointCloud2ConstPtr&
    {
        ROS_INFO("TRYING TO TRANSFORM...");
        try {
            // Usually: target_frame = "odom_combined", source_frame = "head_mount_kinect_ir_optical_frame"
            listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
            listener_.lookupTransform(target_frame, source_frame, ros::Time(0), stamped_transform_);
            tf::transformTFToEigen(stamped_transform_, transform_eigen_);
            pcl::transformPointCloud(*cloud, *buffer_, transform_eigen_);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        //savePointCloudXYZNamed(cloud, "before_transforming");
        //savePointCloudXYZNamed(buffer_, "transformed");
        ROS_INFO("TRANSFORMED!");
        return buffer_;
    }

    PointCloudXYZPtr removeBelowPlane(PointCloudXYZPtr input) {
        PointCloudXYZPtr cloud_odom_combined(new PointCloudXYZ);

        cloud_odom_combined = CloudTransformer::transform(input, "odom_combined", "head_mount_kinect_ir_optical_frame");
        ROS_INFO("TRANSFORMED!");

        // Find the bottom plane
        PointIndices planeIndices(new pcl::PointIndices);
        ROS_INFO("FINDING PLANE");
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setInputCloud(cloud_odom_combined);
        segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(500); // Default is 50 and could be problematic
        segmentation.setAxis(Eigen::Vector3f(0, 0, 1));
        segmentation.setEpsAngle(5.0f * (M_PI / 180.0f)); // plane can be within 30 degrees of X-Z plane
        segmentation.setDistanceThreshold(0.02);  // Distance to model points
        segmentation.setOptimizeCoefficients(true);
        segmentation.segment(*planeIndices, *coefficients);

        PointCloudXYZPtr plane(new PointCloudXYZ);
        plane = extractCluster(cloud_odom_combined, planeIndices, false); // extract the plane

        // savePointCloudXYZNamed(plane, "ground_plane");

        float min_height = plane->points[0].z;
        for (int i = 1; i < plane->points.size(); i++) { // Search for the lowest point on the plane
            if (plane->points[i].z < min_height) {
                min_height = plane->points[i].z;
            }
        }

        PointCloudXYZPtr result(new PointCloudXYZ);
        ROS_INFO("Plane height: %f", min_height);
        ROS_INFO("STARTING PASSTHROUGH FILTER");
        pcl::PassThrough<pcl::PointXYZ> pass_above; // Filter out all points below the min_height
        pass_above.setInputCloud(cloud_odom_combined);
        pass_above.setFilterFieldName("z");
        pass_above.setFilterLimits(min_height, 5.00);
        pass_above.setKeepOrganized(false);
        pass_above.filter(*result);

        savePointCloudXYZNamed(result, "aaaaa");

        return result; // THIS POINTCLOUD IS STILL IN ODOM_COMBINED!

    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    //ros::Publisher pcl_pub_;
    tf::TransformListener listener_;
    tf::StampedTransform stamped_transform_;
    tf::Transform test_transform_;
    Eigen::Affine3d transform_eigen_;
    PointCloudXYZPtr buffer_; // sensor_msgs::PointCloud2::Ptr
}; // End of class CloudTransformer

CloudContainer::CloudContainer() {

    void setInputCloud(PointCloudXYZPtr input);

    void setObjectClouds(std::vector<sensor_msgs::PointCloud2> object_clouds);



};

void CloudContainer::setInputCloud(PointCloudXYZPtr input){
    kinect = input;
}
void CloudContainer::setObjectClouds(std::vector<sensor_msgs::PointCloud2> object_clouds){
    objects = object_clouds;
}


// -- END OF CLASSES -- //


/**
 * Find the object!
 * @param kinect
 */
std::vector<sensor_msgs::PointCloud2> findCluster(PointCloudXYZPtr kinect, ros::NodeHandle n) {

    std::vector<sensor_msgs::PointCloud2> result;

    CloudTransformer transform_cloud(n);

    // the 'f' in the identifier stands for filtered

    PointCloudXYZPtr cloud_plane(new PointCloudXYZ), cloud_cluster(new PointCloudXYZ), cloud_cluster2(
            new PointCloudXYZ), cloud_f(
            new PointCloudXYZ), cloud_3df(
            new PointCloudXYZ), cloud_voxelgridf(new PointCloudXYZ), cloud_mlsf(new PointCloudXYZ), cloud_prism(
            new PointCloudXYZ), cloud_final(new PointCloudXYZ);

    PointIndices plane_indices(new pcl::PointIndices), plane_indices2(new pcl::PointIndices), prism_indices(
            new pcl::PointIndices);


    if (kinect->points.size() <
        500)                              // if PR2 is not looking at anything
    {
        ROS_ERROR("Input from kinect is empty");
        error_message_perc = "Cloud empty. ";
        centroid_stamped_perc = findCenterGazebo();              // Use gazebo data instead
    } else {
        ROS_INFO("Starting Cluster extraction");

        cloud_3df = apply3DFilter(kinect, 0.4, 0.4, 1.5);   // passthrough filter
        cloud_voxelgridf = voxelGridFilter(cloud_3df);      // voxel grid filter
        cloud_mlsf = mlsFilter(cloud_voxelgridf);           // moving least square filter
        cloud_cluster = cloud_mlsf;                         // cloud_f set after last filtering function is applied

        transform_cloud.removeBelowPlane(cloud_cluster);

        // While a segmented plane would be larger than 500 points, segment it.
        bool loop_plane_segmentations = true;
        int amount_plane_segmentations = 0;
        while (loop_plane_segmentations) {
            plane_indices = estimatePlaneIndices(cloud_cluster);
            if (plane_indices->indices.size() > 500)         // is the extracted plane big enough?
            {
                ROS_INFO("plane_indices: %lu", plane_indices->indices.size());
                ROS_INFO("cloud_cluster: %lu", cloud_cluster->points.size());
                cloud_cluster = extractCluster(cloud_cluster, plane_indices, true); // actually extract the object
                amount_plane_segmentations++;
            } else loop_plane_segmentations = false;          // if not big enough, stop looping.
        }
        ROS_INFO("Extracted %d planes!", amount_plane_segmentations);

        cloud_final = outlierRemoval(cloud_cluster);

        /** Speichere Zwischenergebenisse **/

        /**
        savePointCloudXYZNamed(cloud_3df, "1_cloud_3d_filtered");
        savePointCloudXYZNamed(cloud_voxelgridf, "2_cloud_voxelgrid_filtered");
        savePointCloudXYZNamed(cloud_mlsf, "3_cloud_mls_filtered");
        savePointCloudXYZNamed(cloud_cluster, "4_cloud_cluster");
        savePointCloudXYZNamed(cloud_prism, "6_cloud_prism");
        savePointCloudXYZNamed(cloud_cluster2, "7_cluster_2");
        savePointCloudXYZNamed(cloud_final, "result");
        **/


        ROS_INFO("%sExtraction OK", "\x1B[32m");

        if (cloud_cluster->points.size() == 0) {
            ROS_ERROR("Extracted Cluster is empty");
            error_message_perc = "Final extracted cluster was empty. ";
            centroid_stamped_perc = findCenterGazebo(); // Use gazebo data instead
        }

        error_message_perc = "";


        // convert clustered objects
        sensor_msgs::PointCloud2 pcloud2_msg;
        pcl::toROSMsg(*cloud_final, pcloud2_msg);

        result[0] = pcloud2_msg;                            // add clustered objects to result

        return result;

    }
}

/**
 * Returns a fake point (not estimated) with its origin in the model in simulation.
 * It is only called, when findCluster() cannot find a point in simulation
 * @return
 */
geometry_msgs::PointStamped
findCenterGazebo() {

    return centroid_stamped_perc;
}

/**
 * finding the geometrical center of a given pointcloud
 * @param object_cloud
 * @return
 */
std::vector<geometry_msgs::PointStamped> findCenter(const std::vector<sensor_msgs::PointCloud2> object_clouds_in) {
    std::vector<geometry_msgs::PointStamped> result;

    // convert pointclouds
    std::vector<PointCloudXYZPtr> object_clouds;
    PointCloudXYZPtr temp(new PointCloudXYZ);
    for (int i = 0; i < object_clouds_in.size(); i++) {
        pcl::fromROSMsg(object_clouds_in[i], *temp);
        object_clouds[i] = temp;
    }

    // calculate centroids
    for (auto object_cloud : object_clouds) {
        if (object_cloud->points.size() != 0) {
            int cloud_size = object_cloud->points.size();

            Eigen::Vector4f centroid;

            pcl::compute3DCentroid(*object_cloud, centroid);

            centroid_stamped_perc.point.x = centroid.x();
            centroid_stamped_perc.point.y = centroid.y();
            centroid_stamped_perc.point.z = centroid.z();

            ROS_INFO("%sCURRENT CLUSTER CENTER\n", "\x1B[32m");
            ROS_INFO("\x1B[32mX: %f\n", centroid_stamped_perc.point.x);
            ROS_INFO("\x1B[32mY: %f\n", centroid_stamped_perc.point.y);
            ROS_INFO("\x1B[32mZ: %f\n", centroid_stamped_perc.point.z);
            centroid_stamped_perc.header.frame_id = "/head_mount_kinect_ir_optical_frame";

            result.push_back(centroid_stamped_perc);
        } else {
            ROS_ERROR("CLOUD EMPTY. NO POINT EXTRACTED");
        }
    }
}

/**
 * estimating surface normals
 * @param input
 * @return
 */
PointCloudNormalPtr estimateSurfaceNormals(PointCloudXYZPtr input) {
    ROS_INFO("ESTIMATING SURFACE NORMALS");

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    PointCloudNormalPtr cloud_normals(new PointCloudNormal);

    ne.setRadiusSearch(0.03); // Use all neighbors in a sphere of radius 3cm

    ne.compute(*cloud_normals);
    return cloud_normals;
}

/**
 * apply a passthrough Filter to all dimensions, reducing points and
 * narrowing Field of Vision
 * @param input
 * @param x
 * @param y
 * @param z
 * @return
 */
PointCloudXYZPtr apply3DFilter(PointCloudXYZPtr input, float x, float y,
                               float z) {

    //TODO test filtering here
    ROS_INFO("Starting passthrough filter");
    PointCloudXYZPtr input_after_x(new PointCloudXYZ),
            input_after_xy(new PointCloudXYZ), input_after_xyz(new PointCloudXYZ);
    /** Create the filtering object **/
    // Create the filtering object (x-axis)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);
    pass.setKeepOrganized(false);
    pass.filter(*input_after_x);

    // Create the filtering object (y-axis)
    pass.setInputCloud(input_after_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);
    pass.setKeepOrganized(false);
    pass.filter(*input_after_xy);

    // Create the filtering object (z-axis)
    pass.setInputCloud(input_after_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
            0.0, z); // no negative range (the pr2 can't look behind its head)
    pass.setKeepOrganized(false);
    pass.filter(*input_after_xyz);

    if (input_after_xyz->points.size() == 0) {
        ROS_ERROR("Cloud empty after passthrough filtering");
        error_message_perc = "Cloud was empty after filtering. ";
        centroid_stamped_perc = findCenterGazebo(); // Use gazebo data instead
    }

    return input_after_xyz;
}

/**
 * estimating plane indices
 * @param input
 * @return
 */
PointIndices estimatePlaneIndices(PointCloudXYZPtr input) {
    ROS_INFO("Starting plane indices estimation");
    PointIndices planeIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;

    segmentation.setInputCloud(input);
    //segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // PERPENDICULAR
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    //segmentation.setMaxIterations(500); // Default is 50 and could be problematic
    //segmentation.setAxis(Eigen::Vector3f(0,1,0));
    //segmentation.setEpsAngle(30.0); // plane can be within 30 degrees of X-Z plane - 30.0f * (M_PI/180.0f)
    segmentation.setDistanceThreshold(0.01); // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);


    if (planeIndices->indices.size() == 0) {
        ROS_ERROR("No plane (indices) found");
        error_message_perc = "No plane found. ";
        centroid_stamped_perc = findCenterGazebo(); // Use gazebo data instead
    }

    return planeIndices;
}

/**
 * prism segmentation
 * @param input_cloud
 * @param plane
 * @return
 */
PointIndices prismSegmentation(PointCloudXYZPtr input_cloud, PointCloudXYZPtr plane) {
    PointCloudXYZPtr plane_hull = plane;
    ROS_INFO("Starting prism segmentation...");
    pcl::ConvexHull<pcl::PointXYZ> hull;
    PointIndices prism_indices(new pcl::PointIndices);
    hull.setInputCloud(input_cloud);
    hull.reconstruct(*plane_hull);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(input_cloud);
    prism.setInputPlanarHull(plane);
    prism.setHeightLimits(0, 2); // Get everything up to 2 meters above the plane
    prism.segment(*prism_indices);

    return prism_indices;
}

/**
 * extract a pointcloud by indices from an input pointcloud
 * @param input
 * @param indices
 * @param negative
 * @return
 */
PointCloudXYZPtr extractCluster(PointCloudXYZPtr input, PointIndices indices, bool negative) {
    ROS_INFO("CLUSTER EXTRACTION");
    PointCloudXYZPtr objects(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*objects);
    return objects;
}

/**
 * Filtering the input cloud with a moving least squares algorithm
 * @param input
 * @return
 */
PointCloudXYZPtr mlsFilter(PointCloudXYZPtr input) {

    int poly_ord = 1;

    PointCloudXYZPtr result(new PointCloudXYZ);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(input);
    mls.setPolynomialOrder(poly_ord); // the lower the smoother, the higher the more exact
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    mls.process(mls_points);

    for (int i = 0; i < mls_points.size(); i++) {
        pcl::PointXYZ point;

        point.x = mls_points.at(i).x;
        point.y = mls_points.at(i).y;
        point.z = mls_points.at(i).z;
        result->push_back(point);
    }

    return result;
}


/**
 * Filtering the input cloud with a voxel grid
 * @param input
 * @return
 */
PointCloudXYZPtr voxelGridFilter(PointCloudXYZPtr input) {
    PointCloudXYZPtr result(new PointCloudXYZ);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*result);
    return result;
}

PointCloudXYZPtr outlierRemoval(PointCloudXYZPtr input) {
    PointCloudXYZPtr cloud_filtered(new PointCloudXYZ);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(25);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

