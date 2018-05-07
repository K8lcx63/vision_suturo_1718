#include "perception.h"

int best_ia_index = 0;

std::string mesh_array[] = {"cup_eco_orange.pcd",
                            "edeka_red_bowl.pcd",
                            "hela_curry_ketchup.pcd",
                            "ja_milch.pcd",
                            "kellogs_toppas_mini.pcd",
                            "koelln_muesli_knusper_honig_nuss.pcd",
                            "pringles_paprika.pcd",
                            "pringles_salt.pcd",
                            "sigg_bottle.pcd",
                            "tomato_sauce_oro_di_parma.pcd"};

enum mesh_enum {
    CUP_ECO_ORANGE,
    EDEKA_RED_BOWL,
    HELA_CURRY_KETCHUP,
    JA_MILCH,
    KELLOGS_TOPPAS_MINI,
    KOELLN_MUESLI_KNUSPER_HONIG_NUSS,
    PRINGLES_PAPRIKA,
    PRINGLES_SALT,
    SIGG_BOTTLE,
    TOMATO_SAUCE_ORO_DI_PARMA
};

Eigen::Matrix<float,4,4> rot_mat;

PointCloudRGBPtr cloud_global(new PointCloudRGB);
PointCloudRGBPtr cloud_perceived(new PointCloudRGB);
PointCloudRGBPtr cloud_aligned(new PointCloudRGB);
PointCloudRGBPtr cloud_mesh(new PointCloudRGB);
geometry_msgs::PoseStamped pose_global;

Eigen::Matrix4f global_first_transformation;
Eigen::Vector4f global_centroid;



std::string error_message; // Used by the objects_information service
tf::Matrix3x3 global_tf_rotation;

/**
 * Applies all the filters to a PointCloud.
 * @param kinect PointCloud
 * @return Preprocessed PointCloud
 */
PointCloudRGBPtr preprocessCloud(PointCloudRGBPtr kinect) {
    PointCloudRGBPtr cloud_3df(new PointCloudRGB),
            cloud_voxelgridf(new PointCloudRGB),
            cloud_mlsf(new PointCloudRGB),
            cloud_prism(new PointCloudRGB),
            cloud_preprocessed(new PointCloudRGB);
    cloud_3df = apply3DFilter(kinect, 0.4, 0.4, 1.5);   // passthrough filter
// std::cout << "after 3dfilter cluster is of size: " << cloud_3df->size() << std::endl;
    cloud_voxelgridf = voxelGridFilter(cloud_3df);      // voxel grid filter
// std::cout << "after vgfilter cluster is of size: " << cloud_voxelgridf->size() << std::endl;
    cloud_mlsf = mlsFilter(cloud_voxelgridf);           // moving least square filter
//std::cout << "after mlsfilter cluster is of size: " << cloud_mlsf->size() << std::endl;
    cloud_preprocessed = cloud_mlsf; // cloud_f set after last filtering function is applied
    return cloud_preprocessed;
}

/**
 * Segment planes that aren't relevant to the objects.
 * @param cloud_cluster
 */
PointCloudRGBPtr segmentPlanes(PointCloudRGBPtr cloud_cluster) {
    // While a segmented plane would be larger than plane_size_threshold points, segment it.
    bool loop_segmentations = true;
    int segmentations_amount = 0;
    int plane_size_threshold = 8000;
    PointIndices plane_indices(new pcl::PointIndices);
    for (int n = 0; loop_segmentations; n++) {
        plane_indices = estimatePlaneIndices(cloud_cluster);
        if (plane_indices->indices.size() > plane_size_threshold)         // is the extracted plane big enough?
        {
            ROS_INFO("plane_indices: %lu", plane_indices->indices.size());
            ROS_INFO("cloud_cluster: %lu", cloud_cluster->points.size());
            cloud_cluster = extractCluster(cloud_cluster, plane_indices, true); // actually extract the object
            n++;
        } else loop_segmentations = false;          // if not big enough, stop looping.
        segmentations_amount = n;
    }
    ROS_INFO("Extracted %d planes!", segmentations_amount);
    return cloud_cluster;
}

/**
 * Find the objects.
 * @param kinect
 * @return
 */
std::vector<PointCloudRGBPtr> findCluster(PointCloudRGBPtr kinect) {

    ros::NodeHandle n;
    std::vector<PointCloudRGBPtr> result;
    CloudTransformer transform_cloud(n);
    PointCloudRGBPtr cloud_cluster(new PointCloudRGB),
            cloud_preprocessed(new PointCloudRGB);
    PointIndices
            plane_indices2(new pcl::PointIndices),
            prism_indices(new pcl::PointIndices);

    ROS_INFO("Starting Cluster extraction");

    cloud_preprocessed = preprocessCloud(kinect);

    cloud_preprocessed = transform_cloud.extractAbovePlane(cloud_preprocessed);
    cloud_cluster = cloud_preprocessed;

    cloud_cluster = segmentPlanes(cloud_cluster);
    ROS_INFO("Points after segmentation: %lu", cloud_cluster->points.size());
    cloud_global = cloud_cluster;

    /*
     * We currently don't need outlierRemoval here, because euclideanClusterExtraction
     * already has a set minimum point value, which causes smaller clusters / amounts of outliers  to be extracted
     * anyway.
     */

    // Split cloud_final into one PointCloud per object

    result = euclideanClusterExtraction(cloud_cluster);

    ROS_INFO("CALCULATED RESULT!");


    if (cloud_global->points.size() == 0) {
        ROS_ERROR("Extracted Cluster is empty");
        error_message = "Final extracted cluster was empty. ";
    } else {
        ROS_INFO("%sExtraction OK", "\x1B[32m");
        error_message = "";
    }



    /*
    for (int i = 0; i < result.size(); i++){
        std::stringstream obj_files;
        obj_files << "object_" << i;
        savePointCloudRGBNamed(result[i], obj_files.str());
    }
    */


    return result;

}

/**
 * Finds the geometrical center and rotation of an object.
 * @param The pointcloud object_cloud
 * @return The pose of the object contained in object_cloud
 */
geometry_msgs::PoseStamped findPose(const PointCloudRGBPtr input, std::string label) {
    // instantiate objects for results
    PointCloudRGBPtr aligned_cloud(new PointCloudRGB),
                     icp_cloud(new PointCloudRGB),
            mesh_orig(new PointCloudRGB),
            mesh(new PointCloudRGB);
    geometry_msgs::PoseStamped current_pose,
                               map_pose;
    tf::Quaternion quat_tf,
                   quat_tf_map;
    geometry_msgs::QuaternionStamped quat_msg,
                                     quat_msg_map;
    Eigen::Vector4f centroid, centroid_map;
    double x,y,z;
    tf::TransformListener t_listener;


    // add header and time
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = ros::Time(0);

    // Calculate quaternions
    mesh = getTargetByLabel(label, centroid);

    std::string map = "map";
    std::string kinect_frame = "head_mount_kinect_rgb_optical_frame";

    cloud_mesh = mesh;
    ROS_INFO("Alignment: mesh to cluster");

    // initial alignment
    aligned_cloud = iterativeClosestPoint(mesh, input);
    cloud_aligned = aligned_cloud;


    // create original quaternion
    global_tf_rotation.getEulerYPR(z, y, x);
    quat_tf.setEuler(z, y, x);
    quat_tf.normalize();

    quat_msg.header.frame_id = "map";
    quat_msg.quaternion.x = quat_tf.x();
    quat_msg.quaternion.y = quat_tf.y();
    quat_msg.quaternion.z = quat_tf.z();
    quat_msg.quaternion.w = quat_tf.w();

    // calculate and set centroid from mesh
    pcl::compute3DCentroid(*aligned_cloud, centroid);

    geometry_msgs::PointStamped p1,p2;
    p1.header.frame_id = kinect_frame;
    p1.point.x = centroid.x();
    p1.point.y = centroid.y();
    p1.point.z = centroid.z();
    t_listener.transformPoint(map, p1,p2);

    current_pose.pose.position.x = p2.point.x;
    current_pose.pose.position.y = p2.point.y;
    current_pose.pose.position.z = p2.point.z;



    // set original quaternion
    current_pose.pose.orientation = quat_msg.quaternion;

/*
    std::string map = "map";
    t_listener.transformPose(map, current_pose, map_pose);
    map_pose.header.frame_id = map;
    map_pose.header.stamp = ros::Time(0);
    //current_pose = map_pose;

    quat_tf_map.setEuler(z, y+M_PI, x);
    quat_tf_map.normalize();

    // create message quaternion
    //quat_msg_map.header.frame_id = "map";
    quat_msg_map.quaternion.x = quat_tf_map.x();
    quat_msg_map.quaternion.y = quat_tf_map.y();
    quat_msg_map.quaternion.z = quat_tf_map.z();
    quat_msg_map.quaternion.w = quat_tf_map.w();

    map_pose.pose.orientation = quat_msg_map.quaternion;


    map_pose.pose.position.x = centroid.x();
    map_pose.pose.position.y = centroid.y();
    map_pose.pose.position.z = centroid.z();


    pose_global = map_pose;
*/
    pose_global = current_pose;
    return current_pose;
}

/**
 * Estimates surface normals.
 * @param Pointcloud input
 * @return The estimated surface normals of the input Pointcloud
 */
PointCloudNormalPtr estimateSurfaceNormals(PointCloudRGBPtr input) {
    ROS_INFO("ESTIMATING SURFACE NORMALS");


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
 * Applies a PassThrough filter to all dimensions, reducing points and
 * narrowing field of vision.
 * @param input Pointcloud
 * @param x
 * @param y
 * @param z
 * @return Filtered Pointcloud
 */
PointCloudRGBPtr apply3DFilter(PointCloudRGBPtr input,
                               float x,
                               float y,
                               float z) {


    ROS_INFO("Starting passthrough filter");
    PointCloudRGBPtr input_after_x(new PointCloudRGB),
            input_after_xy(new PointCloudRGB), input_after_xyz(new PointCloudRGB);
    /** Create the filtering object **/
    // Create the filtering object (x-axis)
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);

    //pass.setUserFilterValue(0.0f);
    pass.setKeepOrganized(false);
    pass.filter(*input_after_x);

    // Create the filtering object (y-axis)
    pass.setInputCloud(input_after_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);

    //pass.setUserFilterValue(0.0f);
    pass.setKeepOrganized(false);
    pass.filter(*input_after_xy);

    // Create the filtering object (z-axis)
    pass.setInputCloud(input_after_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
            0.0, z); // no negative range (the pr2 can't look behind its head)

    //pass.setUserFilterValue(0.0f);
    pass.setKeepOrganized(true);
    pass.filter(*input_after_xyz);

    if (input_after_xyz->points.size() == 0) {
        ROS_ERROR("Cloud empty after passthrough filtering");
        error_message = "Cloud was empty after filtering. ";
    }


    return input_after_xyz;
}

/**
 * Estimates plane indices of a PointCloud.
 * @param input PointCloud
 * @return Indices of the plane points in the PointCloud.
 */
PointIndices estimatePlaneIndices(PointCloudRGBPtr input) {

    ROS_INFO("Starting plane indices estimation");
    PointIndices planeIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;

    segmentation.setInputCloud(input);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01); // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);


    if (planeIndices->indices.size() == 0) {
        ROS_ERROR("No plane (indices) found");
        error_message = "No plane found. ";
    }

    return planeIndices;
}

/**
 * Extracts a PointCloud from an input PointCloud, using indices.
 * @param input PointCloud
 * @param indices
 * @param bool negative to decide whether to return all points fulfilling the indices,
 * or all points not fulfilling the indices.
 * @return Extracted PointCloud
 */
PointCloudRGBPtr extractCluster(PointCloudRGBPtr input,
                                PointIndices indices,
                                bool negative) {
    ROS_INFO("CLUSTER EXTRACTION");
    PointCloudRGBPtr result(new PointCloudRGB);

    PointCloudRGBPtr objects(new PointCloudRGB);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(input);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.setKeepOrganized(false);
    extract.filter(*objects);
    ROS_INFO("CLUSTER EXTRACTION COMPLETED!");

    return objects;
}

/**
 * Filters the input cloud with a moving least squares algorithm.
 * @param PointCloud input
 * @return The filtered PointCloud
 */
PointCloudRGBPtr mlsFilter(PointCloudRGBPtr input) {
    ROS_INFO("MLS Filter!");
    PointCloudRGBPtr result(new PointCloudRGB);

    int poly_ord = 1;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(input);
    mls.setPolynomialOrder(poly_ord); // the lower the smoother, the higher the more exact
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    mls.process(mls_points);

    for (int i = 0; i < mls_points.size(); i++) {
        pcl::PointXYZRGB point;

        point.x = mls_points.points[i].x;
        point.y = mls_points.points[i].y;
        point.z = mls_points.points[i].z;
        point.r = input->points[i].r;
        point.g = input->points[i].g;
        point.b = input->points[i].b;
        point.rgb = input->points[i].rgb;
        point.rgba = input->points[i].rgba;
        result->push_back(point);
    }
    ROS_INFO("Finished MLS Filter!");
    return result;
}


/**
 * Filters the input cloud with a voxel grid filter.
 * @param PointCloud input
 * @return Filtered PointCloud
 */
PointCloudRGBPtr voxelGridFilter(PointCloudRGBPtr input) {
    PointCloudRGBPtr result(new PointCloudRGB);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(*result);
    return result;
}

/**
 * Removes statistical outliers from a PointCloud.
 * @param input PointCloud
 * @return Filtered PointCloud
 */
PointCloudRGBPtr outlierRemoval(PointCloudRGBPtr input) {
    PointCloudRGBPtr cloud_filtered(new PointCloudRGB);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(2.0);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

/**
 * Estimates features of an object in a PointCloud using VFHSignature308.
 * @param input PointCloud
 * @return VFHSignature308 Features
 */
PointCloudVFHS308Ptr cvfhRecognition(PointCloudRGBPtr input) {
    ROS_INFO("CVFH Recognition!");
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the CVFH descriptors.
    PointCloudVFHS308Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

    // Estimate the normals of the object.
    normals = estimateSurfaceNormals(input);

    // New KdTree to search with.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    // CVFH estimation object.
    pcl::CVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(input);
    cvfh.setInputNormals(normals);
    cvfh.setSearchMethod(kdtree);
    // Set the maximum allowable deviation of the normals,
    // for the region segmentation step.
    cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
    // Set the curvature threshold (maximum disparity between curvatures),
    // for the region segmentation step.
    cvfh.setCurvatureThreshold(1.0);
    // Set to true to normalize the bins of the resulting histogram,
    // using the total number of points. Note: enabling it will make CVFH
    // invariant to scale just like VFH, but the authors encourage the opposite.
    cvfh.setNormalizeBins(false);

    cvfh.compute(*descriptors);

    //float x [308] = descriptors->points[0].histogram; // Save calculated histogram in a float array
    //std::vector<float> result(x, x + sizeof x / sizeof x[0]);
    //std::vector<float> result(std::begin(descriptors->points[0].histogram), std::end(descriptors->points[0].histogram)); // Array to vector

    return descriptors; // to vector
}

/**
 * Seperates clusters from each other using euclidean cluster extraction.
 * @param input PointCloud
 * @return Seperated PointClouds
 */
std::vector<PointCloudRGBPtr> euclideanClusterExtraction(PointCloudRGBPtr input) {
    ROS_INFO("Euclidean Cluster Extraction!");
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input);

    PointIndicesVector cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01); // 4cm
    ec.setMinClusterSize(300);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ROS_INFO("BEFORE EXTRACT");
    ec.extract(cluster_indices);


    ROS_INFO("AFTER EXTRACT");

    std::vector<PointCloudRGBPtr> result;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
        PointCloudRGBPtr cloud_cluster(new PointCloudRGB);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(input->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
                  << std::endl;
        j++;

        result.push_back(cloud_cluster);
    }


    ROS_INFO("Finished Euclidean Cluster Extraction!");
    return result;
}

/**
 * Calculating the alignment of an object to a certain target using sample consensus.
 * @param PointClouds objects
 * @param VFHSignature308 features
 * @param target PointCloud
 * @return Output PointCloud
 */
PointCloudRGBPtr SACInitialAlignment(PointCloudRGBPtr input, PointCloudRGBPtr target) {

    PointCloudRGBPtr result(new PointCloudRGB);
    PointCloudVFHS308Ptr input_feats(new pcl::PointCloud<pcl::VFHSignature308>), target_feats(new pcl::PointCloud<pcl::VFHSignature308>);
    input_feats = cvfhRecognition(input);
    target_feats = cvfhRecognition(target);

    std::cout << "size of input: "  << input->points.size() << std::endl;
    std::cout << "size of input features: "  << input_feats->points.size() << std::endl;

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::VFHSignature308> sac_ia;

    sac_ia.setInputSource(input);
    sac_ia.setSourceFeatures(input_feats);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_feats);
    sac_ia.setMinSampleDistance(0.01);
    sac_ia.setRANSACIterations(50000);
    PointCloudRGB registration_output;
    sac_ia.align(registration_output);

    // get fitness score with max squared distance for correspondence
    float fitness_score = (float) sac_ia.getFitnessScore(0.01f * 0.01f);
    Eigen::Matrix4f transformation_matrix = sac_ia.getFinalTransformation();


    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = transformation_matrix.block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformation_matrix.block<3, 1>(0, 3);




// possible to create Eigen::affine3f from translation and rotation
    // then to tf::transform
    // ROS is right handed rotation (usually) look for ros rotation convention
    // khan academy
    // tf::Transform::setBasis(Rotation Matrix)
    // be careful with configuration of objects
    // pcl::transformPointCloud(*input, registration_output, transformation_matrix);


    //ROS_INFO("Estimate transformation");
    // estimate rigid transformation from input (object cluster) to mesh (object mesh pcd)
    tf::Matrix3x3 tf_rotation(transformation_matrix(0,0),
                              transformation_matrix(0,1),
                              transformation_matrix(0,2),
                              transformation_matrix(1,0),
                              transformation_matrix(1,1),
                              transformation_matrix(1,2),
                              transformation_matrix(2,0),
                              transformation_matrix(2,1),
                              transformation_matrix(2,2));

    global_tf_rotation = tf_rotation;

    *result = registration_output;


    return result;
}

/**
 * Calculates the alignment of an object to a certain target using iterative closest point algorithm.
 * @param input PointCloud
 * @param target PointCloud
 * @return output PointCloud
 */
PointCloudRGBPtr iterativeClosestPoint(PointCloudRGBPtr input,
                                       PointCloudRGBPtr target) {

    PointCloudRGBPtr result (new PointCloudRGB), input_centroid (new PointCloudRGB);


    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(input);
    icp.setInputTarget(target);
    icp.setRANSACIterations(20000);
    icp.setMaximumIterations(20000);
    icp.setMaxCorrespondenceDistance(3.0f); // set Max distance btw source <-> target to include into estimation

    PointCloudRGBPtr final(new PointCloudRGB);
    icp.align(*final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    Eigen::Matrix4f transformation = icp.getFinalTransformation();

    tf::Matrix3x3 tf_rotation(transformation(0,0),
                              transformation(0,1),
                              transformation(0,2),
                              transformation(1,0),
                              transformation(1,1),
                              transformation(1,2),
                              transformation(2,0),
                              transformation(2,1),
                              transformation(2,2));


    global_tf_rotation = tf_rotation;



    return final;
}

/**
 * Gets the color histogram from a PointCloud.
 * @param Input PointCloud cloud
 * @return Concatenated floats (r,g,b) from PointCloud points
 */
std::vector<uint64_t> produceColorHist(PointCloudRGBPtr cloud) {
    int red[8];
    int green[8];
    int blue[8];
    std::vector<uint64_t> result;

    // initialize all array-values with 0
    for (int i = 0; i < 8; i++) {
        red[i] = 0;
        green[i] = 0;
        blue[i] = 0;
    }

    for (int i = 0; i < cloud->size(); i++) {
        pcl::PointXYZRGB p = cloud->points[i];
        // increase value in bin at given index
        if (p.r < 32) {
            red[0]++;

        } else if (p.r >= 32 && p.r < 64) {
            red[1]++;

        } else if (p.r >= 64 && p.r < 96) {
            red[2]++;
        } else if (p.r >= 96 && p.r < 128) {
            red[3]++;

        } else if (p.r >= 128 && p.r < 160) {
            red[4]++;

        } else if (p.r >= 160 && p.r < 192) {
            red[5]++;

        } else if (p.r >= 192 && p.r < 224) {
            red[6]++;

        } else if (p.r >= 224 && p.r < 256) {
            red[7]++;

        }

        if (p.g < 32) {
            green[0]++;

        } else if (p.g >= 32 && p.g < 64) {
            green[1]++;

        } else if (p.g >= 64 && p.g < 96) {
            green[2]++;
        } else if (p.g >= 96 && p.g < 128) {
            green[3]++;

        } else if (p.g >= 128 && p.g < 160) {
            green[4]++;

        } else if (p.g >= 160 && p.g < 192) {
            green[5]++;

        } else if (p.g >= 192 && p.g < 224) {
            green[6]++;

        } else if (p.g >= 224 && p.g < 256) {
            green[7]++;

        }

        if (p.b < 32) {
            blue[0]++;

        } else if (p.b >= 32 && p.b < 64) {
            blue[1]++;

        } else if (p.b >= 64 && p.b < 96) {
            blue[2]++;
        } else if (p.b >= 96 && p.b < 128) {
            blue[3]++;

        } else if (p.b >= 128 && p.b < 160) {
            blue[4]++;

        } else if (p.b >= 160 && p.b < 192) {
            blue[5]++;

        } else if (p.b >= 192 && p.b < 224) {
            blue[6]++;

        } else if (p.b >= 224 && p.b < 256) {
            blue[7]++;

        }
    }

    // concatenate red, green and blue entries
    for (int r = 0; r < 8; r++) {
        result.push_back(red[r]);
    }
    for (int g = 0; g < 8; g++) {
        result.push_back(green[g]);
    }
    for (int b = 0; b < 8; b++) {
        result.push_back(blue[b]);
    }


    return result;

}

/**
 * Gets both CVFH and color features for all object clusters
 * @param PointClouds all_clusters
 */
void getAllFeatures(std::vector<PointCloudRGBPtr> all_clusters, std::vector<float> vfhs_vector,
                    std::vector<uint64_t> color_features_vector) {


    getCVFHFeatures(all_clusters);
    ROS_INFO("Vision: CVFH filling completed");

    // do the same for the color histogram
    getColorFeatures(all_clusters);
    ROS_INFO("Vision: Color Histogram filling completed");
}

/**
 * Gets the CVFH features from PointClouds.
 * @param all_clusters PointCloud
 * @return CVFH features, a histogram of angles between a central viewpoint direction and each normal
 */
std::vector<float> getCVFHFeatures(std::vector<PointCloudRGBPtr> all_clusters) {

    PointCloudVFHS308Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>);
    std::vector<float> result;


    for (int i = 0; i < all_clusters.size(); i++) {

        vfhs = cvfhRecognition(all_clusters[i]);

        for (int x = 0; x < 308; x++) {
            //ROS_INFO("%f", current_features[x]);
            result.push_back(vfhs->points[0].histogram[x]);

        }
    }
    return result;
}


/**
 * Gets the color features from a PointCloud.
 * @param all_clusters PointCloud
 * @param color_features_vector to be filled
 */
std::vector<uint64_t> getColorFeatures(std::vector<PointCloudRGBPtr> all_clusters) {

    std::vector<uint64_t> current_color_features;
    std::vector<uint64_t> result;

    for (int i = 0; i < all_clusters.size(); i++) {
        current_color_features = produceColorHist(all_clusters[i]);

        for (int x = 0; x < 24; x++) {
            //ROS_INFO("%f", current_color_features[x]);
            result.push_back(current_color_features[x]);

        }

    }
    return result;
}

/**
 * Load the correct PCD file for the label given.
 * @param label
 * @return Object PointCloud out of PCD file
 */
PointCloudRGBPtr getTargetByLabel(std::string label, Eigen::Vector4f centroid){
    PointCloudRGBPtr    result(new PointCloudRGB),
                        mesh(new PointCloudRGB);



    if (label == "PringlesPaprika") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/pringles.pcd", *mesh);

    } else if (label == "PringlesSalt") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/pringles.pcd", *mesh);
    } else if (label == "SiggBottle") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/sigg_bottle.pcd", *mesh);
    } else if (label == "JaMilch") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/ja_milch.pcd", *mesh);
        ROS_INFO("THIS IS A JA MILCH!");
    } else if (label == "TomatoSauceOroDiParma") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/tomato_sauce_oro_di_parma.pcd", *mesh);
    } else if (label == "KoellnMuesliKnusperHonigNuss") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/koelln_muesli_knusper_honig_nuss.pcd",
                             *mesh);
    } else if (label == "KelloggsToppasMini") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/kelloggs_toppas_mini.pcd", *mesh);
    } else if (label == "HelaCurryKetchup") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/hela_curry_ketchup.pcd", *mesh);
    } else if (label == "CupEcoOrange") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/cup_eco_orange.pcd", *mesh);
    } else if (label == "EdekaRedBowl") {
        pcl::io::loadPCDFile("../../../src/vision_suturo_1718/vision/meshes/edeka_red_bowl.pcd", *mesh);
    }


    return mesh;
}


PointCloudRGBPtr rigidPoseEstimation(PointCloudRGBPtr input, PointCloudRGBPtr target) {


    PointCloudRGBPtr result(new PointCloudRGB);
    PointCloudVFHS308Ptr input_feats(new pcl::PointCloud<pcl::VFHSignature308>), target_feats(
            new pcl::PointCloud<pcl::VFHSignature308>);
    input_feats = cvfhRecognition(input);
    target_feats = cvfhRecognition(target);
    const float leaf = 0.005f;


    // Perform alignment
    pcl::console::print_highlight("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::VFHSignature308> align;
    align.setInputSource(input);
    align.setSourceFeatures(input_feats);
    align.setInputTarget(target);
    align.setTargetFeatures(target_feats);
    align.setMaximumIterations(50000); // Number of RANSAC iterations
    align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(5); // Number of nearest features to use
    align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
    align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align(*result);
    }

    if (align.hasConverged()) {
        // Print results
        printf("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation();
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1),
                                 transformation(0, 2));
        pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1),
                                 transformation(1, 2));
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1),
                                 transformation(2, 2));
        pcl::console::print_info("\n");
        pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3),
                                 transformation(2, 3));
        pcl::console::print_info("\n");
        pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), input->size());

        global_first_transformation = transformation;
        tf::Matrix3x3 tf_rotation(transformation(0, 0),
                                  transformation(0, 1),
                                  transformation(0, 2),
                                  transformation(1, 0),
                                  transformation(1, 1),
                                  transformation(1, 2),
                                  transformation(2, 0),
                                  transformation(2, 1),
                                  transformation(2, 2));

        global_tf_rotation = tf_rotation;


        return result;


    } else {
        pcl::console::print_error("Alignment failed!\n");
        return result;
    }
}


    bool isObjectAlignedToPlane(PointCloudNormalPtr normal_plane, geometry_msgs::Quaternion quaternion){
        bool is_aligned;
        float threshold = 0.2;
        // prepare
        pcl::Normal n;
        pcl::Normal pre;

        for (int i = 1; i < normal_plane->size(); i++){
            n = normal_plane->points[i];
            pre = normal_plane->points[i-1];

            n.normal_x += pre.normal_x;
            n.normal_y += pre.normal_y;
            n.normal_z += pre.normal_z;

        }

        // normalize vector
        n.normal_x = n.normal_x / normal_plane->points.size();
        n.normal_y = n.normal_y / normal_plane->points.size();
        n.normal_z = n.normal_z / normal_plane->points.size();

        // compare

        if (n.normal_z > quaternion.z + threshold || n.normal_z < quaternion.z + threshold) {
            is_aligned = false;
        } else {
            is_aligned = true;

        }


        return is_aligned;

    }

