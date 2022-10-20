// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    // Create the filtering object: down-sample the dataset using a given leaf size
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_filtered);

    std::vector<int> indices;

    region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region.setInputCloud(cloud_filtered);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int index : indices)
    {
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}

template <typename PointT>
OutputPairType<PointT> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    auto obst_cloud{std::make_unique<pcl::PointCloud<PointT>>()};
    auto plane_cloud{std::make_unique<pcl::PointCloud<PointT>>()};

    for (int index : inliers->indices)
    {
        // fill plane
        plane_cloud->points.push_back(cloud->points[index]);
    }

    // extract obstacles
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*obst_cloud);

    OutputPairType<PointT> segResult(std::move(obst_cloud), std::move(plane_cloud));
    return segResult;
}

template <typename PointT>
OutputPairType<PointT> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    auto inliers{std::make_unique<pcl::PointIndices>()};

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return OutputPairType<PointT>{};
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    OutputPairType<PointT> segResult = SeparateClouds(std::move(inliers), cloud);
    return segResult;
}

// ************************* STUDENT CODE FOR RANSAC **************************************************************
template <typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;

    // Will be used to obtain a seed for the random number engine
    srand(time(NULL));

    if (cloud == nullptr)
        return inliersResult;

    // For max iterations
    while (maxIterations--)
    {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
        {
            inliers.insert(rand() % (cloud->points.size()));
        }

        // Randomly sample subset and fit plane
        auto itr{inliers.begin()};
        float x1{cloud->points[*itr].x};
        float y1{cloud->points[*itr].y};
        float z1{cloud->points[*itr].z};
        ++itr;
        float x2{cloud->points[*itr].x};
        float y2{cloud->points[*itr].y};
        float z2{cloud->points[*itr].z};
        ++itr;
        float x3{cloud->points[*itr].x};
        float y3{cloud->points[*itr].y};
        float z3{cloud->points[*itr].z};

        // find plane constants
        auto a{(y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1)};
        auto b{(z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1)};
        auto c{(x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)};
        auto d{-(a * x1 + b * y1 + c * z1)};

        for (size_t index = 0; index < cloud->points.size(); index++)
        {
            if (inliers.count(index) > 0)
            {
                continue;
            }

            auto point{cloud->points[index]};
            auto d0{sqrtf(a * a + b * b + c * c)};
            float distance{fabs(a * point.x + b * point.y + c * point.z + d) / d0};

            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceTol)
            {
                inliers.insert(index);
            }
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}

template <typename PointT>
OutputPairType<PointT> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    auto inliers_result{Ransac3D<PointT>(cloud, maxIterations, distanceThreshold)};

    // adapt to pcl container
    pcl::PointIndices::Ptr inliers_out(new pcl::PointIndices());
    for (const auto i : inliers_result)
    {
        inliers_out->indices.push_back(i);
    }

    if (inliers_out->indices.empty())
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return OutputPairType<PointT>{};
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    OutputPairType<PointT> segResult = SeparateClouds(std::move(inliers_out), cloud);
    return segResult;
}

// ************************* STUDENT CODE FOR EUCLIDEAN CLUSTERING ****************************************************************
namespace
{
    void euclideanClusterHelper(const std::vector<std::vector<float>> &points, std::vector<int> &cluster, euclidean_clustering::KdTree *tree, std::vector<bool> &visited, const size_t point_id, float distanceTol)
    {
        // mark current point as processed and add it to the current cluster
        visited.at(point_id) = true;
        cluster.push_back(point_id);

        // get a list of every nearby point and iterate through it
        auto neighbors{tree->search(points.at(point_id), distanceTol)};
        for (const auto &neighbor_id : neighbors)
        {
            if (not visited.at(neighbor_id))
                euclideanClusterHelper(points, cluster, tree, visited, neighbor_id, distanceTol);
        }
    }

    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, euclidean_clustering::KdTree *tree, float distanceTol)
    {
        std::vector<std::vector<int>> clusters;
        std::vector<bool> visited(points.size(), false);

        size_t id{0};
        for (const auto &point : points)
        {
            if (not visited.at(id))
            {
                auto current_cluster{std::vector<int>{}};
                euclideanClusterHelper(points, current_cluster, tree, visited, id, distanceTol);
                clusters.push_back(current_cluster);
            }
            ++id;
        }

        return clusters;
    }
}
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringEuclidean(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> cluster_indices;

    std::vector<std::vector<float>> points;
    auto tree{std::make_unique<euclidean_clustering::KdTree>()};
    points.reserve(cloud->points.size());

    // awful for performance but needed
    for (size_t index = 0; index < cloud->points.size(); ++index)
    {
        const auto& point{cloud->points.at(index)};
        points.emplace_back(std::vector<float>{point.x, point.y, point.z});
        tree->insert(points.back(), index);
    }

    auto cluster_result{euclideanCluster(points, tree.get(), clusterTolerance)};

    // adapt results for pcl structure
    for (const auto &cluster : cluster_result)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

        for (const auto &idx : cluster)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }

        if ((cloud_cluster->size() < minSize) or (cloud_cluster->size() > maxSize))
        {
            continue;
        }

        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud)[idx]);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}