/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor
    auto lidar_ptr{std::make_shared<Lidar>(cars, 0)};
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr{lidar_ptr->scan()};
    // renderRays(viewer, Vect3{0.0, 0.0, cars.at(0).dimensions.z + 1}, point_cloud_ptr);
    renderPointCloud(viewer, point_cloud_ptr, "Cloudio", Color{0, 0, 1});
    //  Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor{};
    auto segments{processor.SegmentPlane(point_cloud_ptr, 1000, 0.2)};
    renderPointCloud(viewer, segments.first, "Obstacles", Color{1, 0, 0});
    renderPointCloud(viewer, segments.second, "Plane", Color{0, 1, 0});

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor.Clustering(segments.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = processor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI{};
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // filter point cloud
    auto cloud_filtered{pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-30, -7.0, -3.5, 1), Eigen::Vector4f(30, 7.0, 3.5, 1))};

    // segment point cloud, render plane and pcl obstacles
    auto cloud_segmented = pointProcessorI.SegmentPlane(cloud_filtered, 1000, 0.2);
    renderPointCloud(viewer, cloud_segmented.second, "Ground", Color(0, 1, 0));
    renderPointCloud(viewer, cloud_segmented.first, "Obstacles", Color(1, 0.5, 0));

    // euclidean clustering and box rendering
    auto cloud_clusters = pointProcessorI.Clustering(cloud_segmented.first, 0.6, 20, 300);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1.0, 0.0, 0.0), Color(0.0, 1.0, 0), Color(0.0, 0.0, 1.0)};
    
    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0.5, 0.1), 0.8);

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters)
    {
        auto color{colors[clusterId % colors.size()]};
        std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), color);

        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, color, 0.5);
        ++clusterId;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}