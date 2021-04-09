/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    // RENDER OPTIONS
    bool render_inp = false;
    bool render_filt = false;
    bool render_obst = false;
    bool render_plane = true;
    bool render_clusters = true;
    bool render_box = true;

    // Load the lidar point cloud data
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    if(render_inp)
    {
        ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
        renderPointCloud(viewer, inputCloudI, "cloud");
    }
    
    // Downsample and crop point cloud
    Eigen::Vector4f minPoint(-25.0, -5.0, -3.0, 1.0);
    Eigen::Vector4f maxPoint(25.0, 7.0, 1.0, 1.0);
    float filterRes = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud =  pointProcessor.FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    if(render_filt)
    {
        renderPointCloud(viewer, filterCloud, "filterCloud", Color(1,0,0));
    }

    // Segment plane
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.2);
    if(render_obst)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    }
    if(render_plane)
    {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    }

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.5, 12, 1000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
        }
        if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }

        ++clusterId;

    }

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obst = true;
    bool render_plane = true;
    bool render_clusters = true;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0); // instatiate on the heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    
    // Render the lidar rays and scene or just the point cloud
    // renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // on the stack
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    if(render_obst)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    }
    if(render_plane)
    {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
        }
        if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }

        ++clusterId;

    }

    //renderPointCloud(viewer, segmentCloud.second, "planeCloud");

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // case 1: simulated data
    //simpleHighway(viewer);

    // case 2: real data
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}