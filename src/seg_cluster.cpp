//
// Created by yuan on 4/10/21.
//

#include "seg_cluster.h"

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    int size_of_PCL = cloud->points.size();



    // For max iterations
    while(maxIterations --){
        std::unordered_set<int> inliers;
        // initial a vector to store the indices of samples
        std::vector<int> sample_indices;

        // sample 3 unique points
        while(inliers.size()<4){
            inliers.insert(rand() % size_of_PCL);
        }



        auto itr = inliers.begin();
        // 1st point data
        float x1 = cloud->points[*itr].x;
        float y1 = cloud->points[*itr].y;
        float z1 = cloud->points[*itr].z;
        itr++;
        // 2nd point data
        float x2 = cloud->points[*itr].x;
        float y2 = cloud->points[*itr].y;
        float z2 = cloud->points[*itr].z;

        itr++;
        // 3nd point data
        float x3 = cloud->points[*itr].x;
        float y3 = cloud->points[*itr].y;
        float z3 = cloud->points[*itr].z;

        // line parameters
        float A,B,C,D,i,j,k;
        i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
        A = i;
        B = j;
        C = k;
        D = -(i*x1+j*y1+k*z1);

        // Measure distance between every point and fitted line

        for (int point_index = 0; point_index<size_of_PCL; point_index++){


            float x_point_index = cloud->points[point_index].x;
            float y_point_index = cloud->points[point_index].y;
            float z_point_index = cloud->points[point_index].z;
            float distance = std::abs(A*x_point_index+B*y_point_index+C*z_point_index+D)/sqrt(A*A+B*B+C*C);

            if (distance<=distanceTol){

                inliers.insert(point_index);

            }

        }

        if(inliers.size()>inliersResult.size()){
            inliersResult = inliers;
        }




        // If distance is smaller than threshold count it as inlier

        // Return indicies of inliers from fitted line with most inliers
    }
    return inliersResult;

}
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold){
    // this function is implemented by myself
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    // TODO:: Fill in this function to find inliers for the cloud.
    std::unordered_set<int> inliersResult = Ransac3D(cloud, 50, 0.5);

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZI point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(cloudOutliers, cloudInliers);




    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

void Proximity(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cluster, std::vector<bool>& processed, int i, KdTree3D* tree, float distanceTol){
    processed[i]=true;
    pcl::PointXYZI point_lib;
    point_lib.x = cloud->points[i+1].x;
    point_lib.y = cloud->points[i+1].y;
    point_lib.z = cloud->points[i+1].z;
    point_lib.intensity = cloud->points[i+1].intensity;

    cluster->points.push_back(point_lib);
    std::cout<<i<<std::endl;
    std::vector<float>  point = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
    std::vector<int> nearby_points = tree->search(point,distanceTol);
    for(int id:nearby_points){
        if(!processed[id]){
            Proximity(cloud, cluster, processed, id, tree,  distanceTol);
        }
    }


    int j = 0;
    while(j<nearby_points.size()){
        if(processed[j]==false){
            Proximity( cloud, cluster,processed, j, tree, distanceTol);
            j++;
        }
        else{
            j++;
        }
    }

}
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree3D* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster


    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    std::vector<bool> processed(cloud->size(),false);
    int i = 0;

    while(i<cloud->size()){
        if(processed[i]==true)
        {
            i++;
        }
        else{
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster;


            Proximity(cloud, cluster,processed, i, tree, distanceTol);
            clusters.push_back(cluster);
            i++;
        }

    }


    return clusters;

}


std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();


    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    KdTree3D* tree = new KdTree3D;

    std::cout<<"cloud size:"<<cloud->size()<<std::endl;

    int point_index = 0;
    for (auto point_itr:cloud->points){
        std::vector<float> point = {point_itr.x,point_itr.y,point_itr.z};
        tree->insert(point,point_index);
        point_index++;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = euclideanCluster(cloud, tree, clusterTolerance);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

void cityBlock_Yuan(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();



    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    Eigen::Vector4f min = Eigen::Vector4f::Zero();
    Eigen::Vector4f max = Eigen::Vector4f::Zero();
    min<<-15,-15,-15,1;
    max<<15,15,15,1;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.8 , min,max);


    // cloud segmentation
    ProcessPointClouds<pcl::PointXYZI>* processPointClouds = new ProcessPointClouds<pcl::PointXYZI>();


    // ********************** my implementation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =SegmentPlane(filterCloud, 100, 0.2);
    // **********************


    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // clustering

    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processPointClouds->Clustering(segmentCloud.first, 1.0, 3, 50);
    // ********************** my implementation
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = Clustering(segmentCloud.first, 1.0, 3, 50);
    // **********************


    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,0), Color(0,1,1), Color(1,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processPointClouds->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }


}