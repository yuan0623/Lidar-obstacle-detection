/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <random>
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    int size_of_PCL = cloud->points.size();



    // For max iterations
    for(int i=0; i<maxIterations; i++){

        // initial a vector to store the indices of samples
        std::vector<int> sample_indices;


        // Randomly sample subset and fit line
        for(int j = 0;j<2;j++){

            int sample_index = rand() % size_of_PCL;
            sample_indices.push_back(sample_index);
        }



        // 1st point data
        float x1 = cloud->points[sample_indices[0]].x;
        float y1 = cloud->points[sample_indices[0]].y;

        // 2nd point data
        float x2 = cloud->points[sample_indices[1]].x;
        float y2 = cloud->points[sample_indices[1]].y;

        // line parameters
        float A,B,C;
        A = y1-y2;
        B = x2-x1;
        C = x1*y2-x2*y1;
        std::unordered_set<int> inliers;
        // Measure distance between every point and fitted line

        for (int point_index = 0; point_index<size_of_PCL; point_index++){


            float x_point_index = cloud->points[point_index].x;
            float y_point_index = cloud->points[point_index].y;
            float distance = std::abs(A*x_point_index+B*y_point_index+C)/sqrt(A*A+B*B);
            std::cout<<"distance: "<<distance<<std::endl;
            if (distance<=distanceTol){
                std::cout<<"haha"<<std::endl;
                inliers.insert(point_index);

            }

        }
        std::cout<<"inlier size after: "<<inliers.size()<<std::endl;
        if(inliers.size()>inliersResult.size()){
            inliersResult = inliers;
        }




	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
    }
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
