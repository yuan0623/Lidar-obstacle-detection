//
// Created by yuan on 4/10/21.
//

#ifndef PLAYBACK_SEG_CLUSTER_H
#define PLAYBACK_SEG_CLUSTER_H
#include <random>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../src/processPointClouds.h"
#include "../src/render/render.h"
#include "kdtree3D.h"
// using templates for processPointClouds so also include .cpp to help linker


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold);
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree3D* tree, float distanceTol);
void Proximity(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cluster, std::vector<bool>& processed, int i, KdTree3D* tree, float distanceTol);
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);


void cityBlock_Yuan(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud);
#endif //PLAYBACK_SEG_CLUSTER_H
