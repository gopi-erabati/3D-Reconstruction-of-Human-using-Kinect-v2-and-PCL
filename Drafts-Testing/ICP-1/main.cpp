#include<iostream>
#include<pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc



typedef pcl::PointXYZRGBA PointT;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    // set background to black (R = 0, G = 0, B = 0)
    viewer.setBackgroundColor (0, 0, 0);
}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    // you can add something here, ex:  add text in viewer
}


int
main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGBA>);


  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("../filtered-pointclouds/fcloud1.pcd", *cloud_1)== -1)
   {
     PCL_ERROR ("Couldn't read file fcloud1.pcd");
     return(-1);
   }
  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("../filtered-pointclouds/fcloud2.pcd", *cloud_2)== -1)
   {
     PCL_ERROR ("Couldn't read file fcloud2.pcd");
     return(-1);
   }
 

  pcl::console::TicToc time;
   time.tic ();
  int iterations = 50;

  time.tic ();
  pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setMaximumIterations (iterations);
    //icp.setInputCloud(cloud_1); // setInputSource
    icp.setInputSource(cloud_2);
    icp.setInputTarget(cloud_1);
    //pcl::PointCloud<pcl::PointXYZRGBA> Final;
    icp.align(*cloud_out);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;
   std::cout << "Applied " << iterations << " ICP iteration(s) in " << (time.toc())/1000 << " s" << std::endl;


//write to pcd file
 pcl::io::savePCDFileASCII ("test_pcd_2_1_itr50.pcd", *cloud_out);
 // std::cerr << "Saved " << Final.points.size () << " data points to test_pcd.pcd." << std::endl;

  //for (size_t i = 0; i < Final.points.size (); ++i)
    //std::cerr << "    " << Final.points[i].x << " " << Final.points[i].y << " " << Final.points[i].z << std::endl;

//visual
  pcl::visualization::CloudViewer viewer("Cloud Viewer"); // //// viewer

    // blocks until the cloud is actually rendered
    viewer.showCloud(cloud_out);

    // use the following functions to get access to the underlying more advanced/powerful
    // PCLVisualizer

    // This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);

    // This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ()) {
        // you can also do cool processing here
        // FIXME: Note that this is running in a separate thread from viewerPsycho
        // and you should guard against race conditions yourself...
    }

    cout << "The code is running"<< endl;

  return (0);
}

  
