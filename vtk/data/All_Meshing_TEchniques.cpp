#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/poisson.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/marching_cubes_hoppe.h>

#include <pcl/surface/mls.h>
using namespace std;
using namespace pcl;
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkLineSource.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkDoubleArray.h>
#include <vtkSmartPointer.h>
#include <vtkOBJReader.h>
#include <vtkJPEGReader.h>
#include <vtkTexture.h>
#include <vtkProjectedTexture.h>
#include <vtkDataSetMapper.h>
#include <vtkPointPicker.h>
#include <vtkCommand.h>



#pragma comment(lib,"vtkCommon.lib")
#pragma comment(lib,"vtkImaging.lib")
#pragma comment(lib,"vtkRendering.lib")
#pragma comment(lib,"vtkFiltering.lib")
#pragma comment(lib,"vtkGraphics.lib")
#pragma comment(lib,"vtkHybrid.lib")
#pragma comment(lib,"vtkIO.lib")

#define res 0.0035

    


int
main (int argc, char** argv)
{




  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile ("/home/shubham/FastTriangulation/FinalResult-Gopi-colored.pcd", cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud


// Upsampling (VOXEL_GRID_DILATION)
 MovingLeastSquares<PointXYZRGB, PointXYZRGB> mls;
 mls.setInputCloud (cloud);
 mls.setSearchRadius (0.03);
 mls.setPolynomialFit (true);
 mls.setPolynomialOrder (4);
 mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGB, PointXYZRGB>::VOXEL_GRID_DILATION);
 //mls.setUpsamplingRadius (0.005);
 //mls.setUpsamplingStepSize (0.003);
  mls.setDilationVoxelSize(0.002); 
PointCloud<PointXYZRGB>::Ptr cloud_smoothed (new PointCloud<PointXYZRGB> ());
 mls.process (*cloud_smoothed);

 /*cout << "begin normal estimation" << endl;
       NormalEstimationOMP<PointXYZ, Normal> ne;
       ne.setNumberOfThreads(8);
       ne.setInputCloud(cloud_smoothed);
       ne.setRadiusSearch(0.01);
       Eigen::Vector4f centroid;
       compute3DCentroid(*cloud_smoothed, centroid);
       ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

       PointCloud<Normal>::Ptr normals (new PointCloud<Normal>());
       ne.compute(*normals);
       cout << "normal estimation complete" << endl;
       cout << "reverse normals' direction" << endl;

       for(size_t i = 0; i < normals->size(); ++i){
         normals->points[i].normal_x *= -1;
         normals->points[i].normal_y *= -1;
         normals->points[i].normal_z *= -1;
       }
       cout << "combine points and normals" << endl;*/

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_smoothed);
  n.setInputCloud (cloud_smoothed);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*cloud_normals);
  //* normals should not contain the point normals + surface curvatures
  
  // Concatenate the XYZ and normal fields*

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals*/

   //pcl::io::savePCDFile ("/home/shubham/FastTriangulation/FinalResult_Gopi_normals.pcd", *cloud_normals);

  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud (cloud_with_normals);
  //pcl::io::savePCDFile("/home/shubham/FastTriangulation/cloud_normals.pcd",*cloud_with_normals);
  

 // Initialize objects Greedy Projection Triangulation 


  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  boost::shared_ptr<pcl::PolygonMesh> triangles (new pcl::PolygonMesh);
  pcl::PolygonMesh triangles;
  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (3);//3
  gp3.setMaximumNearestNeighbors (1200);
  gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees  //2
  gp3.setMinimumAngle(M_PI/36); // 10 degrees //5deg
  gp3.setMaximumAngle(2*M_PI/3 + M_PI/6); // 120 degrees  // 150 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  std::cerr << "Begin Laplacian SMoothing...";
  pcl:: PolygonMesh output;
  pcl:: MeshSmoothingLaplacianVTK vtk;
  vtk.setInputMesh(triangles);
  vtk.setNumIter(20000);
  vtk.setConvergence(0.0001);
  vtk.setRelaxationFactor(0.0001);
  vtk.setFeatureEdgeSmoothing(true);
  vtk.setFeatureAngle(M_PI/5);
  vtk.setBoundarySmoothing(true);
  vtk.process(output);
  std::cerr << "Done." << std:: endl;


// Grid Projection


/*pcl:: PolygonMesh triangles;
pcl::GridProjection<pcl::PointNormal> gp;

// Get result
  gp.setInputCloud(cloud_with_normals);
  gp.setSearchMethod(tree2);
  gp.setResolution(res);
  gp.setPaddingSize(3);
  gp.reconstruct(triangles);*/


  std::cout << output.polygons.size() << " Triangles in reconstructed mesh" << std::endl;
  pcl::io::saveVTKFile ("/home/shubham/FastTriangulation/FinalResult_Gopi_colored_mesh2.vtk", output);
  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();


// Poisson Reconstruction 


   /*pcl::Poisson<pcl::PointNormal>poi; 
   poi.setInputCloud(cloud_with_normals); 
   poi.setSearchMethod(tree2);
   //poi.setConfidence(false);
   //poi.setManifold(false);
   //poi.setOutputPolygons(false);
   poi.setDepth(12);
   poi.setScale(1.2);
   poi.setSolverDivide(8);
   poi.setIsoDivide(8);
   poi.setSamplesPerNode(2);
   pcl::PolygonMesh mesh;
   poi.reconstruct(mesh);
   pcl::io::saveVTKFile ("/home/shubham/FastTriangulation/FinalResult_Gopi_mesh1.vtk", mesh);
   std::cout << mesh.polygons.size() << " Triangles in reconstructed mesh" << std::endl;*/


// Marching Cubes

    /*pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
    pcl::PolygonMesh triangles;

    mc.setGridResolution(100, 100, 100);
    mc.setIsoLevel(0.005);
    mc.setPercentageExtendGrid(0);

    mc.setInputCloud(cloud_with_normals);
    mc.setSearchMethod(tree2);
    mc.reconstruct(triangles);
    std::cout << triangles.polygons.size() << " polygons in reconstructed mesh" << std::endl;


    pcl::io::saveVTKFile ("/home/shubham/MarchingCubes1/FinalResult_Gopi_mesh1.vtk", triangles);*/

  // VTK VIEWER
  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();

  reader->SetFileName("/home/shubham/FastTriangulation/FinalResult_Gopi_colored_mesh2.vtk");

  reader->Update();

  vtkSmartPointer<vtkPolyData> poly;
  poly = vtkSmartPointer<vtkPolyData>::New();
  poly->DeepCopy(reader->GetOutput());

  vtkSmartPointer<vtkTransform> transform;
  vtkSmartPointer<vtkMatrix4x4> matrix;
  matrix = vtkSmartPointer<vtkMatrix4x4>::New();

  transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();
  transform->SetMatrix(matrix);
  transform->Update();

  vtkSmartPointer<vtkTransformPolyDataFilter> tfpdf;
  tfpdf = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  tfpdf->SetInput(poly);
  tfpdf->SetTransform(transform);
  tfpdf->Update();


  vtkSmartPointer<vtkPolyDataMapper> mapper;
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(tfpdf->GetOutputPort());
  mapper->SetScalarVisibility(0);

  vtkSmartPointer<vtkActor> actor;
  actor = vtkSmartPointer<vtkActor>::New();
  actor->GetProperty()->SetColor(0.4,0.2,0.5);
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(0,0,0);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkRenderWindow> ren_window;
  ren_window = vtkSmartPointer<vtkRenderWindow>::New();
  ren_window->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> ren_inter = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  ren_inter->SetRenderWindow(ren_window);


// start interaction
  ren_window->Render();
  ren_inter->Start();


  // Finish
  return (0);
}
