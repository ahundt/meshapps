#include "index.h"
#include <vtkVersion.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include "pcl/visualization/pcl_visualizer.h"


bool GetPointNormals(vtkPolyData* polydata);
void TestPointNormals(vtkPolyData* polydata);

int main(int argc, char** argv)
{
    //std::string inst_name("sander");
    //pcl::console::parse_argument(argc, argv, "--i", inst_name);
    //std::string inputFileName = inst_name+".obj";
    //std::string outputFileName = inst_name+".vtk";
 
    std::string path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    pcl::console::parse_argument(argc, argv, "--p", path);
    boost::filesystem::path p(path);
    std::vector< std::string > ret;
    find_files(p, ".obj", ret);
    
    for( size_t i = 0 ; i < ret.size(); i++ )
    {
        vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
        reader->SetFileName((path+ret[i]).c_str());
        reader->Update();

        vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New();
        writer->SetFileName((path+ret[i].substr(0, ret[i].size()-4)+".vtk").c_str());
        writer->SetInputConnection(reader->GetOutputPort());
        writer->Update();
    }
    
    return 0;
}

void TestPointNormals(vtkPolyData* polydata)
{
  std::cout << "In TestPointNormals: " << polydata->GetNumberOfPoints() << std::endl;
  // Try to read normals directly
  bool hasPointNormals = GetPointNormals(polydata);

  if(!hasPointNormals)
    {
    std::cout << "No point normals were found. Computing normals..." << std::endl;

    // Generate normals
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
#if VTK_MAJOR_VERSION <= 5
    normalGenerator->SetInput(polydata);
#else
    normalGenerator->SetInputData(polydata);
#endif
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOff();
    normalGenerator->Update();
    /*
    // Optional settings
    normalGenerator->SetFeatureAngle(0.1);
    normalGenerator->SetSplitting(1);
    normalGenerator->SetConsistency(0);
    normalGenerator->SetAutoOrientNormals(0);
    normalGenerator->SetComputePointNormals(1);
    normalGenerator->SetComputeCellNormals(0);
    normalGenerator->SetFlipNormals(0);
    normalGenerator->SetNonManifoldTraversal(1);
    */

    polydata = normalGenerator->GetOutput();

    // Try to read normals again
    hasPointNormals = GetPointNormals(polydata);

    std::cout << "On the second try, has point normals? " << hasPointNormals << std::endl;

    }
  else
    {
    std::cout << "Point normals were found!" << std::endl;
    }
}

bool GetPointNormals(vtkPolyData* polydata)
{
  std::cout << "In GetPointNormals: " << polydata->GetNumberOfPoints() << std::endl;
  std::cout << "Looking for point normals..." << std::endl;

  // Count points
  vtkIdType numPoints = polydata->GetNumberOfPoints();
  std::cout << "There are " << numPoints << " points." << std::endl;

  // Count triangles
  vtkIdType numPolys = polydata->GetNumberOfPolys();
  std::cout << "There are " << numPolys << " polys." << std::endl;

  ////////////////////////////////////////////////////////////////
  // Double normals in an array
  vtkDoubleArray* normalDataDouble =
    vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));

  if(normalDataDouble)
    {
    int nc = normalDataDouble->GetNumberOfTuples();
    std::cout << "There are " << nc
            << " components in normalDataDouble" << std::endl;
    return true;
    }

  ////////////////////////////////////////////////////////////////
  // Double normals in an array
  vtkFloatArray* normalDataFloat =
    vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));

  if(normalDataFloat)
    {
    int nc = normalDataFloat->GetNumberOfTuples();
    std::cout << "There are " << nc
            << " components in normalDataFloat" << std::endl;
    return true;
    }
    
  ////////////////////////////////////////////////////////////////
  // Point normals
  vtkDoubleArray* normalsDouble =
    vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetNormals());

  if(normalsDouble)
    {
    std::cout << "There are " << normalsDouble->GetNumberOfComponents()
              << " components in normalsDouble" << std::endl;
    return true;
    }

  ////////////////////////////////////////////////////////////////
  // Point normals
  vtkFloatArray* normalsFloat =
    vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());

  if(normalsFloat)
    {
    std::cout << "There are " << normalsFloat->GetNumberOfComponents()
              << " components in normalsFloat" << std::endl;
    return true;
    }
    
  /////////////////////////////////////////////////////////////////////
  // Generic type point normals
  vtkDataArray* normalsGeneric = polydata->GetPointData()->GetNormals(); //works
  if(normalsGeneric)
    {
    std::cout << "There are " << normalsGeneric->GetNumberOfTuples()
              << " normals in normalsGeneric" << std::endl;

    double testDouble[3];
    normalsGeneric->GetTuple(0, testDouble);

    std::cout << "Double: " << testDouble[0] << " "
              << testDouble[1] << " " << testDouble[2] << std::endl;

    // Can't do this:
    /*
    float testFloat[3];
    normalsGeneric->GetTuple(0, testFloat);

    std::cout << "Float: " << testFloat[0] << " "
              << testFloat[1] << " " << testFloat[2] << std::endl;
    */
    return true;
    }


  // If the function has not yet quit, there were none of these types of normals
  std::cout << "Normals not found!" << std::endl;
  return false;

}


/*
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setSize(1280, 800);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);

    pcl::PolygonMesh::Ptr model_mesh(new pcl::PolygonMesh()); 
    std::string inst_name("sander");
    pcl::console::parse_argument(argc, argv, "--i", inst_name);
    pcl::io::loadPolygonFilePLY("mesh/"+inst_name+".ply", *model_mesh); 
    
    pcl::PointCloud<myPointXYZ>::Ptr cloud(new pcl::PointCloud<myPointXYZ>()); 
    pcl::fromPCLPointCloud2(model_mesh->cloud, *cloud);
    //pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
    //pcl::fromPCLPointCloud2(model_mesh->cloud, *cloud_normals);

    //viewer->addPointCloud(cloud, "cloud");
    //viewer->addPointCloudNormals<myPointXYZ, NormalT>(cloud, cloud_normals, 10, 0.03, "normals");
    //viewer->spin();
    //return 0;
    
    //pcl::PointCloud<myPointXYZ>::Ptr center(new pcl::PointCloud<myPointXYZ>());
    //center = ComputeCentroid(cloud);
    
    //myPointXYZ origin = center->at(0);
    //std::cerr << origin.x << " " << origin.y << " " << origin.z << std::endl;
    //pcl::transformPointCloud(*cloud, *cloud, Eigen::Vector3f(-origin.x, -origin.y, -origin.z), Eigen::Quaternion<float> (1,0,0,0));
    
    //pcl::toPCLPointCloud2(*cloud, model_mesh->cloud);
    
    viewer->addPolygonMesh<myPointXYZ>(cloud, model_mesh->polygons, "Mesh");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, "Mesh");
    viewer->spin();            
    
    //pcl::io::savePolygonFileSTL("mesh/"+inst_name+".stl", *model_mesh); 
    //pcl::io::savePolygonFileVTK("mesh/"+inst_name+".vtk", *model_mesh); 
    */
    

    