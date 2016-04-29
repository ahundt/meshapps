#include "index.h"
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <vtkPolyLine.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/filters/fast_bilateral.h>

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);
void viewSPGraph(const pcl::PointCloud<PointT>::Ptr seg_center, 
                 const std::multimap<uint32_t, uint32_t> &graph, 
                 pcl::visualization::PCLVisualizer::Ptr viewer);


int main (int argc, char ** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);

    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::console::print_highlight ("Loading point cloud...\n");
    if (pcl::io::loadPCDFile<PointT> (argv[1], *full_cloud))
    {
        pcl::console::print_error ("Error loading cloud file!\n");
        return (1);
    }
    
    pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
    std::vector<int> idx_ff;
    pcl::removeNaNFromPointCloud(*full_cloud, *tmp_cloud, idx_ff);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*tmp_cloud, *cloud);
    
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (tmp_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1, 2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);
    
    float voxels = 0.05;
    float seeds = 0.1;
    float cw = 0.2;
    float dw = 0.4;
    float nw = 1.0;
    pcl::console::parse_argument(argc, argv, "--voxels", voxels);
    pcl::console::parse_argument(argc, argv, "--seeds", seeds);
    pcl::console::parse_argument(argc, argv, "--cw", cw);
    pcl::console::parse_argument(argc, argv, "--dw", dw);
    pcl::console::parse_argument(argc, argv, "--nw", nw);
    
    float down_ss = -1;
    int min_num = 100;
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    pcl::console::parse_argument(argc, argv, "--mm", min_num);
    pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
    if( down_ss > 0 )
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(down_ss, down_ss, down_ss);
        sor.filter(*down_cloud);
    }
    else
        down_cloud = cloud;
    
    std::vector<pcl::PointCloud<PointT>::Ptr> segs;
    std::multimap<uint32_t, uint32_t> graph;
    
    IDXSET segs_to_cloud;
    pcl::PointCloud<PointLT>::Ptr label_cloud = SPCloud(down_cloud, segs, segs_to_cloud, graph, voxels, seeds, cw, dw, nw);
//    pcl::PointCloud<PointT>::Ptr seg_center = SPCloud(down_cloud, segs, segs_to_cloud, graph, voxels, seeds, cw, dw, nw);
    std::cerr << "SEG NUM: " << segs.size() << std::endl;
    srand(time(NULL));
    int count[100] = {0};
    for( size_t i = 0 ; i < segs.size() ; i++ )
    {
        if( segs[i]->size() < 100 )
            continue;
        if( segs[i]->size() >= 5000 )
            count[99]++;
        else
            count[segs[i]->size()/50]++;
        
        std::stringstream ss;
        ss << i;
        
        float r = (std::rand()%255)/255.0;
        float g = (std::rand()%255)/255.0;
        float b = (std::rand()%255)/255.0;
        
        viewer->addPointCloud(segs[i], "seg"+ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "seg"+ss.str());
    }
    for( int i = 0 ; i < 100 ; i++ )
        std::cerr << count[i] << " ";
    std::cerr << std::endl;
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
//    viewSPGraph(seg_center, graph, viewer);
    while (!viewer->wasStopped ())
        viewer->spinOnce (100);
    
    return (0);
}

void viewSPGraph(const pcl::PointCloud<PointT>::Ptr seg_center, const std::multimap<uint32_t, uint32_t> &graph, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    std::multimap<uint32_t,uint32_t>::const_iterator label_itr = graph.begin ();
    for ( ; label_itr != graph.end (); )
    {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
                        
        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        PointCloudT adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::const_iterator adjacent_itr = graph.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=graph.equal_range (supervoxel_label).second; ++adjacent_itr)
            adjacent_supervoxel_centers.push_back (seg_center->at(adjacent_itr->second));
        
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer(seg_center->at(supervoxel_label), adjacent_supervoxel_centers, ss.str (), viewer);
        //Move iterator forward to next label
        label_itr = graph.upper_bound(supervoxel_label);
    }
}

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}


/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string scene_name("tmp/");
    float degree = 2.0;
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--i", scene_name);
    pcl::console::parse_argument(argc, argv, "--d", degree);
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer());
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    }
   
    double t1, t2;
    std::string workspace(in_path + scene_name);
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(workspace, pcd_files);
    
    for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
    {
        if( exists_test(workspace + "normal_" + (*it)) == false )
            continue;

        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        std::cerr << "Processing-" << workspace + (*it) << std::endl;
        pcl::io::loadPCDFile(workspace + (*it), *full_cloud);
        cv::Mat img = getFullImage(full_cloud);

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        std::vector<int> idx_ff;
        pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);

        pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(workspace + "normal_" + (*it), *cloud_normals);
        
        t1 = get_wall_time();
        
        pcl::search::KdTree<PointT>::Ptr tree = pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>);
        
        //*
        pcl::RegionGrowing<PointT, NormalT> reg;
        reg.setMinClusterSize (600);
        reg.setMaxClusterSize (10000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (cloud);
        //reg.setIndices (indices);
        reg.setInputNormals (cloud_normals);
        reg.setSmoothnessThreshold (degree / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
        //*/
        /*
        pcl::RegionGrowingRGB<PointT> reg;
        reg.setInputCloud (cloud);
        //reg.setIndices (indices);
        reg.setSearchMethod (tree);
        reg.setDistanceThreshold (10);
        reg.setPointColorThreshold (6);
        reg.setRegionColorThreshold (5);
        reg.setMinClusterSize (600);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
        //
        
        t2 = get_wall_time();
        std::cerr << "Segmentation Time: "<< t2 - t1 << std::endl;
        std::cerr << "Cluster Num: " << clusters.size() << std::endl;
        
        //if( view_flag )
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            pcl::PointCloud<PointT>::Ptr colored_cloud = reg.getColoredCloudRGBA();
            viewer->addPointCloud(colored_cloud, "segments");
        
            //viewer->addPointCloud(cloud, "cloud");
            //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
            //viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 10, 0.02, "cloud_normals");
            viewer->spin();
        }
 
    }
        
    return 1;
}
//*/
