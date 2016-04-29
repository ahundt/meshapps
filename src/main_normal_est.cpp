#include "index.h"
#include "pcl/visualization/pcl_visualizer.h"

//*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string scene_name("tmp/");
    float radius = 0.03;
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--i", scene_name);
    pcl::console::parse_argument(argc, argv, "--r", radius);
    
    bool view_flag = false, overwrite_flag = false;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    if( pcl::console::find_switch(argc, argv, "-f") == true )
        overwrite_flag = true;
    
    
    if( view_flag )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer());
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    }
   
    double t1, t2;
    std::string workspace(in_path + scene_name);
    std::string out_path(workspace + "/for_recog/");
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(workspace, pcd_files);
    
    for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
    {
        if( overwrite_flag == false && exists_test(workspace + "normal_" + (*it)) == true )
            continue;

        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        std::cerr << "Processing-" << workspace + (*it) << std::endl;
        pcl::io::loadPCDFile(workspace + (*it), *full_cloud);
        cv::Mat img = getFullImage(full_cloud);
        cv::imwrite(out_path + it->substr(0, it->size()-4) + ".png", img);

        t1 = get_wall_time();
        pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud<PointT>());
        std::vector<int> idx_ff;
        pcl::removeNaNFromPointCloud(*full_cloud, *raw_cloud, idx_ff);

        // Create the filtering object
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (raw_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.1, 2.0);
        //pass.setFilterLimitsNegative (true);
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pass.filter (*cloud);
        
        pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
        computeNormals(cloud, cloud_normals, radius);

        t2 = get_wall_time();
        std::cerr << "Normal Est Time: "<< t2 - t1 << std::endl;

        if( view_flag )
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            viewer->addPointCloud(cloud, "cloud");
            //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
            viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 10, 0.02, "cloud_normals");
            viewer->spin();
        }
        pcl::io::savePCDFile(out_path + (*it), *cloud, true);
        pcl::io::savePCDFile(out_path + "normal_" + (*it), *cloud_normals, true);
 
    }
        
    return 1;
}
