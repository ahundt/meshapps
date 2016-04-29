#include "index.h"
#include "pcl/visualization/pcl_visualizer.h"

void getPCDFiles(std::string path, std::vector<std::string> &files)
{
    boost::filesystem::path p(path);
    find_files(p, ".pcd", files);
}

int main(int argc, char** argv)
{
    std::string in_path(".");
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    
    std::string out_path = in_path + "/for_rviz/";
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    boost::filesystem::create_directories(out_path);
    
    std::vector< std::string > pcd_files;
    getPCDFiles(in_path, pcd_files);    

    for( int j = 0 ; j < pcd_files.size() ; j++ )
    {
        pcl::PointCloud<PointT>::Ptr cloud_rgba(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(in_path + pcd_files[j], *cloud_rgba);
        
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud_rgba, *cloud_rgb);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::vector<int> idx_rr;
        pcl::removeNaNFromPointCloud(*cloud_rgb, *final_rgb, idx_rr);
        
        pcl::io::savePCDFile(out_path + pcd_files[j], *final_rgb, true);
    }
    
    
    return 0;
}


