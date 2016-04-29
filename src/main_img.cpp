//#include <pcl-1.7/pcl/impl/point_types.hpp>

#include "index.h"
#include "pcl/visualization/pcl_visualizer.h"

void getPCDFiles(std::string path, std::vector<std::string> &files)
{
    boost::filesystem::path p(path);
    find_files(p, ".pcd", files);
}

pcl::PointCloud<PointT>::Ptr removePlane(const pcl::PointCloud<PointT>::Ptr scene)
{
    pcl::ModelCoefficients::Ptr plane_coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(scene);
    seg.segment(*inliers, *plane_coef);
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (scene);
    proj.setModelCoefficients (plane_coef);

    pcl::PointCloud<PointT>::Ptr scene_projected(new pcl::PointCloud<PointT>());
    proj.filter (*scene_projected);

    pcl::PointCloud<PointT>::iterator it_ori = scene->begin();
    pcl::PointCloud<PointT>::iterator it_proj = scene_projected->begin();
    
    pcl::PointCloud<PointT>::Ptr scene_f(new pcl::PointCloud<PointT>());
    for( int base = 0 ; it_ori < scene->end(), it_proj < scene_projected->end() ; it_ori++, it_proj++, base++ )
    {
        
        float diffx = it_ori->x-it_proj->x;
        float diffy = it_ori->y-it_proj->y;
        float diffz = it_ori->z-it_proj->z;

        if( diffx * it_ori->x + diffy * it_ori->y + diffz * it_ori->z >= 0 )
            continue;
        //distance from the point to the plane
        float dist = sqrt(diffx*diffx + diffy*diffy + diffz*diffz);
        
        if ( dist >= 0.03 )//fabs((*it_ori).x) <= 0.1 && fabs((*it_ori).y) <= 0.1 )
            scene_f->push_back(*it_ori);
    }
    
    return scene_f;
}


//int main(int argc, char** argv)
//{
//    std::string in_path(".");
//    std::string out_path = in_path;
//    pcl::console::parse_argument(argc, argv, "--p", in_path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    
//    boost::filesystem::create_directories(out_path);
//    
//    std::vector< std::string > pcd_files;
//    getPCDFiles(in_path, pcd_files);    
//
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->initCameraParameters();
//    viewer->addCoordinateSystem(0.1);
//    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
//    viewer->setSize(640, 480);
//    
//    for( int j = 0 ; j < pcd_files.size() ; j++ )
//    {
//        pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
//        pcl::io::loadPCDFile(in_path + pcd_files[j], *cloud2);
//        
//        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//        pcl::fromPCLPointCloud2(*cloud2, *cloud);
//        
//        pcl::PointCloud<PointT>::Ptr scene_f = removePlane(cloud);
//        
//        viewer->addPointCloud(scene_f, "scene_f");
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "scene_f");
//        viewer->spin();
//	viewer->removeAllPointClouds();
//    }
//    
//    
//    return 0;
//}


cv::Mat getImage(const pcl::PointCloud<PointT>::Ptr cloud, float ap_ratio, float fx, float fy, float center_x, float center_y)
{
    int img_h = 480 * ap_ratio;
    int img_w = 640 * ap_ratio;
    
    cv::Mat img = cv::Mat::zeros(img_h, img_w, CV_8UC3);
    for( size_t i = 0 ; i < cloud->size() ; i++ )
    {
        PointT *pt_ptr = &cloud->at(i);
        uint32_t rgb = pt_ptr->rgba;
        
        int img_x = round(((*pt_ptr).x / (*pt_ptr).z * fx + center_x)*ap_ratio);
        int img_y = round(((*pt_ptr).y / (*pt_ptr).z * fy + center_y)*ap_ratio);
        if( img_x < 0 ) img_x = 0; 
        if( img_y < 0 ) img_y = 0;
        if( img_x >= img_w ) img_x = img_w-1;
        if( img_y >= img_h ) img_y = img_h-1;
        
        img.at<uchar>(img_y, img_x*3+2) = (rgb >> 16) & 0x0000ff;
        img.at<uchar>(img_y, img_x*3+1) = (rgb >> 8) & 0x0000ff;
        img.at<uchar>(img_y, img_x*3+0) = (rgb) & 0x0000ff;
    }
    
    return img;
}

#define FOCAL_X 539.22
#define FOCAL_Y 539.17
#define CENTER_X 317.06
#define CENTER_Y 232.76

//int main(int argc, char** argv)
//{
//    std::string in_path(".");
//    pcl::console::parse_argument(argc, argv, "--p", in_path);
//    
//    std::string out_path = in_path;
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    
//    boost::filesystem::create_directories(out_path);
//    
//    std::vector< std::string > pcd_files;
//    getPCDFiles(in_path, pcd_files);    
//
//    for( int j = 0 ; j < pcd_files.size() ; j++ )
//    {
//        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//        pcl::io::loadPCDFile(in_path + pcd_files[j], *cloud);
//        std::cerr << in_path + pcd_files[j] << std::endl;
//        
//        
//	int w = cloud->width;
//        int h = cloud->height;
//        
//        cv::Mat rgb = cv::Mat::zeros(h, w, CV_8UC3);
//        
//        for(size_t i = 0 ; i < cloud->size() ; i++ )
//        {
//            int r_idx = i / w;
//            int c_idx = i % w;
//            
//            uint32_t rgb_tmp = cloud->at(i).rgba;
//            rgb.at<uchar>(r_idx, c_idx*3+2) = (rgb_tmp >> 16) & 0x0000ff;
//            rgb.at<uchar>(r_idx, c_idx*3+1) = (rgb_tmp >> 8)  & 0x0000ff;
//            rgb.at<uchar>(r_idx, c_idx*3+0) = (rgb_tmp)       & 0x0000ff;  
//            
//        }
//        
//        //cv::Mat rgb = getImage(cloud, 1.0, FOCAL_X, FOCAL_Y, CENTER_X, CENTER_Y);
//        
//        cv::imshow("in",rgb);
//        cv::waitKey(1);//wait for key to be pressed
//        cv::imwrite(out_path + pcd_files[j].substr(0, pcd_files[j].size()-4) + ".png", rgb); 
//    }
//    
//    
//    return 0;
//}

int main(int argc, char** argv)
{
    std::string filename("");
    pcl::console::parse_argument(argc, argv, "--f", filename);
    
//    cv::Mat M;
//    readMat(filename, M);
//    
//    std::cerr << M << std::endl;
//    return 1;
    
    
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile(filename, *cloud);
       
        
    int w = cloud->width;
    int h = cloud->height;
    std::cerr << w << " " << h << " " << cloud->size() << std::endl;
        
    cv::Mat rgb = cv::Mat::zeros(h, w, CV_8UC3);
        
    for(size_t i = 0 ; i < cloud->size() ; i++ )
    {
        int r_idx = i / w;
        int c_idx = i % w;
            
        uint32_t rgb_tmp = cloud->at(i).rgba;
        rgb.at<uchar>(r_idx, c_idx*3+2) = (rgb_tmp >> 16) & 0x0000ff;
        rgb.at<uchar>(r_idx, c_idx*3+1) = (rgb_tmp >> 8)  & 0x0000ff;
        rgb.at<uchar>(r_idx, c_idx*3+0) = (rgb_tmp)       & 0x0000ff;  

    }

//    cv::imwrite("tmp.png", rgb);
    cv::imshow("in",rgb);
    cv::waitKey();//waitf or key to be pressed    
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    viewer->addPointCloud(cloud, "cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
    viewer->spin();
    
    return 0;
}


