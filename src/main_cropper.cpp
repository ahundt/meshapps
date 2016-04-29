#include "index.h"
#include "pcl/visualization/pcl_visualizer.h"

#define ELEV 0.132

//pcl::PointCloud<PointT>::Ptr CropCloud(const pcl::PointCloud<PointT>::Ptr scene, float elev = 0.03, float z_max = 1.0, float z_min = 0.1, float x_span = 0.3, float y_span = 0.3)
bool CropCloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr planeCoef, 
        pcl::PointCloud<PointT>::Ptr &cropped_cloud, cv::Mat &idx_3d2d, cv::Mat &rgb, cv::Mat &mask, cv::Mat &depth, float elev = ELEV)
{   
    int w = cloud->width;

    std::vector<int> idx_f;
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    pcl::removeNaNFromPointCloud(*cloud, *cloud_f, idx_f);

    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_f);
    proj.setModelCoefficients (planeCoef);

    pcl::PointCloud<PointT>::Ptr cloud_f_projected(new pcl::PointCloud<PointT>());
    proj.filter (*cloud_f_projected);

    pcl::PointCloud<PointT>::iterator it_ori = cloud_f->begin();
    pcl::PointCloud<PointT>::iterator it_proj = cloud_f_projected->begin();
    
    pcl::PointCloud<PointT>::Ptr init_cloud(new pcl::PointCloud<PointT>());
    std::vector<int> init_idx;
    for( int base = 0 ; it_ori < cloud_f->end(), it_proj < cloud_f_projected->end() ; it_ori++, it_proj++, base++ )
    {
        float diffx = (*it_ori).x-(*it_proj).x;
        float diffy = (*it_ori).y-(*it_proj).y;
        float diffz = (*it_ori).z-(*it_proj).z;

        //distance from the point to the plane
        float dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz);
        
//        if ( dist > elev && it_ori->z < 1.5 && it_ori->z > 0.1 )//&& (*it_ori).x > -0.2 && (*it_ori).y > -0.25 )//fabs((*it_ori).x) <= 0.1 && fabs((*it_ori).y) <= 0.1 )
        {
            init_cloud->push_back(*it_ori);
            int idx_ori = idx_f[base];            
            init_idx.push_back(idx_ori);
        }
    }
    if( init_idx.empty() == true )
        return false;
    
    pcl::PointIndices::Ptr final_idx(new pcl::PointIndices());
    if( init_idx.size() >= 100 )
    {
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (0.015);     // 1.5cm
        ec.setMinClusterSize (400);
        ec.setMaxClusterSize (INF_);
        ec.setSearchMethod (tree);
        ec.setInputCloud (init_cloud);
        ec.extract (cluster_indices);
        
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                final_idx->indices.push_back(init_idx[*pit]);
        if( final_idx->indices.empty() == true )
            final_idx->indices = init_idx;
    }
    else
        final_idx->indices = init_idx;
    
    cropped_cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
    cropped_cloud->clear();
    int max_r = -1000, min_r = 1000, max_c = -1000, min_c = 1000;
    for( std::vector<int>::iterator it = final_idx->indices.begin() ; it < final_idx->indices.end() ; it++ )
    {
        int r = *it / w;
        int c = *it % w;
        if( max_r < r ) max_r = r;
        if( max_c < c ) max_c = c;
        if( min_r > r ) min_r = r;
        if( min_c > c ) min_c = c; 
        
        cropped_cloud->push_back(cloud->at(*it));
    }
    
    //std::cerr << max_r << " " << min_r << " " << max_c << " " << min_c << std::endl;
    
    rgb = cv::Mat::zeros(max_r-min_r+1, max_c-min_c+1, CV_8UC3);
    depth = cv::Mat::zeros(max_r-min_r+1, max_c-min_c+1, CV_16UC1);
    
    for( int i = min_r ; i <= max_r ; i++ ){
        for( int j = min_c ; j <= max_c ; j++ )
        {
            int cur_idx = i*w + j;
            int r_idx = i-min_r;
            int c_idx = j-min_c;
            uint32_t rgb_tmp = cloud->at(cur_idx).rgba;
            rgb.at<uchar>(r_idx, c_idx*3+2) = (rgb_tmp >> 16) & 0x0000ff;
            rgb.at<uchar>(r_idx, c_idx*3+1) = (rgb_tmp >> 8)  & 0x0000ff;
            rgb.at<uchar>(r_idx, c_idx*3+0) = (rgb_tmp)       & 0x0000ff;
            depth.at<uint16_t>(r_idx, c_idx) = (uint16_t)(cloud->at(cur_idx).z * 1000);
        }
    }
    
    mask = cv::Mat::zeros(max_r-min_r+1, max_c-min_c+1, CV_8UC1);
    idx_3d2d = cv::Mat::zeros(final_idx->indices.size(), 2, CV_32SC1);
    int count = 0;
    for( std::vector<int>::iterator it = final_idx->indices.begin() ; it < final_idx->indices.end() ; it++, count++ )
    {
        int r_idx = *it / w - min_r;
        int c_idx = *it % w - min_c;
        idx_3d2d.at<int>(count, 0) = c_idx;
        idx_3d2d.at<int>(count, 1) = r_idx;
        
        mask.at<uchar>(r_idx, c_idx) = 255;
    }
    
    return true;
}

int getSeqID(std::string name)
{
    int id = -1;
    for( int i = name.size() - 1 ; i >= 0 ; i-- )
    {
        if( name[i] == '_' )
        {
            if( name[i-1] <= '9' && name[i-1] >= '0' )
                id = name[i-1] - '0';
            if( name[i-2] <= '9' && name[i-2] >= '0' )
                id += (name[i-2] - '0') * 10;
            break;
        }
    }
    return id;
}

//*
int main(int argc, char **argv)
{
    int c1 = 0, c2 = JHU_INST_MAX;
    std::string inst_name;
    std::string in_path("/home/chi/JHUIT/raw/");
    std::string out_path("/home/chi/JHUIT/filtered_pcd_2/");

    float elev = ELEV;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    pcl::console::parse_argument(argc, argv, "--i", inst_name);
    pcl::console::parse_argument(argc, argv, "--elev", elev);
    
    std::ifstream model_list((in_path + "/models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < JHU_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < c1 )
            continue;
        else if (m_id > c2)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(in_path + class_name);
        std::string savespace(out_path + class_name);
        if( exists_dir(savespace) == false )
            boost::filesystem::create_directories(savespace);
        
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        size_t file_num = pcd_files.size();
        
        for( size_t j = 0 ; j < file_num ; j++ )
        {
            std::string filename(workspace+"/"+pcd_files[j]);
            if( inst_name.empty() == false && pcd_files[j] != inst_name )
                continue;
            
            std::string pure_name = pcd_files[j].substr(0, pcd_files[j].size()-4);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            
            pcl::ModelCoefficients::Ptr planeCoef(new pcl::ModelCoefficients());
            planeCoef->values.resize(4);
            int seq_id = getSeqID(pcd_files[j]);
            if( seq_id < 10 )
                continue;
            std::cerr << filename << std::endl;
            
            std::ifstream fp;
            if( seq_id >= 4 )
            {
                //continue;
                fp.open((workspace+"/PlaneCoef_"+ pure_name + ".txt").c_str(), std::ios::in);
                if( fp.is_open() == false )
                {
                    std::cerr << "Failed to open: " << workspace+"/PlaneCoef_"+pure_name+ ".txt" << std::endl;
                    exit(0);
                }
                fp >> planeCoef->values[0] >> planeCoef->values[1] >> planeCoef->values[2] >> planeCoef->values[3];
                fp.close();
            }
            else
            {
                //continue;
                std::ostringstream ss;
                ss << seq_id;
                fp.open((workspace+"/../PlaneCoef_"+ss.str()+ ".txt").c_str(), std::ios::in);
                if( fp.is_open() == false )
                {
                    std::cerr << "Failed to open: " << workspace+"/../PlaneCoef_"+ss.str()+ ".txt" << std::endl;
                    exit(0);
                }
                fp >> planeCoef->values[0] >> planeCoef->values[1] >> planeCoef->values[2] >> planeCoef->values[3];
                fp.close();
            }
            
            cv::Mat rgb, depth, mask, idx_3d2d;
            pcl::PointCloud<PointT>::Ptr cropped_cloud;
            pcl::PointCloud<NormalT>::Ptr cropped_normals;;
            bool flag = CropCloud(cloud, planeCoef, cropped_cloud, idx_3d2d, rgb, mask, depth, elev);
            if( flag == true )
            {
//                computeNormals(cropped_cloud, cropped_normals, 0.03);

                //save results
                pcl::io::savePCDFile(savespace+"/" +pcd_files[j], *cropped_cloud);
//                pcl::io::savePCDFile(savespace+"/normal_" +pcd_files[j], *cropped_normals);
                saveMat(savespace+"/3D2D_" +pure_name + ".cvmat", idx_3d2d);

                cv::imwrite(savespace+"/" +pure_name + "_rgbcrop.png", rgb);
                cv::imwrite(savespace+"/" +pure_name + "_depthcrop.png", depth);
                cv::imwrite(savespace+"/" +pure_name + "_mask.png", mask);
            }
            else
                std::cerr<<"Crop Failed: " << class_name << "-" << pcd_files[j] << std::endl;
        }
        
    }
    
    model_list.close();
    
    return 0;
}
//*/

/*
int main(int argc, char **argv)
{
    int c1 = 0, c2 = JHU_INST_MAX;
    std::string in_p("/home/chi/JHUIT/filtered");
    std::string out_p("/home/chi/JHUIT/filtered_pcd");
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    std::ifstream model_list((in_p + "/models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < JHU_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < c1 )
            continue;
        else if (m_id > c2)
            break;    
        
        std::cerr << "Processing-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(in_p +"/" + class_name);
        std::string savespace(out_p +"/" + class_name);
        if( exists_dir(savespace) == false )
            boost::filesystem::create_directories(savespace);
        
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        size_t file_num = pcd_files.size();
        
        std::vector< std::vector<std::string> > seq_names(3);
        for( size_t j = 0 ; j < file_num ; j++ )
        {
            //std::string filename(workspace+"/"+pcd_files[j]);
            
            std::string pure_name = pcd_files[j].substr(0, pcd_files[j].size()-4);
            
            int seq_id = getSeqID(pcd_files[j]);
            seq_names[seq_id-4].push_back(pure_name);
        }
        for( int i = 0 ; i < 3 ; i++ )
        {
            int num = seq_names[i].size();
            if( num < 50 )
            {
                std::cerr << class_name << "-" << i << std::endl;
                exit(0);
            }
            for( int j = 0 ; j < 50 ; j++ )
            {
                boost::filesystem::copy_file(workspace+"/"+seq_names[i][j] + ".pcd", savespace+"/"+seq_names[i][j] + ".pcd");
                boost::filesystem::copy_file(workspace+"/normal_"+seq_names[i][j] + ".pcd", savespace+"/normal_"+seq_names[i][j] + ".pcd");
                boost::filesystem::copy_file(workspace+"/"+seq_names[i][j] + "_depthcrop.png", savespace+"/"+seq_names[i][j] + "_depthcrop.png");
                boost::filesystem::copy_file(workspace+"/"+seq_names[i][j] + "_rgbcrop.png", savespace+"/"+seq_names[i][j] + "_rgbcrop.png");
                boost::filesystem::copy_file(workspace+"/"+seq_names[i][j] + "_mask.png", savespace+"/"+seq_names[i][j] + "_mask.png");
                boost::filesystem::copy_file(workspace+"/3D2D_" +seq_names[i][j] + ".cvmat", savespace+"/3D2D_" +seq_names[i][j] + ".cvmat");
            }
            
        }
        
    }
    
    model_list.close();
    
}
//*/
