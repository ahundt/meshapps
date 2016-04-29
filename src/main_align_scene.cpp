#include "index.h"
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/core/core.hpp>

#include "pcl/visualization/pcl_visualizer.h"

struct KeyPt{
    PointT pt;
    int id;
};

void getKeyPt(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<aruco::Marker> &markers, std::vector<KeyPt> &key_set);
pcl::PointCloud<PointT>::Ptr convertToPC(const std::vector<KeyPt> &key_set);
Eigen::Matrix4f matchKeys(const std::vector<KeyPt> &k1, const std::vector<KeyPt> &k2);
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Meshing(const std::vector< std::vector<KeyPt> > &key_set);
void getEdges(const aruco::Marker &marker, cv::Mat img, const pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<PointT>::Ptr edge_cloud);
Eigen::Matrix4f DenseColorICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &initial_guess);

std::vector<std::string> readMesh(std::string mesh_path, std::string pose_path, std::vector<ModelT> &model_set, std::vector<poseT> &gt_poses)
{
    boost::filesystem::path p(mesh_path);
    std::vector< std::string > ret;
    find_files(p, ".obj", ret);
    
    std::vector< std::string > valid_names;
    for( size_t i = 0 ; i < ret.size() ; i++ )
    {
        std::string model_name = ret[i].substr(0, ret[i].size()-4);
        std::string pose_file(model_name+"_0.csv");
        if( exists_test(pose_path + pose_file) == true )
        {
            ModelT cur_model;
            pcl::PolygonMesh::Ptr model_mesh(new pcl::PolygonMesh()); 
            pcl::io::loadPolygonFile(mesh_path + ret[i], *model_mesh); 
            pcl::PointCloud<myPointXYZ>::Ptr model_cloud(new pcl::PointCloud<myPointXYZ>()); 
            pcl::fromPCLPointCloud2(model_mesh->cloud, *model_cloud);
            cur_model.model_mesh = model_mesh;
            cur_model.model_label = model_name;
            cur_model.model_cloud = model_cloud;
            
            model_set.push_back(cur_model);
            readCSV(pose_path + pose_file, model_name, gt_poses);
            
            valid_names.push_back(model_name);
        }
    }
    
    return valid_names;
}

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    std::string inst_name("4drills_simple");
    std::string object_name("");
    
    int max_frames = 100;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--m", mesh_path);
    pcl::console::parse_argument(argc, argv, "--i", inst_name);
    pcl::console::parse_argument(argc, argv, "--max", max_frames);
    pcl::console::parse_argument(argc, argv, "--obj", object_name);
    
    std::string out_path(in_path + inst_name + "/");
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    bool debug_mode = false;
    if( pcl::console::find_switch(argc, argv, "-d") == true )
        debug_mode = true;
    
    bool icp_flag = true;
    if( pcl::console::find_switch(argc, argv, "-noicp") == true )
        icp_flag = false;
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    aruco::MarkerDetector MDetector;
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile("../carmine_109.yml");
    if( CamParam.isValid() == false )
    {
        std::cerr << "The Camera Parameters Failed!" << std::endl;
        return 0;
    }
    
    std::vector<ModelT> model_set;
    std::vector<poseT> gt_set;
    std::vector<std::string> valid_names = readMesh(mesh_path, in_path + inst_name + "/", model_set, gt_set);
    
    std::cerr << model_set.size() << std::endl;
    std::cerr << gt_set.size() << std::endl;
    //std::cin.get();
    
    std::vector< std::vector<KeyPt> > key_set;
    std::vector< pcl::PointCloud<PointT>::Ptr > cloud_set;
    std::vector< pcl::PointCloud<PointT>::Ptr > edge_set;
    for( int j = 0 ; j < max_frames ; j++ )
    {
        std::stringstream ss;
        ss << j;
        
        std::string filename(in_path + inst_name + "/" + inst_name + "_" + ss.str() + ".pcd");
        std::cerr << "Loading-"<< filename << std::endl;
    
        if( exists_test(filename) == false )
            continue;
        
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(filename, *cloud);
                
        pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
        std::vector<int> in_idx;
        pcl::removeNaNFromPointCloud(*cloud, *temp_cloud, in_idx);
        cloud_set.push_back(temp_cloud);
        
        int w = cloud->width;
        int h = cloud->height;
        
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
        CamParam.resize(rgb.size());

        std::vector< aruco::Marker > markers;
        //MDetector.detect(rgb, markers);
        MDetector.detect(rgb, markers, CamParam, 0.07);
        
        pcl::PointCloud<PointT>::Ptr edge_cloud(new pcl::PointCloud<PointT>());
        for (unsigned int i=0;i<markers.size();i++) {
            markers[i].draw(rgb, cv::Scalar(0,0,255), 2);
            getEdges(markers[i], rgb, cloud, edge_cloud);
        }
        edge_set.push_back(edge_cloud);
        
        std::vector<KeyPt> temp_key;
        getKeyPt(cloud, markers, temp_key);
        key_set.push_back(temp_key);
        
        /*
        std::cerr << edge_cloud->size() << std::endl;
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud, "scene");
        viewer->addPointCloud(edge_cloud, "edge_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "edge_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "edge_cloud");
        viewer->spin();
        //*/
        
    }
    //*
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_set = Meshing(key_set);
    
    int total = key_set.size();
    std::vector<pcl::PointCloud<PointT>::Ptr> tran_cloud_set(total);
    tran_cloud_set[0] = edge_set[0];
    if( debug_mode == true )
        viewer->addPointCloud(cloud_set[0], "cloud0");
    for( int j = 1 ; j < total ; j++ )
    {
        std::cerr << "Registering --- " << j << std::endl;
        
        std::stringstream jj;
        jj << j;
        
        if( rot_set[j] == Eigen::Matrix4f::Identity() )
        {
            std::cerr << "Aruco Code Registration Failed!" << std::endl;
            continue;
        }
        //Eigen::Matrix4f cur_tran = DenseColorICP(cloud_set[j], cloud_set[j-1], init_tran);
        Eigen::Matrix4f cur_tran = DenseColorICP(edge_set[j], tran_cloud_set[0], rot_set[j]);
        //Eigen::Matrix4f cur_tran = rot_set[j];
        
        //tran_cloud_set[j] = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
        //pcl::transformPointCloud(*edge_set[j], *tran_cloud_set[j], cur_tran);
        
        if( debug_mode == true )
        {
//            viewer->removeAllPointClouds();
            pcl::PointCloud<PointT>::Ptr tran_cloud(new pcl::PointCloud<PointT>());
            pcl::transformPointCloud(*cloud_set[j], *tran_cloud, cur_tran);
            
            viewer->addPointCloud(tran_cloud, "tran_cloud"+jj.str());
            viewer->spin();
            continue;
        }
        
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud_set[j], "temp_cloud");
        
        std::vector<poseT> new_poses;
        for(size_t k = 0 ; k < gt_set.size() ; k++ )
        {
            std::stringstream kk;
            kk << k;
            
            ModelT *ptr;
            bool flag = false;
            for(size_t i = 0 ; i < model_set.size(); i++ )
                if(model_set[i].model_label == gt_set[k].model_name)
                {
                    ptr = &model_set[i];
                    flag = true;
                    break;
                }
            if( flag == false)
            {
                std::cerr << "No Matching Model!" << std::endl;
                exit(0);
            }
//            std::cerr << ptr->model_label << " " << object_name << std::endl;
            if(object_name.empty()==false && ptr->model_label != object_name)
                continue;
            
            pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
            //pcl::transformPointCloud(*(ptr->model_cloud), *buf_cloud, gt_set[k].shift, gt_set[k].rotation);
            Eigen::Matrix4f init_tran = Eigen::Matrix4f::Identity();
            init_tran(0, 3) = gt_set[k].shift(0);
            init_tran(1, 3) = gt_set[k].shift(1);
            init_tran(2, 3) = gt_set[k].shift(2);
            Eigen::Matrix3f init_rot(gt_set[k].rotation);
            init_tran.block<3,3>(0,0) = init_rot;
             
            Eigen::Matrix4f tran_buf = cur_tran.inverse();
            tran_buf = tran_buf * init_tran;
            
            // doing icp 
            if( icp_flag )
            {
                pcl::PointCloud<PointT>::Ptr temp_rgb(new pcl::PointCloud<PointT>());
                pcl::copyPointCloud(*(ptr->model_cloud), *temp_rgb);
                tran_buf = DenseColorICP(temp_rgb, cloud_set[j], tran_buf);
            }
            pcl::transformPointCloud(*(ptr->model_cloud), *buf_cloud, tran_buf);
            
            poseT temp_pose;
            temp_pose.model_name = std::string(gt_set[k].model_name);
            Eigen::Matrix3f rot;
            rot << tran_buf(0, 0), tran_buf(0, 1), tran_buf(0, 2), 
                   tran_buf(1, 0), tran_buf(1, 1), tran_buf(1, 2), 
                   tran_buf(2, 0), tran_buf(2, 1), tran_buf(2, 2);
            
            temp_pose.shift = Eigen::Vector3f (tran_buf(0, 3), tran_buf(1, 3), tran_buf(2, 3));
            temp_pose.rotation = rot;
            
            new_poses.push_back(temp_pose);
                    
            viewer->addPointCloud(buf_cloud, "cloud" + kk.str());
            viewer->removePolygonMesh("cloud" + kk.str());
            viewer->addPolygonMesh<myPointXYZ>(buf_cloud, ptr->model_mesh->polygons, "cloud" + kk.str());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, "cloud"+kk.str());
        }
        for(size_t k = 0 ; k < valid_names.size() ; k++ )
        {
            if( object_name.empty()==false && valid_names[k] != object_name )
                continue;
            std::cerr << "Writing---" << in_path + inst_name + "/" + valid_names[k]+"_"+jj.str()+".csv" << std::endl;
            writeCSV(in_path + inst_name + "/" + valid_names[k]+"_"+jj.str()+".csv", valid_names[k], new_poses);
        }
        viewer->spin();
    }
    viewer->spin();
    
    return 0;
}


void getKeyPt(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<aruco::Marker> &markers, std::vector<KeyPt> &key_set)
{
    int w = cloud->width;
    int h = cloud->height;
    for( size_t i = 0 ; i < markers.size() ; i++ )
    {
        for(int j = 0 ; j < 4 ; j++ )
        {
            int r = round(markers[i][j].y);
            int c = round(markers[i][j].x);
            int idx = r*w + c;
            if( r >= 0 && r < h && c >= 0 && c < w && pcl_isfinite(cloud->at(idx).z) == true )
            {
                KeyPt cur_pt;
            
                cur_pt.pt = cloud->at(idx);
                cur_pt.id = markers[i].id * 4 + j;
                key_set.push_back(cur_pt);
            }
        } 
    }
}

pcl::PointCloud<PointT>::Ptr convertToPC(const std::vector<KeyPt> &key_set)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    for(std::vector<KeyPt>::const_iterator it = key_set.begin() ; it < key_set.end() ; it++ )
        cloud->push_back(it->pt);
    return cloud;
}


Eigen::Matrix4f matchKeys(const std::vector<KeyPt> &k1, const std::vector<KeyPt> &k2)
{
    //match m1 to m2
    pcl::PointCloud<PointT>::Ptr key_cloud_1 = convertToPC(k1);
    pcl::PointCloud<PointT>::Ptr key_cloud_2 = convertToPC(k2);
    
    pcl::registration::TransformationEstimationSVD<PointT, PointT> SVD;
    pcl::CorrespondencesPtr corrs (new pcl::Correspondences ());
    for(size_t i = 0 ; i < k1.size(); i++ ){
        for(size_t j = 0 ; j < k2.size(); j++ ){
            if( k1[i].id == k2[j].id )
            {
                pcl::Correspondence temp;
                temp.index_query = i;
                temp.index_match = j;
                temp.distance = 0;
                corrs->push_back(temp);
                break;
            }
        }
    }
    
    
    Eigen::Matrix4f svdRt = Eigen::Matrix4f::Identity();
    if( corrs->size() >= 3 )
    {
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> crsc;
        crsc.setInlierThreshold( 0.005 );
        crsc.setMaximumIterations( 2000 );
        
        crsc.setInputSource( key_cloud_1 );
	crsc.setInputTarget( key_cloud_2 );
        
	crsc.setInputCorrespondences( corrs ); 
        pcl::CorrespondencesPtr in_corr (new pcl::Correspondences());
	crsc.getCorrespondences( *in_corr );
    
        svdRt = crsc.getBestTransformation();
    }
    return svdRt;
}


std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Meshing(const std::vector< std::vector<KeyPt> > &key_set)
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_set(key_set.size());
    
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    rot_set[0] = identity;
    for( size_t j = 1 ; j < rot_set.size() ; j++ )
    {
        Eigen::Matrix4f init_tran;
        init_tran = matchKeys(key_set[j],key_set[0]);
        
        if( init_tran == identity )
        {
            std::cerr << "Identity!" << std::endl;
            for( int k = j-1 ; k > 0 ; k-- )
            {
                init_tran = matchKeys(key_set[j],key_set[k]);
                if( init_tran != identity )
                {
                    init_tran = rot_set[k] * init_tran;
                    break;
                }
                else
                    std::cerr << "Identity!" << std::endl;
            }
        }
        
        rot_set[j] = init_tran;
    }

    return rot_set;
}

void getEdges(const aruco::Marker &marker, cv::Mat img, const pcl::PointCloud<PointT>::Ptr in_cloud, pcl::PointCloud<PointT>::Ptr edge_cloud)
{
    int w = in_cloud->width;
    cv::LineIterator it1(img, marker[0], marker[1], 8);
    for(int k = 0; k < it1.count ; k++, it1++ )
    {
        int idx = round(it1.pos().y) * w + round(it1.pos().x);
        if( pcl_isfinite(in_cloud->at(idx).z) == true )
            edge_cloud->push_back(in_cloud->at(idx));
    }
    cv::LineIterator it2(img, marker[1], marker[2], 8);
    for(int k = 0; k < it2.count ; k++, it2++ )
    {
        int idx = round(it2.pos().y) * w + round(it2.pos().x);
        if( pcl_isfinite(in_cloud->at(idx).z) == true )
            edge_cloud->push_back(in_cloud->at(idx));
    }
    cv::LineIterator it3(img, marker[2], marker[3], 8);
    for(int k = 0; k < it3.count ; k++, it3++ )
    {
        int idx = round(it3.pos().y) * w + round(it3.pos().x);
        if( pcl_isfinite(in_cloud->at(idx).z) == true )
            edge_cloud->push_back(in_cloud->at(idx));

    }
    cv::LineIterator it4(img, marker[3], marker[0], 8);
    for(int k = 0; k < it4.count ; k++, it4++ )
    {
        int idx = round(it4.pos().y) * w + round(it4.pos().x);
        if( pcl_isfinite(in_cloud->at(idx).z) == true )
            edge_cloud->push_back(in_cloud->at(idx));

    }
}


Eigen::Matrix4f DenseColorICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &initial_guess)
{
    //if the ICP fail to converge, return false. Otherwise, return true!
    size_t Iter_num = 50;
    float icp_inlier_threshold = 0.01;
    pcl::PointCloud<PointT>::Ptr init_source(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*source, *init_source, initial_guess);
    
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr source_hue(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr target_hue(new pcl::PointCloud<pcl::PointXYZHSV>);
    ExtractHue(init_source, source_hue);
    ExtractHue(target, target_hue);

    Eigen::Matrix4f final_tran = initial_guess;
    pcl::search::KdTree<pcl::PointXYZHSV> target_tree;
    target_tree.setInputCloud(target_hue);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZHSV, pcl::PointXYZHSV> SVD;
    
    double last_error = 100000;
    for( size_t iter = 0 ; iter < Iter_num ; iter++ )
    {
        //In each round of ICP, transform the model_hue cloud
        float diffh, diffs, diffv, diffsum;
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
        for( size_t i = 0 ; i < source_hue->size() ; i++ )
        {
            std::vector<int> pointIdx;
            std::vector<float> pointDistance;
            if ( pcl_isfinite(source_hue->at(i).z) && target_tree.radiusSearch (source_hue->at(i), 0.01, pointIdx, pointDistance) > 0 )
            {
                float diffmin = 1000;
                int diffmin_idx = -1;
                for (size_t j = 0; j < pointIdx.size (); j++)
                {
                    if( pointDistance.at(j) < icp_inlier_threshold )
                    {
                        diffh = std::min( fabs(target_hue->points[pointIdx[j]].h  - source_hue->points[i].h), 
                                std::min(fabs(target_hue->points[pointIdx[j]].h - 1 - source_hue->points[i].h), fabs(target_hue->points[pointIdx[j]].h + 1 - source_hue->points[i].h)));
                        diffs = fabs(target_hue->points[pointIdx[j]].s - source_hue->points[i].s);
                        diffv = fabs(target_hue->points[pointIdx[j]].v - source_hue->points[i].v);
                        
                        diffsum = diffh + diffs + diffv;
                        if( diffmin > diffsum )
                        {
                            diffmin = diffsum;
                            diffmin_idx = j;
                        }
                    }	
                }
                if( diffmin <= 10000 )
                {
                    pcl::Correspondence temp;
                    temp.index_query = i;
                    temp.index_match = pointIdx[diffmin_idx];
                    temp.distance = pointDistance.at(diffmin_idx);
                    model_scene_corrs->push_back(temp);
                }
            }
        }
        Eigen::Matrix4f svdRt;

        SVD.estimateRigidTransformation(*source_hue, *target_hue, *model_scene_corrs, svdRt);
        
        pcl::transformPointCloud(*source_hue, *source_hue, svdRt);
        
        final_tran = svdRt * final_tran ;
        std::cerr<<"Ratio "<<(model_scene_corrs->size()+0.0) / source_hue->size()<<std::endl;
        if( (model_scene_corrs->size()+0.0) / source_hue->size() >= 0.2 ) //sufficient inlier found
        {
            size_t corrs = model_scene_corrs->size();
            double this_error=0;
            for( size_t j = 0; j < corrs; j++ )
            {
                pcl::PointXYZHSV source_pt = source_hue->points[model_scene_corrs->at(j).index_query];
                pcl::PointXYZHSV target_pt = target_hue->points[model_scene_corrs->at(j).index_match];
                double diffx = source_pt.x - target_pt.x, diffy = source_pt.y - target_pt.y, diffz = source_pt.z - target_pt.z;
                double dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz );

                this_error += dist;
            }
            this_error = this_error / corrs;
            if( fabs(this_error - last_error) < 1e-6 )  //Convergence reach
            {
                std::cerr<<"Convergence Reached. Error: "<<this_error<<std::endl;
                std::cerr<<"Iter Num: "<<iter<<std::endl;
                return final_tran;
            }
            else
                last_error = this_error;
        }
    }

    return final_tran;
}
