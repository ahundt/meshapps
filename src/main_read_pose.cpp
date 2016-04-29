#include "index.h"
#include "pcl/visualization/pcl_visualizer.h"

int color[11][3] =
{ {255, 255, 255}, 
  {255, 0, 0},
  {0, 255, 0},
  {0, 0, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255},
  {255, 128, 0},
  {255, 0, 128},
  {0, 128, 255},
  {128, 0, 255},
};    

std::map<std::string, int> model_name_map;
void setObjID(std::map<std::string, int> &model_name_map)
{
    model_name_map["drill"] = 1;
    model_name_map["driller_small"] = 2;
    model_name_map["drill_flat"] = 3;
    model_name_map["drill_point"] = 4;
    model_name_map["mallet_ball_pein"] = 5;
    model_name_map["mallet_black_white"] = 6;
    model_name_map["mallet_drilling"] = 7;
    model_name_map["mallet_fiber"] = 8;
    model_name_map["old_hammer"] = 9;
    model_name_map["sander"] = 10;
}

std::vector<std::string> readMesh(std::string mesh_path, std::string pose_path, std::string file_id, 
        std::vector<ModelT> &model_set, std::vector<poseT> &gt_poses)
{
    boost::filesystem::path p(mesh_path);
    std::vector< std::string > ret;
    find_files(p, ".obj", ret);
    
    std::vector< std::string > valid_names;
    for(size_t i = 0 ; i < ret.size() ; i++ )
    {
        std::string model_name = ret[i].substr(0, ret[i].size()-4);
        std::string pose_file(model_name+"_"+file_id+".csv");
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

pcl::PointCloud<PointT>::Ptr genSeg(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map)
{
    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*scene, *scene_xyz);
    
    pcl::search::KdTree<myPointXYZ> tree;
    tree.setInputCloud (scene_xyz);
    
    std::vector<int> obj_labels(scene->size(), -1);
    std::vector<float> obj_dist(scene->size(), 1000);
    float T = 0.015;
    
    for(size_t k = 0 ; k < gt_poses.size() ; k++ )
    {
        std::stringstream kk;
        kk << k;

        int model_idx = -1;
        int obj_id = -1;
        for(size_t i = 0 ; i < model_set.size(); i++ ){
            if(model_set[i].model_label == gt_poses[k].model_name)
            {
                model_idx = i;
                obj_id = model_map[model_set[i].model_label];
                //std::cerr << model_set[i].model_label << " " << obj_id << std::endl;
                break;
            }
        }
        if( obj_id <= 0 )
        {
            std::cerr << "No Matching Model!" << std::endl;
            exit(0);
        }
        
        pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*(model_set[model_idx].model_cloud), *buf_cloud, gt_poses[k].shift, gt_poses[k].rotation);
        
        for(pcl::PointCloud<myPointXYZ>::iterator it = buf_cloud->begin() ; it < buf_cloud->end() ; it++ )
        {
            std::vector<int> idx;
            std::vector<float> dist;
    
            tree.radiusSearch(*it, T, idx, dist, buf_cloud->size());
            for( size_t j = 0 ; j < idx.size() ; j++ )
            {
                if( obj_dist[idx[j]] > dist[j] )
                {
                    obj_labels[idx[j]] = obj_id;
                    obj_dist[idx[j]] = dist[j];
                } 
            }   
        }
    }
    
    pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < scene->size() ; i++ )
    {
        if( obj_labels[i] > 0 )
        {
            PointT new_pt = scene->at(i);
            new_pt.rgba = obj_labels[i];
            seg_cloud->push_back(new_pt);
        }
    }
    return seg_cloud;
}

void viewSeg(const pcl::PointCloud<PointT>::Ptr seg_cloud, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    pcl::PointCloud<PointT>::Ptr view_cloud(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < seg_cloud->size() ; i++ )
    {
        PointT new_pt = seg_cloud->at(i);
        int id = new_pt.rgba;
        int r = color[id][0];
        int g = color[id][1];
        int b = color[id][2];
        
        new_pt.rgba = (r << 16) | (g << 8) | (b & 0x0000ff);
        
        view_cloud->push_back(new_pt);
        
    }
    viewer->addPointCloud(view_cloud, "seg");
}

void viewPose(const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> model_name_map, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    for(size_t k = 0 ; k < gt_poses.size() ; k++ )
    {
        std::stringstream kk;
        kk << k;

        int model_idx = -1;
        for(size_t i = 0 ; i < model_set.size(); i++ )
            if(model_set[i].model_label == gt_poses[k].model_name)
            {
                model_idx = i;
                break;
            }
        if( model_idx < 0  )
        {
            std::cerr << "No Matching Model!" << std::endl;
            exit(0);
        }

        pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*(model_set[model_idx].model_cloud), *buf_cloud, gt_poses[k].shift, gt_poses[k].rotation);

        int id = model_name_map[gt_poses[k].model_name];
        viewer->removePolygonMesh("mesh" + kk.str());
        viewer->addPolygonMesh<myPointXYZ>(buf_cloud, model_set[model_idx].model_mesh->polygons, "mesh" + kk.str());
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, "mesh"+kk.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[id][0]/255.0, color[id][1]/255.0, color[id][2]/255.0, "mesh"+kk.str());
    }
}

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    std::string inst_name("labpod_0");
    
    int max_frames = 100;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--m", mesh_path);
    pcl::console::parse_argument(argc, argv, "--i", inst_name);
    pcl::console::parse_argument(argc, argv, "--max", max_frames);
    
    int frame_id = -1;
    pcl::console::parse_argument(argc, argv, "--id", frame_id);
    
    std::string out_path("GT/");
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    bool vp_flag = false;
    if( pcl::console::find_switch(argc, argv, "-vp") == true )
        vp_flag = true;
    bool vs_flag = false;
    if( pcl::console::find_switch(argc, argv, "-vs") == true )
        vs_flag = true;
    
    setObjID(model_name_map);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
//    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.0, 0, 0, 1, 0, -1, 0);
    viewer->setSize(640, 480);
    
    int s_idx = 0, e_idx = max_frames;
    if( frame_id >= 0 )
    {
        s_idx = frame_id;
        e_idx = frame_id+1;
    }
    for( int j = s_idx ; j < e_idx ; j++ )
    {
        std::stringstream ss;
        ss << j;
        
        std::string filename(in_path + inst_name + "/" + inst_name + "_" + ss.str() + ".pcd");
        std::cerr << "Loading-"<< filename << std::endl;
    
        if( exists_test(filename) == false )
            continue;
        
        std::vector<ModelT> model_set;
        std::vector<poseT> gt_set;
        std::vector<std::string> model_names = readMesh(mesh_path, in_path + inst_name + "/poses/", ss.str(), model_set, gt_set);
    
        if( model_names.empty() == true )
            continue;
        //
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(filename, *cloud);
                
        pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
        std::vector<int> in_idx;
        pcl::removeNaNFromPointCloud(*cloud, *temp_cloud, in_idx);
        
        if( vs_flag == true )
        {
            pcl::PointCloud<PointT>::Ptr seg_cloud = genSeg(temp_cloud, model_set, gt_set, model_name_map);
            viewer->removeAllPointClouds();
            viewer->addPointCloud(temp_cloud, "scene");
            viewSeg(seg_cloud, viewer);
            viewer->spin();
            pcl::io::savePCDFile(out_path + inst_name + "_" + ss.str() + "_gt.pcd", *seg_cloud, true);
            pcl::io::savePCDFile(out_path + inst_name + "_" + ss.str() + "_ori.pcd", *temp_cloud, true);
        }
        else if( vp_flag == true )
        {
            viewer->removeAllPointClouds();
            viewer->addPointCloud(temp_cloud, "scene");
            viewPose(model_set, gt_set, model_name_map, viewer);
//            cv::Mat img = getFullImage(cloud);
//            cv::imshow("Img", img);
//            cv::waitKey(1);
            //viewer->spinOnce(1);
            //cv::waitKey();
            viewer->spin();
        }
        else
        {
            std::cerr << "Please select view mode!" << std::endl;
            break;
        }
        
    }
    return 0;
}
