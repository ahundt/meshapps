#include <pcl-1.7/pcl/visualization/common/common.h>
#include "pcl/visualization/pcl_visualizer.h"

#include "index.h"

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

float vectorNorm(const std::vector<float>& A)
{
    return sqrt(A[0]*A[0] + A[1]*A[1] + A[2]*A[2]);
}

std::vector<float> crossProduct(const std::vector<float>& A, const std::vector<float>& B )
{
    std::vector<float> product(3, 0);
    
    if( A.size() != 3 || B.size() != 3 )
    {
        std::cerr << "A.size() != 3 || B.size() != 3" << std::endl;
        return product;
    }
    
    product[0] = A[1]*B[2] - A[2]*B[1];
    product[1] = A[2]*B[0] - A[0]*B[2];
    product[2] = A[0]*B[1] - A[1]*B[0];
    
    float norm = vectorNorm(product);
    product[0] /= norm;
    product[1] /= norm;
    product[2] /= norm;
    
    return product;
}

#define OBJMAX 10

std::vector<std::string> readMesh(std::string mesh_path, std::vector<ModelT> &model_set, std::map<std::string, int> model_name_map)
{
    boost::filesystem::path p(mesh_path);
    std::vector< std::string > ret;
    find_files(p, ".obj", ret);
   
    std::vector< std::string > valid_names;
    model_set.clear();
    model_set.resize(OBJMAX+1);
    for(size_t i = 0 ; i < ret.size() ; i++ )
    {
        std::string model_name = ret[i].substr(0, ret[i].size()-4);
        int id = model_name_map[model_name];

        ModelT cur_model;
        pcl::PolygonMesh::Ptr model_mesh(new pcl::PolygonMesh()); 
        pcl::io::loadPolygonFile(mesh_path + ret[i], *model_mesh); 
        pcl::PointCloud<myPointXYZ>::Ptr model_cloud(new pcl::PointCloud<myPointXYZ>()); 
        pcl::fromPCLPointCloud2(model_mesh->cloud, *model_cloud);
        cur_model.model_mesh = model_mesh;
        cur_model.model_label = model_name;
        cur_model.model_cloud = model_cloud;

        model_set[id] = cur_model;
        valid_names.push_back(model_name);
    }
    
    return valid_names;
}

void readPose(std::string pose_path, std::string file_id, std::vector<std::string> model_names, std::vector<poseT> &gt_poses)
{
    for(size_t i = 0 ; i < model_names.size() ; i++ )
    {
        std::string pose_file(model_names[i]+"_"+file_id+".csv");
        if( exists_test(pose_path + pose_file) == true )
            readCSV(pose_path + pose_file, model_names[i], gt_poses);
    }
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

void viewPose(const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> model_name_map, pcl::visualization::PCLVisualizer::Ptr viewer, int &viewport)
{
    std::stringstream ss;
    ss << viewport;
    for(size_t k = 0 ; k < gt_poses.size() ; k++ )
    {
        std::stringstream kk;
        kk << k;
        
        int model_idx = model_name_map[gt_poses[k].model_name];
        if( model_idx < 0  )
        {
            std::cerr << "No Matching Model!" << std::endl;
            exit(0);
        }

        pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*(model_set[model_idx].model_cloud), *buf_cloud, gt_poses[k].shift, gt_poses[k].rotation);

        viewer->removePolygonMesh(ss.str() + "_" + kk.str(), viewport);
        viewer->addPolygonMesh<myPointXYZ>(buf_cloud, model_set[model_idx].model_mesh->polygons, ss.str() + "_" + kk.str(), viewport);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[model_idx][0]/255.0, color[model_idx][1]/255.0, color[model_idx][2]/255.0, ss.str() + "_" + kk.str(), viewport);
    }
}

void drawBoundingBox(const pcl::PointCloud<myPointXYZ>::Ptr cloud, pcl::visualization::PCLVisualizer::Ptr viewer, int id)
{
    float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000, max_z = -1000, min_z = 1000;
    for( pcl::PointCloud<myPointXYZ>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        if( max_x < it->x ) max_x = it->x;
        if( max_y < it->y ) max_y = it->y;
        if( max_z < it->z ) max_z = it->z;
        
        if( min_x > it->x ) min_x = it->x;
        if( min_y > it->y ) min_y = it->y;
        if( min_z > it->z ) min_z = it->z;
    }
//    std::cerr << min_x << " " << max_x << " " << min_y << " " << max_y << " " << min_z << " " << max_z << std::endl;
//    std::cerr << "Hello!" << std::endl;
    std::stringstream ss;
    ss<<id;
    viewer->addCube(min_x, max_x, min_y, max_y, min_z, max_z, color[id][0]/255.0, color[id][1]/255.0, color[id][2]/255.0, "Cube " + ss.str());
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "Cube " + ss.str());
}

void drawMissed(const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, const std::vector<poseT> &est_poses, std::map<std::string, int> model_name_map, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    for(size_t i = 0 ; i < gt_poses.size() ; i++ )
    {
        bool flag = false;
        for(size_t j = 0 ; j < est_poses.size() ; j++ )
        {
            if( gt_poses[i].model_name == est_poses[j].model_name )
                flag = true;
        }
        if( flag == false )
        {
            int model_idx = model_name_map[gt_poses[i].model_name];
            pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
            pcl::transformPointCloud(*(model_set[model_idx].model_cloud), *buf_cloud, gt_poses[i].shift, gt_poses[i].rotation);
            
            
            drawBoundingBox(buf_cloud, viewer, model_idx);
        }
        

    }
}

void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, int colors[][3], bool red_flag, int viewport)
{
    if( red_flag )
    {
        pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::copyPointCloud(*label_cloud, *buf_cloud);
        viewer->addPointCloud(buf_cloud, "foreground", viewport);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "foreground", viewport);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "foreground", viewport);
        
    }
    else
    {
        pcl::PointCloud<PointT>::Ptr view_cloud(new pcl::PointCloud<PointT>());
        for( pcl::PointCloud<PointLT>::const_iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
        {
            if( pcl_isfinite(it->z) == false )
                continue;

            PointT pt;
            pt.x = it->x;
            pt.y = it->y;
            pt.z = it->z;
            pt.rgba = colors[it->label][0] << 16 | colors[it->label][1] << 8 | colors[it->label][2];
            view_cloud->push_back(pt);
        }
        viewer->addPointCloud(view_cloud, "label_cloud", viewport);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "label_cloud", viewport);
    }
}

int frame_count = 0;
void saveFrame(pcl::visualization::PCLVisualizer::Ptr viewer)
{
    std::stringstream ss;
    ss << frame_count;
    viewer->spinOnce(1);
    viewer->spin();
    viewer->saveScreenshot("better_video/failure_" + ss.str() +".png");
    frame_count++;
}

void setPos(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<float> x_axis, std::vector<float> center, std::vector<float> new_pos, float dist)
{
    std::vector<float> dir(3,0);
    dir[0] = -(new_pos[0] - center[0]);
    dir[1] = -(new_pos[1] - center[1]);
    dir[2] = -(new_pos[2] - center[2]);
    
    float dir_norm = vectorNorm(dir);
    
    float ratio = dist / dir_norm;
    
    std::vector<float> final_pos(3, 0);
    final_pos[0] = center[0] - dir[0]*ratio;
    final_pos[1] = center[1] - dir[1]*ratio;
    final_pos[2] = center[2] - dir[2]*ratio;
    
    std::vector<float> up_axis = crossProduct(x_axis, dir);
    
//    std::cerr << new_pos[0] << " " << new_pos[1] << " " << new_pos[2] << " "
//              <<center[0]-new_pos[0] << " " << center[1]-new_pos[1] << " " << center[2]-new_pos[2] << " "
//              << up_axis[0] << " " << up_axis[1] << " " << up_axis[2] << std::endl;
              
    viewer->setCameraPosition(final_pos[0], final_pos[1], final_pos[2], center[0], center[1], center[2], up_axis[0], up_axis[1], up_axis[2]);
//    viewer->spinOnce(1);
        
    saveFrame(viewer);
//    viewer->spin();
}

void MovingCamera(pcl::visualization::PCLVisualizer::Ptr viewer, const pcl::PointCloud<PointLT>::Ptr label_cloud)
{
    std::vector<float> x_axis(3, 0);
    x_axis[0] = 1.0;
    
    std::vector<float> center(3,0);
    int count = 0;
    for( pcl::PointCloud<PointLT>::const_iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == true && it->label > 0)
        {
            center[0] += it->data[0];
            center[1] += it->data[1];
            center[2] += it->data[2];
            count++;
        }
    }
    
    center[0] /= count;
    center[1] /= count;
    center[2] /= count;
    
//    pcl::PointCloud<PointT>::Ptr center_cloud(new pcl::PointCloud<PointT>());
//    center_cloud->resize(1);
//    center_cloud->at(0).x = center[0];
//    center_cloud->at(0).y = center[1];
//    center_cloud->at(0).z = center[2];
//    
//    viewer->addPointCloud(center_cloud, "center_cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "center_cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "center_cloud");
    
//    float dist = vectorNorm(center);
    float center_view = -center[0];
    float range = 1.0;
    int num = 30;
    float step = range*2.0 / num;
    
    for( int i = 0 ; i < num ; i++ )
    {
        std::vector<float> cur_pose(3, 0);
        cur_pose[0] = -range - center_view + step*i;
        setPos(viewer, x_axis, center, cur_pose, 1.0);
    }
}


int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    std::string pose_path("est_poses/tr0/");
    std::string gt_seg_path("../../data_pool/GT_segs/");
    std::string seg_path("../../data_pool/semanticLabels/");
    
    std::vector<std::string> inst_vec;
    std::vector<std::string> idx_vec;

// Weakly Cluttered Scenes
//    inst_vec.push_back("office_0");idx_vec.push_back("2");
//    inst_vec.push_back("labpod_1");idx_vec.push_back("10");
//    inst_vec.push_back("barrett_2");idx_vec.push_back("0");
//    inst_vec.push_back("box_2");idx_vec.push_back("9");
    
// Densely Cluttered Scenes    
//    inst_vec.push_back("office_3");idx_vec.push_back("4");
//    inst_vec.push_back("office_4");idx_vec.push_back("5");
//    inst_vec.push_back("office_5");idx_vec.push_back("60");
//    
//    inst_vec.push_back("labpod_6");idx_vec.push_back("17");
//    inst_vec.push_back("labpod_9");idx_vec.push_back("0");
//    
//    inst_vec.push_back("barrett_1");idx_vec.push_back("29");
//    inst_vec.push_back("barrett_3");idx_vec.push_back("42");
//    
//    inst_vec.push_back("box_3");idx_vec.push_back("95");
//    inst_vec.push_back("box_8");idx_vec.push_back("12");
//    inst_vec.push_back("box_9");idx_vec.push_back("19");
//    
//    inst_vec.push_back("shelf_1");idx_vec.push_back("31");
//    inst_vec.push_back("shelf_3");idx_vec.push_back("70");

// Failure Cases
    inst_vec.push_back("office_6");idx_vec.push_back("11");
    inst_vec.push_back("labpod_8");idx_vec.push_back("78");
    inst_vec.push_back("box_7");idx_vec.push_back("44");
    
    setObjID(model_name_map);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
//    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.0, 0, 0, 1, 0, -1, 0);
    viewer->setSize(640*2, 480);
    
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->addText("Frame 1", 10, 430, 40, 1.0, 1.0, 0.0, "v1 id", v1);
    viewer->addText("Viewport 1", 10, 10, 40, 1.0, 0.0, 0.0, "v1 text", v1);
    int v2(1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->addText("Frame 1", 10, 430, 40, 1.0, 1.0, 0.0, "v2 id", v2);
    viewer->addText("Viewport 2", 10, 10, 40, 1.0, 0.0, 0.0, "v2 text", v2);
    
    std::vector<ModelT> models;
    std::vector<std::string> model_names = readMesh(mesh_path, models, model_name_map);

    for( int j = 0 ; j < inst_vec.size() ; j++ )
    {
        std::stringstream ss;
        ss << j;
        
        std::string inst_name = inst_vec[j];
        std::string idx = idx_vec[j];
        
        std::string filename(in_path + inst_name + "/" + inst_name + "_" + idx + ".pcd");
        std::string seg_file(seg_path + inst_name + "/" + inst_name + "_" + idx + "_seg.pcd");
        std::string gt_seg_file(gt_seg_path + inst_name + "/" + inst_name + "_" + idx + "_gtseg.pcd");
        std::cerr << "Loading-"<< filename << std::endl;
    
        if( exists_test(filename) == false )
            continue;
        
        std::vector<poseT> gt_set, est_set;
        readPose(in_path + inst_name + "/poses/", idx, model_names, gt_set);
        readPose(pose_path + inst_name + "/", idx, model_names, est_set);
        
        if( gt_set.empty() == true )
            continue;
        //
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(filename, *cloud);
                
        pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
        std::vector<int> in_idx;
        pcl::removeNaNFromPointCloud(*cloud, *temp_cloud, in_idx);
        
        pcl::PointCloud<PointLT>::Ptr label_cloud(new pcl::PointCloud<PointLT>());
        pcl::io::loadPCDFile(seg_file, *label_cloud);
        
        pcl::PointCloud<PointLT>::Ptr gt_cloud(new pcl::PointCloud<PointLT>());
        pcl::io::loadPCDFile(gt_seg_file, *gt_cloud);
        
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->setCameraPosition(0, 0, 0.0, 0, 0, 1, 0, -1, 0);
        
        viewer->addText("#Frame "+ss.str(), 10, 430, 40, 1.0, 1.0, 0.0, "v1 id");
//        viewer->addText("#Frame "+ss.str(), 10, 430, 40, 1.0, 1.0, 0.0, "v2 id");
        
        viewer->addText("Original Scene", 10, 10, 40, 1.0, 0.0, 0.0, "v1 text", v1);
        viewer->addText("Foreground Extraction", 10, 10, 40, 1.0, 0.0, 0.0, "v2 text", v2);
        viewer->addPointCloud(temp_cloud, "scene1", v1);
        viewer->addPointCloud(temp_cloud, "scene2", v2);
        visualizeLabels(label_cloud, viewer, color, true, v2);
        saveFrame(viewer);
        
        viewer->removeAllPointClouds();
        viewer->updateText("Original Scene", 10, 10, 40, 1.0, 0.0, 0.0, "v1 text");
        viewer->updateText("Semantic Segmentation", 10, 10, 40, 1.0, 0.0, 0.0, "v2 text");
        viewer->addPointCloud(temp_cloud, "scene1", v1);
        viewer->addPointCloud(temp_cloud, "scene2", v2);
        visualizeLabels(label_cloud, viewer, color, false, v2);
        saveFrame(viewer);
        
        viewer->removeAllPointClouds();
        viewer->updateText("Original Scene", 10, 10, 40, 1.0, 0.0, 0.0, "v1 text");
        viewer->updateText("Pose Estimation", 10, 10, 40, 1.0, 0.0, 0.0, "v2 text");
        viewer->addPointCloud(temp_cloud, "scene1", v1);
        viewer->addPointCloud(temp_cloud, "scene2", v2);
        viewPose(models, est_set, model_name_map, viewer, v2);
        saveFrame(viewer);
        
        viewer->removeAllPointClouds();
        viewer->updateText("Pose Groundtruth", 10, 10, 40, 1.0, 0.0, 0.0, "v1 text");
        viewer->updateText("Pose Estimation", 10, 10, 40, 1.0, 0.0, 0.0, "v2 text");
        viewer->addPointCloud(temp_cloud, "scene1", v1);
        viewPose(models, gt_set, model_name_map, viewer, v1);
        viewer->addPointCloud(temp_cloud, "scene2", v2);
        viewPose(models, est_set, model_name_map, viewer, v2);
        
        drawMissed(models, gt_set, est_set, model_name_map, viewer);
//        saveFrame(viewer);
        MovingCamera(viewer, gt_cloud);
        
//        viewer->spin();
    }
    return 0;
}


//int main(int argc, char** argv)
//{
//    std::string in_path("/home/chi/JHUIT/scene/");
//    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
//    std::string pose_path("est_poses/tr0/");
//    std::string gt_seg_path("../../data_pool/GT_segs/");
//    std::string seg_path("../../data_pool/semanticLabels/");
//    
//    std::vector<std::string> inst_vec;
//    std::vector<std::string> idx_vec;
//
//// Weakly Cluttered Scenes
//    inst_vec.push_back("office_0");idx_vec.push_back("2");
//    inst_vec.push_back("labpod_1");idx_vec.push_back("10");
//    inst_vec.push_back("barrett_2");idx_vec.push_back("0");
//    inst_vec.push_back("box_2");idx_vec.push_back("9");
//    
//// Densely Cluttered Scenes    
////    inst_vec.push_back("office_3");idx_vec.push_back("4");
////    inst_vec.push_back("office_4");idx_vec.push_back("5");
////    inst_vec.push_back("office_5");idx_vec.push_back("60");
////    inst_vec.push_back("office_8");idx_vec.push_back("25");
////    
////    inst_vec.push_back("labpod_2");idx_vec.push_back("4");
////    inst_vec.push_back("labpod_6");idx_vec.push_back("17");
////    inst_vec.push_back("labpod_9");idx_vec.push_back("0");
////    
////    inst_vec.push_back("barrett_1");idx_vec.push_back("29");
////    inst_vec.push_back("barrett_3");idx_vec.push_back("42");
////    inst_vec.push_back("barrett_4");idx_vec.push_back("12");
////    
////    inst_vec.push_back("box_1");idx_vec.push_back("5");
////    inst_vec.push_back("box_3");idx_vec.push_back("95");
////    inst_vec.push_back("box_8");idx_vec.push_back("12");
////    inst_vec.push_back("box_9");idx_vec.push_back("19");
////    
////    inst_vec.push_back("shelf_0");idx_vec.push_back("10");
////    inst_vec.push_back("shelf_1");idx_vec.push_back("31");
////    inst_vec.push_back("shelf_2");idx_vec.push_back("15");
////    inst_vec.push_back("shelf_3");idx_vec.push_back("70");
////    inst_vec.push_back("shelf_5");idx_vec.push_back("1");
////    inst_vec.push_back("shelf_6");idx_vec.push_back("7");
//    
//// Failure Cases
////    inst_vec.push_back("office_6");idx_vec.push_back("11");
////    inst_vec.push_back("labpod_8");idx_vec.push_back("78");
////    inst_vec.push_back("box_7");idx_vec.push_back("44");
//    
//    
////    std::string inst_name("office_0"); 
////    int max_frames = 100;
////    pcl::console::parse_argument(argc, argv, "--i", inst_name);
//    
//    setObjID(model_name_map);
//    
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
//    viewer->initCameraParameters();
//    viewer->addCoordinateSystem(0.1);
//    viewer->setCameraPosition(0, 0, 0.0, 0, 0, 1, 0, -1, 0);
//    viewer->setSize(640*2, 480*2);
//    
//    int v1(0);
//    viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v1);
//    viewer->addText("Pose Estimation", 10, 10, 50, 1.0, 0.0, 0.0, "v1 text", v1);
//    int v2(1);
//    viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v2);
//    viewer->addText("Pose Groundtruth", 10, 10, 50, 1.0, 0.0, 0.0, "v2 text", v2);
//    int v3(2);
//    viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v3);
//    viewer->addText("Original Scene", 10, 10, 50, 1.0, 0.0, 0.0, "v3 text", v3);
//    int v4(3);
//    viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v4);
//    viewer->addText("Semantic Segmentation", 10, 10, 50, 1.0, 0.0, 0.0, "v4 text", v4);
//    
//    std::vector<ModelT> models;
//    std::vector<std::string> model_names = readMesh(mesh_path, models, model_name_map);
//
////    for( int j = 0 ; j < max_frames ; j++ )
//    for( int j = 0 ; j < inst_vec.size() ; j++ )
//    {
////        std::stringstream ss;
////        ss << j;
////        std::string idx = ss.str();
//
//        std::string inst_name = inst_vec[j];
//        std::string idx = idx_vec[j];
//        
//        std::string filename(in_path + inst_name + "/" + inst_name + "_" + idx + ".pcd");
//        std::string seg_file(seg_path + inst_name + "/" + inst_name + "_" + idx + "_seg.pcd");
//        std::string gt_seg_file(gt_seg_path + inst_name + "/" + inst_name + "_" + idx + "_gtseg.pcd");
//        std::cerr << "Loading-"<< filename << std::endl;
//    
//        if( exists_test(filename) == false )
//            continue;
//        
//        std::vector<poseT> gt_set, est_set;
//        readPose(in_path + inst_name + "/poses/", idx, model_names, gt_set);
//        readPose(pose_path + inst_name + "/", idx, model_names, est_set);
//        
//        if( gt_set.empty() == true )
//            continue;
//        //
//        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//        pcl::io::loadPCDFile(filename, *cloud);
//                
//        pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
//        std::vector<int> in_idx;
//        pcl::removeNaNFromPointCloud(*cloud, *temp_cloud, in_idx);
//        
//        pcl::PointCloud<PointLT>::Ptr label_cloud(new pcl::PointCloud<PointLT>());
//        pcl::io::loadPCDFile(seg_file, *label_cloud);
//        
//        pcl::PointCloud<PointLT>::Ptr gt_cloud(new pcl::PointCloud<PointLT>());
//        pcl::io::loadPCDFile(gt_seg_file, *gt_cloud);
//        
//        viewer->removeAllPointClouds();
//        
//        viewer->addPointCloud(temp_cloud, "scene1", v3);
////        visualizeLabels(label_cloud, viewer, color, true, v3);
//        
//        viewer->addPointCloud(temp_cloud, "scene2", v4);
//        visualizeLabels(label_cloud, viewer, color, false, v4);
//        
//        viewer->addPointCloud(temp_cloud, "scene3", v1);
//        viewPose(models, est_set, model_name_map, viewer, v1);
//        
//        viewer->addPointCloud(temp_cloud, "scene4", v2);
//        viewPose(models, gt_set, model_name_map, viewer, v2);
//        
//        MovingCamera(viewer, gt_cloud);
////        viewer->spin();
//    }
//    return 0;
//}
