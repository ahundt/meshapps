#include "index.h"
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/core/core.hpp>

#include "pcl/visualization/pcl_visualizer.h"

#define ELEV 0.135

struct KeyPt{
    PointT pt;
    int id;
    
};

pcl::PointCloud<PointT>::Ptr FilterBoundary(const pcl::PointCloud<PointT>::Ptr cloud);
pcl::PointCloud<PointT>::Ptr cropCloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr planeCoef, float elev);
Eigen::Matrix4f DenseColorICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &initial_guess);
void getKeyPt(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<aruco::Marker> &markers, std::vector<KeyPt> &key_set);

pcl::PointCloud<PointT>::Ptr convertToPC(const std::vector<KeyPt> &key_set);
Eigen::Matrix4f getMarkerPose(const aruco::Marker &marker);
Eigen::Matrix4f matchKeys(const std::vector<KeyPt> &k1, const std::vector<KeyPt> &k2);
Eigen::Matrix4f matchMarkers(const std::vector<aruco::Marker> &m1, const std::vector<aruco::Marker> &m2);
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Meshing(const std::vector< std::vector<KeyPt> > &key_set);

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/raw_mesh/");
    std::string out_path("../mesh/");
    std::string inst_name("sander");
    std::string seq_id("0");
    
    float elev = ELEV;
    int max_frames = 60;
    int sample_rate = 1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--s", seq_id);
    pcl::console::parse_argument(argc, argv, "--i", inst_name);
    pcl::console::parse_argument(argc, argv, "--elev", elev);
    pcl::console::parse_argument(argc, argv, "--max", max_frames);
    pcl::console::parse_argument(argc, argv, "--ss", sample_rate);
        
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    std::ifstream fp;
    fp.open((in_path+"/PlaneCoef_"+ seq_id + ".txt").c_str(), std::ios::in);
    if( fp.is_open() == false )
    {
        std::cerr << "Failed to open: " << in_path+"/PlaneCoef_"+ seq_id + ".txt" << std::endl;
        exit(0);
    }
    pcl::ModelCoefficients::Ptr planeCoef(new pcl::ModelCoefficients());
    planeCoef->values.resize(4);
    fp >> planeCoef->values[0] >> planeCoef->values[1] >> planeCoef->values[2] >> planeCoef->values[3];
    fp.close();
    
    
    aruco::MarkerDetector MDetector;
    aruco::CameraParameters CamParam;
    CamParam.readFromXMLFile("../carmine_109.yml");
    if ( CamParam.isValid() == false )
    {
        std::cerr << "The Camera Parameters Failed!" << std::endl;
        return 0;
    }
    
    std::vector< std::vector<KeyPt> > key_set;
    std::vector< pcl::PointCloud<PointT>::Ptr > cloud_set;
    std::vector< pcl::PointCloud<PointT>::Ptr > edge_set;
    for( int j = 0 ; j < max_frames ; j++ )
    {
        std::stringstream ss;
        ss << j;
        
        if ( j % sample_rate != 0 )
            continue;
        
        std::string filename(in_path + inst_name + "/" + inst_name + "_" + seq_id + "_" + ss.str() + ".pcd");
        std::cerr << "Loading-"<< filename << std::endl;
    
        if( exists_test(filename) == false )
            continue;
        
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(filename, *cloud);
        
        pcl::PointCloud<PointT>::Ptr temp_cloud = cropCloud(cloud, planeCoef, 0.110);
        
        temp_cloud = FilterBoundary(temp_cloud);
        
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (temp_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.1, 1.0);
        pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>());
        pass.filter (*cropped_cloud);
        
        cloud_set.push_back(cropped_cloud);
        
        //viewer->removeAllPointClouds();
        //viewer->addPointCloud(cropped_cloud, "cropped_cloud");
        //viewer->spin();
        
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
            
            {
                cv::LineIterator it1(rgb, markers[i][0], markers[i][1], 8);
                for(int k = 0; k < it1.count ; k++, it1++ )
                {
                    int idx = round(it1.pos().y) * w + round(it1.pos().x);
                    if( pcl_isfinite(cloud->at(idx).z) == true )
                        edge_cloud->push_back(cloud->at(idx));
                }
            }
            {
                cv::LineIterator it2(rgb, markers[i][1], markers[i][2], 8);
                for(int k = 0; k < it2.count ; k++, it2++ )
                {
                    int idx = round(it2.pos().y) * w + round(it2.pos().x);
                    if( pcl_isfinite(cloud->at(idx).z) == true )
                        edge_cloud->push_back(cloud->at(idx));
                }
            }
            {
                cv::LineIterator it3(rgb, markers[i][2], markers[i][3], 8);
                for(int k = 0; k < it3.count ; k++, it3++ )
                {
                    int idx = round(it3.pos().y) * w + round(it3.pos().x);
                    if( pcl_isfinite(cloud->at(idx).z) == true )
                        edge_cloud->push_back(cloud->at(idx));
                
                }
            }
            {
                cv::LineIterator it4(rgb, markers[i][3], markers[i][0], 8);
                for(int k = 0; k < it4.count ; k++, it4++ )
                {
                    int idx = round(it4.pos().y) * w + round(it4.pos().x);
                    if( pcl_isfinite(cloud->at(idx).z) == true )
                        edge_cloud->push_back(cloud->at(idx));
                
                }
            }
        }
        
        //cv::imshow("in",rgb);
        //cv::waitKey();//wait for key to be pressed
        
        std::vector<KeyPt> temp_key;
        
        getKeyPt(cloud, markers, temp_key);
        key_set.push_back(temp_key);
        
        edge_set.push_back(edge_cloud);
        /*
        std::cerr << edge_cloud->size() << std::endl;
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cropped_cloud, "cropped_cloud");
        viewer->addPointCloud(edge_cloud, "edge_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "edge_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "edge_cloud");
        viewer->spin();
        */
        //if ( CamParam.isValid() ){
        //    for (unsigned int i=0;i<markers.size();i++) 
        //        aruco::CvDrawingUtils::draw3dCube(rgb, markers[i], CamParam);
        //}
        /*
        std::cerr << markers.size() << std::endl;
        Eigen::Matrix4f tran = getMarkerPose(markers[1]);
        pcl::PointCloud<PointT>::Ptr cur_cloud(new pcl::PointCloud<PointT>());
        tran = tran.inverse();
        pcl::transformPointCloud(*cropped_cloud, *cur_cloud, tran);
        
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cur_cloud, "cur_cloud");
        viewer->spin();
        //*/
        /*
        pcl::PointCloud<PointT>::Ptr keys = convertToPC(key_set[j]);
        
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud, "full_cloud");
        viewer->addPointCloud(keys, "keys");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keys");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keys");
        viewer->spin();
        //*/
        
    }
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*cloud_set[0], *full_cloud);
    
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_set = Meshing(key_set);
    
    int total = key_set.size();
    std::vector<pcl::PointCloud<PointT>::Ptr> tran_cloud_set(total);
    tran_cloud_set[0] = edge_set[0];
    for( int j = 1 ; j < total ; j++ )
    {
        std::cerr << "Registering --- " << j << std::endl;
        
        //Eigen::Matrix4f init_tran = matchKeys(key_set[j],key_set[j-1]);//matchMarkers(marker_set[j], marker_set[j-1]);
        //Eigen::Matrix4f cur_tran = DenseColorICP(cloud_set[j], cloud_set[j-1], init_tran);
        Eigen::Matrix4f cur_tran = DenseColorICP(edge_set[j], tran_cloud_set[0], rot_set[j]);
        //Eigen::Matrix4f cur_tran = rot_set[j];
        
        pcl::PointCloud<PointT>::Ptr cur_cloud(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cloud_set[j], *cur_cloud, cur_tran);
        
        tran_cloud_set[j] = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*edge_set[j], *tran_cloud_set[j], cur_tran);
        //tran_cloud_set[j] = cur_edge;
        
        full_cloud->insert(full_cloud->end(), cur_cloud->begin(), cur_cloud->end());
        full_cloud->is_dense = false;
        
        viewer->removeAllPointClouds();
        viewer->addPointCloud(full_cloud, "full_cloud");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "full_model");
        //viewer->addPointCloudNormals<PointT, NormalT> (full_model, full_model_normals, 5, 0.01, "normals");
        viewer->spinOnce(1);
    }
    pcl::PointCloud<PointT>::Ptr temp_model = cropCloud(full_cloud, planeCoef, elev);
    pcl::PointCloud<PointT>::Ptr final_model(new pcl::PointCloud<PointT>());
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(temp_model);
    sor.setLeafSize(0.0015, 0.0015, 0.0015);
    sor.filter(*final_model);
    
    std::vector<int> in_idx;
    pcl::removeNaNFromPointCloud(*final_model, *final_model, in_idx);
    
    pcl::io::savePCDFile(out_path+"full_"+inst_name+".pcd", *final_model);
    pcl::PLYWriter ply_writer;
    ply_writer.write(out_path+"full_"+inst_name+".ply", *final_model, true);
    
    viewer->removeAllPointClouds();
    viewer->addPointCloud(final_model, "final_model");
    viewer->spin();
    
    return 0;
}


void getKeyPt(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<aruco::Marker> &markers, std::vector<KeyPt> &key_set)
{
    int w = cloud->width;
    for( size_t i = 0 ; i < markers.size() ; i++ )
    {
        for(int j = 0 ; j < 4 ; j++ )
        {
            int r = round(markers[i][j].y);
            int c = round(markers[i][j].x);
            int idx = r*w + c;
            if( pcl_isfinite(cloud->at(idx).z) == true )
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

Eigen::Matrix4f getMarkerPose(const aruco::Marker &marker)
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    bool invalid=false;
    for (int i=0;i<3 && !invalid ;i++)
    {
        if (marker.Tvec.at<float>(i,0)!=-999999) invalid |= false;
        if (marker.Rvec.at<float>(i,0)!=-999999) invalid |= false;
    }
    if (invalid) 
    {
        std::cerr << "Pose Invalid!" << std::endl;
        return pose;
    }
    cv::Mat Rot(3,3,CV_32FC1),Jacob;
    cv::Rodrigues(marker.Rvec, Rot, Jacob);

    double para[3][4];
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++) 
            pose(i, j) = Rot.at<float>(i,j);
    //now, add the translation
    pose(0, 3) = marker.Tvec.at<float>(0,0);
    pose(1, 3) = marker.Tvec.at<float>(1,0);
    pose(2, 3) = marker.Tvec.at<float>(2,0);

    return pose;
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

Eigen::Matrix4f matchMarkers(const std::vector<aruco::Marker> &m1, const std::vector<aruco::Marker> &m2)
{
    //match m1 to m2
    for(int i = 0 ; i < m1.size(); i++ ){
        for(int j = 0 ; j < m2.size(); j++ )
        {
            if( m1[i].id == m2[j].id )
            {
                Eigen::Matrix4f tran1 = getMarkerPose(m1[i]);
                Eigen::Matrix4f tran2 = getMarkerPose(m2[j]);
                
                Eigen::Matrix4f tran = tran1.inverse() * tran2;
                return tran;
            }
        }
    }
    
    std::cerr << "Matching Failed!" << std::endl;
    return Eigen::Matrix4f::Identity();
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Meshing(const std::vector< std::vector<KeyPt> > &key_set)
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_set(key_set.size());
    
    Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    rot_set[0] = identity;
    for( int j = 1 ; j < rot_set.size() ; j++ )
    {
        Eigen::Matrix4f init_tran;
        init_tran = matchKeys(key_set[j],key_set[0]);
        
        if( init_tran == identity )
        {
            for( int k = j-1 ; k > 0 ; k-- )
            {
                init_tran = matchKeys(key_set[j],key_set[k]);
                if( init_tran != identity )
                {
                    init_tran = rot_set[k] * init_tran;
                    break;
                }
            }
        }
        
        rot_set[j] = init_tran;
    }

    return rot_set;
}


pcl::PointCloud<PointT>::Ptr cropCloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr planeCoef, float elev)
{
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*cloud, *cloud_f);
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_f);
    proj.setModelCoefficients (planeCoef);

    pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>());
    proj.filter (*cloud_projected);
    for(size_t i = 0 ; i < cloud_f->size() ; i++ )
    {
        if( pcl_isfinite(cloud_f->at(i).z) == true )
        {
            float diffx = cloud_f->at(i).x-cloud_projected->at(i).x;
            float diffy = cloud_f->at(i).y-cloud_projected->at(i).y;
            float diffz = cloud_f->at(i).z-cloud_projected->at(i).z;

            float dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz);
            if ( dist <= elev )
            {
                cloud_f->at(i).x = NAN;
                cloud_f->at(i).y = NAN;
                cloud_f->at(i).z = NAN;
            }
        }
    }
    cloud_f->is_dense = false;
    return cloud_f;
}

pcl::PointCloud<PointT>::Ptr FilterBoundary(const pcl::PointCloud<PointT>::Ptr cloud)
{
    float threshold = 0.04;
    int BLen = 5;
    
    int w = cloud->width;
    int h = cloud->height;
    int num = cloud->size();
    cv::Mat depthmap = cv::Mat::ones(h, w, CV_32FC1)*-1000;
    
    for(int i = 0 ; i < num; i++ ){
        if( pcl_isfinite(cloud->at(i).z) == true ){
            int r_idx = i / w;
            int c_idx = i % w;
            depthmap.at<float>(r_idx, c_idx) = cloud->at(i).z;
        }
    }
    
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    for(int i=0 ; i<num; i++ ){
        
        int row = i / w;
        int col = i % w;
        if( pcl_isfinite(cloud->at(i).z) == true )
        {
            float zCur = depthmap.at<float>(row, col);
            float max = zCur, min = zCur;
            for( int j = row - BLen ; j <= row + BLen ; j++ ){
                for( int k = col - BLen ; k <= col + BLen ; k++ )
                {
                    if( j < 0 || k < 0 || j >= h || k >= h || pcl_isfinite(depthmap.at<float>(j, k)) == false )
                        continue;
                    if( depthmap.at<float>(j, k) > max )
                        max = depthmap.at<float>(j, k);
                    if( depthmap.at<float>(j, k) < min )
                        min = depthmap.at<float>(j, k);
                }
            }
            
            if( fabs(max-zCur) <= threshold && fabs(min-zCur) <= threshold)
                cloud_f->push_back(cloud->at(i));
        }
    }
    return cloud_f;
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
                    //std::cerr<<"Finding..."<<scene_hue->points[pointIdx[j]].h <<" "<<model_hue->points[i].h<<std::endl;
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
                //std::cerr<<diffcurvature<<" ";
                //std::cerr<<diffmin<<" ";
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
