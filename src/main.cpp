#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/common/actor_map.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/area_picking_event.h>
#include <pcl/visualization/interactor_style.h>
#include <sstream>
#include "../include/jenga_game.hpp"
#include "../include/k2g.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    if(argc!=4)
    {
        cout << "argc must be 4!" << endl;
        cout << "example: ./jenga file_path num_of_layers(3) which_mode(1)" << endl;
        cout << "notice: mode 0 means leftbottom_3_rectangle, while 1 means leftbottom_1_rectangle" << endl;
        return -1;
    }
    string file_path = argv[1];
    int jenga_layer = atoi(argv[2]);  // 14
    bool jenga_bottom = atoi(argv[3]);  //

//    Processor freenectprocessor = OPENGL;
//    std::vector<int> ply_file_indices;
//
//    // TODO: color and depth image capture, should be better in ROS
//    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
//    K2G k2g(freenectprocessor);
//    cloud = k2g.getCloud();
//
//    k2g.printParameters();
//
//    cloud->sensor_orientation_.w() = 0.0;
//    cloud->sensor_orientation_.x() = 1.0;
//    cloud->sensor_orientation_.y() = 0.0;
//    cloud->sensor_orientation_.z() = 0.0;

    cv::Mat color, depth;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr capColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PCPtr bigJengaPC(new PC);
    PCPtr worldJengaCloud(new PC);
    PCPtr CenterPoint(new PC);
    PCPtr initCenterPoint(new PC);
    PCPtr testPoint(new PC);
    PCPtr cloud_no_plane(new PC);
    PCPtr cloud_no_out(new PC);
    PCPtr cloud_seg_unique(new PC);
    PCPtr modelVertexPose(new PC);
    vector<PC > eachJengaVertexPose; //记录每个积木的顶点坐标
    Eigen::Matrix4f icpTrans;
    pcl::io::loadPLYFile(file_path, *capColorCloud);

    // Jenga construct
    Jenga j_game(jenga_layer,jenga_bottom);
    modelVertexPose = j_game.getJengaVertex();
    j_game.regisPointCloud(capColorCloud, CenterPoint);
    bigJengaPC = j_game.getModel();
    worldJengaCloud = j_game.getCloudSegUnique();
    testPoint = j_game.getCloudPassed();
    icpTrans = j_game.getModeltoWorldTrans();
    initCenterPoint = j_game.getInitCenPC();

    // view init
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    viewer->setBackgroundColor(0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//
//    PlySaver ps(cloud, false, false, k2g);
//    viewer->registerKeyboardCallback(KeyboardEventOccurred, (void *) &ps);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> icp_view(new pcl::visualization::PCLVisualizer("icp view"));
    icp_view->setBackgroundColor(0.0, 0, 0);
    icp_view->initCameraParameters();


    int v1, v2, v3, v4;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> test_view(new pcl::visualization::PCLVisualizer("test view"));
    test_view->setBackgroundColor(0.0, 0, 0);
    test_view->initCameraParameters();
    test_view->addCoordinateSystem(0.01);
    test_view->addPointCloud(testPoint);
    test_view->createViewPort(0.0,0.0,0.5,0.5,v1);
    test_view->createViewPort(0.5,0.0,1.0,0.5,v2);
    test_view->createViewPort(0.0,0.5,0.5,1.0,v3);
    test_view->createViewPort(0.5,0.5,1.0,1.0,v4);


    cout << icpTrans << endl;
    int icp_v1,icp_v2;
    icp_view->createViewPort(0.0,0.0,0.5,1.0,icp_v1);
    icp_view->createViewPort(0.5,0.0,1.0,1.0,icp_v2);
    icp_view->addCoordinateSystem(0.04, "icp_v1", icp_v1);
    icp_view->addCoordinateSystem(0.04, "icp_v2", icp_v2);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> sources_cloud_color(bigJengaPC,
                                                                                        250, 0, 0);
    icp_view->addPointCloud(bigJengaPC, sources_cloud_color, "source_cloud", icp_v1);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(worldJengaCloud,
//                                                                                       0, 250, 0);
    icp_view->addPointCloud(worldJengaCloud, "target_cloud", icp_v1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> center_cloud_color(CenterPoint,
                                                                                       0, 250, 250);
    icp_view->addPointCloud(CenterPoint, center_cloud_color, "center_cloud", icp_v1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> center_cloud_color_v2(initCenterPoint,
                                                                                       0, 250, 250);
    icp_view->addPointCloud(initCenterPoint, center_cloud_color_v2, "center_cloud_v2", icp_v2);



    // repleace world jenga centers with big sphere for better view
    for (int i = 0; i < CenterPoint->points.size(); i++) {
        stringstream ss;
        ss << "sphere_" << i;
        icp_view->addSphere(CenterPoint->points[i], 0.003, ss.str(), icp_v1);
//        icp_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
//        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, ss.str());
    }

    // add local coordinate for each small jenga
    Eigen::Affine3f t;
    vector<Eigen::Matrix4f > eachMatrix;
    eachMatrix = j_game.getEachTransMatrix();
    for (int i = 0; i < eachMatrix.size(); i++)
    {
        stringstream ss;
        ss << "coordinate_" << i;
        t.matrix() =  icpTrans * eachMatrix[i];
        icp_view->addCoordinateSystem(0.04, t, ss.str(), icp_v1);
        test_view->addCoordinateSystem(0.04, t, ss.str(), v1);
    }

    // transform simulated vertexes into world vertexes
    for(int i = 0; i < eachMatrix.size(); i++)
    {
        PCPtr tempPC(new PC);
        pcl::transformPointCloud(*modelVertexPose, *tempPC, icpTrans * eachMatrix[i]);
        eachJengaVertexPose.push_back(*tempPC);
    }

    // repleace world jenga vertexes with big sphere for better view
    for (int i = 0; i < eachJengaVertexPose.size(); i++) {
        stringstream ss;

        for(int j = 0; j < 8; j++)
        {
            ss << "sphere_" << i << "_" << j;
            icp_view->addSphere(eachJengaVertexPose[i].points[j], 0.003, ss.str(), icp_v1);
            test_view->addSphere(eachJengaVertexPose[i].points[j], 0.003, ss.str(), v1);
        }

        //        icp_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        //        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, ss.str());
    }

    // add local coordinate for each small jenga in icp_v2
    for (int i = 0; i < eachMatrix.size(); i++)
    {
        stringstream ss;
        ss << "coordinate_v2_" << i;
        t.matrix() = icpTrans * eachMatrix[i];
        icp_view->addCoordinateSystem(0.04, t, ss.str(), icp_v2);
    }
//    icp_view->addPointCloud(capCloudSegUnique);

    // add point clouds to view
    testPoint = j_game.getCloudPassed();
    cloud_no_plane = j_game.getCloudNoPlane();
    cloud_no_out = j_game.getCloudNoOut();
    cloud_seg_unique = j_game.getCloudSegUnique();
    test_view->addPointCloud(testPoint, "cloud_passed", v1);
    test_view->addPointCloud(cloud_no_plane, "cloud_no_plane", v2);
    test_view->addPointCloud(cloud_no_out, "cloud_no_out", v3);
    test_view->addPointCloud(cloud_seg_unique, "cloud_seg", v4);

    Eigen::Vector4f last_centroid = Eigen::Vector4f(0,0,0,0);
    Eigen::Vector4f current_centroid;
    vector<PC > frameClouds;
    double rotate_angle = 0;
    bool getFitInitPose = false;
    test_view->spin();

//    while(!viewer->wasStopped()){
////        bigJengaView->spinOnce();
//        viewer->spinOnce ();
//        std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();
//
//        k2g.get(color, depth, cloud);
//        // Showing only color since depth is float and needs conversion
////        cv::imshow("color", color);
//        int c = cv::waitKey(1);
//
////        worldJengaCloud->clear();
////        if(!getFitInitPose)
////        {
////            j_game.regisPointCloud(cloud, CenterPoint);
////            icpTrans = j_game.getModeltoWorldTrans();
////            std::cout << "icp_trans: " << icpTrans << std::endl;
////            bigJengaPC = j_game.getModel();
////            pcl::compute3DCentroid(*bigJengaPC, current_centroid);
////            float centroidDis = (current_centroid-last_centroid).squaredNorm();
////            std::cout << "centroid error: " << last_centroid << std::endl << current_centroid << std::endl;
////            std::cout << centroidDis << std::endl;
////            if(centroidDis < 1e-05)
////            {
////                getFitInitPose = true;
////            }
////            last_centroid = current_centroid;
////
////            worldJengaCloud = j_game.getCloudSegUnique();
////            initCenterPoint = j_game.getInitCenPC();
////            icp_view->updatePointCloud(bigJengaPC, sources_cloud_color, "source_cloud");
////            icp_view->updatePointCloud(worldJengaCloud, "target_cloud");
////            icp_view->updatePointCloud(CenterPoint, center_cloud_color, "center_cloud");
////            icp_view->updatePointCloud(initCenterPoint, center_cloud_color_v2, "center_cloud_v2");
////            for (int i = 0; i < CenterPoint->points.size(); i++) {
////                stringstream ss;
////                ss << "sphere_" << i;
////                icp_view->updateSphere(CenterPoint->points[i], 0.003, 0.5, 0.5, 0.5, ss.str());
////    //        icp_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
////    //        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, ss.str());
////            }
////
////            vector<Eigen::Matrix4f > upEachMatrix;
////            upEachMatrix = j_game.getEachTransMatrix();
////            eachJengaVertexPose.clear();
////            for (int i = 0; i < eachMatrix.size(); i++)
////            {
////                PC tempPC;
////                stringstream ss;
////                ss << "coordinate_" << i;
////                t.matrix() = icpTrans;
////                icp_view->updateCoordinateSystemPose(ss.str(), t);  //在原坐标的基础上乘以一个旋转矩阵
////                Eigen::Matrix4f tempMatrix;
////                tempMatrix = icpTrans * eachMatrix[i];
////                pcl::transformPointCloud(*modelVertexPose, tempPC, tempMatrix);
////                eachJengaVertexPose.push_back(tempPC);
////            }
////            if(jenga_bottom!=(jenga_layer%2))
////            {
////                rotate_angle = asin(abs(eachJengaVertexPose[eachJengaVertexPose.size()-1].points[2].x-
////                                        eachJengaVertexPose[eachJengaVertexPose.size()-1].points[6].x)/0.144f);
////                rotate_angle = rotate_angle / M_PI * 180;
////            } else
////            {
////                rotate_angle = M_PI/2-(abs(eachJengaVertexPose[eachJengaVertexPose.size()-1].points[2].x-
////                                           eachJengaVertexPose[eachJengaVertexPose.size()-1].points[6].x)/0.144f);
////                rotate_angle = rotate_angle / M_PI * 180;
////            }
//////            std::cout << eachJengaVertexPose[eachJengaVertexPose.size()-1].points[]
////            for (int i = 0; i < eachJengaVertexPose.size(); i++) {
////                stringstream ss;
////                for(int j = 0; j < 8; j++)
////                {
////                    ss << "sphere_" << i << "_" << j;
////                    icp_view->updateSphere(eachJengaVertexPose[i].points[j], 0.003, 0.0,0.5,0.5,ss.str());
////                }
////
////                //        icp_view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
////                //        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, ss.str());
////            }
////            testPoint = j_game.getCloudPassed();
////            cloud_no_plane = j_game.getCloudNoPlane();
////            cloud_no_out = j_game.getCloudNoOut();
////            cloud_seg_unique = j_game.getCloudSegUnique();
////            test_view->updatePointCloud(testPoint,"cloud_passed");
////            test_view->updatePointCloud(cloud_no_plane, "cloud_no_plane");
////            test_view->updatePointCloud(cloud_no_out, "cloud_no_out");
////            test_view->updatePointCloud(cloud_seg_unique, "cloud_seg");
////
////        }
//        std::cout << "Rotate Angle: " << rotate_angle << std::endl;
//
//        std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
////        std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
//        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//        viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
////        bigJengaView->updatePointCloud(bigJengaCloud);
//
//    }


	return 0;
}