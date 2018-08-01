#ifndef _JENGA_GAME_
#define _JENGA_GAME_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
//#include <pcl/common/impl/io.hpp> //copyPointCloud
#include <pcl/features/normal_3d_omp.h> //法线
#include <pcl/features/shot_omp.h>  //描述子
#include <pcl/features/board.h>
#include <pcl/features/fpfh_omp.h>    //包含fpfh加速计算的omp(多核并行计算)
#include <pcl/correspondence.h>   //对应表示两个实体之间的匹配（例如，点，描述符等）。
#include <pcl/registration/ia_ransac.h> //pcl::SampleConsensusInitialAlignment
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/recognition/cg/hough_3d.h> //hough算子
#include <pcl/recognition/cg/geometric_consistency.h> //几何一致性
#include <pcl/filters/uniform_sampling.h> //均匀采样
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>  //配准方法
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>  //转换矩阵
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <string>
using namespace std;

enum InitMode{leftbottom_3_rectangle, leftbottom_1_rectangle};


typedef OpenMesh::PolyMesh_ArrayKernelT<> MyMesh;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PC;
typedef pcl::PointCloud<PointT>::Ptr PCPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PCCPtr;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

class Jenga
{
public:
	Jenga(int jenga_layer, bool mode = leftbottom_3_rectangle, float bigJLeafSize = 0.005):
        JengaLength(0.144), JengaWidth(0.048), JengaHeight(0.025), JengaLayer(jenga_layer), bigJengaNumSample(jenga_layer * 20000),
        bigJengaLeafSize(bigJLeafSize), MapCloud(new PC), capCloudPassed(new PC), capCloudNoPlane(new PC),
        capCloudNoOut(new PC), capCloudSegUnique(new PC), initCenCloud(new PC), Mode(mode), JengaModel(new PC),
        model(new PC), model_keypoints(new PC), scene(new PC), scene_keypoints(new PC), initBigJengaPC(new PC),
        model_normals(new pcl::PointCloud<pcl::Normal>),scene_normals(new pcl::PointCloud<pcl::Normal>),
        model_descriptors(new pcl::PointCloud<pcl::SHOT352>), scene_descriptors(new pcl::PointCloud<pcl::SHOT352>),
        JengaVertex(new PC)
	{
        model_ss_ = 0.01f;
        scene_ss_ = 0.03f;
        descr_rad_ = 0.02f;
        rf_rad_ = 0.015f;
        cg_size_ = 0.01f;
        cg_thresh_ = 5.0f;
        BigJengaName = "big_jenga.ply";
        JengaName = "jenga_model.ply";
		BigJengaSim();
        initJengaModel();
		MeshToPC(BigJengaName, initBigJengaPC, bigJengaNumSample);
        MeshToPC(JengaName, JengaModel, bigJengaNumSample/3);
        initCentroid();
	}

	~Jenga(){};

    /**
     * compute each small world jenga center
     * @param map_cloud world cloud
     * @param point_center_out estimate jenga center
     */
    void regisPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & map_cloud, PCPtr & point_center_out);

    /**
     * get each small registered jenga vertexes
     * @return
     */
    PCPtr getJengaVertex() {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*JengaVertex, *tempPC);
        return tempPC;
    }

    /**
     * get big simulated dense jenga cloud
     * @return
     */
	PCPtr getModel() {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*model, *tempPC);
        return tempPC;
    }

    /**
     *
     */
    PCPtr getSmallModel() {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*JengaModel, *tempPC);
        return tempPC;
    }

    /**
     * get passed points
     * @return
     */
    PCPtr getCloudPassed()
    {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*capCloudPassed, *tempPC);
        return tempPC;
    }

    /**
     * get removed plane cloud
     * @return
     */
    PCPtr getCloudNoPlane()
    {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*capCloudNoPlane, *tempPC);
        return tempPC;
    }

    /**
     * get a point cloud with all unregistered simulate small jenga centers
     * @return
     */
    PCPtr getInitCenPC()
    {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*initCenCloud, *tempPC);
        return tempPC;
    }

    /**
     * get a point cloud with no isolated points
     * @return
     */
    PCPtr getCloudNoOut()
    {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*capCloudNoOut, *tempPC);
        return tempPC;
    }

    /**
     * same as getModel()
     * @return
     */
    PCPtr getInitBigJengaPC()
    {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*initBigJengaPC, *tempPC);
        return tempPC;
    }

    /**
     * get segmented cloud
     * @return
     */
    PCPtr getCloudSegUnique()
    {
        PCPtr tempPC(new PC);
        pcl::copyPointCloud(*capCloudSegUnique, *tempPC);
        return tempPC;
    }

    /**
     * get pose of big jenga
     * @return
     */
    Eigen::Matrix4f getModeltoWorldTrans(){
        Eigen::Matrix4f tmpMatrix;
        tmpMatrix = ModelToWorldTrans;
        return tmpMatrix;
    };

    /**
     * get pose of each small jenga
     * @return
     */
    vector<Eigen::Matrix4f > getEachTransMatrix(){
        vector<Eigen::Matrix4f > tmpMatrix(eachTrans);
        return tmpMatrix;
    }

    /**
     * get pose of each small jenga
     * @return
     */
    vector<Eigen::Affine3f > getEachTransAffine(){
        vector<Eigen::Affine3f > tmpAffine(eachTrans3);
        return tmpAffine;
    }


private:
    bool show_keypoints_;
    bool show_correspondences_;
    bool use_cloud_resolution_;
    bool use_hough_;
    float model_ss_;    //均匀采样
    float scene_ss_;
    float descr_rad_;   //计算关键点描述子的搜索半径
    float rf_rad_;      //参考帧搜索半径
    float cg_size_;     //霍夫空间设置每个bin的大小
    float cg_thresh_;   //阈值

    bool Mode;  //3 small rectangles position in the bottom
    int JengaLayer;
    // how many points to fill in mesh
	int bigJengaNumSample;
    // decide the density of the simulated jenga point cloud
	float bigJengaLeafSize;
    // length of a small jenga
	float JengaLength;
    // width of a small jenga
    float JengaWidth;
    // height of a small jenga
	float JengaHeight;
    string BigJengaName;
    string JengaName;
    PCPtr initBigJengaPC; //模拟大积木稠密点云
    PCPtr model; //模拟大积木稠密点云
    PCPtr model_keypoints; // 模型角点
    PCPtr scene; // 目标点云
    PCPtr scene_keypoints; //目标角点
    pcl::PointCloud<pcl::Normal>::Ptr model_normals; //法线
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals;
    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors; //描述子
    pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors;

    // a point cloud with each simulate small jenga vertexs(8 vertexs)
    PCPtr JengaVertex;
    // small jenga dense point cloud
    PCPtr JengaModel;
    // a point cloud with all unregistered simulate small jenga centers
    PCPtr initCenCloud;
    PCPtr MapCloud;
    PCPtr capCloudPassed;
    PCPtr capCloudNoPlane;
    PCPtr capCloudNoOut;
    PCPtr capCloudSegUnique;
    Eigen::Matrix4f ICPTrans;
    // pose from simulated big jenga to world one
    Eigen::Matrix4f ModelToWorldTrans;
    vector<Eigen::Matrix4f > eachTrans;
    vector<Eigen::Affine3f > eachTrans3;
    PCPtr lastTransPC;
    PCPtr currentTransPC;

    /**
    * passes points in a cloud based on constraints for one particular field of the point type.
    * @param cloud input cloud
    * @param cloud_filtered output cloud
    */
    void PointPass(PCPtr &cloud, PCPtr &cloud_filtered);    //直通滤波

    /**
     * remove plane with RANSAC
     * @param cloud input cloud
     * @param cloud_no_plane output cloud
     */
    void PointRemovePlane(PCPtr &cloud, PCPtr &cloud_no_plane);     //去除平面

    /**
     * filters points in a cloud based on the number of neighbors they have.
     * @param cloud input cloud
     * @param cloud_filtered output cloud
     */
    void PointOutRemove(PCPtr &cloud, PCPtr &cloud_filtered);       //去除离群点

    // TODO: the prior information may not be true
    /**
     * seg points with prior fixed size and choose the biggest one as jenga
     * @param cloud input cloud
     * @param cloud_unique output cloud
     */
    void PointSegUnique(PCPtr &cloud, PCPtr &cloud_unique);     //分割出目标积木堆

    /**
     * compute pose with ICP
     * @param cloud_sources source cloud
     * @param cloud_target target cloud
     * @param matrix4 pose
     */
    void PointICP(PCPtr &cloud_sources, PCPtr &cloud_target, Eigen::Matrix<float,4,4> & matrix4);

    /**
     * construct a small jenga mesh with 8 vertexes
     */
    void initJengaModel();

    /**
     * compute all small jenga center points into a point cloud according to layers
     */
    void initCentroid();

    /**
     * fill points into mesh's faces and get a dense point cloud
     * @param mesh_name mesh name
     * @param output_cloud dense point cloud from a sparse mesh
     * @param sample_points how many points to fill in mesh
     */
    void MeshToPC(string & mesh_name, PCPtr & output_cloud, int sample_points);

    /**
     * compute fpfh feature
     * @param input_cloud input cloud
     * @param tree search tree
     * @return computed features
     */
    fpfhFeature::Ptr compute_fpfh_feature(PCPtr input_cloud,pcl::search::KdTree<PointT>::Ptr tree);

    /**
     * register point cloud with fpfh features
     * @param source
     * @param target
     */
    void regisWithFPFH(PCPtr & source, PCPtr & target);

    /**
     * assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
     * @param in_out_cloud input output cloud
     * @param leaf_size downsample level
     */
    void computeVoxelGrid(PCPtr &in_out_cloud, float leaf_size);

    /**
     * simulation of a big jenga
     */
    void BigJengaSim();
};






#endif