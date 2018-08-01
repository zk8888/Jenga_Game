#include "jenga_game.hpp"

void Jenga::initCentroid() {
	initCenCloud->width = JengaLayer * 3;
	initCenCloud->height = 1;
	initCenCloud->points.resize(initCenCloud->width*initCenCloud->height);
    eachTrans.clear();
    eachTrans3.clear();
    eachTrans.reserve(JengaLayer * 3);
    eachTrans3.reserve(JengaLayer * 3);
    Eigen::Matrix4f tmp1Matrix;
    Eigen::Matrix4f tmp2Matrix;
    Eigen::Affine3f tmpAffine;
    Eigen::AngleAxisf rotationVector(-1*M_PI/2,Eigen::Vector3f::UnitZ()); //(M_PI/2, Eigen::Vector3f(0,0,1))
    Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
    rotationMatrix = rotationVector.toRotationMatrix();
    tmp1Matrix << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
    tmp2Matrix << rotationMatrix(0,0), rotationMatrix(0,1),rotationMatrix(0,2),0,
            rotationMatrix(1,0),rotationMatrix(1,1),rotationMatrix(1,2),0,
            rotationMatrix(2,0),rotationMatrix(2,1),rotationMatrix(2,2),0,
            0,0,0,1;
	if(!Mode)    //左面底部为3个小矩形块
	{
		for(int i = 0; i < JengaLayer; i++)
		{
			if(!(i%2))
			{
				initCenCloud->points[i*3+0].x = -1 * JengaWidth;
                initCenCloud->points[i*3+0].y = 0;
                initCenCloud->points[i*3+0].z = JengaHeight / 2 + i * JengaHeight;
                tmp1Matrix(0,3) = -1 * JengaWidth;
                tmp1Matrix(1,3) = 0;
                tmp1Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp1Matrix);
                tmpAffine.matrix() = tmp1Matrix;
                eachTrans3.push_back(tmpAffine);
                initCenCloud->points[i*3+1].x = 0;
                initCenCloud->points[i*3+1].y = 0;
                initCenCloud->points[i*3+1].z = JengaHeight / 2 + i * JengaHeight;
                tmp1Matrix(0,3) = 0;
                tmp1Matrix(1,3) = 0;
                tmp1Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp1Matrix);
                tmpAffine.matrix() = tmp1Matrix;
                eachTrans3.push_back(tmpAffine);
                initCenCloud->points[i*3+2].x = JengaWidth;
                initCenCloud->points[i*3+2].y = 0;
                initCenCloud->points[i*3+2].z = JengaHeight / 2 + i * JengaHeight;
                tmp1Matrix(0,3) = JengaWidth;
                tmp1Matrix(1,3) = 0;
                tmp1Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp1Matrix);
                tmpAffine.matrix() = tmp1Matrix;
                eachTrans3.push_back(tmpAffine);
            } else
            {
                initCenCloud->points[i*3+0].x = 0;
                initCenCloud->points[i*3+0].y = -1 * JengaWidth;
                initCenCloud->points[i*3+0].z = JengaHeight / 2 + i * JengaHeight;
                tmp2Matrix(0,3) = 0;
                tmp2Matrix(1,3) = -1 * JengaWidth;
                tmp2Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp2Matrix);
                tmpAffine.matrix() = tmp2Matrix;
                eachTrans3.push_back(tmpAffine);
                initCenCloud->points[i*3+1].x = 0;
                initCenCloud->points[i*3+1].y = 0;
                initCenCloud->points[i*3+1].z = JengaHeight / 2 + i * JengaHeight;
                tmp2Matrix(0,3) = 0;
                tmp2Matrix(1,3) = 0;
                tmp2Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp2Matrix);
                tmpAffine.matrix() = tmp2Matrix;
                eachTrans3.push_back(tmpAffine);
                initCenCloud->points[i*3+2].x = 0;
                initCenCloud->points[i*3+2].y = JengaWidth;
                initCenCloud->points[i*3+2].z = JengaHeight / 2 + i * JengaHeight;
                tmp2Matrix(0,3) = 0;
                tmp2Matrix(1,3) = JengaWidth;
                tmp2Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp2Matrix);
                tmpAffine.matrix() = tmp2Matrix;
                eachTrans3.push_back(tmpAffine);
            }
		}
	}
    else
    {
        for(int i = 0; i < JengaLayer; i++)
        {
            if(i%2)
            {
                initCenCloud->points[i*3+0].x = -1 * JengaWidth;
                initCenCloud->points[i*3+0].y = 0;
                initCenCloud->points[i*3+0].z = JengaHeight / 2 + i * JengaHeight;
                tmp1Matrix(0,3) = -1 * JengaWidth;
                tmp1Matrix(1,3) = 0;
                tmp1Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp1Matrix);
                initCenCloud->points[i*3+1].x = 0;
                initCenCloud->points[i*3+1].y = 0;
                initCenCloud->points[i*3+1].z = JengaHeight / 2 + i * JengaHeight;
                tmp1Matrix(0,3) = 0;
                tmp1Matrix(1,3) = 0;
                tmp1Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp1Matrix);
                initCenCloud->points[i*3+2].x = JengaWidth;
                initCenCloud->points[i*3+2].y = 0;
                initCenCloud->points[i*3+2].z = JengaHeight / 2 + i * JengaHeight;
                tmp1Matrix(0,3) = JengaWidth;
                tmp1Matrix(1,3) = 0;
                tmp1Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp1Matrix);
            } else
            {
                initCenCloud->points[i*3+0].x = 0;
                initCenCloud->points[i*3+0].y = -1 * JengaWidth;
                initCenCloud->points[i*3+0].z = JengaHeight / 2 + i * JengaHeight;
                tmp2Matrix(0,3) = 0;
                tmp2Matrix(1,3) = -1 * JengaWidth;
                tmp2Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp2Matrix);
                initCenCloud->points[i*3+1].x = 0;
                initCenCloud->points[i*3+1].y = 0;
                initCenCloud->points[i*3+1].z = JengaHeight / 2 + i * JengaHeight;
                tmp2Matrix(0,3) = 0;
                tmp2Matrix(1,3) = 0;
                tmp2Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp2Matrix);
                initCenCloud->points[i*3+2].x = 0;
                initCenCloud->points[i*3+2].y = JengaWidth;
                initCenCloud->points[i*3+2].z = JengaHeight / 2 + i * JengaHeight;
                tmp2Matrix(0,3) = 0;
                tmp2Matrix(1,3) = JengaWidth;
                tmp2Matrix(2,3) = JengaHeight / 2 + i * JengaHeight;
                eachTrans.push_back(tmp2Matrix);
            }
        }
    }
}

void Jenga::initJengaModel() {
    MyMesh mesh;
    MyMesh::VertexHandle vhandle[8];
    vhandle[0] = mesh.add_vertex(MyMesh::Point(-1*JengaWidth/2, -1*JengaLength/2, -1*JengaHeight/2));
    vhandle[1] = mesh.add_vertex(MyMesh::Point(JengaWidth/2, -1*JengaLength/2, -1*JengaHeight/2));
    vhandle[2] = mesh.add_vertex(MyMesh::Point(JengaWidth/2, -1*JengaLength/2, JengaHeight/2));
    vhandle[3] = mesh.add_vertex(MyMesh::Point(-1*JengaWidth/2,-1*JengaLength/2,JengaHeight/2));
    vhandle[4] = mesh.add_vertex(MyMesh::Point(-1*JengaWidth/2,JengaLength/2,-1*JengaHeight/2));
    vhandle[5] = mesh.add_vertex(MyMesh::Point(JengaWidth/2,JengaLength/2,-1*JengaHeight/2));
    vhandle[6] = mesh.add_vertex(MyMesh::Point(JengaWidth/2,JengaLength/2,JengaHeight/2));
    vhandle[7] = mesh.add_vertex(MyMesh::Point(-1*JengaWidth/2,JengaLength/2,JengaHeight/2));
    JengaVertex->width = 8;
    JengaVertex->height = 1;
    JengaVertex->points.resize(JengaVertex->width * JengaVertex->height);
    JengaVertex->points[0] = PointT(-1*JengaWidth/2, -1*JengaLength/2, -1*JengaHeight/2);
    JengaVertex->points[1] = PointT(JengaWidth/2, -1*JengaLength/2, -1*JengaHeight/2);
    JengaVertex->points[2] = PointT(JengaWidth/2, -1*JengaLength/2, JengaHeight/2);
    JengaVertex->points[3] = PointT(-1*JengaWidth/2,-1*JengaLength/2,JengaHeight/2);
    JengaVertex->points[4] = PointT(-1*JengaWidth/2,JengaLength/2,-1*JengaHeight/2);
    JengaVertex->points[5] = PointT(JengaWidth/2,JengaLength/2,-1*JengaHeight/2);
    JengaVertex->points[6] = PointT(JengaWidth/2,JengaLength/2,JengaHeight/2);
    JengaVertex->points[7] = PointT(-1*JengaWidth/2,JengaLength/2,JengaHeight/2);

    // generate (quadrilateral) faces
    std::vector<MyMesh::VertexHandle> face_vhandles;
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[2]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[3]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[5]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[6]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[6]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[7]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[4]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[0]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[5]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[1]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[6]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[7]);
    mesh.add_face(face_vhandles);
    //write mesh to output.obj
    if (!OpenMesh::IO::write_mesh(mesh, JengaName)){
        std::cerr << "Cannot write mesh to file 'JengaModel.obj'" << std::endl;
    }

}
void Jenga::BigJengaSim()
{
	MyMesh mesh;
	MyMesh::VertexHandle vhandle[7];
	float halfLength = JengaLength / 2;
	float bigJengaHeight = JengaHeight * JengaLayer;
	vhandle[0] = mesh.add_vertex(MyMesh::Point(-halfLength, -halfLength, 0));
	vhandle[1] = mesh.add_vertex(MyMesh::Point(halfLength, -halfLength, 0));
	vhandle[2] = mesh.add_vertex(MyMesh::Point(halfLength, halfLength, 0));
	vhandle[3] = mesh.add_vertex(MyMesh::Point(-halfLength, -halfLength, bigJengaHeight));
	vhandle[4] = mesh.add_vertex(MyMesh::Point(halfLength, -halfLength, bigJengaHeight));
	vhandle[5] = mesh.add_vertex(MyMesh::Point(halfLength, halfLength, bigJengaHeight));
	vhandle[6] = mesh.add_vertex(MyMesh::Point(-halfLength, halfLength, bigJengaHeight));
	// generate (quadrilateral) faces
	std::vector<MyMesh::VertexHandle> face_vhandles;
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[4]);
	mesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[0]);
	face_vhandles.push_back(vhandle[4]);
	face_vhandles.push_back(vhandle[3]);
	mesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[2]);
	face_vhandles.push_back(vhandle[5]);
	mesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[1]);
	face_vhandles.push_back(vhandle[5]);
	face_vhandles.push_back(vhandle[4]);
	mesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[4]);
	face_vhandles.push_back(vhandle[5]);
	mesh.add_face(face_vhandles);
	face_vhandles.clear();
	face_vhandles.push_back(vhandle[3]);
	face_vhandles.push_back(vhandle[5]);
	face_vhandles.push_back(vhandle[6]);
	mesh.add_face(face_vhandles);
	//write mesh to output.obj
	if (!OpenMesh::IO::write_mesh(mesh, BigJengaName)){
		std::cerr << "Cannot write mesh to file 'output.obj'" << std::endl;
	}
}

void Jenga::PointPass(PCPtr &cloud, PCPtr &cloud_filtered)
{
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0,1);
	pass.filter(*cloud_filtered);
}

void Jenga::PointRemovePlane(PCPtr &cloud, PCPtr &cloud_no_plane)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//�����ָ����
	pcl::SACSegmentation<PointT> seg;
	//��ѡ����
	seg.setOptimizeCoefficients(true);
	//��������
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.015);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}
	std::cerr << inliers->indices[1000] << endl;
	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
			  << coefficients->values[1] << " "
			  << coefficients->values[2] << " "
			  << coefficients->values[3] << std::endl;
	pcl::ExtractIndices<PointT> extract;
	//�����ڲ�
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
//    extract.setKeepOrganized(true);
	extract.setNegative(true);
	extract.filter(*cloud_no_plane);
}

void Jenga::PointOutRemove(PCPtr &cloud, PCPtr &cloud_filtered)
{
	pcl::RadiusOutlierRemoval<PointT> outrem; //创建滤波器
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.01);       //设置在0.01半径内找临近点
	outrem.setMinNeighborsInRadius(5); //设置查询点的邻近点集数小于30的删除
	outrem.filter(*cloud_filtered);
}

void Jenga::PointSegUnique(PCPtr &cloud, PCPtr &cloud_unique)
{
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<PointT> vg;
	pcl::PointCloud<PointT>::Ptr cloud_down_filtered(new pcl::PointCloud<PointT>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.005f, 0.005f, 0.005f);
	vg.filter(*cloud_down_filtered);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud_down_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(0.020); // 2cm
	ec.setMinClusterSize((JengaHeight*10*JengaLayer*JengaLength*10*2+JengaLength*10*JengaLength*10)/2);
	ec.setMaxClusterSize(JengaHeight*10*JengaLayer*JengaLength*10*2+JengaLength*10*JengaLength*10+10000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_down_filtered);
	ec.extract(cluster_indices);

	int j = 0;
	size_t lastnum = 0;
	size_t currentnum = 0;
	int choose = 0;
	vector<pcl::PointCloud<PointT> > PC(cluster_indices.size());
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			PC[j].points.push_back(cloud_down_filtered->points[*pit]); //*
		PC[j].width = PC[j].points.size();
		PC[j].height = 1;
		PC[j].is_dense = true;

		//std::cout << "PointCloud representing the Cluster: " << PC[j].points.size() << " data points." << std::endl;
		//std::stringstream ss;
		//ss << "cloud_cluster_" << j << ".pcd";
		//writer.write<pcl::PointXYZ>(ss.str(), PC[j], false); //*
		if(!j)
		{
			lastnum = PC[j].width;
			currentnum = lastnum;
			choose = 0;
		} else
		{
            // choose the biggest cluster as jenga
			currentnum = PC[j].width;
			if(currentnum > lastnum)
				choose = j;
			lastnum = currentnum;
		}
		j++;
	}
	pcl::copyPointCloud(PC[choose],*cloud_unique);

}

void Jenga::PointICP(PCPtr &cloud_sources, PCPtr &cloud_target, Eigen::Matrix<float,4,4> & matrix4)
{
		pcl::IterativeClosestPoint<PointT, PointT> icp;   //创建icp的实例类
//        pcl::VoxelGrid<pcl::PointXYZ> vg;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sources_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//        vg.setInputCloud(cloud_sources);
//        vg.setLeafSize(0.01f, 0.01f, 0.01f);
//        vg.filter(*cloud_sources_filtered);

		pcl::PointCloud<PointT>::Ptr cloud_target_filtered(new pcl::PointCloud<PointT>);
		pcl::VoxelGrid<PointT> vg1;
		vg1.setInputCloud(cloud_target);
		vg1.setLeafSize(0.01f, 0.01f, 0.01f);
		vg1.filter(*cloud_target_filtered);
		icp.setInputSource(cloud_sources);
		icp.setInputTarget(cloud_target_filtered);
		icp.setMaxCorrespondenceDistance(1.5);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(0.007);
		icp.setMaximumIterations(1000);
		icp.setRANSACIterations(100);
		icp.align(*cloud_sources);
		matrix4 = icp.getFinalTransformation();
        std::cout << icp.getFitnessScore(1.0) << std::endl;
}

void Jenga::regisPointCloud(pcl::PointCloud<PointT>::Ptr &map_cloud, PCPtr & point_center_out)
{
	pcl::copyPointCloud(*map_cloud, *MapCloud);
	PointPass(MapCloud, capCloudPassed);
	PointRemovePlane(capCloudPassed, capCloudNoPlane);
	PointOutRemove(capCloudNoPlane, capCloudNoOut);
	PointSegUnique(capCloudNoOut, capCloudSegUnique);
    pcl::copyPointCloud(*initBigJengaPC, *model);
    computeVoxelGrid(capCloudSegUnique, 0.01f);
    computeVoxelGrid(model, 0.01f);
    bool use_fpfh = true;
    if(!use_fpfh)
    {
        PointICP(model,capCloudSegUnique,ICPTrans);
        pcl::transformPointCloud(*initCenCloud, *point_center_out, ICPTrans);
    } else{
        regisWithFPFH(model, capCloudSegUnique);
        Eigen::Matrix4f matrix4f;
        PointICP(model, capCloudSegUnique, matrix4f);
        ModelToWorldTrans = matrix4f * ModelToWorldTrans;
        pcl::transformPointCloud(*initCenCloud, *point_center_out, ModelToWorldTrans);
    }

}

fpfhFeature::Ptr Jenga::compute_fpfh_feature(PCPtr input_cloud, pcl::search::KdTree<PointT>::Ptr tree) {
    //法向量
    pcl::PointCloud<pcl::Normal>::Ptr point_normal (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT,pcl::Normal> est_normal;
    est_normal.setInputCloud(input_cloud);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(10);
    est_normal.compute(*point_normal);
    //fpfh 估计
    fpfhFeature::Ptr fpfh (new fpfhFeature);
    //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
    pcl::FPFHEstimationOMP<PointT,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
    est_fpfh.setNumberOfThreads(6); //指定4核计算
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
    est_fpfh.setInputCloud(input_cloud);
    est_fpfh.setInputNormals(point_normal);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(10);
    est_fpfh.compute(*fpfh);

    return fpfh;
}

void Jenga::regisWithFPFH(PCPtr & source, PCPtr & target) {
    Eigen::Matrix<float,4,4> matrix4;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
    fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);

    //对齐
    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target);
    sac_ia.setTargetFeatures(target_fpfh);
    //  sac_ia->setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
    sac_ia.setCorrespondenceRandomness(6);  //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
    sac_ia.align(*source);
    ModelToWorldTrans = sac_ia.getFinalTransformation();
//    pcl::transformPointCloud(*initCenCloud, *point_center_out, ModelToWorldTrans);
}

void Jenga::computeVoxelGrid(PCPtr &in_out_cloud, float leaf_size) {
    pcl::VoxelGrid<PointT> vg1;
    vg1.setInputCloud(in_out_cloud);
    vg1.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg1.filter(*in_out_cloud);
}
