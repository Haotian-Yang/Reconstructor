#include <registrator.h>


void Registrator::setRegPointCloudPair(PointCloud::Ptr &source, PointCloud::Ptr &target){
    cloud_tgt = target;
    cloud_src = source;
}

float Registrator::pairAlign(float maxCorrespondence, float reduction, int iteration, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample, int leafSize){

    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);

    // filter the original point cloud with downsampling to boost computational speed
    pcl::VoxelGrid<PointT> grid;
    if (downsample){
        grid.setLeafSize(leafSize, leafSize, leafSize);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else{
        src = cloud_src;
        tgt = cloud_tgt;
    }

    // compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(100);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);


    // Instantiate custom point representation
    PointCurvature point_representation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues(alpha);


    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon(1e-4);
    // set the maximum distance between two correspondences (src<->tgt) to 50mm
    reg.setMaxCorrespondenceDistance(maxCorrespondence);
    reg.setPointRepresentation(boost::make_shared<const PointCurvature> (point_representation));
    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Initialize the transformation of i-th iteration
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, T_st;

    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);

    for (int i =0 ; i < iteration ; i++){
        PCL_INFO("Iteration: %d.\n", i+1);

        // save for visualization
        points_with_normals_src = reg_result;
        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        Ti = reg.getFinalTransformation() * Ti;
        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon()){
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() * reduction);
            std::cout << reg.getMaxCorrespondenceDistance() << std::endl;
        }

        prev = reg.getLastIncrementalTransformation();
        if(visual){
            showCloudResult(points_with_normals_src, points_with_normals_tgt);
        }


    }


    if(visual){
        std::cout << "press q to continue" << std::endl;
        p->spin();
    }
    // transformation from target to src
    T_st = Ti;
    float CorrDis = reg.getMaxCorrespondenceDistance();

    // transform source back to target
    pcl::transformPointCloud(*cloud_src, *output, T_st);
    *output += *cloud_tgt;

    final_transform = T_st;
    transformation = T_st;

    return CorrDis;
}


void Registrator::showCloudOriginal(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt){
    p->removePointCloud("vp1_source");
    p->removePointCloud("vp1_target");

    PointCloudColorHandlerCustom<PointT> src_h(cloud_src, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> tgt_h(cloud_tgt, 255, 0, 0);

    p->addPointCloud(cloud_src, src_h, "vp1_source", vp1);
    p->addPointCloud(cloud_tgt, tgt_h, "vp1_target", vp1);
    PCL_INFO("Press q to registrate\n");

    p->spin();
}

void Registrator::showCloudResult(const PointCloudWithNormals::Ptr cloud_src, const PointCloudWithNormals::Ptr cloud_tgt){
   p->removePointCloud("vp2_source");
   p->removePointCloud("vp2_target");

   PointCloudColorHandlerCustom<PointNormalT> src_color_handler (cloud_src, 0, 255, 0);
   PointCloudColorHandlerCustom<PointNormalT> tgt_color_handler (cloud_tgt, 255, 0, 0);

   p->addPointCloud(cloud_src, src_color_handler, "vp2_source", vp2);
   p->addPointCloud(cloud_tgt, tgt_color_handler, "vp2_target", vp2);
   p->spinOnce();
}

void Registrator::createViewPort(){
    p = new pcl::visualization::PCLVisualizer("registration");
    p->createViewPort(0.0, 0.0, 0.5, 1.0, vp1);
    p->createViewPort(0.5, 0.0, 1.0, 1.0, vp2);
    visual = true;
}
