cmake_minimum_required(VERSION 3.21)
project(pcl_learning)

set(CMAKE_CXX_STANDARD 14)

add_executable(pcl_learning
        00-base/00pcd_write/pcd_write.cpp
        00-base/01matrix_transform/matrix_transform.cpp
        02-kdtree/1kdtree_search/kdtree_search.cpp
        03-octree/1point_cloud_compression/point_cloud_compression.cpp
        "03-octree/2Spatial Partitioning and Search Operations with Octrees/1octree_search/octree_search.cpp"
        "03-octree/2Spatial Partitioning and Search Operations with Octrees/2octree_change_detection/octree_change_detection.cpp"
        03-octree/3point_cloud_compression/point_cloud_compression.cpp
        05-sampleconsensus抽样一致性模块/1随机采样一致性算法/random_sample_consensus.cpp
        05-sampleconsensus抽样一致性模块/2随机采样一致性算法/random_sample_consensus.cpp
        06-range-images深度图像/1从点云创建深度图/range_image_creation.cpp
        06-range-images深度图像/2从深度图中提取边界/range_image_border_extraction.cpp
        08-IO输入输出/1write_pcd/write_pcd.cpp
        08-IO输入输出/2read_pcd/read_pcd.cpp
        08-IO输入输出/3concatenate_clouds/concatenate_clouds.cpp
        08-IO输入输出/4openni_grabber/openni_grabber.cpp
        09-filters滤波/1直通滤波器进行滤波处理/passthrough.cpp
        09-filters滤波/2VoxelGrid滤波器对点云进行下采样/voxel_grid.cpp
        09-filters滤波/3滤波器移除离群点/statistical_removal.cpp
        09-filters滤波/4使用参数化模型投影点云/project_inliers.cpp
        09-filters滤波/5从一个点云中提取索引/extract_indices.cpp
        09-filters滤波/6用ConditionalRemoval或RadiusOutlinerRemoval移除离群点/remove_outliers.cpp
        09-filters滤波/7双边滤波/bilateral_filter.cpp
        10-features特征/1估计整个点云表面法向量/normal_estimation.cpp
        "10-features特征/2估计点云子集的表面法线(error)/normal_estimation_subdset_points.cpp"
        10-features特征/3使用积分图进行法线估计/normal_estimation_using_integral_images.cpp
        10-features特征/4点特征直方图（PFH）描述子/PFH.cpp
        10-features特征/4点特征直方图（PFH）描述子/PFH1.cpp
        "10-features特征/5快速点特征直方图(FPFH)描述子/FPFH.cpp"
        "10-features特征/6从一个深度图像（range image）中提取NARF特征/narf_feature_extraction.cpp"
        "11-surface表面 /1基于多项式重构的平滑和法线估计/resampling.cpp"
        "11-surface表面 /2在平面模型上提取凸（凹）多边形/hull_2d.cpp"
        "11-surface表面 /3无序点云的快速三角化/greedy_projection.cpp"
        "11-surface表面 /4将修剪的B样条曲线拟合到无序点云/bspline_fitting.cpp"
        "11-surface表面 /5平面点云B样条曲线拟合/fitting_curve_2d.cpp"
        12-segmentation分割/1planar_segmentation/planar_segmentation.cpp
        12-segmentation分割/2cylinder_segmentation/cylinder_segmentation.cpp
        12-segmentation分割/3欧式聚类提取/cluster_extraction.cpp
        13-recognition识别/1基于对应分组的三维物体识别/correspondence_grouping.cpp
        "13-recognition识别/2隐式形状模型 ISM (Implicit Shape Model)/implicit_shape_model.cpp"
        13-recognition识别/33D物体识别的假设检验/global_hypothesis_verification.cpp
        "14-registration配准/1使用迭代最近点算法(ICP)/iterative_closest_point.cpp"
        14-registration配准/2如何逐步匹配多幅点云/pairwise_incremental_registration.cpp
        14-registration配准/3配准之交互式ICP/interactive_icp.cpp
        "14-registration配准/4正态分布变换配准(NDT)/normal_distributions_transform.cpp"
        14-registration配准/5刚性物体的鲁棒姿态估计/alignment_prerejective.cpp
        15-visualization可视化/0base/demo.cpp
        15-visualization可视化/1cloud_viewer/cloud_viewer.cpp
        15-visualization可视化/2range_image_visualization/range_image_visualization.cpp
        15-visualization可视化/3.0pcl_visualizer_simple/1pcl_visualizer_simple.cpp
        15-visualization可视化/3pcl_visualizer_demo/pcl_visualizer_demo.cpp
        15-visualization可视化/4multi_pcl_visualizer/multi_pcl_visualizer.cpp
        "16-keypoints关键点/1从深度图像中提取 NARF关键点/narf_keypoint_extraction.cpp"
        17-Point-cloud-transform点云转换/1-xyz2uv/main.cpp
        practice/01pcd和text相互转换/pcd_to_txt.cpp
        practice/01pcd和text相互转换/txt_to_pcd.cpp
        practice/02直线拟合/01lineFit.cpp)
