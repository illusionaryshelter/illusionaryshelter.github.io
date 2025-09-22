---
title: "Fast-LIO+GICP与ScanContext的分析"
date: 2025-04-21 22:30:00 +0800 # 文章的精确发布时间，+0800代表东八区

categories: 
  - SLAM
  - LIO
tags:
  - C++
  - Fast-LIO
  - GICP
  - Point Registration
  - Loop Closure Detection
excerpt_separator: "" # 定义摘要分隔符, 下文会解释

header:
  overlay_image: /assets/images/b1.jpg
  actions:
    - label: "More Info"
      url: "https://github.com/illusionaryshelter/FAST-LIO-Localization-SC-QN2"

---

## 1. 动机：为何选择 GICP 与 Scan Context？

### 1.1 GICP：scan-to-map的配准升维

Fast-LIO 的scan-to-map matching依赖于点到K-D树的距离计算，这本质上是一种点到线/面的残差模型。在理想的结构化环境中，这种方法表现优异。但在稀疏、半结构化或充满平面的场景（如室内走廊、隧道）中，简单的点状约束可能导致错误的收敛方向。

**GICP(Generalized-ICP)** 将问题提升了一个维度。它不再将点云视为孤立点的集合，而是为每个点估计一个局部平面，然后最小化这些平面之间的距离。

- **核心思想**：GICP 假设每个点及其邻域共同定义了一个局部平面。配准的误差函数是基于“面到面”的概率模型，这使得它对点云中的噪声和稀疏性具有更强的鲁棒性。
- **优势**：在特征较少的平面或边缘上，GICP 能提供比传统 ICP 更稳定、更准确的约束，从而提高里程计的精度。

### 1.2 Scan Context：轻量级、非学习式的回环检测

激光雷达里程计的累计漂移是不可避免的。Loop Closure Detection是抑制全局漂移、构建全局一致性地图的关键。Fast-LIO 作为一个里程计系统，其开源版本并未集成强大的回环模块。

**Scan Context** 是一个global descriptor。
Scancontext使用Ring和Sector把3维信息压缩到极坐标，Ring类似于极坐标中的角度，Sector类似于极坐标中的长度，3D物理空间被Ring和Sector划分成2D的区块空间，每个区块由Ring和Sector唯一确定。同时，将所需相关信息存放在每个区块中，比如：区块中的点云数量、区块中的点云max height、区块中的点云mean height。描述子本身以Ring-Sector矩阵表示，除去水平方向的位移偏差，具有旋转不变性。

理论上，使用当前帧Scan-Context遍历历史所有Scan-Context一定能找到对应的结果，即相似度的全局极值，不过为了更快引入Ring-key，这也是一种旋转不变描述子，具体表示为一位数组，其中每个元素是Ring的编码值，按照距离原点从近到远排布，相当于对原先的数据进行了压缩和降维。
截至目前Scancontext++已经被TRO接受，可以考虑更换为Scancontext++，因为Scancontext本身可以设计为module便于更替。
- **核心思想**：将三维点云投影成一个二维的 `[环数 x 扇区数]` 的描述子图像。这个描述子通过编码每个格网内点的最大高度信息，实现了对旋转变化的鲁棒性（通过列平移匹配）。
- **优势**：
    - **高效性**：生成和匹配描述子的速度极快，对系统性能影响微乎其微。
    - **无需训练**：与基于深度学习的方法不同，它无需数据集训练，泛化能力强。
    - **鲁棒性**：对视角变化和旋转不敏感，适合 SLAM 场景。

## 2. 集成实现细节

我们的实验基于 Fast-LIO 的开源代码库。主要修改分为前端配准和后端回环两个部分。

### 2.1 GICP 替换前端配准
此处选择了vectr-ucla的[nano-gicp](https://github.com/vectr-ucla/direct_lidar_odometry)，用于轻量级点云扫描与交叉对象数据共享匹配，本质上是将开源的FastGICP和NanoFLANN两个包进行合并，使用NanoGICP的NanoFLANN来高效构建KD-tree, 然后通过FastGICP进行高效匹配，不过据说small_gicp的效果现在更好。

```cpp
RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &src,
                                  const pcl::PointCloud<PointType> &dst) {
    RegistrationOutput reg_output;
    aligned_.clear();
    // merge subkeyframes before ICP
    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;
    nano_gicp_.setInputSource(src_cloud);
    nano_gicp_.calculateSourceCovariances();
    nano_gicp_.setInputTarget(dst_cloud);
    nano_gicp_.calculateTargetCovariances();
    nano_gicp_.align(aligned_);

    // handle results
    reg_output.score_ = nano_gicp_.getFitnessScore();
    // if matchness score is lower than threshold, (lower is better)
    if (nano_gicp_.hasConverged() &&
        reg_output.score_ < config_.gicp_config_.icp_score_thr_) {
      reg_output.is_valid_ = true;
      reg_output.is_converged_ = true;
      reg_output.pose_between_eig_ =
          nano_gicp_.getFinalTransformation().cast<double>();
    }
    return reg_output;
}
```
直接用 GICP 的结果覆盖 IKFOM 的状态更新是不明智的。更优的策略是：将 GICP 计算得到的点到面的残差和协方差，作为外部测量信息，融入到 esekf 的更新公式中。但这需要对公式进行深度修改。一个简化的替代方案是，用 GICP 的结果作为 IKFOM 迭代的更精确初值。

### 2.2 Scancontext 集成

具体流程为：读取单帧3D点云数据，建立Scan-Context，在Mapping过程中生成KeyFrame中查找，使用Ring-key在KD-tree下查找最近邻结果，结果计算统计得分，得到最佳匹配，完成回环检测。

其中Scan-Context的结构与Ring-Key的生成示意如下
```cpp
const int NO_POINT = -1000;
Eigen::MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
...
MatrixXd SCManager::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * summary: rowwise mean vector
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;
} 
```
然后可以使用nanoflann自带的KDTree查找候选描述子
```cpp
nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) );
...
for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
{
  MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
  std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
  double candidate_dist = sc_dist_result.first;
  int candidate_align = sc_dist_result.second;

  if( candidate_dist < min_dist )
  {
  min_dist = candidate_dist;
  nn_align = candidate_align;

  nn_idx = candidate_indexes[candidate_iter_idx];
  }
}
```
根据阈值判断是否检测到回环
```
if( min_dist < SC_DIST_THRES )
{
     loop_id = nn_idx; 
    
     // std::cout.precision(3); 
     cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
     cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
}
else
{
     std::cout.precision(3); 
     cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size()-1 << " and " << nn_idx << "." << endl;
     cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
}
```

