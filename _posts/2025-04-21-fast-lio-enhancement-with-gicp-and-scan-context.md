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
      url: "[https://unsplash.com](https://github.com/illusionaryshelter/FAST-LIO-Localization-SC-QN2)"

---

## 摘要

Fast-LIO 系列算法作为当前激光惯性 SLAM 领域的里程碑之作，以其高效率和高精度在众多应用中大放异彩。然而，原生的 Fast-LIO 作为一个纯粹的激光惯性里程计（LIO），仍然存在两个主要的提升空间：一是其前端点云配准的鲁棒性，二是在大场景下无法避免的累积漂移。本文旨在探讨并实践两个主流的改进方向：使用 **GICP (Generalized-ICP)** 替换原有的点云配准，以增强前端的精度和鲁棒性；引入 **Scan Context** 作为后端的回环检测模块，以消除累积误差，实现全局一致性。

---

## 1. Fast-LIO 核心思想回顾

在深入改进之前，我们简要回顾 Fast-LIO 的核心思想。它是一个紧耦合的激光惯性里程计系统，其精髓在于：

- **基于迭代卡尔曼滤波 (Iterated Kalman Filter) 的状态估计**：将 IMU 和 LiDAR 的测量在一个统一的框架下进行融合。
- **反向传播去除运动畸变**：利用 IMU 的高频测量，将一帧激光扫描中的每个点都校正到统一的时刻，极大地提高了精度。
- **直接点云配准**：将当前扫描的点直接与地图（由 ikd-Tree 维护的体素地图）进行配准，计算残差并用于更新卡尔曼滤波的状态。

然而，Fast-LIO 的原生实现主要关注前端里程计，这意味着：
1.  **前端配准**：虽然高效，但在结构稀疏或退化场景下，其点到面的配准方式仍有提升空间。
2.  **全局漂移**：系统没有回环检测和全局优化的能力，当机器人回到曾经经过的位置时，它无法识别出来，导致轨迹和地图会随着时间累积漂移。

## 2. 改进一：从标准ICP到GICP

### 为什么选择GICP？

标准的 ICP 算法（如点到点、点到面）在进行点云配准时，通常只考虑了点的位置，而忽略了点云自身在不同方向上的不确定性。例如，从一个平坦的墙面扫描得到的点云，其在平行于墙面的方向上不确定性很大，而在垂直于墙面的方向上不确定性则很小。

**GICP (Generalized-ICP)** 正是为了解决这个问题而生。它将每个点及其周围的点拟合成一个局部平面，并为这个平面（即这个点）计算一个协方差矩阵。这样，每个点就被看作一个概率分布，而不仅仅是一个三维空间中的坐标。配准过程就从“点对点”的距离最小化，变为了“概率分布之间”的距离最小化。这种 **“面到面 (plane-to-plane)”** 的配准方式能够更好地利用场景的结构信息，从而在结构化环境中提供更高的鲁棒性和精度。

### 实现思路

在 Fast-LIO 的框架中替换 ICP 模块相对直接。主要步骤如下：

1.  **定位配准模块**：找到 Fast-LIO 中执行点云到地图配准的核心代码。
2.  **替换为GICP**：使用一个成熟的 GICP 库（例如 PCL 中的 `pcl::GeneralizedIterativeClosestPoint`）来替换原有的配准函数。
3.  **数据传递**：将当前帧处理后的点云 (`input_source`) 和 ikd-Tree 构建的局部地图 (`target_map`) 传递给 GICP 算法。

一个简化的伪代码示例如下：
```cpp
#include <pcl/registration/gicp.h>

// ...

void performGICPRegistration(const PointCloud::Ptr& source, const PointCloud::Ptr& target) {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    // 设置GICP参数
    gicp.setMaxCorrespondenceDistance(1.0);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setEuclideanFitnessEpsilon(1e-5);
    gicp.setMaximumIterations(30);

    PointCloud Final;
    gicp.align(Final);

    if (gicp.hasConverged()) {
        Eigen::Matrix4f transformation = gicp.getFinalTransformation();
        // 使用这个 transformation 更新位姿
    }
}
