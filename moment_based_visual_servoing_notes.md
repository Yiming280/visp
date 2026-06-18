# 基于矩的视觉伺服实现笔记

## 引言

基于矩的视觉伺服是一种高级的视觉伺服技术，它使用图像矩（moments）作为视觉特征来控制机器人运动。与传统的点特征或线特征相比，矩特征能够描述物体的全局形状和姿态信息，提供更丰富的视觉信息。本文档基于ViSP库中的三个示例程序（servoMomentImage.cpp、servoMomentPoints.cpp、servoMomentPolygon.cpp）进行深度分析，拆解伺服方法的实现过程。

ViSP（Visual Servoing Platform）是一个开源的视觉伺服库，提供丰富的视觉特征提取、伺服控制算法和机器人接口。这些示例展示了如何使用矩特征进行眼在手上（eye-in-hand）的视觉伺服。

## 共同的伺服框架

所有三个示例都使用了一个名为`servoMoment`的类来封装伺服逻辑。这个类提供了统一的接口来处理不同的对象容器类型（图像、点、多边形）。

### 类结构

```cpp
class servoMoment {
public:
  // 构造函数和析构函数
  servoMoment();
  ~servoMoment();

  // 初始化方法
  void init(vpHomogeneousMatrix &cMo, vpHomogeneousMatrix &cdMo);
  void initScene();      // 初始化场景
  void initFeatures();   // 初始化特征

  // 执行伺服
  void execute(unsigned int nbIter);

  // 辅助方法
  void refreshScene(vpMomentObject &obj);
  void planeToABC(vpPlane &pl, double &A, double &B, double &C);
  void paramRobot();
  void init_visp_plot(vpPlot &ViSP_plot);

protected:
  // 成员变量
  unsigned int m_width, m_height;  // 图像尺寸
  vpHomogeneousMatrix m_cMo, m_cdMo;  // 当前和期望位姿
  vpSimulatorCamera/vpSimulatorAfma6 m_robot;  // 机器人模拟器
  vpImage<vpRGBa> m_Iint;  // 内部显示图像
  vpServo m_task;  // 伺服任务
  vpCameraParameters m_cam;  // 相机参数
  double m_error;  // 当前误差

  // 矩相关对象
  vpMomentObject m_src, m_dst;  // 源和目标矩对象
  vpMomentCommon *m_moments, *m_momentsDes;  // 矩计算器
  vpFeatureMomentCommon *m_featureMoments, *m_featureMomentsDes;  // 矩特征

  vpServo::vpServoIteractionMatrixType m_interaction_type;  // 交互矩阵类型
};
```

### 伺服流程

1. **初始化**：设置相机参数、机器人状态、期望位姿
2. **特征提取**：从图像中提取矩特征
3. **控制计算**：基于特征误差计算控制律
4. **执行**：将控制命令发送给机器人
5. **迭代**：重复直到收敛

## 示例1: servoMomentImage.cpp - 使用图像作为对象容器

### 概述

这个示例使用整个图像作为对象容器，通过图像分割（阈值128）来提取物体轮廓，然后计算基于图像的矩特征。

### 图像特征的选择和计算

#### 1. 对象初始化

```cpp
// 初始化源对象（当前图像）
m_src.setType(vpMomentObject::DENSE_FULL_OBJECT);
m_src.fromImage(m_src_img, 128, m_cam);

// 初始化目标对象（期望图像）
m_dst.setType(vpMomentObject::DENSE_FULL_OBJECT);
m_dst.fromImage(m_dst_img, 128, m_cam);
```

- `vpMomentObject::DENSE_FULL_OBJECT`：表示使用密集的全对象矩计算
- `fromImage()`：从灰度图像创建矩对象，128为阈值

#### 2. 矩特征选择

使用`vpMomentCommon`类计算常用的矩特征：

```cpp
m_moments = new vpMomentCommon(
    vpMomentCommon::getSurface(m_dst),     // 面积矩
    vpMomentCommon::getMu3(m_dst),         // 三阶中心矩
    vpMomentCommon::getAlpha(m_dst),       // 方向矩
    vec[2]                                 // Z坐标
);
```

自动包含的矩：
- **重心坐标**：Xg, Yg（归一化）
- **面积矩**：An（归一化面积）
- **形状矩**：C1, C2（不变矩）
- **方向矩**：Alpha（方向角）

#### 3. 视觉特征

```cpp
m_featureMoments = new vpFeatureMomentCommon(*m_moments);
```

提取的特征向量包括：
- 归一化重心：x_n, y_n
- 归一化面积：a_n
- 形状参数：sx, sy（尺度）
- 方向：alpha

### 计算部分的实现

#### 平面参数计算

```cpp
void planeToABC(vpPlane &pl, double &A, double &B, double &C) {
  A = -pl.getA() / pl.getD();
  B = -pl.getB() / pl.getD();
  C = -pl.getC() / pl.getD();
}
```

将平面方程ax+by+cz=d转换为1/Z = Ax + By + C的形式，用于透视投影。

#### 特征更新

```cpp
// 更新矩计算
m_moments->updateAll(obj);

// 更新特征（需要平面参数）
m_featureMoments->updateAll(A, B, C);
```

### 交互矩阵和控制律计算

#### 伺服任务设置

```cpp
m_task.setServo(vpServo::EYEINHAND_CAMERA);
m_task.setInteractionMatrixType(vpServo::CURRENT);
m_task.setLambda(1.);

// 添加特征
m_task.addFeature(m_featureMoments->getFeatureGravityNormalized(),
                  m_featureMomentsDes->getFeatureGravityNormalized());
m_task.addFeature(m_featureMoments->getFeatureAn(),
                  m_featureMomentsDes->getFeatureAn());
m_task.addFeature(m_featureMoments->getFeatureCInvariant(),
                  m_featureMomentsDes->getFeatureCInvariant(),
                  (1 << 10) | (1 << 11));
m_task.addFeature(m_featureMoments->getFeatureAlpha(),
                  m_featureMomentsDes->getFeatureAlpha());
```

#### 控制律计算

```cpp
vpColVector v = m_task.computeControlLaw();
```

内部实现：
1. 计算特征误差：e = s - s*
2. 计算交互矩阵：L = ∂s/∂v
3. 应用控制增益：λ
4. 求解控制速度：v = -λ L^+ e

其中L^+是交互矩阵的伪逆。

## 示例2: servoMomentPoints.cpp - 使用离散点作为对象容器

### 概述

这个示例使用离散的3D点来定义对象形状，通过点的投影来计算矩特征。

### 图像特征的选择和计算

#### 1. 对象初始化

```cpp
// 定义3D点
vpColVector X[4];
X[0] = [-0.2, -0.1, 0];
X[1] = [0.2, -0.1, 0];
X[2] = [0.2, 0.1, 0];
X[3] = [-0.2, 0.1, 0];

// 创建图像模拟器
vpImageSimulator imsim;
imsim.init(tmp_img, X);
```

#### 2. 矩计算

与图像示例类似，使用相同的矩特征集合。

### 计算部分的实现

#### 场景刷新

```cpp
void refreshScene(vpMomentObject &obj) {
  m_cur_img = 0;
  m_imsim.setCameraPosition(m_cMo);
  m_imsim.getImage(m_cur_img, m_cam);
  obj.fromImage(m_cur_img, 128, m_cam);
}
```

通过图像模拟器生成当前视角下的图像，然后提取矩。

## 示例3: servoMomentPolygon.cpp - 使用多边形作为对象容器

### 概述

这个示例使用多边形（由顶点定义）作为对象容器，直接从几何形状计算矩特征。

### 图像特征的选择和计算

#### 1. 对象初始化

```cpp
std::vector<vpPoint> src_pts;
double x[5] = {0.2, 0.2, -0.2, -0.2, 0.2};
double y[5] = {-0.1, 0.1, 0.1, -0.1, -0.1};

for(int i = 0; i < 4; i++) {
  vpPoint p(x[i], y[i], 0.0);
  p.track(m_cMo);  // 投影到当前位姿
  src_pts.push_back(p);
}

m_src.setType(vpMomentObject::DENSE_POLYGON);
m_src.fromVector(src_pts);
```

#### 2. 矩计算

```cpp
m_moments = new vpMomentCommon(
    vpMomentCommon::getSurface(m_dst),
    vpMomentCommon::getMu3(m_dst),
    vpMomentCommon::getAlpha(m_dst),
    vec[2]
);
```

注意：这里没有第三个参数`true`，表示不使用对称性假设。

### 计算部分的实现

#### 场景刷新

```cpp
void refreshScene(vpMomentObject &obj) {
  std::vector<vpPoint> cur_pts;
  // 重新计算点在当前位姿下的位置
  for(int i = 0; i < nbpoints; i++) {
    vpPoint p(x[i], y[i], 0.0);
    p.track(m_cMo);
    cur_pts.push_back(p);
  }
  obj.fromVector(cur_pts);
}
```

直接从几何点计算矩，无需图像处理。

## 三个方法的区别

| 方面 | servoMomentImage | servoMomentPoints | servoMomentPolygon |
|------|------------------|-------------------|-------------------|
| **对象容器** | 整个图像 | 离散3D点 | 多边形顶点 |
| **特征提取** | 图像分割+矩计算 | 点投影+图像模拟+矩计算 | 直接几何计算 |
| **计算效率** | 中等 | 低（需要图像渲染） | 高（直接计算） |
| **适用场景** | 任意形状物体 | 已知3D点 | 已知多边形 |
| **精度** | 依赖分割质量 | 依赖点定义 | 几何精确 |
| **对称性处理** | 使用对称假设 | 使用对称假设 | 不使用对称假设 |

### 关键区别分析

1. **计算路径**：
   - Image：图像 → 二值化 → 轮廓提取 → 矩计算
   - Points：3D点 → 图像投影 → 图像模拟 → 矩计算
   - Polygon：几何顶点 → 直接矩计算

2. **矩初始化参数**：
   - Image/Points：使用`true`参数启用对称性优化
   - Polygon：不使用对称性优化

3. **更新频率**：
   - 所有方法都在每次迭代中更新矩和特征

## 使用ViSP库实现视觉伺服

### 基本步骤

1. **包含头文件**
```cpp
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeatureMomentCommon.h>
#include <visp3/core/vpMomentCommon.h>
#include <visp3/core/vpMomentObject.h>
```

2. **初始化伺服任务**
```cpp
vpServo task;
task.setServo(vpServo::EYEINHAND_CAMERA);  // 眼在手上
task.setInteractionMatrixType(vpServo::CURRENT);  // 使用当前交互矩阵
task.setLambda(0.5);  // 设置控制增益
```

3. **创建矩对象和特征**
```cpp
// 创建源和目标矩对象
vpMomentObject src(6), dst(6);
src.setType(vpMomentObject::DENSE_POLYGON);  // 或其他类型
// ... 初始化src和dst ...

// 创建矩计算器
vpMomentCommon moments(vpMomentCommon::getSurface(dst),
                      vpMomentCommon::getMu3(dst),
                      vpMomentCommon::getAlpha(dst), Z_dst);

// 创建特征
vpFeatureMomentCommon feature_moments(moments);
vpFeatureMomentCommon feature_moments_des(moments_des);

// 更新特征
moments.updateAll(src);
feature_moments.updateAll(A, B, C);
```

4. **添加特征到任务**
```cpp
task.addFeature(feature_moments.getFeatureGravityNormalized(),
                feature_moments_des.getFeatureGravityNormalized());
task.addFeature(feature_moments.getFeatureAn(),
                feature_moments_des.getFeatureAn());
task.addFeature(feature_moments.getFeatureAlpha(),
                feature_moments_des.getFeatureAlpha());
```

5. **伺服循环**
```cpp
while(error > threshold) {
    // 更新当前矩和特征
    moments.updateAll(current_obj);
    feature_moments.updateAll(A, B, C);

    // 计算控制律
    vpColVector v = task.computeControlLaw();

    // 发送给机器人
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);

    // 计算误差
    error = task.getError().sumSquare();
}
```

### 高级用法

- **自定义特征**：继承`vpFeatureMomentCommon`创建自定义矩特征
- **多任务伺服**：组合不同类型的特征
- **鲁棒性**：添加特征权重和选择矩阵
- **优化**：使用不同的交互矩阵类型（期望、平均等）

### 注意事项

1. 确保相机标定正确
2. 选择合适的控制增益λ
3. 处理奇异配置
4. 考虑特征的可观测性
5. 实现适当的停止条件

这个实现提供了基于矩的视觉伺服的完整框架，可以根据具体应用调整特征选择和控制参数。

## 图像矩的定义和计算

ViSP库中的图像矩是基于图像几何形状的统计特征，用于描述物体的位置、形状、方向和尺度信息。`vpMomentCommon`类集成了常用的矩计算，以下详细介绍各矩的数学定义和计算方法。

### 1. 基本矩 (Basic Moments) - vpMomentBasic

#### 数学定义
基本矩是图像矩的基础，定义为像素坐标的幂次积分或求和：

**连续情况（密集对象）**：
\f[m_{ij} = \iint_{O} x^i y^j \, dx \, dy\f]

**离散情况（离散点集）**：
\f[m_{ij} = \sum_{k=1}^{n} x_k^i y_k^j\f]

其中：
- \f$(x, y)\f$ 是图像坐标系中的像素坐标
- \f$i, j\f$ 是矩的阶数
- \f$O\f$ 是物体区域
- \f$n\f$ 是离散点的数量

#### 特殊情况
- \f$m_{00}\f$：连续情况下等于物体面积\f$a\f$，离散情况下等于点数\f$n\f$
- \f$m_{10}, m_{01}\f$：用于计算重心坐标

#### 计算实现
基本矩的计算在`vpMomentObject`类中完成，`vpMomentBasic::compute()`方法为空实现，因为所有计算都在对象层面完成。

### 2. 重心矩 (Gravity Center Moments) - vpMomentGravityCenter

#### 数学定义
重心坐标定义为：
\f[x_g = \frac{m_{10}}{m_{00}}, \quad y_g = \frac{m_{01}}{m_{00}}\f]

#### 计算实现
```cpp
void vpMomentGravityCenter::compute() {
  double m00 = getObject().get(0, 0);
  double m10 = getObject().get(1, 0);
  double m01 = getObject().get(0, 1);

  m_xg = m10 / m00;
  m_yg = m01 / m00;
}
```

### 3. 中心矩 (Centered Moments) - vpMomentCentered

#### 数学定义
中心矩相对于重心进行计算：

**连续情况**：
\f[\mu_{ij} = \iint_{O} (x - x_g)^i (y - y_g)^j \, dx \, dy\f]

**离散情况**：
\f[\mu_{ij} = \sum_{k=1}^{n} (x_k - x_g)^i (y_k - y_g)^j\f]

#### 重要性质
- \f$\mu_{00} = m_{00}\f$（面积不变）
- \f$\mu_{10} = \mu_{01} = 0\f$（重心处一阶矩为零）
- \f$\mu_{11}\f$：反映物体的倾斜程度
- \f$\mu_{20}, \mu_{02}\f$：反映物体的伸长方向

#### 计算实现
中心矩通过基本矩计算得到：
```cpp
for(unsigned int i = 0; i <= order; i++) {
  for(unsigned int j = 0; j <= order - i; j++) {
    double value = 0.0;
    for(size_t k = 0; k < points.size(); k++) {
      double dx = points[k].x - xg;
      double dy = points[k].y - yg;
      value += pow(dx, i) * pow(dy, j);
    }
    set(i, j, value);
  }
}
```

### 4. 归一化重心矩 (Gravity Center Normalized) - vpMomentGravityCenterNormalized

#### 数学定义
归一化重心坐标用于视觉伺服中的尺度不变性：
\f[x_n = \frac{x_g}{\sqrt{a}}, \quad y_n = \frac{y_g}{\sqrt{a}}\f]

其中\f$a\f$是物体面积。

#### 应用
- 提供尺度不变的重心位置
- 用于控制物体的平移运动

### 5. 归一化面积矩 (Area Normalized) - vpMomentAreaNormalized

#### 数学定义
归一化面积用于深度估计：
\f[a_n = Z^* \sqrt{\frac{a^*}{a}}\f]

其中：
- \f$Z^*\f$：期望深度
- \f$a^*\f$：期望面积
- \f$a\f$：当前面积

#### 计算方法
```cpp
double current_area = (object_type == DISCRETE) ?
  (centered_moments.get(2,0) + centered_moments.get(0,2)) :
  basic_moments.get(0,0);

double normalized_area = desired_depth * sqrt(desired_area / current_area);
```

### 6. 方向矩 (Alpha Moment) - vpMomentAlpha

#### 数学定义
物体在平面内的方向角：
\f[\alpha = \frac{1}{2} \tan^{-1}\left(\frac{2\mu_{11}}{\mu_{20} - \mu_{02}}\right)\f]

取值范围：\f$[-\pi/2, \pi/2]\f$

#### 扩展到\f$[-\pi, \pi]\f$
对于非对称物体，使用三阶中心矩进行180度歧义消除：
\f[\alpha_{extended} = \alpha + k\pi\f]

其中\f$k\f$根据\f$\mu_{30}, \mu_{03}, \mu_{21}, \mu_{12}\f$的符号确定。

#### 计算实现
```cpp
double mu11 = centered_moments.get(1,1);
double mu20 = centered_moments.get(2,0);
double mu02 = centered_moments.get(0,2);

double alpha = 0.5 * atan2(2 * mu11, mu20 - mu02);
```

### 7. C型不变矩 (C-Invariants) - vpMomentCInvariant

#### 数学定义
C型不变矩是平移、旋转、尺度不变的特征，用于控制绕X和Y轴的旋转：

\f[C_1 = \frac{\mu_{20} + \mu_{02}}{\mu_{00}^{2}}\f]

\f[C_2 = \frac{(\mu_{20} - \mu_{02})^2 + 4\mu_{11}^2}{\mu_{00}^{4}}\f]

\f[C_3 = \frac{(\mu_{30} - 3\mu_{12})^2 + (\mu_{03} - 3\mu_{21})^2}{\mu_{00}^{5}}\f]

\f[C_4 = \frac{(\mu_{30} + \mu_{12})^2 + (\mu_{03} + \mu_{21})^2}{\mu_{00}^{5}}\f]

#### P型不变矩（非对称物体）
\f[P_x = \frac{\mu_{30} + \mu_{12}}{\mu_{00}^{2.5}}, \quad P_y = \frac{\mu_{03} + \mu_{21}}{\mu_{00}^{2.5}}\f]

#### S型不变矩（对称物体）
\f[S_x = \frac{\mu_{30} - 3\mu_{12}}{\mu_{00}^{2.5}}, \quad S_y = \frac{\mu_{03} - 3\mu_{21}}{\mu_{00}^{2.5}}\f]

#### 计算实现
C不变矩通过中心矩计算，涉及复杂的代数运算以确保不变性。

### 矩的依赖关系

ViSP中的矩计算有严格的依赖顺序：

```
vpMomentBasic
├── vpMomentGravityCenter
│   ├── vpMomentCentered
│   │   ├── vpMomentAreaNormalized
│   │   ├── vpMomentCInvariant
│   │   └── vpMomentAlpha
│   └── vpMomentGravityCenterNormalized
└── vpMomentArea
```

### 计算流程

1. **基本矩计算**：直接从图像或点集计算\f$m_{ij}\f$
2. **重心计算**：\f$x_g, y_g\f$从\f$m_{10}, m_{01}, m_{00}\f$得到
3. **中心矩计算**：\f$\mu_{ij}\f$基于重心坐标计算
4. **导出矩计算**：基于中心矩计算各种不变矩和归一化矩

### 应用中的注意事项

1. **对象类型影响**：
   - `DENSE_FULL_OBJECT`：适用于完整图像分割
   - `DENSE_POLYGON`：适用于几何多边形
   - `DISCRETE`：适用于离散点集

2. **矩阶数要求**：
   - 基本矩：任意阶数
   - 中心矩：至少2阶
   - C不变矩：至少3阶
   - Alpha矩：至少2阶

3. **数值稳定性**：
   - 避免除零（面积为零的情况）
   - 处理奇异配置（物体退化）
   - 数值精度对高阶矩影响显著

4. **计算效率**：
   - 基本矩：O(N)复杂度，N为像素/点数
   - 高阶矩：O(order² × N)复杂度
   - 建议根据需要选择合适的矩阶数

这些矩特征为视觉伺服提供了丰富的几何信息，使得系统能够精确控制物体的6DOF运动。

## 交互矩阵类型和计算

### vpServo中的交互矩阵类型

ViSP库中的`vpServo`类提供了四种不同的交互矩阵计算类型：

#### 1. CURRENT（当前交互矩阵）
\f[L_s = \frac{\partial s}{\partial v}\bigg|_{s}\f]

使用当前视觉特征\f$s\f$计算的交互矩阵。这是默认和最常用的类型。

#### 2. DESIRED（期望交互矩阵）
\f[L_{s^*} = \frac{\partial s}{\partial v}\bigg|_{s^*}\f]

使用期望视觉特征\f$s^*\f$计算的交互矩阵。

#### 3. MEAN（平均交互矩阵）
\f[L = \frac{L_s + L_{s^*}}{2}\f]

使用当前和期望交互矩阵的平均值，可以提高在特征空间大范围运动时的稳定性。

#### 4. USER_DEFINED（用户自定义）
允许用户手动设置交互矩阵，通常用于特殊应用场景。

### 交互矩阵计算流程

在`vpServo::computeInteractionMatrix()`中：

```cpp
switch (interactionMatrixType) {
case CURRENT:
  computeInteractionMatrixFromList(featureList, featureSelectionList, L);
  break;
case DESIRED:
  computeInteractionMatrixFromList(desiredFeatureList, featureSelectionList, L);
  break;
case MEAN:
  computeInteractionMatrixFromList(featureList, featureSelectionList, L);
  computeInteractionMatrixFromList(desiredFeatureList, featureSelectionList, Lstar);
  L = (L + Lstar) / 2;
  break;
}
```

### 矩特征的交互矩阵

对于基于矩的视觉特征，交互矩阵通过`vpFeatureMomentCommon`类计算。每个矩特征都有对应的交互矩阵：

#### 重心归一化特征的交互矩阵
\f[\frac{\partial (x_n, y_n)}{\partial v} = \begin{bmatrix} L_{x_n} \\ L_{y_n} \end{bmatrix} = \begin{bmatrix} -1 & 0 & x_n & x_n y_n & -(1+x_n^2) & y_n \\ 0 & -1 & y_n & 1+y_n^2 & -x_n y_n & -x_n \end{bmatrix}\f]

#### 面积归一化特征的交互矩阵
\f[\frac{\partial a_n}{\partial v} = L_{a_n} = \begin{bmatrix} 0 & 0 & -a_n & 0 & 0 & 0 \end{bmatrix}\f]

#### 方向特征的交互矩阵
\f[\frac{\partial \alpha}{\partial v} = L_{\alpha} = \begin{bmatrix} 0 & 0 & 0 & 0 & 0 & -1 \end{bmatrix}\f]

#### C不变矩的交互矩阵
C不变矩的交互矩阵较为复杂，涉及高阶导数计算，用于控制空间旋转。

## 视觉特征误差计算

### 误差定义

视觉伺服中的特征误差定义为：
\f[e = s - s^*\f]

其中：
- \f$s\f$：当前视觉特征向量
- \f$s^*\f$：期望视觉特征向量

### 矩特征误差的具体计算

在`vpServo::computeError()`中，误差通过每个特征的`error()`方法计算：

```cpp
vectTmp = current_s->error(*desired_s, select);
```

#### 矩特征误差示例

对于重心归一化特征：
\f[e_{x_n} = x_n - x_n^*\f]
\f[e_{y_n} = y_n - y_n^*\f]

对于面积归一化特征：
\f[e_{a_n} = a_n - a_n^*\f]

对于方向特征：
\f[e_{\alpha} = \alpha - \alpha^*\f]

### 复合特征误差

对于包含多个子特征的复合矩特征（如C不变矩），误差向量会级联所有子特征的误差。

## 控制律的矩阵形式

### 基本视觉伺服控制律

\f[v = -\lambda \hat{L}^+ e\f]

其中：
- \f$v\f$：相机速度向量（6×1）
- \f$\lambda\f$：控制增益
- \f$\hat{L}^+\f$：交互矩阵的伪逆
- \f$e\f$：特征误差向量

### 扩展到机器人空间

对于眼在手上配置：
\f[v_c = v\f]

对于眼到手上配置，需要坐标系变换：
\f[v_c = {^c}V_e v_e\f]

### 任务雅可比矩阵

完整的控制律涉及任务雅可比矩阵\f$J_1\f$：
\f[J_1 = L \cdot {^c}V_a \cdot {^a}J_e\f]

其中：
- \f$L\f$：特征交互矩阵
- \f${^c}V_a\f$：相机到机器人基座的速度变换
- \f${^a}J_e\f$：机器人雅可比矩阵

### 最终控制速度

\f[v = -\lambda J_1^+ e\f]

其中\f$J_1^+\f$是任务雅可比矩阵的伪逆。

## 基于矩的视觉伺服完整数学框架

### 特征向量定义

基于矩的视觉特征向量\f$s\f$通常包括：
\f[s = \begin{bmatrix} x_n \\ y_n \\ a_n \\ s_x \\ s_y \\ \alpha \end{bmatrix}\f]

其中：
- \f$(x_n, y_n)\f$：归一化重心坐标
- \f$a_n\f$：归一化面积
- \f$(s_x, s_y)\f$：形状参数（从C不变矩导出）
- \f$\alpha\f$：方向角

### 交互矩阵结构

完整的6×6交互矩阵\f$L\f$为：
\f[L = \begin{bmatrix} L_{x_n} \\ L_{y_n} \\ L_{a_n} \\ L_{s_x} \\ L_{s_y} \\ L_{\alpha} \end{bmatrix}\f]

### 控制稳定性

基于矩的视觉伺服的稳定性依赖于：
1. **特征可观测性**：交互矩阵\f$L\f$满秩
2. **控制增益**：\f$\lambda\f$的选择
3. **奇异配置避免**：防止\f$L\f$接近奇异

### 收敛条件

伺服过程收敛当：
\f[\|e\| = \|s - s^*\| < \epsilon\f]

其中\f$\epsilon\f$是预设的收敛阈值。

这个数学框架提供了基于矩的视觉伺服的完整理论基础，确保了算法的稳定性和收敛性。
