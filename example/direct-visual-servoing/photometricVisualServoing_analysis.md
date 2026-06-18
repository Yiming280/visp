# photometricVisualServoing.cpp 深度解析

## 1. 程序入口

程序入口为 `example/direct-visual-servoing/photometricVisualServoing.cpp` 中的 `main(int argc, const char **argv)`。

入口流程：

1. 解析命令行参数：`getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display, opt_niter)`。
2. 从环境变量 `VISP_INPUT_IMAGE_PATH` 或命令行 `-i` 获取图像数据目录。
3. 读取纹理图像 `Klimt/Klimt.pgm`。
4. 使用 `vpImageSimulator` 创建虚拟图像平面并生成相机图像。
5. 初始化 desired/curr 视觉特征 `vpFeatureLuminance`。
6. 创建 `vpServo` 任务，设置控制律、增益和交互矩阵类型。
7. 在循环中：仿真相机取像、计算当前视觉特征、求解控制律、发送相机速度、更新相机位姿。

程序入口函数文件：

- `example/direct-visual-servoing/photometricVisualServoing.cpp`

## 2. 库结构与模块

这个例子依赖 ViSP 的多个模块：

- `modules/core`：基础数学、图像、矩阵、投影与相机参数。
  - 例如：`vpImage.h`、`vpCameraParameters.h`、`vpHomogeneousMatrix.h`、`vpMath.h`、`vpImageTools.h`、`vpIoTools.h`
- `modules/io`：图像读写、参数解析。
  - 例如：`vpImageIo.h`、`vpParseArgv.h`
- `modules/visual_features`：视觉特征定义。
  - 例如：`vpFeatureLuminance.h`、`vpFeatureLuminance.cpp`
- `modules/vs`：视觉伺服控制器。
  - 例如：`vpServo.h`、`vpServo.cpp`
- `modules/robot`：机器人/相机仿真。
  - 例如：`vpImageSimulator.h`、`vpSimulatorCamera.h`

从程序来看，关键计算在视觉特征与视觉伺服模块中：

- 视觉特征的构造和交互矩阵：`vpFeatureLuminance`
- 控制律的计算：`vpServo`
- 图像读取与仿真：`vpImageIo`、`vpIoTools`、`vpImageSimulator`

## 3. 图片是怎么读取的

该程序读取一张灰度纹理图：

```cpp
vpImage<unsigned char> Itexture;
filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
vpImageIo::read(Itexture, filename);
```

关键点：

- `vpIoTools::getViSPImagesDataPath()` 返回 ViSP 示例图像所在目录。
- `vpIoTools::createFilePath(parent, child)` 拼接目录和文件名。
- `vpImageIo::read(Itexture, filename)` 根据文件扩展名读取图像。
  - `vpImageIo` 支持 PGM、PPM、JPEG、PNG 等格式。
  - 在本例中，读取灰度图像 `vpImage<unsigned char>`。

所以，图像读取入口在 `vpImageIo::read()`，它会根据文件名后缀选择读 PGM 或其它格式。

### 图片加载后如何使用

读取后，`Itexture` 被用作纹理图像，随后传给 `vpImageSimulator::init`：

```cpp
sim.init(Itexture, X);
```

`Itexture` 作为平面纹理，投影到 3D 平面上，再由虚拟相机采样生成 `I` 和 `Id`。

## 4. 具体实现在哪：算法整体结构

这个程序实现的是基于亮度的直接视觉伺服 (photometric visual servoing)，核心思路是：

- 目标图像 `Id` 是“期望图像”。
- 当前图像 `I` 是实时相机图像。
- 通过图像亮度误差 `I - Id` 构造视觉误差。
- 使用亮度特征的交互矩阵求得相机速度，使当前图像向目标图像收敛。

程序关键实现位置：

- `vpFeatureLuminance::buildFrom(vpImage<unsigned char> &I)`
  - 构建亮度特征并计算梯度。
- `vpFeatureLuminance::interaction(vpMatrix &L)`
  - 计算亮度特征交互矩阵。
- `vpFeatureLuminance::error(const vpBasicFeature &s_star, vpColVector &e)`
  - 计算视觉误差向量。
- `vpServo::computeError()`
  - 将所有特征误差拼接成任务误差向量。
- `vpServo::computeInteractionMatrix()`
  - 将所有特征交互矩阵拼接成任务交互矩阵。
- `vpServo::computeControlLaw()`
  - 求解视觉伺服控制律，输出相机速度。
- `vpSimulatorCamera::setVelocity()` / `getPosition()`
  - 将控制速度应用到仿真相机上，更新位姿。

这些函数联合完成了“从图像到速度”的闭环控制。

## 5. 关键函数与调用关系

### 5.1 命令行参数解析

- `getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display, opt_niter)`
  - 使用 `vpParseArgv::parse()` 解析参数。
  - 选项：
    - `-i <input image path>`：图像路径。
    - `-c`：禁用点击继续。
    - `-d`：关闭显示。
    - `-n <number>`：最大迭代次数。

`vpParseArgv` 是通用参数解析工具，属于 `modules/io`。

### 5.2 图像与仿真初始化

1. 定义 4 个顶点 `X[4]` 表示纹理平面的 3D 角点。
2. 创建 `vpImageSimulator sim;`
3. `sim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);`
4. `sim.init(Itexture, X);`
5. 定义相机内参：

```cpp
vpCameraParameters cam(870, 870, 160, 120);
```

6. 创建当前图像 `I(240, 320, 0)`，`Id` 等。
7. 设置期望相机位姿 `cdMo[2][3] = 1;`
8. `sim.setCameraPosition(cdMo);`
9. `sim.getImage(I, cam);` 获得期望图像 `Id`。

这里 `vpImageSimulator` 将 `Itexture` 投射到由 `X` 定义的 3D 平面上，再由虚拟相机采样获得 `I`。

### 5.3 当前图像与差分图

- 初始位置 `cMo.buildFrom(...)` 设定当前相机初始姿态。
- `sim.setCameraPosition(cMo);`
- `sim.getImage(I, cam);` 获取当前图像 `I`。
- `vpImageTools::imageDifference(I, Id, Idiff);` 计算图像差分并显示。

`imageDifference` 仅用于可视化，不直接参与控制律。

### 5.4 视觉特征初始化

程序定义两个亮度特征：

```cpp
vpFeatureLuminance sI;
sI.init(I.getHeight(), I.getWidth(), Z);
sI.setCameraParameters(cam);
sI.buildFrom(I);

vpFeatureLuminance sId;
sId.init(I.getHeight(), I.getWidth(), Z);
sId.setCameraParameters(cam);
sId.buildFrom(Id);
```

- `init(rows, cols, Z)` 初始化特征维度与深度常数。
- `setCameraParameters(cam)` 设置相机内参。
- `buildFrom(I)` 依据图像构建亮度值与梯度。

### 5.5 视觉伺服任务配置

```cpp
vpServo servo;
servo.setServo(vpServo::EYEINHAND_CAMERA);
servo.addFeature(sI, sId);
servo.setLambda(30);
servo.setInteractionMatrixType(vpServo::CURRENT);
```

- `setServo(EYEINHAND_CAMERA)`：使用眼端手法，速度计算在相机坐标系下。
- `addFeature(sI, sId)`：加入当前与期望特征对。
- `setLambda(30)`：控制增益为 30。
- `setInteractionMatrixType(CURRENT)`：交互矩阵计算使用当前特征。

### 5.6 机器人仿真

```cpp
vpSimulatorCamera robot;
robot.setSamplingTime(0.04);
wMc = wMo * cMo.inverse();
robot.setPosition(wMc);
robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
```

- `vpSimulatorCamera` 是一个 6-自由度自由飞行相机仿真器。
- `setVelocity(..., v)` 将计算出的相机速度应用到仿真机器人上。
- `getPosition()` 返回新的世界到相机变换 `wMc`。

## 6. 图像误差和机器人速度是怎么算的

### 6.1 亮度特征的构建与梯度

在 `vpFeatureLuminance::buildFrom(vpImage<unsigned char> &I)` 中执行以下步骤：

1. 计算像素对应的相机坐标平面点 `(x, y)`：
   - 通过 `vpPixelMeterConversion::convertPoint(cam, j, i, x, y)`
   - 这将像素坐标 `(j, i)` 转成以米为单位的成像平面坐标。
2. 计算图像梯度：
   - `Ix = px * vpImageFilter::derivativeFilterX(I, i, j);`
   - `Iy = py * vpImageFilter::derivativeFilterY(I, i, j);`
   - 其中 `px = cam.get_px()`，`py = cam.get_py()`。
3. 存储亮度值和梯度：
   - `pixInfo[l].I = I[i][j];`
   - `pixInfo[l].Ix = Ix;`
   - `pixInfo[l].Iy = Iy;`
   - `s[l] = I[i][j];`

这一步得到当前图像在每个像素位置上的亮度和导数信息。

### 6.2 亮度特征交互矩阵

`vpFeatureLuminance::interaction(vpMatrix &L)` 计算每个像素的 6×1 行：

```cpp
L[m][0] = Ix * Zinv;
L[m][1] = Iy * Zinv;
L[m][2] = -(x * Ix + y * Iy) * Zinv;
L[m][3] = -Ix * x * y - (1 + y * y) * Iy;
L[m][4] = (1 + x * x) * Ix + Iy * x * y;
L[m][5] = Iy * x - Ix * y;
```

其中：

- `Ix`、`Iy` 是亮度梯度。
- `x`、`y` 是像素在相机成像平面上的坐标。
- `Zinv = 1 / Z`，此例中 `Z` 是常数 1。

这个矩阵是亮度特征的经典视觉互动矩阵，用来将图像亮度变化与相机速度联系起来。

### 6.3 误差计算

亮度误差由 `vpFeatureLuminance::error` 给出：

```cpp
e[i] = s[i] - s_star[i];
```

也就是逐像素的亮度差分。对于整个特征，`vpServo::computeError()` 会把每个特征的误差拼接成一个长向量 `error`。

在例程中，误差向量对应的是当前图像亮度 `I` 与期望图像亮度 `Id` 的差异。

### 6.4 控制律计算

在 `vpServo::computeControlLaw()` 中，主要计算步骤为：

1. `computeInteractionMatrix()`：得到任务交互矩阵 `L`。
2. `computeError()`：得到任务误差向量 `error`。
3. 构造任务雅可比：
   - `J1 = L * cVa * aJe;`
   - 对于 `EYEINHAND_CAMERA`，`cVa` 和 `aJe` 通常为单位矩阵。
4. 求伪逆：
   - `J1.pseudoInverse(J1p, sv, m_pseudo_inverse_threshold, imJ1, imJ1t);`
5. 计算主任务速度：
   - `e1 = WpW * J1p * error;`
   - `e = -lambda(e1) * e1;`

因此最终相机速度向量为：

```text
v = - lambda * J1^+ * error
```

其中 `J1^+` 是任务雅可比的伪逆，`lambda` 由 `servo.setLambda(30)` 设置。

在当前程序中，`J1` 的行数等于亮度特征数，列数等于 6，对应相机的 6 个速度自由度。

### 6.5 机器人速度应用

控制法线性输出是相机速度 `v`：

```cpp
v = servo.computeControlLaw();
robot.setVelocity(vpRobot::CAMERA_FRAME, v);
wMc = robot.getPosition();
cMo = wMc.inverse() * wMo;
```

- `robot.setVelocity(vpRobot::CAMERA_FRAME, v)` 将速度作用于仿真相机。
- `robot.getPosition()` 返回新的相机位姿 `wMc`。
- `cMo` 则通过 `wMc.inverse() * wMo` 得到相机相对目标位姿。

随后下一次循环，`sim.setCameraPosition(cMo)` 使虚拟相机按照新位姿取图。

## 7. 循环执行细节

主循环如下：

```cpp
do {
  sim.setCameraPosition(cMo);
  sim.getImage(I, cam);
  vpImageTools::imageDifference(I, Id, Idiff);
  sI.buildFrom(I);
  v = servo.computeControlLaw();
  normError = servo.getError().sumSquare();
  robot.setVelocity(vpRobot::CAMERA_FRAME, v);
  wMc = robot.getPosition();
  cMo = wMc.inverse() * wMo;
} while (normError > 10000 && iter < opt_niter);
```

每次迭代流程：

1. 把仿真相机位置设为当前 `cMo`。
2. 生成当前相机图像 `I`。
3. 计算图像差分 `Idiff` 并显示。
4. 更新当前亮度特征 `sI`。
5. 调用 `servo.computeControlLaw()` 计算相机速度 `v`。
6. 计算误差范数 `normError = servo.getError().sumSquare()`。
7. 将速度命令发送给仿真机器人，更新相机位姿。
8. 继续直到误差低于阈值或达到最大迭代次数。

## 8. 重要函数总览

| 功能 | 函数 | 文件 |
|---|---|---|
| 命令行解析 | `vpParseArgv::parse` | `modules/io/src/tools/vpParseArgv.cpp` |
| 图像读取 | `vpImageIo::read` | `modules/io/include/visp3/io/vpImageIo.h` |
| 图像路径辅助 | `vpIoTools::getViSPImagesDataPath` / `createFilePath` | `modules/core/include/visp3/core/vpIoTools.h` |
| 纹理投影仿真 | `vpImageSimulator::init` / `setCameraPosition` / `getImage` | `modules/robot/include/visp3/robot/vpImageSimulator.h` |
| 特征构建 | `vpFeatureLuminance::buildFrom` | `modules/visual_features/src/visual-feature/vpFeatureLuminance.cpp` |
| 交互矩阵 | `vpFeatureLuminance::interaction` | `modules/visual_features/src/visual-feature/vpFeatureLuminance.cpp` |
| 误差计算 | `vpFeatureLuminance::error` | `modules/visual_features/src/visual-feature/vpFeatureLuminance.cpp` |
| 任务交互矩阵 | `vpServo::computeInteractionMatrix` | `modules/vs/src/vpServo.cpp` |
| 任务误差 | `vpServo::computeError` | `modules/vs/src/vpServo.cpp` |
| 控制律计算 | `vpServo::computeControlLaw` | `modules/vs/src/vpServo.cpp` |
| 仿真相机速度 | `vpSimulatorCamera::setVelocity` / `getPosition` | `modules/robot/include/visp3/robot/vpSimulatorCamera.h` |

## 9. 关键公式总结

### 9.1 误差公式

视觉误差向量 `e` 为：

```text
e = s - s*.
```

这里 `s` 是当前亮度值向量，`s*` 是期望亮度值向量。

### 9.2 交互矩阵 (单像素)

每个像素贡献一行 `L_m`：

```text
L_m = [Ix/Z, Iy/Z, -(x Ix + y Iy)/Z,
       -Ix*x*y - (1+y^2)Iy,
        (1+x^2)Ix + Iy*x*y,
        Iy*x - Ix*y]
```

### 9.3 控制律

对任务雅可比 `J1`，最终速度为：

```text
v = -lambda * J1^+ * error
```

`J1 = L * cVa * aJe`，由于例程使用眼端相机并且仿真中相机与末端一致，`cVa` 与 `aJe` 近似单位矩阵。

## 10. 结论

这个例程的核心是“亮度直接视觉伺服”：

- 通过将当前图像和期望图像的亮度差作为误差，
- 使用亮度特征交互矩阵将误差映射到相机速度，
- 再由仿真相机执行速度命令，逐步让图像收敛。

关键模块分工清晰：

- `vpImageIo` 负责读图，
- `vpImageSimulator` 负责生成当前/目标图像，
- `vpFeatureLuminance` 负责亮度特征与交互矩阵，
- `vpServo` 负责视觉伺服控制律，
- `vpSimulatorCamera` 负责模拟相机运动。

通过这个 `.md` 文档，你可以快速定位每个实现部分和关键函数，并理解误差与速度的计算路径。

---

# photometricVisualServoingWithoutVpServo.cpp 深度解析

## 概述

`photometricVisualServoingWithoutVpServo.cpp` 是 `photometricVisualServoing.cpp` 的变体版本，主要区别在于不使用 `vpServo` 类来封装视觉伺服任务，而是手动实现控制律计算。本例程展示了如何在没有 `vpServo` 抽象的情况下，直接使用 `vpFeatureLuminance` 进行亮度视觉伺服，并采用 Levenberg-Marquardt (LM) 优化方法来求解控制律。

## 与 photometricVisualServoing.cpp 的主要区别

| 方面 | photometricVisualServoing.cpp | photometricVisualServoingWithoutVpServo.cpp |
|------|-------------------------------|--------------------------------------------|
| 伺服封装 | 使用 `vpServo` 类 | 手动实现控制律 |
| 优化方法 | 伪逆求解 | Levenberg-Marquardt 优化 |
| 交互矩阵计算 | 动态计算（当前特征） | 固定在期望位置计算 |
| 控制律公式 | `v = -λ J⁺ e` | `v = -λ H L^T e` (其中 H 是 LM Hessian) |
| 迭代策略 | 固定增益 | 自适应阻尼参数 μ |

## 程序入口与库结构

### 程序入口

程序入口与第一个例程相同：

```cpp
int main(int argc, char **argv)
{
  // 参数解析
  getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display, opt_niter);

  // 图像读取与仿真初始化
  // ... (与第一个例程相同)

  // 主循环
  do {
    // ... 伺服计算与机器人控制
  } while(normError > 1e-8 && iter < opt_niter);
}
```

### 库依赖

与第一个例程相比，移除了 `vpServo` 的依赖，但增加了对矩阵运算的直接使用：

- `modules/core`：矩阵运算 (`vpMatrix`, `vpColVector`)
- `modules/visual_features`：亮度特征 (`vpFeatureLuminance`)
- `modules/robot`：仿真器 (`vpSimulatorCamera`, `vpImageSimulator`)
- `modules/io`：图像 I/O (`vpImageIo`)

## 图像读取与仿真设置

图像读取和仿真初始化与第一个例程完全相同：

- 使用 `vpImageIo::read()` 读取纹理图像
- `vpImageSimulator` 初始化纹理投影
- 相机内参设置：`vpCameraParameters cam(870, 870, 160, 120)`
- 期望图像通过 `sim.getImage(Id, cam)` 获取

## 视觉特征初始化

与第一个例程类似，创建两个亮度特征：

```cpp
vpFeatureLuminance sI, sId;
sI.init(I.getHeight(), I.getWidth(), Z);
sI.setCameraParameters(cam);
sId.init(I.getHeight(), I.getWidth(), Z);
sId.setCameraParameters(cam);
```

但在主循环中，只有当前特征 `sI` 会动态更新：

```cpp
sI.buildFrom(I);  // 每次迭代更新当前特征
```

期望特征 `sId` 在循环外构建一次：

```cpp
sId.buildFrom(Id);  // 只在初始化时构建
```

## 交互矩阵计算

**关键区别**：交互矩阵在期望位置计算，而不是当前位置。

```cpp
vpMatrix Lsd(I.getHeight() * I.getWidth(), 6);
sId.interaction(Lsd);  // 在期望特征位置计算交互矩阵
```

这与第一个例程的 `servo.setInteractionMatrixType(vpServo::CURRENT)` 不同，这里使用期望位置的交互矩阵，这是一种常见的视觉伺服策略，可以提供更好的收敛特性。

## 控制律计算

### Levenberg-Marquardt 优化

本例程使用 LM 方法手动实现控制律：

```cpp
// 计算 Hessian 矩阵
vpMatrix Hsd = Lsd.AtA();  // Lsd^T * Lsd

// LM 阻尼参数
double mu = 0.01;

// 对角线 Hessian 用于 LM
vpMatrix diagHsd = Hsd;
for(unsigned int i = 0; i < Hsd.getRows(); i++) {
  for(unsigned int j = 0; j < Hsd.getCols(); j++) {
    if(i != j) diagHsd[i][j] = 0;
  }
}

// LM Hessian: H = (μ * diag(Hsd) + Hsd)^(-1)
vpMatrix H = (mu * diagHsd + Hsd).inverseByLU();

// 计算误差向量
vpColVector error(I.getHeight() * I.getWidth());
sI.error(sId, error);

// 中间控制向量: e = H * Lsd^T * error
vpColVector e = H * Lsd.t() * error;

// 最终速度: v = -λ * e
double lambda = 30.0;
vpColVector v = -lambda * e;
```

### 迭代优化策略

程序实现了自适应 LM 参数调整：

```cpp
if (iter > 90) {
  mu = 0.0;  // 90 次迭代后切换到 Gauss-Newton 方法
}
```

这意味着前 90 次迭代使用 LM 优化（有阻尼），之后使用纯 Gauss-Newton 方法（无阻尼）。

## 误差计算

误差计算与第一个例程相同：

```cpp
vpColVector error(I.getHeight() * I.getWidth());
sI.error(sId, error);
```

这是逐像素的亮度差：`error[i] = sI[i] - sId[i]`

误差范数用于终止条件：

```cpp
double normError = error.sumSquare();
```

## 机器人速度应用

与第一个例程相同：

```cpp
robot.setVelocity(vpRobot::CAMERA_FRAME, v);
wMc = robot.getPosition();
cMo = wMc.inverse() * wMo;
```

## 关键公式总结

### 误差公式

```
e = s - s*
```

其中 `s` 是当前亮度向量，`s*` 是期望亮度向量。

### 交互矩阵

在期望位置计算：

```
L* ∈ ℝ^{(M×N)×6}
```

其中 M×N 是图像像素数。

### Levenberg-Marquardt Hessian

```
H_sd = L*^T * L*
H = (μ * diag(H_sd) + H_sd)^{-1}
```

### 控制律

```
e = H * L*^T * error
v = -λ * e
```

## 实现优势与特点

### 优势

1. **更灵活的优化**：可以实现 LM 或其他优化方法
2. **更好的收敛**：期望位置交互矩阵 + LM 阻尼
3. **可定制性**：可以修改 Hessian 计算或添加约束

### 特点

1. **手动实现**：不依赖 `vpServo` 封装
2. **固定交互矩阵**：在期望位置计算一次
3. **自适应阻尼**：迭代过程中调整 LM 参数 μ

## 结论

`photometricVisualServoingWithoutVpServo.cpp` 展示了如何在不使用 `vpServo` 的情况下手动实现亮度视觉伺服。通过在期望位置计算交互矩阵并使用 Levenberg-Marquardt 优化，这个实现提供了比伪逆方法更好的收敛特性和鲁棒性。

与使用 `vpServo` 的版本相比，这个手动实现版本：
- 更复杂但更灵活
- 允许自定义优化策略
- 提供了对算法内部工作原理的更深入理解

两个版本共同展示了 ViSP 中视觉伺服的两种实现方式：封装式（`vpServo`）和手动式，每种方法都有其适用场景。
