# 无人机动力学模型与控制

## 1. 动力系统模型

### 1.1 拉力与转速关系
拉力 $T$ 与转速的关系为：
$$
T = k \omega^2 + b \omega + c
$$


### 1.2 输出转矩
输出转矩 $M$ 由以下公式给出：
$$
M = K_T (I_m - I_{m0})
$$
其中，$I_m$ 和 $I_{m0}$ 分别为电机当前的转动惯量和初始转动惯量。

### 1.3 电池模型
电池的功率模型为：
$$
P_{mot} = \frac{c_d \Omega^3}{\eta}
$$
其中，$P_{mot}$ 为电池输出的功率，$\Omega$ 为转速，$\eta$ 为效率。

### 1.4 转速模型
转速的动态变化由以下方程描述：
$$
\dot{\Omega} = \frac{1}{k_{mot}} (\Omega_{ss} - \Omega)
$$
其中，$\Omega_{ss}$ 为稳态转速。

稳态转速 $\Omega_{ss}$ 可以通过电池电压 $U_{bat}$ 和电机控制指令 $u_{cmd}$ 来表示：
$$
\Omega_{ss} \sim 1 + U_{bat} + \sqrt{u_{cmd}} + u_{cmd} + U_{bat} \sqrt{u_{cmd}}
$$

## 2. 动力学模型

### 2.1 位置和速度关系
无人机的运动方程如下：
$$
^w \dot{p} = {^w v}
$$
$$
^{w} \dot{v} = {^w g} + \frac{1}{m} \left( R({^b f_{prop}} + {^b f_{aero}}) \right)
$$
其中，$^w \dot{p}$ 和 $^w v$ 分别是位置的变化率和速度，$^w g$ 为重力加速度，$m$ 为无人机的质量，$R$ 为旋转矩阵，${^b f_{prop}}$ 和 ${^b f_{aero}}$ 分别是推进力和空气阻力。

### 2.2 速度与转速的关系
无人机的速度与转速的关系为：
$$
^{w} v = R \cdot {^b v} \rightarrow {^{w} \dot{v}} = R \cdot {^b \dot{v}} + \dot{R} \cdot {^b v}
$$
其中，${^b \dot{v}}$ 为体坐标系下的加速度，$\dot{R}$ 为旋转矩阵的导数。

### 2.3 动力学方程
无人机的动力学方程为：
$$
^{b} \dot{v} = R^T (^{w} \dot{v} - \dot{R}^b v) = {^w g} R^T + \frac{^b f}{m} - [\omega]_{\times} {^b v}
$$
$$
\dot{R} = R [\omega]_{\times}
$$
$$
\dot{\omega} = J^{-1} (\eta_{prop} + \eta_{mot} - \omega \times J \omega)
$$
其中，$\omega$ 为角速度，$J$ 为转动惯量矩阵。

### 2.4 推进力与扭矩
推进力 $f_{prop}$ 由所有桨叶产生的推力之和给出：
$$
f_{prop} = \sum f_i, \quad f_i = c_l \Omega^2
$$
推进力产生的扭矩 $\eta_{prop}$ 包括三个部分：
- $\eta_x = \eta_y = r \times f_{prop}$，代表体坐标系下 x 和 y 方向的推力作用产生的扭矩。
- $\eta_z = \sum c_d \Omega_i^2$，代表由于桨叶拨动空气产生的反作用扭矩。
  
电机的反作用扭矩 $\eta_{mot}$ 由桨叶加速产生：
$$
\eta_{mot} = J_{m+p} \sum \dot{\Omega_i}
$$

## 3. 气动模型

气动阻力可以分解为沿体坐标系的三个轴方向（x, y, z）：
$$
{^b f_{aero}} = {^b f_x} + {^b f_y} + {^b f_z}
$$
对于不同方向，空气阻力的表达式如下：
- ${^b f_x} \sim {^b v_x} + {^b v_x}|{^b v_x}| + \Omega^2 + {^b v_x} \Omega^2$
- ${^b f_y} \sim {^b v_y} + {^b v_y}|{^b v_y}| + \Omega^2 + {^b v_y} \Omega^2$
- ${^b f_z} \sim {^b v_z} + {^b v_z}|{^b v_z}| + {^b v_z} \Omega^2 + {^b v_{xy}} + {^b v_{xy}}^2 + {^b v_{xy}} \Omega^2 + {^b v_z} {^b v_{xy}} \Omega^2$

## 4. SE3 控制框架

![SE3 Control Framework](imgs/SE3.png)

### 4.1 推力与扭矩转矩矩阵
推力和转矩矩阵可以表示为：
$$
\begin{bmatrix}
f \\
M_1 \\
M_2 \\
M_3
\end{bmatrix}
=
\begin{bmatrix}
1 & 1 & 1 & 1 \\
0 & -d & 0 & d \\
d & 0 & -d & 0 \\
-c_{\tau f} & c_{\tau f} & -c_{\tau f} & c_{\tau f}
\end{bmatrix}
\begin{bmatrix}
f_1 \\
f_2 \\
f_3 \\
f_4
\end{bmatrix}
$$

### 4.2 推力角速度环控制
控制指令为：
$$
cmd = [f_{cmd}, w_x, w_y, w_z]^T = [f_{cmd}, w_{cmd}]^T
$$
可观测状态为：
$$
state = [R, w, v, a]
$$
其中，$J$ 为飞机机身的转动惯量。

转矩控制器为：
$$
M = J \cdot PID(R^T \cdot w_{cmd} - w) + w \times (J \cdot w)
$$

### 4.3 速度环控制
控制指令为：
$$
cmd = [\theta_{yaw}, v_x, v_y, v_z] = [\theta_{yaw}, v_{cmd}]
$$
速度残差 $v_{res}$ 为：
$$
v_{res} = v_{cmd} - v
$$
加速度指令为：
$$
a_{des} = PID(v_{res})
$$
所需推力为：
$$
f_{des} = m(a + g), \quad f_{cmd} = f_{des} \cdot Re_3
$$
计算法向量：
$$
\vec{n_z} = \frac{f_{des}}{|f_{des}|}, \quad n_y = \text{norm}(n_z \times [\cos(\theta_{yaw}), \sin(\theta_{yaw}), 0]^T), \quad n_x = n_y \times n_z
$$
期望旋转矩阵为：
$$
R_{des} = [n_x, n_y, n_z], \quad R_{res} = [R^T R_{des} - R_{des}^T R]^{\vee}
$$
最终的控制转矩为：
$$
M_{cmd} = PID(R_{res}) + w \times Jw
$$
