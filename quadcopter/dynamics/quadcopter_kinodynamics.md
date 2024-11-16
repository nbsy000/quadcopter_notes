# 无人机动力学

## 动力系统模型
拉力$T = C_T \rho(\frac{N}{60})D_p^4$ ，拉力与转速平方成正比，与叶片半径四次成正比

输出转矩$M = K_T(I_m - I_{m0})$ 

电池模型：$P_{mot} = \frac{c_d\Omega^3}{\eta}$

转速模型：

$\dot\Omega = \frac{1}{k_{mot}}(\Omega_{ss} - \Omega)$，$\Omega_{ss}$代表稳态时的转速

$\Omega_{ss} \sim 1 + U_{bat} + \sqrt{u_{cmd}} + u_{cmd} + U_{bat}\sqrt{u_{cmd}}$， 代表稳态转速与电池电压以及电机控制指令相关

## 动力学模型

$^w\dot p = {^wv}$ 

${^{w}\dot{v}} = {^wg} + \frac{1}{m} (R({^bf_{prop}} + {^bf_{aero}}))={^wg}+\frac{1}{m}R·{^bf}$

${^{w}v} = R · {^bv} \rightarrow {^{w}\dot{v}} =R·^b\dot{v} +\dot{R}·{^bv}$ 

$^{b}\dot{v} =R^T(^{w}\dot{v} -\dot{R}^{b}v) ={^wg}R^T+\frac{^bf}{m} - [\omega]_{\times} {^{b}v}$

$\dot{R} = R [\omega]_{\times}$

$\dot{\omega} = J^{-1} (\eta_{prop} + \eta_{mot} - \omega \times J\omega)$

其中：

$f_{prop} = \sum f_i \ , f_i = c_l\Omega^2$

$\eta_{prop} = \eta_x + \eta_y + \eta_z$

$\eta_x = \eta_y = r\times f_{prop}$ 也就是代表body系z轴方向推力作用产生的扭矩

$\eta_z = \sum c_d\Omega_i^2$，代表桨叶由于拨动空气产生的反作用扭矩

$\eta_{mot} = J_{m+p} \sum \dot\Omega_i$ 代表由于桨叶加速产生的反作用扭矩

## 气动模型
${^bf_{areo}} = {^bf_x} +{^bf_y} + {^bf_z}$ 也就是说body系的空气阻力可以分成x，y，z三个轴进行分析，对于x，y轴参数基本相同，z轴有比较大的不同。

${^bf_x} \sim {^bv_x} + {^bv_x}|{^bv_x}|+\Omega^2+{^bv_x}\Omega^2$

${^bf_y} \sim {^bv_y} + {^bv_y}|{^bv_y}|+\Omega^2+{^bv_y}\Omega^2$

${^bf_z} \sim {^bv_z} + {^bv_z}|{^bv_z}|+{^bv_z}\Omega^2 +{^bv_{xy}} + {^bv_{xy}}^2 + {^bv_{xy}}\Omega^2 + {^bv_z}{^bv_{xy}}\Omega^2$