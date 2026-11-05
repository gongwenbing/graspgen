# 桌面标记只读对照结论

用户将95mm、ID0标记平放桌面，与瓶子同时可见，机械臂不动。20组同步图像/深度/采图时刻TF观测全部通过标记重投影、姿态歧义和深度覆盖筛选。

- PnP标记中心在base_link下Z中位数：-24.161mm。
- 标记内部深度点在base_link下Z中位数：-23.966mm。
- 相同像素的实测深度与PnP标记平面相比，base Z差中位数：+0.097mm。
- 对应相机Z深度差中位数：-0.105mm。
- 标记XYZ峰峰值：[0.146,0.424,0.619]mm；重投影误差中位数0.103px。

实时link6到camera_color_optical_frame的TF与handeye_result.yaml完全一致（浮点误差量级）；URDF及手眼文件哈希均与原经验profile一致，故没有证据表明加载了另一份标定/模型。

在标记实际位于base_link Z=0桌面这一前提下，主偏差位于相机到基座的外参/运动学/物理参考链；不能将其解释为深度图单独偏移约24mm，也与TCP无关。单姿态只能确定该姿态下的竖直残差，不能唯一确定通用外参修正；不得据此直接将相机外参某个分量增加24mm或降低桌面碰撞体。

后续需在标记空间位置不变时采集其他末端方向的对应观测，确认误差是否随姿态变化，再选择参考基准校正或重新手眼标定。未修改TCP、手眼、桌面或运动门控，未发送任何运动指令。

数据：calibration/marker_depth_inspection_20261105.json；calibration/handeye_live_tf_inspection_20261105.json。只读复测工具：scripts/inspect_marker_depth.py。
