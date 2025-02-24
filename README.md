# wust_nav
## 武汉科技大学导航仓库
参考：深圳北理莫斯科大学[pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav?tab=readme-ov-file#pb2025_sentry_nav)框架
# 项目介绍
使用mid360激光雷达配合point_lio里程计，采用先验点云配准smail_gicp,通过point_lio构建点云地图，使用pcdtogrid生成预制静态全局/局部代价地图，运用terrain_analysis点云分隔过滤非地面障碍物，使用输出点云构建局部代价地图障碍物层，采用NAV2全局路径规划， pb_omni_pid_pursuit_controller路径跟踪控制器。
新增主动避障模式
# roadmap
- [ ] 配合自瞄系统的跟随模式
- [ ] 多传感器融合，增加深度相机感知系统
- [ ] 基于先验点云的差异点云分割
- [ ] 决策行为树
- [ ] 强化？学习？
