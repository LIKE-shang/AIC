# 安装说明

本次版本为 20261018_audit_v2。请使用配套部署包 slam_navigation_ground_audit_20261018_v2 内的 INSTALL_CN.md 和 install.sh，同时更新定位与地面控制代码。旧版 v3 安装脚本只安装导航，无法修复 transform_fusion 的地图坐标误标问题。

已有 config/straight_run.yaml 将保留，安装后必须审核其内容。README_CN.md 给出完整启动顺序和实机验收限制。

重要更正：v2 包内底盘启动指引有误。请先阅读 ../_deploy/CHASSIS_ENTRY_CORRECTION_20261018_CN.md，使用厂家的 minimal.launch（rosserial），不要用 minimal_uart.launch。无需重新安装本导航代码。
