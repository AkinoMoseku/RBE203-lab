# RBE203 Lab — 双轮平衡车

基于 Arduino Uno 的双轮平衡车实验项目，包含控制程序、硬件诊断程序、依赖库和实验资料。

## 项目内容

| 路径 | 内容 |
| --- | --- |
| `two_wheel_robot/` | 平衡车主程序及辅助源码 |
| `robot_diagnostic/` | MPU6050 和电机诊断程序 |
| `dependencies/libraries/` | 项目使用的 Arduino 依赖库 |
| `dependencies/*.zip` | 依赖库压缩包 |
| `dependencies/backups/` | 本地已有的历史源码备份；当前程序位于 `two_wheel_robot/` |
| `库文件/` | 随实验提供的库文件和使用说明 |
| `library_for_two_wheel_robot.rar` | 原始库文件压缩包 |
| `Stabilization of two-wheel robot.pdf` | 实验参考资料 |

`tmp/` 中的编译产物、串口日志和临时分析文件不纳入版本控制。

## 使用

1. 在 Arduino IDE 中安装 Arduino AVR Boards，选择 **Arduino Uno**。
2. 将 `dependencies/libraries/` 下的库安装到 Arduino sketchbook 的 `libraries` 目录。主要依赖为 PinChangeInt、MsTimer2、BalanceCar、KalmanFilter、I2Cdev 和 MPU6050；Wire 由开发板核心提供。
3. 打开 `two_wheel_robot/two_wheel_robot.ino` 并编译。诊断程序的入口为 `robot_diagnostic/robot_diagnostic.ino`。
4. 串口监视器波特率为 **9600**。

当前源码包含固定的陀螺仪零偏及控制参数；更换硬件后需重新核对。运行电机诊断前应将车轮悬空，诊断程序不执行平衡控制。

## 参与修改

欢迎通过 Fork → 修改 → Pull Request 提交改进，也可以通过 Issues 反馈问题。

公开仓库允许所有人查看和 Fork。直接向本仓库推送修改需要由仓库所有者邀请为协作者；GitHub 没有向所有互联网用户统一开放直接写入的设置。

第三方库保留各自原有的许可证及版权声明。
