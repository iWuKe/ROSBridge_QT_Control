# ROSBridge_Qt_Client 基于ROSBridge开发的ROS控制端

## 界面预览
![c53d4b47b4d7c23a12a4e92e8db5fbd](https://user-images.githubusercontent.com/43928335/223720356-0ccec1f9-2a4e-4806-a888-0c642aa1ecc9.png)<br>
![36def5cda4e6860f140c0b51819a746](https://user-images.githubusercontent.com/43928335/223720378-7d2d8f84-d961-4190-88eb-a193e044e2ae.png)<br>


## 使用说明：

1. 由Qt5.15.2构建并编译运行成功<br>
![image](https://user-images.githubusercontent.com/43928335/223720108-8314cbda-d6ba-4d18-b98b-bca3707f6db1.png)<br>
![bf25e909cca2f8a8f96526accba43c1](https://user-images.githubusercontent.com/43928335/223720155-b300b548-1a1a-40e6-be44-67bc2670a5cb.png)<br>

2. 可在Qt使用安卓套件编译生成安卓程序，已经配置好了横屏全屏<br>
![21f98e987de741fabd0b302d070b3f6](https://user-images.githubusercontent.com/43928335/223720292-fb5eccca-d3ee-4bb7-a458-607bc0fb47bb.jpg)<br>


## 注意事项：

1. 工程的保存建图和加载地图功能其实是没有完成的，因为能力不够，且资料少，没研究出来怎么使用ROSBridge或者其他方法去实现它。
2. 该工程是个人研究ROS和Qt的学习之作，存在有很多不足，代码结构较为混乱。
<<<<<<< HEAD
3. 程序的优化较差。
=======
3. 程序的优化较差，因为WebSocket接收到的消息都在主线程处理，并没有使用多线程去处理。
>>>>>>> 673ebbfef8c19efa77594e23fd0699d518e0e5cd
4. 一些功能并不是ROS官方的功能包发送的话题，如robot_pose，car_state,control_cmd等，robot_pose是机器人位置话题，car_state是简易状态，control_cmd是发送特定data实现特定命令
5. scan_filtered是雷达话题，雷达数据是经过laser_filters功能包屏蔽了，直接修改为你雷达的数据的话题就可以了，不过它的使用位置依赖于机器人的位置，你要想办法获取机器人的位置，并进行tf坐标变换，我这里的是很简略且不规范的，勉强能用而已

## 其他

1. 项目参考借鉴了古月居的部分代码https://github.com/chengyangkj/Ros_Qt5_Gui_App
2. 如果您想讨论或者能完善部分功能，欢迎与我建立联系
