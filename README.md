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

1. 工程的保存建图和加载地图功能其实是没有完成的，因为能力不够，且资料少，没研究出来怎么使用ROSBridge或者其他方法去实现它。有另一种实现方式参考该readme下的“关于如何保存地图和启动地图服务的说明”，但考虑到不是ROSBridge的方式我就不再完善了。
2. 该工程是个人研究ROS和Qt的学习之作，存在有很多不足，代码结构较为混乱。
3. 程序的优化较差。
4. 一些功能并不是ROS官方的功能包发送的话题，如robot_pose，car_state,control_cmd等，robot_pose是机器人位置话题，car_state是简易状态，control_cmd是发送特定data实现特定命令
5. scan_filtered是雷达话题，雷达数据是经过laser_filters功能包屏蔽了，直接修改为你雷达的数据的话题就可以了，不过它的使用位置依赖于机器人的位置，你要想办法获取机器人的位置，并进行tf坐标变换，我这里的是很简略且不规范的，勉强能用而已

## 关于如何保存地图和启动地图服务的说明
1. 可以考虑使用ssh的方式连接到运行ROS的设备，使用命令启动地图服务和加载地图，在Qt中有QSsh库可以用（需要自己下载源码编译）
2. 使用此方式需要借用map_server的功能包，参考命令如下，在连接进ssh后运行命令rosrun map_server map_server /home/nvidia/map/xxx.yaml，可以启动地图服务并加载/home/nvidia/map路径下名为xxx的地图
3. 运行rosrun map_server map_saver -f /home/nvidia/map/xxx，可以启动保存地图服务并将地图保存到/home/nvidia/map路径下，命名为xxx，生成相应的xxx.yaml和xxx.pgm

## 其他

1. 项目参考借鉴了古月居的部分代码，https://github.com/chengyangkj/Ros_Qt5_Gui_App
2. ROSBridge V2协议，https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md
3. 综合本程序总结，使用ROSBridge开发一款ROS控制软件，本质上是使用websocket方式，通过json数据订阅和发送消息，并将订阅的东西展示出来，当然，运行ROS的设备需要有ROSBridge功能包并启动了ROSBridge服务。除了websocket外，还有tcp，udp方式，但我都没试过，想来应该是类似的
4. 如果您想讨论或者能完善部分功能，欢迎与我建立联系

## 更新-20230312
1. 新增了路径显示，优化了View，使其可以旋转。
2. 新增设置页可订阅和取消订阅部分话题。
