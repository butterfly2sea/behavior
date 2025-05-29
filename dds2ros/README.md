# DDS2ROS

## 介绍
包括提供给地面端使用的SDK及机载端使用的DDS2ROS转发程序，用于机地及机间相互通信；地面站通过对应接口将信息通过DDS消息发送，
机载端将收到的信息转发为ROS消息而将收到的ROS消息转发为DDS消息；机载端为可独立运行的程序，地面站为包括调用接口的动态库。
|      | 分支名称          | 功能描述                           |
| ---- | ----------------- | ---------------------------------- |
| 1    | junYi             | 基础功能上添加机间命令消息及目标消息转发       |
| 2    | guokeda           | 基于科大自定义消息     |
| 3    | 54suo             | 基于54所自定义消息 |
## 依赖
###fastDDS
1.  机载板卡端安装ros2时已安装了fastDDS库不用单独安装
2.  windows端下载二进制安装包进行安装
从https://www.eprosima.com/product-download
下载window版本,如eProsima_Fast-DDS-2.7.1-Windows.exe
或从钉钉中下载https://alidocs.dingtalk.com/i/nodes/a9E05BDRVQNzklQPCEPxxxKeJ63zgkYA?utm_scene=team_space
###json库
参考 https://e.coding.net/g-zkva6329/jiqunfenbushi/common.git中说明
###custom_msgs库
为分布式机载端自定义ros2消息库，参考https://e.coding.net/g-zkva6329/jiqunfenbushi/custom_msgs.git
##编译
###机载端(linux)
-  创建~/build/DDS2ROS
-  cd ~/build/DDS2ROS
-  cmake ~/DDS2ROS/nx/DDS2ROS
-  make
###地面端(windows)
1.  打开cmake-gui工具
2.  选择DDS2ROS源文件目录(包括CMakeLists.txt)的目录、选择要生成工程信息的保存目录
3.  依次点击Configure、Generate按钮以生成工程
4.  去工程信息保存目录用vs2017或更高版本打开方案文件DDSCommu.sln
5.  右击commu_api或GrdControl选择生成，以进行编译
6.  成功后在工程生成目录的Release目录中包括给分布式地面站使用的commu_api.lib、commu_api.dll库文件，及测试使用的demo地面站GrdControl.exe
<div style="display: flex; justify-content: space-between;">
  <img src="images/win-configure.png" alt="选择源码及生成目录" width="45%"/>
  <img src="images/win-generate.png" alt="生成" width="45%"/>
</div>
打开工程
<p align="center">
     <img src="images/win-sln.png" width = "30%" height="300"/>
</p>

##使用
1.  机载端运行DDS2ROS程序
    -  cd ~/build/DDS2ROS
    -  ./DDS2ROS
2.  地面端启动GrdControl程序
运行工程生成目录Release文件夹中GrdControl.exe
3.  通信接口库提供给分布式地面站使用
    1.  拷贝头文件CommApi.h,Camera.h
    2.  拷贝库文件commu_api.lib,commu_api.dll
<div style="display: flex; justify-content: space-between;">
  <img src="images/dds2ros.png" alt="启动机载端" width="45%" height="300"/>
  <img src="images/grd.png" alt="启动demo地面站" width="45%" height="300"/>
</div>








