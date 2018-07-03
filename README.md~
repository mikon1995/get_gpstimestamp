# GPS时间戳提取
## 依赖库
* Ubantu 14.04
* PCL 1.3.0+
* boost 
* 详细请见 *pcap2pcd/PcapDump/makefile*
## 代码运行
cd get_timestamp/pcap2pcd/PcapDump  
make  
./PcapDump.exe -cal HDL-64.xml  
结果示例：  
![result1](https://github.com/mikon1995/get_gpstimestamp/raw/master/imgs/result1.png)
![result2](https://github.com/mikon1995/get_gpstimestamp/raw/master/imgs/result2.png)

## 基本概念
### 数据存储格式
* **pcap**: 一种通用的数据流格式,可从网络特定端口抓取数据存储。
* **pcd**: Point Cloud Data，一种存储点云数据的文件格式。  
  雷达扫描频率为10Hz, 故一帧为0.1s；此项目中默认按帧存储pcd文件，当雷达水平角度从0°扫至360°时为一帧。
* **datapacket**: 雷达原始udp数据包。  
  总长度为1248 = 42(头部)+1206（数据）+4（gpstimestamp）+1(status type)+1(status value)  
![datapacket](https://github.com/mikon1995/get_gpstimestamp/raw/master/imgs/datapacket.png)

### Velodyne HDL64 gpstimestamp
* **目标格式**： YYYY-MM-DD-HH-MI-SS-MS
* **时间戳构成**:  
1. **Status type & Status value**(1-byte & 1-byte): 每个数据包中只存储一个类型：Y/N/D/H/M/S 和其对应的值。  
  在此把Status type 所对应的值整合为**datetime**，表示为年-月-日-时-分-秒   
  |Y=year | N=month | D=day | H=hour | M=minute | S=second |  
![gpstimestamp_per_datapacket](https://github.com/mikon1995/get_gpstimestamp/raw/master/imgs/gpstimestamp_per_datapacket.png)
2. **gpstimestamp**(4-byte）：代表从初始小时开始记录的微秒数。  
示例图：
## 工作流程
*此文件中gps时间戳的提取是基于pcl::HDLGrabber实现的*
1.**定义数据包结构体，将原始udp数据包存储至HDLDataPacket。**
		struct HDLDataPacket
		{
		HDLFiringData firingData[HDL_FIRING_PER_PKT];
		unsigned int gpsTimestamp;
		unsigned char blank1; //Status type
		unsigned char blank2; //Status value
		};
2. **在存储每一帧点云数据时，添加一个参数返回时间戳。**  
* 一帧点云数据为0.1s, datetime精确度为s，故一帧中udp数据包中存储的datetime是相同的。
* 在 pcl::HDLGrabber中，判断水平角度从0°扫至360°时为一帧，在存储新的一帧即初始位置时，返回当前的gpstimestamp，
  故每次返回的gpstimestamp为0°的微秒数。
3. **解析 Status type，Status value 并整合为datatime。**  
./MyHdlGrabber/MyHdlGrabber.cpp  
![get_gpstimestamp](https://github.com/mikon1995/get_gpstimestamp/raw/master/imgs/get_gpstimestamp.png)
4. **解析 gpstimestamp 并转化为毫秒数。**
5. **以gps时间戳（3 + 4）同步命名并存储pcd文件。**  
./PcapDump/PcapDump.cpp  
![return_gpstimestamp](https://github.com/mikon1995/get_gpstimestamp/raw/master/imgs/return_gpstimestamp.png)
 
