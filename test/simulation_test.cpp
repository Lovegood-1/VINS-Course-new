//
// Created by yy on 22-7-22.
//
//
// Created by yy on 22-7-22.
//
#include <unistd.h>//函数名: usleep()
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>//　iomanip的作用:　主要是对cin,cout之类的一些操纵运算子，比如setfill,setw,setbase,setprecision等等

 
#include <opencv2/opencv.hpp>
 
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/yzp/files/code_program/LearningVIO/vio_data_simulation/bin/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;
 ///读取imu数据文件中的IMU数据，并把数据打包成(时间dStampNSec，角速度vGyr，加速度vAcc)的形式调用系统函数PubImuData进行处理：
void PubImuData()///读IMU数据并调用系统接口去处理imu数据
{
    string sImu_data_file = sConfig_path +  "imu_pose_noise.txt";
    cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
    ifstream fsImu;
    fsImu.open(sImu_data_file.c_str());
    if (!fsImu.is_open())
    {
        cerr << "Failed to open imu file! " << sImu_data_file << endl;
        return;
    }
    double tmp;
    std::string sImu_line;
    double dStampNSec = 0.0;
    Vector3d vAcc;
    Vector3d vGyr;

///imu data :timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)
    while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
    {
        // timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)
        std::istringstream ssImuData(sImu_line);
        ssImuData >> dStampNSec;///timestamp
        for(int i=0;i<7;i++)
            ssImuData>>tmp;///imu quaternion(4)，imu position(3)
        ssImuData>>vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
        // 时间单位为 s
        pSystem->PubImuData(dStampNSec, vGyr, vAcc);
        usleep(5000*nDelayTimes);
    }

    fsImu.close();
}
 
///先从MH_05_cam0.txt文件读取数据，打包成（时间dStampNSec，图像sImgFileName）。
///再调用系统的PubImageData函数进行图像数据的处理。
void PubImageData()///读图像数据并调用系统接口去处理图像数据
{///cam data:timestamp (1)，cam quaternion(4)，cam position(3)，imu gyro(3)，imu acc(3)
    string sImage_file = sConfig_path + "cam_pose.txt";// 含时间戳的文件

    cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

    ifstream fsImage;
    fsImage.open(sImage_file.c_str());
    if (!fsImage.is_open())
    {
        cerr << "Failed to open image file! " << sImage_file << endl;
        return;
    }

    std::string sImage_line;
    double dStampNSec;
    string sImgFileName;

    int n=0;

    // cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
    //这个循环是遍历所有的相机
    while (std::getline(fsImage, sImage_line) && !sImage_line.empty())//sImage_line是cam_pose每行的数据流
    {
        std::istringstream ssImgData(sImage_line);//是cam_pose每行的内容
        ssImgData >> dStampNSec;   //读入时间戳
        cout<<"cam time: "<<fixed<<dStampNSec<<endl;
        // all_points_ 文件存储的是house模型的线特征，每行4个数，对应该线两端点在归一化平面的坐标
        //all_points_  文件每行的内容是 x, y, z, 1, u, v  这里的u v是归一化下的x ,y 不是像素坐标
        //在函数PubSimImageData中会算出具体特征点的像素坐标
        string all_points_file_name = "/home/yzp/files/code_program/LearningVIO/vio_data_simulation/bin/keyframe/all_points_" + to_string(n)+ ".txt";  //第n个相机对应的观测数据的文件名
        cout<<"points_file: "<<all_points_file_name<<endl;//0-599 all_point_0-599.txt
        vector<cv::Point2f> FeaturePoints;//容器FeaturePoints存放一个相机的特征点(归一化坐标)
        std::ifstream f;
        f.open(all_points_file_name);

        //这个循环是遍历每个相机的特征点信息
        // file content in each line: x, y, z, 1, u, v
        //经过这个循环把all_points_的特征点都放在FeaturePoints了
        while(!f.eof())
        {
            std::string s; 
            std::getline(f,s);//得到all_points_的文件流s
            // 一行两个点连成线，获取每行点判断一下是否之前获取过
            if(!s.empty())
            {
                std::stringstream ss;//
                ss << s;//ss得到每行的内容

                double tmp;//跳过  x y z 1
                for(int i=0;i<4;i++)
                    ss>>tmp;

                float px,py;
                ss >> px;
                ss >> py;
                cv::Point2f pt( px, py);//归一化坐标

                FeaturePoints.push_back(pt);
            }
        }

//        cout << "All points:" << endl;
//        for(auto point : FeaturePoints){
//            cout << point << " ";
//        }
//        cout << endl;


        pSystem->PubSimImageData(dStampNSec, FeaturePoints);//把每一个图片的特征点 放进VINS系统里

        usleep(50000*nDelayTimes);
        n++;
    }


    fsImage.close();
}

#ifdef __APPLE__
// support for MacOS
void DrawIMGandGLinMainThrd(){
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	pSystem->InitDrawGL();
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		//pSystem->PubImageData(dStampNSec / 1e9, img);
		cv::Mat show_img;
		cv::cvtColor(img, show_img,  cv:::COLOR_GRAY2RGB);
		if (SHOW_TRACK)
		{
			for (unsigned int j = 0; j < pSystem->trackerData[0].cur_pts.size(); j++)
			{
				double len = min(1.0, 1.0 *  pSystem->trackerData[0].track_cnt[j] / WINDOW_SIZE);
				cv::circle(show_img,  pSystem->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}

			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
		  // cv::waitKey(1);
		}

		pSystem->DrawGLFrame();
		usleep(50000*nDelayTimes);
	}
	fsImage.close();

}
#endif
int main(int argc, char **argv)
{
//    if(argc != 3)
//    {
//        cerr << "./simulation-test 特征点文件路径 配置文件/config \n"
//             << endl;
//        return -1;
//    }
//    sData_path = argv[1];
//    sConfig_path = argv[2];

    pSystem.reset(new System(sConfig_path));

    std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem); // 最重要！！

    // sleep(5);
    std::thread thd_PubImuData(PubImuData); // 获取IMU数据的线程

    std::thread thd_PubImageData(PubImageData); //获取图像数据的线程

    std::thread thd_Draw(&System::Draw, pSystem); // 画图的线程


    thd_PubImuData.join();
    thd_PubImageData.join();
    thd_BackEnd.join();

    thd_Draw.join();

    cout << "main end... see you ..." << endl;
    return 0;
}

