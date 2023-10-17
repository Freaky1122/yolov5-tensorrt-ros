//std include
#include <memory>
#include <thread>
#include <random>
#include <ctime>
#include <cmath>
#include <string>
#include <vector>
#include <sys/time.h>
//yolo include
#include "class_timer.hpp"
#include "class_detector.h"
#include "yolov5/Target.h"
//ros include
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
//realsense include
#include <librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
//opencv include
#include<opencv2/highgui/highgui_c.h>

//lcm include
#include "target_t.hpp"
#include "lcm/lcm-cpp.hpp"
//my include
#include "filter.cpp"
#include "log.cpp"


//定义D435i摄像头相关的参数
#define WIDTH 640
#define HEIGHT 480
#define FPS 30

//function declare
float get_depth_scale(rs2::device dev);
float measure_distance(const cv::Mat& depth,const rs2::frame& depth_frame,std::vector<cv::Point> &points,float *Pcc3,rs2::pipeline_profile profile);

int main(int argc,char *argv[])
{

    //创建数据管道
    rs2::pipeline pipe;
    rs2::config pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH,WIDTH,HEIGHT,RS2_FORMAT_Z16,30);
    pipe_config.enable_stream(RS2_STREAM_COLOR,WIDTH,HEIGHT,RS2_FORMAT_BGR8,30);
	//start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);
    
    //lcm init
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    target::target_t lcm_msg;
    //定义一个变量去转换深度到距离
    float depth_clipping_distance = 1.f;
 
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    
    //create log 
    DataLog log("./.log");
    std::string log_str = "time_all,target_x,target_y,correct_x,correct_y\n";
    log.addLog(log_str);
	//ros node init
	ros::init(argc,argv,"yolov5_d435i");
	ros::NodeHandle n;
	//publisher
	ros::Publisher pub = n.advertise<yolov5::Target>("detector",1000);


	//添加配置文件和权重文件
	Config config_v5;
	config_v5.net_type = YOLOV5;
	config_v5.detect_thresh = 0.5;
	config_v5.file_model_cfg = "/home/nvidia/dev/IARC_ws/src/yolov5-tensorrt-ros/configs/yolov5-5.0/yolov5s.cfg";
	config_v5.file_model_weights = "/home/nvidia/dev/IARC_ws/src/yolov5-tensorrt-ros/configs/yolov5-5.0/best.weights";
	config_v5.inference_precison = FP16;

	std::unique_ptr<Detector> detector(new Detector());
	detector->init(config_v5);

	std::vector<BatchResult> batch_res;
	std::vector<cv::Mat> batch_img;
	yolov5::Target target;
	char key;
	double fps;
	//double t;
    //每次循环时间，用于Singer模型，为简化根据FPS设置成固定时间
    static float time_each = 0.033;
    static float time_all = 0;
    const double alpha = 1.0/30; 
    //随机数种子
    srand((unsigned)time(nullptr));

    //timer
    timeval t1,t2;
	//加入卡尔曼滤波
	cv::KalmanFilter KFx(3, 1, 0);
    cv::KalmanFilter KFy(3, 1, 0);

    //Initialize transitionMatrix
    float A[] = {1,time_each,(alpha*time_each-1+std::exp(-alpha*time_each))/std::pow(alpha,2),0,1,(1-std::exp(-alpha*time_each))/alpha,0,0,std::exp(-alpha*time_each)};
    KFx.transitionMatrix = cv::Mat(3, 3, CV_32F, A).clone();
    KFy.transitionMatrix = cv::Mat(3, 3, CV_32F, A).clone();

    //Singer模型
    float sigma = 10.0;
    float q11 = 2*alpha*sigma*(1-std::exp(-2*alpha*time_each)+2*alpha*time_each+2*std::pow(time_each,3)*std::pow(alpha,3)/3-2*std::pow(time_each,2)*std::pow(alpha,2)-4*alpha*time_each*std::exp(-alpha*time_each))/(2*std::pow(alpha,5));
    float q12 = 2*alpha*sigma*(std::exp(-2*alpha*time_each)+1-2*std::exp(-alpha*time_each)-2*alpha*time_each+std::pow(time_each,2)*std::pow(alpha,2)+2*alpha*time_each*std::exp(-alpha*time_each))/(2*std::pow(alpha,4));
    float q13 = 2*alpha*sigma*(1-std::exp(-2*alpha*time_each)-2*alpha*time_each*std::exp(-alpha*time_each))/(2*std::pow(alpha,3));
    float q22 = 2*alpha*sigma*(4*std::exp(-alpha*time_each)-3-std::exp(-2*alpha*time_each)+2*alpha*time_each)/(2*std::pow(alpha,3));
    float q23 = 2*alpha*sigma*(std::exp(-2*alpha*time_each)+1-2*alpha*time_each)/(2*std::pow(alpha,2));
    float q33 = 2*alpha*sigma*(1-std::exp(-2*alpha*time_each))/(2*alpha);

    
    // Initialize other Kalman filter parameters.
    float Q[] = {q11,q12,q13,q12,q22,q23,q13,q23,q33};
    KFx.processNoiseCov = cv::Mat(3, 3, CV_32F, Q).clone();
    KFy.processNoiseCov = cv::Mat(3, 3, CV_32F, Q).clone();
    cv::setIdentity(KFx.measurementMatrix);
    cv::setIdentity(KFx.measurementNoiseCov, cv::Scalar(1e-5));
    cv::setIdentity(KFx.errorCovPost, cv::Scalar(1));

    cv::setIdentity(KFy.measurementMatrix);
    cv::setIdentity(KFy.measurementNoiseCov, cv::Scalar(1e-5));
    cv::setIdentity(KFy.errorCovPost, cv::Scalar(1));
    // choose random initial state
    cv::Mat measurement_x = cv::Mat::zeros(1, 1, CV_32F);
    cv::Mat measurement_y = cv::Mat::zeros(1, 1, CV_32F);
    //中值滤波核的大小
    const int Size = 5;
    float depth_array[Size]={0};

	while(ros::ok())
	{
        //开始计时
        gettimeofday(&t1, nullptr);
		//prepare batch data
		rs2::frameset frames = pipe.wait_for_frames();//等待所有配置的流生成框架
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        cv::Mat frame(cv::Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat frame_depth(cv::Size(WIDTH,HEIGHT), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
		
		cv::Mat temp0 = frame.clone();
		batch_img.push_back(temp0);
		//detect
		detector->detect(batch_img, batch_res);
        
		
		//disp
		if(!batch_res[0].empty())
		{
			for (const auto &r : batch_res[0])
			{
				//std::cout <<"batch "<<i<< " id:" << r.id << " prob:" << r.prob << " rect:" << r.rect << std::endl;
				//rosmsg发送
				target.id = r.id;
				target.prob = r.prob;
				target.x = (int32_t)(r.rect.x + r.rect.width/2);
				target.y = (int32_t)(r.rect.y + r.rect.height/2);
                //lcm发送
                lcm_msg.target[0] = (int16_t)target.x;
                lcm_msg.target[1] = (int16_t)target.y;
                lcm.publish("YOLO",&lcm_msg);
				pub.publish(target);
				ROS_INFO("%d\n",r.id);
                
                //最终用于读取深度的区域
                cv::Rect roi_rect(target.x-r.rect.width/8, target.y-r.rect.height/8, r.rect.width/4,r.rect.height/4);
                cv::Point detect_point(target.x,target.y);
                //select ten random point in roi rect
                std::vector<cv::Point> points;
                points.push_back(cv::Point(target.x,target.y));
                for(int i=0; i<9; i++)
                {
                    cv::Point point(target.x+rand()%(r.rect.width/4)-r.rect.width/8,target.y+rand()%(r.rect.height/4)-r.rect.height/8);
                    points.push_back(point);
                }
                // for(std::vector<cv::Point>::iterator it = points.begin();it!=points.end();it++)
                //     cv::circle(batch_img[0],cv::Point(it->x,it->y),2,cv::Scalar(255,0,0),2);

                float pcc3[3];
                //use depth image aligned
                float depth=measure_distance(frame_depth,depth_frame,points,pcc3,profile);

                ROS_INFO("coordinate: x  %f, y  %f, z  %f",pcc3[0],pcc3[1],pcc3[2]);
                //median filter
                for(int i=Size-1;i>0;i--)
                {
                    depth_array[i] = depth_array[i-1];
                }
                depth_array[0] = depth;

                float depth_filtered=medianFilter(depth_array, Size);
                ROS_INFO("depth_filter: %f",depth_filtered);
                //先验估计值
                cv::Mat prediction_x = KFx.predict();
                cv::Mat prediction_y = KFy.predict();
		        cv::Point predict_point = cv::Point(prediction_x.at<float>(0),prediction_y.at<float>(0));
                //更新测量值
                measurement_x.at<float>(0) = (float)target.x;
		        measurement_y.at<float>(0) = (float)target.y;
                //后验估计值
		        KFx.correct(measurement_x);
                KFy.correct(measurement_y);
                cv::Point correct_point(KFx.statePost.at<float>(0),KFy.statePost.at<float>(0));

                //写入日志文件
                log.addLog(to_string(time_all)+","+to_string(target.x)+","+to_string(target.y)+","+to_string(correct_point.x)+","+to_string(correct_point.y)+"\n");
    
				cv::rectangle(batch_img[0], r.rect, cv::Scalar(255, 0, 0), 2);
                cv::rectangle(batch_img[0],roi_rect , cv::Scalar(0, 255, 0), 1);
                cv::circle(batch_img[0],correct_point,6,cv::Scalar(0,0,255),2);
                cv::circle(batch_img[0],detect_point,6,cv::Scalar(255,0,0),2);
	
				std::stringstream stream;
				stream << std::fixed << std::setprecision(2) << "id:" << r.id << "  score:" << r.prob<<"  depth: "<<depth_filtered;
				cv::putText(batch_img[0], stream.str(), cv::Point(r.rect.x, r.rect.y - 5), 0, 0.5, cv::Scalar(0, 0, 255), 2);	

			}
		}
		else
		{
			target.id = -1;
			pub.publish(target);
			ROS_INFO("-1");
		}
        //结束计时
        gettimeofday(&t2, nullptr);
        //计算此次循环时间
        time_each = ((t2.tv_sec-t1.tv_sec) * 1000000 + t2.tv_usec-t1.tv_usec)*0.001;
        //计算程序运行时间
        time_all += time_each;
        fps = 1000.0/time_each;
		cv::putText(batch_img[0],"FPS: "+ std::to_string(fps),cv::Point(0,400), 0, 1, cv::Scalar(0, 255, 255), 2);
		cv::namedWindow("video", cv::WINDOW_NORMAL);
		cv::imshow("video", batch_img[0]);
		batch_img.pop_back();
		
		key = cv::waitKey(1);
        if(key == 27) 
            break;

		ros::spinOnce();

	}

}




//深度图对齐到彩色图函数
float measure_distance(const cv::Mat& depth,const rs2::frame& depth_frame,std::vector<cv::Point> &points,float *Pcc3,rs2::pipeline_profile profile){
    
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();                  //profile 是声明的数据流　　　RS2_STREAM_DEPTH是深度数据流
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();               //RS2_STREAM_COLOR颜色数据流
 
    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    auto extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    auto extrinColor2Depth=color_stream.get_extrinsics_to(depth_stream);
 
    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3];
 
    //获取深度像素与现实单位比例（D435默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    int y=0,x=0;
    int effective_pixel = 0;
    float distance_sum = 0.0,distance=0.0;
 
    //对深度图像遍历
    for(std::vector<cv::Point>::iterator it = points.begin();it!=points.end();it++)
    {
        //将当前的(x,y)放入数组pc_uv，表示当前RGB图的点
        pc_uv[0]=it->x;
        pc_uv[1]=it->y;
        rs2_project_color_pixel_to_depth_pixel(pd_uv,reinterpret_cast<const uint16_t*>(depth_frame.get_data()),depth_scale,0.2,10.0,&intrinDepth, &intrinColor,
            &extrinColor2Depth , &extrinDepth2Color , pc_uv);
        //取当前点对应的深度值
        uint16_t depth_value=depth.at<uint16_t>(pd_uv[1],pd_uv[0]);

        if(it == points.begin())
        {
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_value);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
        }
        if(depth_value!=0)
        {
            //换算到米
            float depth_m=depth_value*depth_scale;
            distance_sum += depth_m;
            effective_pixel++;
        }
    }

    distance = distance_sum/effective_pixel;
    return distance;
}


//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors　　检查设备的传感器
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor　　检查设备是否是一个深度传感器
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();           //函数的作用是检索深度图像和米之间的映射，返回的是深度，单位是米
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");                   //throw是抛出异常
}
