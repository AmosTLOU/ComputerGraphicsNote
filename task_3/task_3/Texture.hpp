//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

using namespace std;

class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }
	//��ҵ��ԭ����rasterizer����ʹ����std::optionalģ���ࣨC++17����ģ�����Texture������vs2015��֧��c++17�������������µ�default���캯���������������ϵ�������ˣ�
	Texture()
	{	}

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

		

		//��ҵ��ԭ��û������4�У���������Щimage_data������ʱ���Խ�硣��˵��Object�ļ����uv���������⣿
		u_img = u_img < width-1 ? u_img : width-1;
		u_img = u_img > 0 ? u_img : 0;
		v_img = v_img < height-1 ? v_img : height-1;
		v_img = v_img > 0 ? v_img : 0;
		
		//if (u_img > 512 && v_img < 512) {
		//	cout << "color" << endl;
		//	cout << v_img << "    " << u_img << endl;
		//	/*cout << Eigen::Vector3f(color[0], color[1], color[2]) << endl;*/
		//}

		auto color = image_data.at<cv::Vec3b>((int)v_img, (int)u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
