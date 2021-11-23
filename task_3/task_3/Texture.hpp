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
	//作业中原本在rasterizer类里使用了std::optional模板类（C++17），模板就是Texture。由于vs2015不支持c++17，因此添加了如下的default构造函数。（具体因果关系不解释了）
	Texture()
	{	}

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

		

		//作业里原本没有以下4行，但不加这些image_data索引的时候会越界。这说明Object文件里的uv坐标有问题？
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
