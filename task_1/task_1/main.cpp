#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "test.h"

constexpr double MY_PI = 3.1415926;

using namespace std;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
				0, 1, 0, -eye_pos[1], 
				0, 0, 1, -eye_pos[2], 
				0, 0, 0, 1;
    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	rotation_angle = rotation_angle * MY_PI / 180.f; 
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f rotate;
	rotate << cos(rotation_angle), -sin(rotation_angle), 0, 0, 
				sin(rotation_angle), cos(rotation_angle), 0, 0, 
				0, 0, 1, 0,
				0, 0, 0, 1;
	model = rotate * model;

    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float rotation_angle) {

	rotation_angle = rotation_angle * MY_PI / 180.f;
	Eigen::Matrix4f model, mat4_rotate;
	model = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f mat_N, mat3_rotate, mat_identity;
	mat_identity = Eigen::Matrix3f::Identity();

	float x, y, z;
	x = axis.x();
	y = axis.y();
	z = axis.z();
	mat_N << 0, -z, y,
			z, 0, -x,
			-y, x, 0;
	
	//可以发现Eigen的vector默认是列向量！！！
	mat3_rotate = cos(rotation_angle) * mat_identity + ( 1 - cos(rotation_angle)) * axis * axis.transpose() + sin(rotation_angle) * mat_N;
	//cout << "mark" << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			mat4_rotate(i, j) = mat3_rotate(i, j);
		}
		mat4_rotate(i, 3) = 0.f;
	}
	for (int j = 0; j < 3; j++)
		mat4_rotate(3, j) = 0.f;
	mat4_rotate(3, 3) = 1.f;
	model = mat4_rotate * model;

	return model;
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	zNear = -zNear;
	zFar = -zFar;
	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f persp_to_ortho, ortho_1, ortho_2, viewport;
	persp_to_ortho << zNear, 0, 0, 0, 
						0, zNear, 0, 0, 
						0, 0, zNear + zFar, -zNear*zFar, 
						0, 0, 1, 0;
	float h_half = tan(eye_fov * MY_PI / 360.f) * -zNear;
	float w_half = h_half * aspect_ratio;
	ortho_1 << 1, 0, 0, 0,
				0, 1, 0, 0, 
				0, 0, 1, -(zNear + zFar)/2.f,
				0, 0, 0, 1;
	// 以下是我当初上课时写的原代码，肯定是跑通了。但应该是写错了，应该第一行是w_half，第二行是h_half。所以虽然跑通了但显示比例可能有点问题。
	ortho_2 << 1 / h_half, 0, 0, 0,
				0, 1 / w_half, 0, 0,
				0, 0, 2/(zNear - zFar), 0,
				0, 0, 0, 1;
	projection = ortho_2 * ortho_1 * persp_to_ortho * projection;
	cout << projection << endl;
    return projection;
}

int main(int argc, const char** argv)
{
	//test();
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

	//视口变换的视口大小，由rasterizer负责确定。
    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

	//2个load函数功能是把三角形数据放到rasterizer对象的库里，同时返回库中的编号。
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
	/*cout << "hh:\n";
	cout << pos_id.pos_id << endl;
	cout << ind_id.ind_id << endl;*/

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//作业里上一行这块儿原来写的是0.1和50，让我以为n和f是正的  但不是！！！n和f是负值，是z坐标！！！！！！

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		Eigen::Vector3f n_axis = {0, 0, 1};
		r.set_model(get_rotation(n_axis, angle));
		//r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//作业里上一行这块儿原来写的是0.1和50，让我以为n和f是负的？不是！！！n和f是负值，是z坐标！！！！！！

		//值得注意 这个程序里的rasterizerh绘制三角形的方法是：变换阶段【只变换3个顶点】，最后的绘图阶段将连线画出来（画连线的时候还用了一些数学技巧）。
		//而不是在变换阶段就变换所有最后需要画的点，并在最后直接画出3条边，这样就不需要数学技巧了。
		//但这样其实不可能，因为变换前不可能知道变换后被画出的三条边上的点分别是哪些，也不可能处理三条边上所有点吧？因为有无数个点。
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		//frame_buffer是个vector，大小是700*700，frame_buffer用来给image喂图像数据。映射关系就是经典的从一行的最左到最右，一行结束再换下一行。
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);


		//waitkey负责2件事：显示图像（没错，光有cv::imshow还不够）&监听键盘输入（有输入就终止显示图像）
        key = cv::waitKey(10);
        std::cout << "frame count: " << frame_count++ << '\n';
        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
