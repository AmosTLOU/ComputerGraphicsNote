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
	
	//���Է���Eigen��vectorĬ����������������
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
	// �������ҵ����Ͽ�ʱд��ԭ���룬�϶�����ͨ�ˡ���Ӧ����д���ˣ�Ӧ�õ�һ����w_half���ڶ�����h_half��������Ȼ��ͨ�˵���ʾ���������е����⡣
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

	//�ӿڱ任���ӿڴ�С����rasterizer����ȷ����
    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

	//2��load���������ǰ����������ݷŵ�rasterizer����Ŀ��ͬʱ���ؿ��еı�š�
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
		//��ҵ����һ������ԭ��д����0.1��50��������Ϊn��f������  �����ǣ�����n��f�Ǹ�ֵ����z���꣡����������

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
		//��ҵ����һ������ԭ��д����0.1��50��������Ϊn��f�Ǹ��ģ����ǣ�����n��f�Ǹ�ֵ����z���꣡����������

		//ֵ��ע�� ����������rasterizerh���������εķ����ǣ��任�׶Ρ�ֻ�任3�����㡿�����Ļ�ͼ�׶ν����߻������������ߵ�ʱ������һЩ��ѧ���ɣ���
		//�������ڱ任�׶ξͱ任���������Ҫ���ĵ㣬�������ֱ�ӻ���3���ߣ������Ͳ���Ҫ��ѧ�����ˡ�
		//��������ʵ�����ܣ���Ϊ�任ǰ������֪���任�󱻻������������ϵĵ�ֱ�����Щ��Ҳ�����ܴ��������������е�ɣ���Ϊ���������㡣
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		//frame_buffer�Ǹ�vector����С��700*700��frame_buffer������imageιͼ�����ݡ�ӳ���ϵ���Ǿ���Ĵ�һ�е��������ң�һ�н����ٻ���һ�С�
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);


		//waitkey����2���£���ʾͼ��û������cv::imshow��������&�����������루���������ֹ��ʾͼ��
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
