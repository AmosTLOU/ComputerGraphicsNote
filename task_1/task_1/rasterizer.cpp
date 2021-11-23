//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

using namespace std;


// �о�����д���е����⣬�����������ͬһ�������� �����������main�б����ã�id��0���¸�������load_indices���ٱ����ã�id�ͱ��1��
rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);
	rst::pos_buf_id tem;
	tem.pos_id = id;
	return tem;
	/*auto id = get_next_id();
	pos_buf.emplace(id, positions);
	return {id};*/
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);
	rst::ind_buf_id tem;
	tem.ind_id = id;
	return tem;
	/*auto id = get_next_id();
	ind_buf.emplace(id, indices);
	return {id};*/
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
// CSDN���� https://blog.csdn.net/qq_36711003/article/details/83277698
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
	// ���Կ��������µĻ����߲���ֻ����x��y���꣨��begin��end������ά���꣬xyz���߶��У����ע��һ�£�
	auto x1 = begin.x();
	auto y1 = begin.y();
	auto x2 = end.x();
	auto y2 = end.y();
	
	Eigen::Vector3f line_color = { 255, 0, 0 };
	Eigen::Vector3f vertex_color = { 255, 255, 255 };

	// x��y������������Χ[0, width-1], [0, height-1]������ָ�����ĸ�����
	int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

	dx = x2 - x1;
	dy = y2 - y1;
	dx1 = fabs(dx);
	dy1 = fabs(dy);
	px = 2 * dy1 - dx1;
	py = 2 * dx1 - dy1;

	//dx1����x����λ�Ʒ��򣬷���y����λ�Ʒ���
	if (dy1 <= dx1)
	{
		//ȷ������ĵ�Ϊ��ʼ�㡣��ȷ���Ժ�֮��ֱ�ߵ�����Ϳ��Է�����2��������������
		if (dx >= 0)
		{
			x = x1;
			y = y1;
			xe = x2;
		}
		else
		{
			x = x2;
			y = y2;
			xe = x1;
		}
		Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.f);
		set_pixel(point, line_color);
		for (i = 0; x<xe; i++)
		{
			x = x + 1;
			if (px<0)
			{
				px = px + 2 * dy1;
			}
			else
			{
				//�ж�ֱ���������ϻ�����
				if ((dx<0 && dy<0) || (dx>0 && dy>0))
				{
					y = y + 1;
				}
				else
				{
					y = y - 1;
				}
				px = px + 2 * (dy1 - dx1);
			}
			//            delay(0);
			Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.f);
			set_pixel(point, line_color);
		}
	}
	else
	{
		//ȷ�����µĵ�Ϊ��ʼ�㡣��ȷ���Ժ�֮��ֱ�ߵ�����Ϳ��Է�����2��������������
		if (dy >= 0)
		{
			x = x1;
			y = y1;
			ye = y2;
		}
		else
		{
			x = x2;
			y = y2;
			ye = y1;
		}
		Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.f);
		set_pixel(point, line_color);
		for (i = 0; y<ye; i++)
		{
			y = y + 1;
			if (py <= 0)
			{
				py = py + 2 * dx1;
			}
			else
			{
				//�ж�ֱ������������������
				if ((dx<0 && dy<0) || (dx>0 && dy>0))
				{
					x = x + 1;
				}
				else
				{
					x = x - 1;
				}
				py = py + 2 * (dx1 - dy1);
			}
			//            delay(0);
			Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.f);
			set_pixel(point, line_color);
		}
	}
	
	Eigen::Vector3f point1 = Eigen::Vector3f((int)x1, (int)y1, 1.f);
	set_pixel(point1, vertex_color);
	Eigen::Vector3f point2 = Eigen::Vector3f((int)x2, (int)y2, 1.f);
	set_pixel(point2, vertex_color);
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//ÿ�ε����������draw����������һ���������Σ������ǰѿռ������������ζ����ˡ�
void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
	if (type != rst::Primitive::Triangle)
	{
		throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
	}
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];

	float f1 = (100 - 0.1) / 2.0;
	float f2 = (100 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
		Triangle t;

		Eigen::Vector4f v[] = {
			//���任ʱ��Ҫ����������ά������Ϊ��ά
			//���Է���Eigen��vectorĬ����������������
			mvp * to_vec4(buf[i[0]], 1.0f),
			mvp * to_vec4(buf[i[1]], 1.0f),
			mvp * to_vec4(buf[i[2]], 1.0f)
		};
		
		// �ⲽҲ����Ҫ��ͶӰ�任���4ά�ܿ��ܲ�����1��Ҫ��������ԭ��������任����ı��4ά��
		for (auto& vec : v) {
			vec /= vec.w();
		}

		//�ӿڱ任
		for (auto & vert : v)
		{
			vert.x() = 0.5*width*(vert.x() + 1.0);
			vert.y() = 0.5*height*(vert.y() + 1.0);
			//���Ͻ����ӿڱ任û˵Ҫ�任z���ꡣ����ı任�õ������z�����ϵļ�����ˣ����ҵ��z�������Ϊ����ֵ��
			//������һ��������Ĳ��������ó������û���õ���
			vert.z() = vert.z() * f1 + f2;
		}
		
		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i].head<3>());	//���任ʱ��Ҫ����������ά������Ϊ��ά������任�ñ����ˣ��Ϳ��Ա����ά�ˡ�
			t.setVertex(i, v[i].head<3>()); //head˵���� https://blog.csdn.net/liufengl138/article/details/78405652
			t.setVertex(i, v[i].head<3>());
			// ��˵����ΪɶҪд3�顣����Ӧ��1��͹��˰�
		}
		
		//��Ҫע�⣬����������ζ������ɫ���Ը�ֵ��ʵ�������Ƴ�����ͼ��û���κ�Ӱ�졣
		//��Ϊ����ͼ����OpenCV��������OpenCV��ͼ�����ݻ�������frame_buf������frame_buf��ֵ��Ψһ������ʹ��set_pixel���������ú���ֻ��draw_line�����ﱻ�����ˡ���draw����rasterize_wireframe����draw_line��
		t.setColor(0, 255.0, 0.0, 0.0);
		t.setColor(1, 0.0, 255.0, 0.0);
		t.setColor(2, 0.0, 0.0, 255.0);

		rasterize_wireframe(t);
	}
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
	draw_line(t.c(), t.a());
	draw_line(t.c(), t.b());
	draw_line(t.b(), t.a());
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
}

// x��y�Ƕ�ά���꣨ԭ�������½ǵĵѿ�������ϵ�� ��ת��Ϊ1ά�����������ո�����Ƶ�frame_buff��depth_buff�ã�
int rst::rasterizer::get_index(int x, int y)
{
	return (height -1 - y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	if (point.x() < 0 || point.x() >= width ||
		point.y() < 0 || point.y() >= height) return;
	// ������2ά�ĵѿ������꣨��������İ汾��ԭ�������½ǣ� �� 1ά������ӳ��
	auto ind = (height - 1 - point.y())*width + point.x();
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//��ҵ����һ������д����(get_index������Ҳд����)��ԭ��д���� height - point.y()�������Ļ�y=0ʱind��Խ�磬����700*700��Ӧ����height -1 - point.y()
	frame_buf[ind] = color;
}

