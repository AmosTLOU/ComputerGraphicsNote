//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>

using namespace std;


// 感觉这里写的有点问题，明明代表的是同一个三角形 这个函数先在main中被调用，id是0，下个函数（load_indices）再被调用，id就变成1了
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
// CSDN讲解 https://blog.csdn.net/qq_36711003/article/details/83277698
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
	// 可以看到，以下的画连线操作只用了x和y坐标（但begin和end都是三维坐标，xyz三者都有，这点注意一下）
	auto x1 = begin.x();
	auto y1 = begin.y();
	auto x2 = end.x();
	auto y2 = end.y();
	
	Eigen::Vector3f line_color = { 255, 0, 0 };
	Eigen::Vector3f vertex_color = { 255, 255, 255 };

	// x、y都是整数，范围[0, width-1], [0, height-1]，负责指定是哪个像素
	int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

	dx = x2 - x1;
	dy = y2 - y1;
	dx1 = fabs(dx);
	dy1 = fabs(dy);
	px = 2 * dy1 - dx1;
	py = 2 * dx1 - dy1;

	//dx1大，则x是主位移方向，否则y是主位移方向。
	if (dy1 <= dx1)
	{
		//确定靠左的点为起始点。（确定以后，之后直线的走向就可以分上下2种走向来分析）
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
				//判断直线走向是上还是下
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
		//确定靠下的点为起始点。（确定以后，之后直线的走向就可以分左右2种走向来分析）
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
				//判断直线走向是向左还是向右
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

//每次调用下面这个draw函数，画【一个】三角形，而不是把空间内所有三角形都画了。
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
			//做变换时需要将本来的三维点扩充为四维
			//可以发现Eigen的vector默认是列向量！！！
			mvp * to_vec4(buf[i[0]], 1.0f),
			mvp * to_vec4(buf[i[1]], 1.0f),
			mvp * to_vec4(buf[i[2]], 1.0f)
		};
		
		// 这步也很重要！投影变换后第4维很可能不再是1，要将向量还原。（仿射变换不会改变第4维）
		for (auto& vec : v) {
			vec /= vec.w();
		}

		//视口变换
		for (auto & vert : v)
		{
			vert.x() = 0.5*width*(vert.x() + 1.0);
			vert.y() = 0.5*height*(vert.y() + 1.0);
			//课上讲的视口变换没说要变换z坐标。下面的变换让点与点在z方向上的间距变宽了，并且点的z坐标均变为了正值。
			//可能是一项有意义的操作，但该程序后续没有用到。
			vert.z() = vert.z() * f1 + f2;
		}
		
		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i].head<3>());	//做变换时需要将本来的三维点扩充为四维，做完变换该保存了，就可以变回三维了。
			t.setVertex(i, v[i].head<3>()); //head说明见 https://blog.csdn.net/liufengl138/article/details/78405652
			t.setVertex(i, v[i].head<3>());
			// 话说这里为啥要写3遍。。。应该1遍就够了吧
		}
		
		//需要注意，这里给三角形对象的颜色属性赋值其实对最后绘制出来的图像没有任何影响。
		//因为最后绘图的是OpenCV，而传入OpenCV的图像数据缓冲区是frame_buf，而给frame_buf赋值的唯一方法是使用set_pixel函数，而该函数只在draw_line函数里被调用了。（draw调用rasterize_wireframe调用draw_line）
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

// x和y是二维坐标（原点在左下角的笛卡尔坐标系） 被转换为1维索引（给最终负责绘制的frame_buff和depth_buff用）
int rst::rasterizer::get_index(int x, int y)
{
	return (height -1 - y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	if (point.x() < 0 || point.x() >= width ||
		point.y() < 0 || point.y() >= height) return;
	// 以下是2维的笛卡尔坐标（就是最常见的版本，原点在左下角） 到 1维索引的映射
	auto ind = (height - 1 - point.y())*width + point.x();
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//作业里上一行这块儿写错了(get_index函数里也写错了)，原来写的是 height - point.y()，这样的话y=0时ind会越界，超过700*700。应该是height -1 - point.y()
	frame_buf[ind] = color;
}

