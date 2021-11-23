// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <array>
#include <tuple>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);
	rst::pos_buf_id tem;
	tem.pos_id = id;

    return tem;
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);
	rst::ind_buf_id tem;
	tem.ind_id = id;

    return tem;
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);
	rst::col_buf_id tem;
	tem.col_id = id;

    return tem;
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
	Eigen::Vector2f line[3];
	line[0] << _v[1].x() - _v[0].x(), _v[1].y() - _v[0].y();
	line[1] << _v[2].x() - _v[1].x(), _v[2].y() - _v[1].y();
	line[2] << _v[0].x() - _v[2].x(), _v[0].y() - _v[2].y();

	bool flag_pos = 0, flag_inside = 1;
	for (int i = 0; i < 3; i++) {
		Eigen::Matrix3f mat;
		mat << 0, 0, line[i].y(),
			0, 0, -line[i].x(),
			-line[i].y(), line[i].x(), 0;
		Eigen::Vector3f v_result = mat * Eigen::Vector3f(x - _v[i].x(), y - _v[i].y(), 0.f);
		if (0 == i) {
			if (v_result.z() >= 0)
				flag_pos = 1;
		}
		else {
			if (v_result.z() >= 0 && 1 != flag_pos) {
				flag_inside = 0;
				break;
			}
			else if (v_result.z() < 0 && 0 != flag_pos) {
				flag_inside = 0;
				break;
			}
		}
	}
	return flag_inside;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	//作业给的原始程序在第一行这里报错了，解决方案：在开头添加 #include <array>
	//我发现原因好像是：如果在一个文件里使用auto的话，那么auto变量要接收的那个类型必须得在这个文件里有定义！
	//在这里v接收的数据类型是三角形类的toVector4方法所返回的array类型，array类型虽然在三角形类的cpp里定义了（通过#include <array>），但在这个文件里<array>本来没有被include！
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
	float max_x = -DBL_MAX, max_y = -DBL_MAX;
	float min_x = DBL_MAX, min_y = DBL_MAX;
	for (int i = 0; i < 3; i++) {
		if (v[i].x() > max_x)
			max_x = v[i].x();
		if (v[i].y() > max_y)
			max_y = v[i].y();
		if (v[i].x() < min_x)
			min_x = v[i].x();
		if (v[i].y() < min_y)
			min_y = v[i].y();
	}
	Eigen::Vector3f _v[3];
	for (int i = 0; i < 3; i++) {
		_v[i] << v[i].x(), v[i].y(), v[i].z();
	}

	bool flag_MSAA = 0;
	for (int x = (int)(min_x); x <= (int)(max_x); x++) {
		for (int y = (int)(min_y); y <= (int)(max_y); y++) {
			if (flag_MSAA) {
				float x_[4], y_[4];
				x_[0] = x + 1 / 3.f;
				x_[1] = x + 2 / 3.f;
				x_[2] = x + 1 / 3.f;
				x_[3] = x + 2 / 3.f;
				y_[0] = y + 1 / 3.f;
				y_[1] = y + 1 / 3.f;
				y_[2] = y + 2 / 3.f;
				y_[3] = y + 2 / 3.f;
				int num_inside = 0;
				for (int n = 0; n <= 3; n++) {
					if (!insideTriangle(x_[n], y_[n], _v))
						continue;
					std::tuple<float, float, float> tuple_barycentric;
					float alpha, beta, gamma;
					//从下一行到z_interpolated *= w_reciprocal这一行，是在通过三角形三个顶点的xyz坐标和某个点的xy坐标，来推断出该点的z坐标的绝对值。
					tuple_barycentric = computeBarycentric2D(x_[n], y_[n], t.v);
					//获取tuple元组内数据的方式
					alpha = std::get<0>(tuple_barycentric);
					beta = std::get<1>(tuple_barycentric);
					gamma = std::get<2>(tuple_barycentric);
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;
					z_interpolated = -z_interpolated;//作业里没有这一行，没有这行的话z_interpolated不会是正的！（但作业里又说“已将z_interpolated处理为正数”...很无语）
					//此时被处理完毕的z_interpolated都是【正数】，越大代表离相机越远
					int ind = get_index(x, y);
					if (z_interpolated < depth_buf_MSAA[ind][n]) {
						num_inside++;
						depth_buf_MSAA[ind][n] = z_interpolated;
					}	
				}
				if(num_inside)
					set_pixel(Eigen::Vector3f(x, y, 0.f), t.getColor() * num_inside / 4.);
			}
			else {
				if (insideTriangle(x + 0.5f, y + 0.5f, _v)) { //判断某像素是否在三角形内部时，要用其中心的坐标（x+0.5, y+0.5）来判断！！！！！！ 但是set_pixel的时候还是要用(x,y),因为像素索引肯定还是整数。
					std::tuple<float, float, float> tuple_barycentric;
					float alpha, beta, gamma;
					tuple_barycentric = computeBarycentric2D(x + 0.5f, y + 0.5f, t.v);
					//获取tuple元组内数据的方式
					alpha = std::get<0>(tuple_barycentric);
					beta = std::get<1>(tuple_barycentric);
					gamma = std::get<2>(tuple_barycentric);
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;
					z_interpolated = -z_interpolated;//作业里没有这一行，没有这行的话z_interpolated不会是正的！（但作业里又说“已将z_interpolated处理为正数”...很无语）
					//此时被处理完毕的z_interpolated都是【正数】，越大代表离相机越远
					int ind = get_index(x, y);
					if (z_interpolated < depth_buf[ind]) {
						depth_buf[ind] = z_interpolated;
						set_pixel(Eigen::Vector3f(x, y, 0.f), t.getColor());
					}
				}
			}
		}	
	}
   
    
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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
	
	std::fill(depth_buf_MSAA.begin(), depth_buf_MSAA.end(), Eigen::Vector4f(DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX));
	
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
	depth_buf.resize(w * h);
	depth_buf_MSAA.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
	if (point.x() < 0 || point.x() >= width ||
		point.y() < 0 || point.y() >= height) return;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on