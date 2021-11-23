//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <array>
#include <string>

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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

	rst::col_buf_id tem;
	tem.col_id = id;

    return tem;
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// 作业2里该函数没提供，这次提供了，但形参x和y类型本来写的是int，我感觉不合理，改成了float。
// 因为根据课上讲的内容，应该判断的是像素中心点在不在三角形中，而像素中心点坐标是(x + 0.5f, y + 0.5f) 浮点类型。
// 另外，这次提供的算法跟课上讲的不一样。（我不知道这个算法的原理是什么，跟课上讲的算法肯定可以互相转化，但怎么转化的我也不知道。暂时都没研究）
static bool insideTriangle(float x, float y, const Vector3f* _v){
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
	if ((p.dot(f0)*f0.dot(v[2]) > 0) && (p.dot(f1)*f1.dot(v[0]) > 0) && (p.dot(f2)*f2.dot(v[1]) > 0))
		return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
			vec.z()/= vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
		// viewspace_pos就是三角形3个顶点在投影变换之前的三维坐标
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;
	// zp = - zp; //这行我加的!

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);

	
	auto v = t.toVector4();

	//Find out the bounding box of current triangle.
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
				//float x_[4], y_[4];
				//x_[0] = x + 1 / 3.f;
				//x_[1] = x + 2 / 3.f;
				//x_[2] = x + 1 / 3.f;
				//x_[3] = x + 2 / 3.f;
				//y_[0] = y + 1 / 3.f;
				//y_[1] = y + 1 / 3.f;
				//y_[2] = y + 2 / 3.f;
				//y_[3] = y + 2 / 3.f;
				//int num_inside = 0;
				//for (int n = 0; n <= 3; n++) {
				//	if (!insideTriangle(x_[n], y_[n], _v))
				//		continue;
				//	std::tuple<float, float, float> tuple_barycentric;
				//	float alpha, beta, gamma;
				//	//从下一行到z_interpolated *= w_reciprocal这一行，是在通过三角形三个顶点的xyz坐标和某个点的xy坐标，来推断出该点的z坐标的绝对值。
				//	tuple_barycentric = computeBarycentric2D(x_[n], y_[n], t.v);
				//	//获取tuple元组内数据的方式
				//	alpha = std::get<0>(tuple_barycentric);
				//	beta = std::get<1>(tuple_barycentric);
				//	gamma = std::get<2>(tuple_barycentric);
				//	float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				//	float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				//	z_interpolated *= w_reciprocal;
				//	z_interpolated = -z_interpolated;//作业里没有这一行，没有这行的话z_interpolated不会是正的！（但作业里又说“已将z_interpolated处理为正数”...很无语）
				//									 //此时被处理完毕的z_interpolated都是【正数】，越大代表离相机越远
				//	int ind = get_index(x, y);
				//	if (z_interpolated < depth_buf_MSAA[ind][n]) {
				//		num_inside++;
				//		depth_buf_MSAA[ind][n] = z_interpolated;
				//	}
				//}
				//if (num_inside)
				//	set_pixel(Eigen::Vector3f(x, y, 0.f), t.getColor() * num_inside / 4.);
			}
			else {
				if (insideTriangle(x + 0.5f, y + 0.5f, _v)) { //判断某像素是否在三角形内部时，要用其中心的坐标（x+0.5, y+0.5）来判断！！！！！！ 但是set_pixel的时候还是要用(x,y),因为像素索引肯定还是整数。
					std::tuple<float, float, float> tuple_barycentric;
					float alpha, beta, gamma;
					/*Vector3f a = view_pos[2] - view_pos[0];
					Vector3f b = view_pos[1] - view_pos[0];
					Vector3f c = b.cross(a);
					Matrix4f mat_reverse, mat_reverse_1, mat_reverse_2;
					mat_reverse_1 << 1, 0, 0, -c.x(),
										0, 1, 0, -c.y(),
										0, 0, 1, -c.z(),
										0, 0, 0, 1;
					Vector3f g_cross_t = (-c).cross(Vector3f(0, 1, 0));
					mat_reverse_2 << g_cross_t.x(), g_cross_t.y(), g_cross_t.z(), 0,
										0, 1, 0, 0,
										c.x(), c.y(), c.z(), 0,
										0, 0, 0, 1;
					mat_reverse = mat_reverse_2 * mat_reverse_1;
					Vector4f v_before_projection[3];
					for (int i = 0; i < 3; i++) {
						v_before_projection[i] = mat_reverse * Vector4f(view_pos[i].x(), view_pos[i].y(), view_pos[i].z(), 1.f);
						v_before_projection[i] /= v_before_projection[i].w();
					}
					Vector4f pixel_before_projection = { x + 0.5f, y + 0.5f, 0.1f, 1};
					pixel_before_projection = mat_reverse * pixel_before_projection;
					pixel_before_projection /= pixel_before_projection.w();
					tuple_barycentric = computeBarycentric2D(pixel_before_projection.x(), pixel_before_projection.y(), v_before_projection);*/
					tuple_barycentric = computeBarycentric2D(x + 0.5f, y + 0.5f, t.v);
					//获取tuple元组内数据的方式
					alpha = std::get<0>(tuple_barycentric);
					beta = std::get<1>(tuple_barycentric);
					gamma = std::get<2>(tuple_barycentric);
					float z_interpolated = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();
					z_interpolated = -z_interpolated;//作业里没有这一行，没有这行的话z_interpolated不会是正的！（但作业里又说“已将z_interpolated处理为正数”...很无语）
					//此时被处理完毕的z_interpolated都是【正数】，越大代表离相机越远

					int ind = get_index(x, y);
					if (z_interpolated < depth_buf[ind]) {
						depth_buf[ind] = z_interpolated;

						fragment_shader_payload payload;
						Vector3f interpolated_normal = alpha * t.normal[0] + beta * t.normal[1] + gamma * t.normal[2];
						Vector3f interpolated_color = alpha * t.color[0] + beta * t.color[1] + gamma * t.color[2];
						std::array<Vector3f, 3> tex_colors;
						for (int j = 0; j < 3; j++)
							tex_colors[j] = texture.getColor(t.tex_coords[j].x(), t.tex_coords[j].y());
						Vector3f interpolated_texcolor = alpha * tex_colors[0] + beta * tex_colors[1] + gamma * tex_colors[2];
						Vector2f interpolated_texcoords = alpha * t.tex_coords[0] + beta * t.tex_coords[1] + gamma * t.tex_coords[2];											
						Vector3f interpolated_shadingcoords = alpha * view_pos[0] + beta * view_pos[1] + gamma * view_pos[2]; //像素点在投影前的坐标，也是通过插值来得出来！！！！！！
						payload.normal = interpolated_normal.normalized();
						payload.color = interpolated_color;
						payload.tex_color = interpolated_texcolor;
						payload.tex_coords = interpolated_texcoords;
						payload.texture = &texture;
						payload.view_pos = interpolated_shadingcoords;
						
						auto pixel_color = fragment_shader(payload);
						set_pixel(Eigen::Vector2i(x, y), pixel_color);
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
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

	//以下是作业原来的写法，但由于vs2015不支持c++17，因此删掉。
	//texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

