#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"
#include <fstream>

using namespace std;

float get_vector3f_length(Eigen::Vector3f v) {
	return sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2));
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
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
		0, 0, 1, -(zNear + zFar) / 2.f,
		0, 0, 0, 1;
	ortho_2 << 1 / h_half, 0, 0, 0,
		0, 1 / w_half, 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;
	projection = ortho_2 * ortho_1 * persp_to_ortho * projection;

	return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
	//normalized()正则化，具体怎么正则化的不太清楚，网上也没查到。但从结果来说，各个维度之间的比例关系不变，等于肯定还是之前的向量，暂时知道这点就行了。
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
	//如果fragment的纹理颜色直接用三角形3个顶点【插值出的纹理颜色】的话，得到的结果很不精确。
	//但是如果用三角形3个顶点【插值出的纹理坐标】的话，再用该纹理坐标去索引纹理图中的纹理颜色的话，结果就准确了。
	Eigen::Vector3f texture_color = payload.tex_color;
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
		// 本次作业提供的纹理图大小应该跟三维模型的大小差不多是一致的。因为这里没有做任何“纹理图过小”或“纹理图过大”时应该做的操作（作业里也没有要求），
		// 最后画出的图效果也很好。
		texture_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

		Vector3f light_diffuse(0, 0, 0), light_route;
		light_route = light.position - point;
		float n_dot_l = (normal / get_vector3f_length(normal)).dot(light_route / get_vector3f_length(light_route));
		float n_dot_l_ = n_dot_l > 0 ? n_dot_l : 0;
		for (int i = 0; i < 3; i++)
			light_diffuse[i] = kd[i] * light.intensity[i];
		light_diffuse *= (n_dot_l_ / pow(get_vector3f_length(light_route), 2));
		result_color += light_diffuse;

		Vector3f light_specular(0, 0, 0), view_route, h;
		view_route = eye_pos - point;
		h = (view_route + light_route);
		h /= get_vector3f_length(h);
		float n_dot_h = (normal / get_vector3f_length(normal)).dot(h);
		float n_dot_h_ = n_dot_h > 0 ? n_dot_h : 0;
		n_dot_h_ = pow(n_dot_h_, p);
		for (int i = 0; i < 3; i++)
			light_specular[i] = ks[i] * light.intensity[i];
		light_specular *= (n_dot_h_ / pow(get_vector3f_length(light_route), 2));
		result_color += light_specular;

		Vector3f light_ambient(0, 0, 0);
		for (int i = 0; i < 3; i++)
			light_ambient[i] = ka[i] * amb_light_intensity[i];
		result_color += light_ambient;

    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	//Eigen::Vector3f kd = {0.1f, 0.1f, 0.1f};
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
		
		Vector3f light_diffuse(0, 0, 0), light_route;
		light_route = light.position - point;
		float n_dot_l = (normal/get_vector3f_length(normal)).dot(light_route/ get_vector3f_length(light_route));
		float n_dot_l_ = n_dot_l > 0 ? n_dot_l : 0;
		for (int i = 0; i < 3; i++) 
			light_diffuse[i] = kd[i] * light.intensity[i];
		light_diffuse *= (n_dot_l_ / pow(get_vector3f_length(light_route), 2) );
		result_color += light_diffuse;

		Vector3f light_specular(0, 0, 0), view_route, h;
		view_route = eye_pos - point;
		h = (view_route + light_route);
		h /= get_vector3f_length(h);
		float n_dot_h = (normal / get_vector3f_length(normal)).dot(h);
		float n_dot_h_ = n_dot_h > 0 ? n_dot_h : 0;
		n_dot_h_ = pow(n_dot_h_, p);
		for (int i = 0; i < 3; i++)
			light_specular[i] = ks[i] * light.intensity[i];
		light_specular *= (n_dot_h_ /  pow(get_vector3f_length(light_route), 2));
		result_color += light_specular;

		Vector3f light_ambient(0, 0, 0);
		for (int i = 0; i < 3; i++)
			light_ambient[i] = ka[i] * amb_light_intensity[i];
		result_color += light_ambient;
    }
	
    return result_color * 255.f;
}


// 作业没给，那自己定义1个简单的h函数。双线性插值得到连续的结果。
float h(float u, float v) {
	
	/*int u1, u2, v1, v2;
	float l1, l2, h1, h2, result_up, result_down, result_final, derivation;
	derivation = 5;
	u1 = (int)u;
	v1 = (int)v;
	u2 = u1 + 1;
	v2 = v1 + 1;
	if (0 == (u1 + v1) % 2) {
		l1 = h2 = -derivation;
		l2 = h1 = derivation;
	}
	else {
		l1 = h2 = derivation;
		l2 = h1 = -derivation;
	}
	result_up = h2 * (u - u1) / (u2 - u1) + l2 * (u2 - u) / (u2 - u1);
	result_down = h1 * (u - u1) / (u2 - u1) + l1 * (u2 - u) / (u2 - u1);

	result_final = result_up * (v - v1) / (v2 - v1) + result_down * (v2 - v) / (v2 - v1);

	return result_final;*/

	int num_span;
	float u1, u2, v1, v2;
	float span, l1, l2, r1, r2, result_up, result_down, result_final, derivation;
	span = 1.f;
	derivation = 10;
	num_span = (int)(u / span) + (int)(v / span);
	u = u - (int)(u / span) * span;
	v = v - (int)(v / span) * span;
	// 现在u和v都是0到span之间的数了。
	u1 = 0;
	v1 = 0;
	u2 = span;
	v2 = span;
	if (0 == (num_span) % 2) {
		l1 = r2 = -derivation;
		l2 = r1 = derivation;
	}
	else {
		l1 = r2 = derivation;
		l2 = r1 = -derivation;
	}
	result_up = r2 * (u - u1) / (u2 - u1) + l2 * (u2 - u) / (u2 - u1);
	result_down = r1 * (u - u1) / (u2 - u1) + l1 * (u2 - u) / (u2 - u1);

	result_final = result_up * (v - v1) / (v2 - v1) + result_down * (v2 - v) / (v2 - v1);
	/*if ((abs(u - u1) / span < 0.1 && abs(v - v1) / span < 0.1) || (abs(u - u2) / span < 0.1 && abs(v - v2) / span < 0.1))
		result_final = l1;
	else
		result_final = l2;*/
	//cout << result_final << endl;
	return result_final;
}

// 以下2个函数displacement和bump的实现似乎和课上讲的概念不太吻合？（没能成功达到pdf里的效果。明明实现了伪代码，设置了h函数。不知道是二者中哪个出了问题）
// 前者是用改变后的法向量计算3种光照作为该点颜色。（课上讲的是会改变原本模型的实际数值，但这里没有）
// 后者是直接拿改变后的法向量作为该点颜色。根本没算光照。（课上的定义是用改变后的法向量计算3种光照作为该点颜色。即正好是这里displacement的实现！！）
Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

	float x, y, z;
	x = normal.x();
	y = normal.y();
	z = normal.z();
	Eigen::Vector3f t = { x*y / sqrt(x*x + z*z), sqrt(x*x + z*z), z*y / sqrt(x*x + z*z) };
	Eigen::Vector3f b = normal.cross(t);
	Eigen::Matrix3f TBN;
	for (int j = 0; j < 3; j++)
		TBN(0, j) = t[j];
	for (int j = 0; j < 3; j++)
		TBN(1, j) = b[j];
	for (int j = 0; j < 3; j++)
		TBN(2, j) = normal[j];
	float u = payload.tex_coords[0];
	float v = payload.tex_coords[1];
	float dU = kh * kn * (h(u + 1, v) - h(u, v));
	float dV = kh * kn * (h(u, v + 1) - h(u, v));
	Eigen::Vector3f ln = { -dU, -dV, 1 };
	normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

		Vector3f light_diffuse(0, 0, 0), light_route;
		light_route = light.position - point;
		float n_dot_l = (normal / get_vector3f_length(normal)).dot(light_route / get_vector3f_length(light_route));
		float n_dot_l_ = n_dot_l > 0 ? n_dot_l : 0;
		for (int i = 0; i < 3; i++)
			light_diffuse[i] = kd[i] * light.intensity[i];
		light_diffuse *= (n_dot_l_ / pow(get_vector3f_length(light_route), 2));
		result_color += light_diffuse;

		Vector3f light_specular(0, 0, 0), view_route, h;
		view_route = eye_pos - point;
		h = (view_route + light_route);
		h /= get_vector3f_length(h);
		float n_dot_h = (normal / get_vector3f_length(normal)).dot(h);
		float n_dot_h_ = n_dot_h > 0 ? n_dot_h : 0;
		n_dot_h_ = pow(n_dot_h_, p);
		for (int i = 0; i < 3; i++)
			light_specular[i] = ks[i] * light.intensity[i];
		light_specular *= (n_dot_h_ / pow(get_vector3f_length(light_route), 2));
		result_color += light_specular;

		Vector3f light_ambient(0, 0, 0);
		for (int i = 0; i < 3; i++)
			light_ambient[i] = ka[i] * amb_light_intensity[i];
		result_color += light_ambient;

    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
	
    float kh = 0.2, kn = 0.1;

	//该函数中，从这里往上算的内容都是作业里本来就有的，但几乎都删去也并不影响。

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
	
	float x, y, z;
	x = point.x();
	y = point.y();
	z = point.z();
	Eigen::Vector3f n = {x, y, z};
	Eigen::Vector3f t = { x*y / sqrt(x*x + z*z), sqrt(x*x + z*z), z*y / sqrt(x*x + z*z) };
	Eigen::Vector3f b = n.cross(t);
	Eigen::Matrix3f TBN;
	for (int j = 0; j < 3; j++)
		TBN(0, j) = t[j];
	for (int j = 0; j < 3; j++)
		TBN(1, j) = b[j];
	for (int j = 0; j < 3; j++)
		TBN(2, j) = normal[j];
	float u = payload.tex_coords[0];
	float v = payload.tex_coords[1];
	float dU = kh * kn * (h(u + 1, v) - h(u, v));
	float dV = kh * kn * (h(u, v + 1) - h(u, v));
	Eigen::Vector3f ln = { -dU, -dV, 1 };
	//cout << ln[0] << " " << ln[1] << " " << ln[2] << endl;
	n = (TBN * ln).normalized();
	
    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = n;

    return result_color * 255.f;
}


int main(int argc, const char** argv)
{
	/*Vector3f tri[3];
	tri[0] << 1, 1, 1;
	tri[1] << 1, 2, 1;
	tri[2] << 2, 1, 1;
	cout << insideTriangle(1.1, 1.1, tri);
	exit(0);*/

    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
	//作业这里原本不是models而是../models。这是因为在作业的设计中，
	//程序会在build文件夹里！所以要退回到上一级目录（../），才能找到models。     
	//这是一个我以前遗漏的知识点！写绝对路径当然没有风险，但写相对路径的话应该是以【最终执行的程序（如.exe）】为基准！！！！！！！！！而不是以c、cpp、h、hpp文件为基准！！！！！！！！！
	//（会产生以后者为基准的错觉是因为vs、codeblocks、pycharm等ide当中，前者和后者往往在一个路径下，但按照作业的设计就不再是一个路径下了！）
    std::string obj_path = "models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("models/spot/spot_triangulated_good.obj");
	
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }
    rst::rasterizer r(700, 700);
	//auto texture_path = "4colors.jpg";
	auto texture_path = "spot_texture.png";
    r.set_texture(Texture(obj_path + texture_path));

	//std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = normal_fragment_shader; //缺省的fragment shader
	//std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;
	//std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;
	std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = bump_fragment_shader;
	//std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = displacement_fragment_shader;
	
	

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
