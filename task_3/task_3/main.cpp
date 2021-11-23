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
	//normalized()���򻯣�������ô���򻯵Ĳ�̫���������Ҳû�鵽�����ӽ����˵������ά��֮��ı�����ϵ���䣬���ڿ϶�����֮ǰ����������ʱ֪���������ˡ�
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
	//���fragment��������ɫֱ����������3�����㡾��ֵ����������ɫ���Ļ����õ��Ľ���ܲ���ȷ��
	//���������������3�����㡾��ֵ�����������꡿�Ļ������ø���������ȥ��������ͼ�е�������ɫ�Ļ��������׼ȷ�ˡ�
	Eigen::Vector3f texture_color = payload.tex_color;
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
		// ������ҵ�ṩ������ͼ��СӦ�ø���άģ�͵Ĵ�С�����һ�µġ���Ϊ����û�����κΡ�����ͼ��С��������ͼ����ʱӦ�����Ĳ�������ҵ��Ҳû��Ҫ�󣩣�
		// ��󻭳���ͼЧ��Ҳ�ܺá�
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


// ��ҵû�������Լ�����1���򵥵�h������˫���Բ�ֵ�õ������Ľ����
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
	// ����u��v����0��span֮������ˡ�
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

// ����2������displacement��bump��ʵ���ƺ��Ϳ��Ͻ��ĸ��̫�Ǻϣ���û�ܳɹ��ﵽpdf���Ч��������ʵ����α���룬������h��������֪���Ƕ������ĸ��������⣩
// ǰ�����øı��ķ���������3�ֹ�����Ϊ�õ���ɫ�������Ͻ����ǻ�ı�ԭ��ģ�͵�ʵ����ֵ��������û�У�
// ������ֱ���øı��ķ�������Ϊ�õ���ɫ������û����ա������ϵĶ������øı��ķ���������3�ֹ�����Ϊ�õ���ɫ��������������displacement��ʵ�֣�����
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

	//�ú����У�����������������ݶ�����ҵ�ﱾ�����еģ���������ɾȥҲ����Ӱ�졣

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
	//��ҵ����ԭ������models����../models��������Ϊ����ҵ������У�
	//�������build�ļ��������Ҫ�˻ص���һ��Ŀ¼��../���������ҵ�models��     
	//����һ������ǰ��©��֪ʶ�㣡д����·����Ȼû�з��գ���д���·���Ļ�Ӧ�����ԡ�����ִ�еĳ�����.exe����Ϊ��׼��������������������������c��cpp��h��hpp�ļ�Ϊ��׼������������������
	//��������Ժ���Ϊ��׼�Ĵ������Ϊvs��codeblocks��pycharm��ide���У�ǰ�ߺͺ���������һ��·���£���������ҵ����ƾͲ�����һ��·�����ˣ���
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

	//std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = normal_fragment_shader; //ȱʡ��fragment shader
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
