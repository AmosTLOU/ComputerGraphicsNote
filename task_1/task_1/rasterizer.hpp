//
// Created by goksu on 4/6/19.
//

#pragma once

#include "Triangle.hpp"
#include <algorithm>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

namespace rst {
enum class Buffers
{
    Color = 1,
    Depth = 2
};

inline Buffers operator|(Buffers a, Buffers b)
{
    return Buffers((int)a | (int)b);
}

inline Buffers operator&(Buffers a, Buffers b)
{
    return Buffers((int)a & (int)b);
}

enum class Primitive
{
    Line,
    Triangle
};

/*
 * For the curious : The draw function takes two buffer id's as its arguments.
 * These two structs make sure that if you mix up with their orders, the
 * compiler won't compile it. Aka : Type safety
 * */
struct pos_buf_id
{
	int pos_id = 0;
	//int pos_id;
};

struct ind_buf_id
{
	int ind_id = 0;
	//int ind_id;
};

class rasterizer
{
  public:
    rasterizer(int w, int h);
    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
    ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);

    void set_model(const Eigen::Matrix4f& m);
    void set_view(const Eigen::Matrix4f& v);
    void set_projection(const Eigen::Matrix4f& p);

    void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

    void clear(Buffers buff);

    void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);

    std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

  private:
    void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
    void rasterize_wireframe(const Triangle& t);

  private:
    Eigen::Matrix4f model;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;

	//����2��map�����������ε���Ϣ key�������εı�ţ�pos_buf�������ε�λ�ã�3�������ģ���ind_buf�������εĶ��������
    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;

	//����2��vector�������յĻ��� size��w*h frame_buff������ÿ�������ϵ���ɫ��depth_buf������ÿ�������ϵ���С��ȣ�����Z Buffer��
	//֮����û�н���������Ϊ��ά�ģ���x��y��������ȷ�����ص�λ�ã�����һά�ģ���һ��i��ȷ�����ص�λ�ã�������Ϊ����Ҫ��opencv��ͼ��opencvҪ��������ָ�ʽ��frame_buff
    std::vector<Eigen::Vector3f> frame_buf;
    std::vector<float> depth_buf;
    int get_index(int x, int y);

    int width, height;

    int next_id = 0;
    int get_next_id() { return next_id++; }
};
} // namespace rst
