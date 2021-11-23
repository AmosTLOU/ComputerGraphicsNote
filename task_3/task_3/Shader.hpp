//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"


struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex) :
         color(col), normal(nor), tex_coords(tc), texture(tex) {}


    Eigen::Vector3f view_pos; //ָ�õ�����
    Eigen::Vector3f color;    //ָ�õ���ɫ
    Eigen::Vector3f normal;   //ָ�õ㷨����
	Eigen::Vector2f tex_coords; //ָ�õ��Ӧ������ͼ��ά����
	Eigen::Vector3f tex_color; //�����Ա�������Լ��ӵġ�ָ�õ��Ӧ��������ɫ��ͨ����������3������������ɫ���в�ֵ�õ���
    Texture* texture;
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;
};

#endif //RASTERIZER_SHADER_H
