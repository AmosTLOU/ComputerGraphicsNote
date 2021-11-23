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


    Eigen::Vector3f view_pos; //指该点坐标
    Eigen::Vector3f color;    //指该点颜色
    Eigen::Vector3f normal;   //指该点法向量
	Eigen::Vector2f tex_coords; //指该点对应的纹理图二维坐标
	Eigen::Vector3f tex_color; //这个成员变量我自己加的。指该点对应的纹理颜色（通过对三角形3个顶点纹理颜色进行插值得到）
    Texture* texture;
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;
};

#endif //RASTERIZER_SHADER_H
