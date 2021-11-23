//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <stdlib.h>

using namespace std;

extern int hit_nothing;
extern int ind_castRay;

void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
	return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
	const Ray &ray,
	const std::vector<Object*> &objects,
	float &tNear, uint32_t &index, Object **hitObject)
{
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
			*hitObject = objects[k];
			tNear = tNearK;
			index = indexK;
		}
	}


	return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here

	Vector3f L_dir = Vector3f(0.0);
	Vector3f L_indir = Vector3f(0.0);
	Intersection inter, inter_sampleLight, inter_check_blocked;
	float pdf_light = 0.0;
	inter = intersect(ray);
	if (!inter.happened) {
		//会有一部分castRay打不到任何物体，这部分区域就是最后结果里的上下左右4个边框;
		return Vector3f(0);
	}
	else if (inter.obj->hasEmit()) { //如果castRay打到的位置就是光源
		return Vector3f(1);
	}
	// 从这里往下的部分，就相当于是课上讲的shade函数。
	// 以下部分，我基本是照着作业指导pdf里的伪代码写的。但伪代码有一些问题，比如有些方向可能反了，有些地方没有归一化。
	sampleLight(inter_sampleLight, pdf_light);

	Vector3f ws = inter.coords - inter_sampleLight.coords;
	inter_check_blocked = intersect(Ray(inter_sampleLight.coords, normalize(ws)));
	if (inter_check_blocked.happened) {
		Vector3f bias = inter_check_blocked.coords - inter.coords;
		float limit = 0.1;
		if (bias.x < limit && bias.y < limit && bias.z < limit) { // light not blocked
			Vector3f wo = -ray.direction;
			Vector3f N = inter.normal;
			Vector3f NN = inter_sampleLight.normal;

			//求cos的点乘一定要记得将向量先归一化！！！
			L_dir = inter_sampleLight.emit * inter.m->eval(ws, wo, N) * dotProduct(normalize(-ws), normalize(N))
				* dotProduct(normalize(ws), normalize(NN)) / dotProduct(ws, ws) / pdf_light;
			//总结 L_dir（光对该点的贡献 direct代表直接光照）= 光的emission * 该点BRDF * cos theta * cos theta prime / 点到光源距离的平方 / 光源采样pdf
			// 光的emission 具体数值见main中light材质的定义，Material构造函数的第二个参数
			// 注意！！！凡是说该点，就是这一点。通常写作p，不是光源，也不是再继续传播下去而打到的q点
		}
	}

	Intersection inter_q;
	float P, P_RR;
	P_RR = RussianRoulette;
	P = (float)rand() / RAND_MAX;
	//std::cout << "P: " << P << endl;
	if (P < P_RR) {
		Vector3f wo = -ray.direction;
		Vector3f wi;
		wi = inter.m->sample(wo, inter.normal); // sample函数根本没有用到第一个参数，只使用了第二个参数法向量。sample函数返回的wi，方向是朝外的。
		Ray p_to_q = Ray(inter.coords, normalize(wi));
		inter_q = intersect(p_to_q);
		if (inter_q.happened && !inter_q.obj->hasEmit()) { // 如果打到了物体q，且物体q不是光源（光源的贡献已经在L_dir直接光照里算过了）。
			Vector3f color_q = castRay(p_to_q, ++depth);
			//求cos的点乘一定要记得将向量先归一化！！！
			L_indir = color_q * inter.m->eval(wi, wo, inter.normal) * dotProduct(normalize(wi), normalize(inter.normal))
				/ inter.m->pdf(wi, wo, inter.normal) / P_RR;
			// 总结 L_indir（非光源的q点对该点的贡献  indirect代表间接光照）= q点颜色 * 该点BRDF * cos theta / 该点采样pdf / RR概率
			// 注意！！！凡是说该点，就是这一点。通常写作p，不是光源，也不是再继续传播下去而打到的q点
		}
	}

	return L_dir + L_indir;
}