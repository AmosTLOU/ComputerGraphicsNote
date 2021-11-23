//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects

	//��Ϊ��AABB�������İ�Χ�У����Բ���Ҫʹ���������ָ����ӵ���ʽ��
	/*std::array<Vector3f, 3> n1;
	std::array<Vector3f, 3> n2;
	n1[0] = { 0, 0, -1 }; //xy
	n1[1] = { 0, -1, 0 }; //xz
	n1[2] = { -1, 0, 0 }; //yz
	n2[0] = { 0, 0, 1 }; //xy
	n2[1] = { 0, 1, 0 }; //xz
	n2[2] = { 1, 0, 0 }; //yz
	for (int i = 0; i < 3; i++) {
	t[i][0] = dotProduct((pMin - ray.origin), n1[i]) / dotProduct(ray.direction, n1[i]);
	t[i][1] = dotProduct((pMax - ray.origin), n2[i]) / dotProduct(ray.direction, n2[i]);
	}*/

	//�����β�invDir��dirIsNeg��û��ʹ�á�
	float t[3][2], t_enter, t_exit, t_tmp;
	t[0][0] = (pMin.z - ray.origin.z) / ray.direction.z;
	t[0][1] = (pMax.z - ray.origin.z) / ray.direction.z;
	t[1][0] = (pMin.y - ray.origin.y) / ray.direction.y;
	t[1][1] = (pMax.y - ray.origin.y) / ray.direction.y;
	t[2][0] = (pMin.x - ray.origin.x) / ray.direction.x;
	t[2][1] = (pMax.x - ray.origin.x) / ray.direction.x;
	for (int i = 0; i < 3; i++) {
		if (t[i][0] > t[i][1]) {
			t_tmp = t[i][0];
			t[i][0] = t[i][1];
			t[i][1] = t_tmp;
		}
	}
	t_enter = t[0][0] > t[1][0] ? t[0][0] : t[1][0];
	t_enter = t[2][0] > t_enter ? t[2][0] : t_enter;
	t_exit = t[0][1] < t[1][1] ? t[0][1] : t[1][1];
	t_exit = t[2][1] < t_exit ? t[2][1] : t_exit;

	if (t_enter <= t_exit && t_exit >= 0)
		return true;
	else
		return false;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
