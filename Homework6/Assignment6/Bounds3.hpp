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
	//这两个点代表长方体的最左下点和最右上点
	Vector3f pMin, pMax; // two points to specify the bounding box
	Bounds3()
	{
		double minNum = std::numeric_limits<double>::lowest();
		double maxNum = std::numeric_limits<double>::max();
		//这里把pMax初始化为最小，pMin初始化为最大，方便后续更新
		pMax = Vector3f(minNum, minNum, minNum);
		pMin = Vector3f(maxNum, maxNum, maxNum);
	}
	Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
	Bounds3(const Vector3f p1, const Vector3f p2)
	{
		//简单的Bounding Box求法
		pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
		pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
	}

	Vector3f Diagonal() const { return pMax - pMin; }
	//该函数判断哪个分量最大
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

	//计算表面积
	double SurfaceArea() const
	{
		Vector3f d = Diagonal();
		return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
	}
	//计算中心
	Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
	//两个包围盒的交集
	Bounds3 Intersect(const Bounds3& b)
	{
		return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
			fmax(pMin.z, b.pMin.z)),
			Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
				fmin(pMax.z, b.pMax.z)));
	}
	//o是p占各边的比例
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
	//是否有重叠
	bool Overlaps(const Bounds3& b1, const Bounds3& b2)
	{
		bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
		bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
		bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
		return (x && y && z);
	}
	//点p是否在包围盒b内
	bool Inside(const Vector3f& p, const Bounds3& b)
	{
		return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
			p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
	}
	//Bounds3[0]代表pMin,否则是pMax(一般用p[1])
	inline const Vector3f& operator[](int i) const
	{
		return (i == 0) ? pMin : pMax;
	}

	inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
		const std::array<int, 3>& dirisNeg) const;
};


//判断光线是否与boundingbox相交
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
	const std::array<int, 3>& dirIsNeg) const
{
	// invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
	// dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
	// TODO test if ray bound intersects
	//注意需要考虑光线方向正负的问题,为负的时候是先与pMax交再与pMin交
	double t1 = 0, t2 = 0;
	t1 = (pMin.x - ray.origin.x) * invDir.x;
	t2 = (pMax.x - ray.origin.x) * invDir.x;
	double txmin = dirIsNeg[0] > 0 ? t1 : t2;
	double txmax = dirIsNeg[0] > 0 ? t2 : t1;

	t1 = (pMin.y - ray.origin.y) * invDir.y;
	t2 = (pMax.y - ray.origin.y) * invDir.y;
	double tymin = dirIsNeg[1] > 0 ? t1 : t2;
	double tymax = dirIsNeg[1] > 0 ? t2 : t1;

	t1 = (pMin.z - ray.origin.z) * invDir.z;
	t2 = (pMax.z - ray.origin.z) * invDir.z;
	double tzmin = dirIsNeg[2] > 0 ? t1 : t2;
	double tzmax = dirIsNeg[2] > 0 ? t2 : t1;

	double t_min = std::max(std::max(txmin, tymin), tzmin);
	double t_max = std::min(std::min(txmax, tymax), tzmax);

	return t_min <= t_max && t_max >= 0;
}
//取两个包围盒的并集
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
	ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
	return ret;
}
//取点与包围盒的并集(简单的替代以扩张)
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b.pMin, p);
	ret.pMax = Vector3f::Max(b.pMax, p);
	return ret;
}

#endif // RAYTRACING_BOUNDS3_H
