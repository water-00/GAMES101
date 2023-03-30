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

    Vector3f Diagonal() const { return pMax - pMin; } // 对角线 pMin->pMax

    // 返回包围盒三个维度长度中的最大者
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

    // 包围盒表面积之和
    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    // 包围盒中心
    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

    // 返回两个包围盒的相交部分（也是一个包围盒）
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    // 返回(p-pMin)与(pMax-pMin)的各维度比值
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

    // 判断b1是否完全覆盖b2
    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    // 判断点p是否在包围盒b内
    bool Inside(const Vector3f& p, const Bounds3& b) const
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }

    // Bounds3[0] 返回pMin；Bounds3[1] 返回pMax
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }


    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg = {0, 0, 0}) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    
    if (Inside(ray.origin, *this)) {
        return true;
    }

    // 写六个平面的法向量，前三个是pMin所在的平面，后三个是pMax所在的平面
    Vector3f n1(1, 0, 0), n2(0, 1, 0), n3(0, 0, 1), n4(-1, 0, 0), n5(0, -1, 0), n6(0, 0, -1);
    double tmin1, tmin2, tmin3, tmax1, tmax2, tmax3;
    tmin1 = (pMin.x - ray.origin.x) * invDir.x;
    tmin2 = (pMin.y - ray.origin.y) * invDir.y;
    tmin3 = (pMin.z - ray.origin.z) * invDir.z;
    tmax1 = (pMax.x - ray.origin.x) * invDir.x;
    tmax2 = (pMax.y - ray.origin.y) * invDir.y;
    tmax3 = (pMax.z - ray.origin.z) * invDir.z;

    // 别人的代码有这个，不理解，先放一下
    // 如果direction某一维度为负，那么当这个维度上的O->pMax长度大于O->pMin（假设两个值都为正）时，会得到 tmax < tmin < 0
    if (ray.direction.x < 0) std::swap(tmin1, tmax1);
    if (ray.direction.y < 0) std::swap(tmin2, tmax2);
    if (ray.direction.z < 0) std::swap(tmin3, tmax3);

    double t_enter = fmax(tmin1, fmax(tmin2, tmin3));
    double t_exit = fmin(tmax1, fmin(tmax2, tmax3));
    if ((t_enter < t_exit) && t_exit >= 0)
        return true;
    else
        return false;
}

// 返回一个恰好包含b1, b2的包围盒
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

// 返回一个恰好包含b和点p的包围盒
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
