//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;
    Vector3f coords; // 坐标，在不同inter中有不同含义
    Vector3f tcoords;
    Vector3f normal; // 面光源法向量
    Vector3f emit; // 发光强度L_i
    double distance; // 光线从光源到相交点的时间
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
