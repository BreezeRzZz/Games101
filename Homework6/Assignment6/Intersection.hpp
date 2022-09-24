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
    bool happened;              //是否相交
    Vector3f coords;            //相交点
    Vector3f normal;            //相交平面法向量
    double distance;            //与光源的距离(用t衡量)
    Object* obj;                //相交对象
    Material* m;                //材质
};
#endif //RAYTRACING_INTERSECTION_H
