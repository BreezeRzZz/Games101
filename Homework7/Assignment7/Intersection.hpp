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
    Vector3f coords;            //相交坐标
    Vector3f tcoords;           //纹理坐标
    Vector3f normal;            //相交平面法向量
    Vector3f emit;              //标识是否是发光体，不发光为(0,0,0)，发光则表示光线方向
    double distance;            //传播时间
    Object* obj;                //相交物体
    Material* m;                //该物体材质
};
#endif //RAYTRACING_INTERSECTION_H
