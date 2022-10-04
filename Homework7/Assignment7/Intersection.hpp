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
    bool happened;              //�Ƿ��ཻ
    Vector3f coords;            //�ཻ����
    Vector3f tcoords;           //��������
    Vector3f normal;            //�ཻƽ�淨����
    Vector3f emit;              //��ʶ�Ƿ��Ƿ����壬������Ϊ(0,0,0)���������ʾ���߷���
    double distance;            //����ʱ��
    Object* obj;                //�ཻ����
    Material* m;                //���������
};
#endif //RAYTRACING_INTERSECTION_H
