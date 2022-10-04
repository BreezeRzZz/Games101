//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
	return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
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
	const Ray& ray,
	const std::vector<Object*>& objects,
	float& tNear, uint32_t& index, Object** hitObject)
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
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here
	Vector3f light_dir, light_indir;

	Intersection wo_inter = intersect(ray);
	if (!wo_inter.happened)
	{
		return { 0.f,0.f,0.f };
	}
	else if (wo_inter.m->hasEmission())
	{
		light_dir += wo_inter.m->getEmission();
	}

	float pdf;
	Intersection ws_inter;
	sampleLight(ws_inter, pdf);


	Vector3f p = wo_inter.coords;

	Vector3f x = ws_inter.coords;

	Vector3f wo = ray.direction;
	Vector3f ws = (x - p).normalized();

	Vector3f N = wo_inter.normal.normalized();
	Vector3f NN = ws_inter.normal.normalized();

	Vector3f emit = ws_inter.emit;

	Ray ws_ray(p, ws);
	Intersection ws_ray_intersection = intersect(ws_ray);

	if ((ws_ray_intersection.coords - x).norm() <= EPSILON)
	{
		float dis = (x - p).norm();
		light_dir += emit * wo_inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (dis * dis) / pdf;
	}

	float ksi = get_random_float();
	if (ksi > RussianRoulette)
		return light_dir + light_indir;

	Vector3f wi = wo_inter.m->sample(wo, N).normalized();
	Ray wi_ray(p, wi);
	Intersection wi_inter = intersect(wi_ray);

	if (wi_inter.happened && !wi_inter.m->hasEmission())
	{
		light_indir = castRay(wi_ray, depth + 1) * wo_inter.m->eval(wo, wi, N) * dotProduct(wi, N) / wo_inter.m->pdf(wo, wi, N) / RussianRoulette;
	}

	return light_dir + light_indir;
}