#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
	SplitMethod splitMethod)
	: maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
	primitives(std::move(p))
{
	time_t start, stop;
	time(&start);
	if (primitives.empty())
		return;

	root = recursiveBuild(primitives);

	time(&stop);
	double diff = difftime(stop, start);
	int hrs = (int)diff / 3600;
	int mins = ((int)diff / 60) - (hrs * 60);
	int secs = (int)diff - (hrs * 3600) - (mins * 60);

	printf(
		"\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
		hrs, mins, secs);
}
//根据object列表，递归构建BVH二叉树
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	//将所有object的包围盒并起来，这作为根节点(本次递归的当前结点)的bounds
	for (int i = 0; i < objects.size(); ++i)
		bounds = Union(bounds, objects[i]->getBounds());
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		//叶子结点才有对象
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });

		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else {
		Bounds3 centroidBounds;
		//这里将各个object的中心用于union
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		int dim = centroidBounds.maxExtent();
		switch (dim) {
		case 0:
			//按照object中心x从小到大的方式排序，下同（排序的是centroidBounds的最大分量)
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().x <
					f2->getBounds().Centroid().x;
				});
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().y <
					f2->getBounds().Centroid().y;
				});
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().z <
					f2->getBounds().Centroid().z;
				});
			break;
		}

		auto beginning = objects.begin();
		auto middling = objects.begin() + (objects.size() / 2);
		auto ending = objects.end();

		
		//SAH part
		const int buckets_num = 12;
		float mincost = FLT_MAX;
		int diff = 0;
		std::vector<Bounds3> buckets(buckets_num);
		std::vector<int>	 bucketsCapacity(buckets_num);	//每个桶的物体数

		//求每个物体在CentroidBounds在长轴上的offset以确定桶位置
		for (int i = 0; i < objects.size(); ++i)
		{
			int pos;
			switch (dim)
			{
			case 0:pos = buckets_num * centroidBounds.Offset(objects[i]->getBounds().Centroid()).x; break;
			case 1:pos = buckets_num * centroidBounds.Offset(objects[i]->getBounds().Centroid()).y; break;
			case 2:pos = buckets_num * centroidBounds.Offset(objects[i]->getBounds().Centroid()).z; break;
			}
			if (pos == buckets_num)
				pos = buckets_num - 1;
			++bucketsCapacity[pos];
			//扩张桶的包围盒(因为是对CentroidBounds的划分，扩充也用Centroid)
			buckets[pos] = Union(buckets[pos], objects[i]->getBounds().Centroid());
		}

		//对buckets_num个桶做buckets_num-1次划分
		for (int i = 1; i < buckets_num; ++i)
		{
			diff = 0;
			Bounds3 A, B;
			int countA = 0, countB = 0;
			for (int j = 0; j < i; ++j)
			{
				A = Union(A, buckets[j]);
				countA += bucketsCapacity[j];
			}
			for (int j = i; j < buckets_num; ++j)
			{
				B = Union(B, buckets[j]);
				countB += bucketsCapacity[j];
			}

			float cost = (countA * A.SurfaceArea() + countB * B.SurfaceArea()) / bounds.SurfaceArea();
			if (cost < mincost)
			{
				mincost = cost;
				for (int k = 0; k < i; ++k)
					diff += bucketsCapacity[k];
				middling = beginning + diff;
			}
		}

		auto leftshapes = std::vector<Object*>(beginning, middling);
		auto rightshapes = std::vector<Object*>(middling, ending);

		assert(objects.size() == (leftshapes.size() + rightshapes.size()));
		//二分构建二叉树
		node->left = recursiveBuild(leftshapes);
		node->right = recursiveBuild(rightshapes);

		node->bounds = Union(node->left->bounds, node->right->bounds);
	}

	return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
	Intersection isect;
	if (!root)
		return isect;
	isect = BVHAccel::getIntersection(root, ray);
	return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
	// TODO Traverse the BVH to find intersection
	// 我们得到的是一颗二叉树的根，需要遍历对每个结点判断
	// 按照正常逻辑，应先对大区域判断，再进一步对小区域判断，所以应该用前序遍历

	//若不相交
	if (!node->bounds.IntersectP(ray, ray.direction_inv, { int(ray.direction.x > 0),int(ray.direction.y > 0),int(ray.direction.z > 0) }))
		return Intersection();

	//叶子结点:对每个对象判断是否相交(本实验中叶子结点就只有一个对象)
	if (node->left == nullptr && node->right == nullptr)
		return node->object->getIntersection(ray);

	Intersection hit1, hit2;
	if (node->left != nullptr)
		hit1 = getIntersection(node->left, ray);
	if (node->right != nullptr)
		hit2 = getIntersection(node->right, ray);
	return hit1.distance < hit2.distance ? hit1 : hit2;
}