#pragma once

#include "Core.h"
#include "Sampling.h"

// 將宏定義更改為變量，以支持運行時設置
extern int USE_MAX_TRIANGLES;
extern float USE_TRAVERSE_COST;
extern float USE_TRIANGLE_COST;
extern int USE_BUILD_BINS;
extern float RAY_EPSILON;

// 保留預設值作為初始化值
#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 16   // BVH構建時的分割數

#define EPSILON 0.001f // 浮點誤差容忍度 - 用於一般浮點比較

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		
		float denom = n.dot(r.dir);

		if (fabs(denom) < 1e-6)
			return false;

		t = -(n.dot(r.o) + d) / denom;

		if (t < 0)
			return false;

		return true;
	}

};



class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[1].p - vertices[0].p;
		e2 = vertices[2].p - vertices[0].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 p = r.dir.cross(e2);
		float det = Dot(p, e1);
 
		// parellel ray check
		if (std::abs(det) < EPSILON)
			return false;
 
		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[0].p;
 
		u = Dot(T, p) * invDet;
 
		if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u - 1) > EPSILON))
			return false;
 
		Vec3 q = T.cross(e1);
		v = Dot(r.dir, q) * invDet;
 
		if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
			return false;
 
		t = Dot(e2, q) * invDet;
 
		if (t < EPSILON)
			return false;
		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		float tmin = 0.0f;
		float tmax = FLT_MAX;
		
		// 检查x轴
		if (std::abs(r.dir.x) > EPSILON) {
			float tx1 = (min.x - r.o.x) * r.invDir.x;
			float tx2 = (max.x - r.o.x) * r.invDir.x;
			
			tmin = std::max(tmin, std::min(tx1, tx2));
			tmax = std::min(tmax, std::max(tx1, tx2));
		}
		else if (r.o.x < min.x || r.o.x > max.x) {
			return false;
		}
		
		// 检查y轴
		if (std::abs(r.dir.y) > EPSILON) {
			float ty1 = (min.y - r.o.y) * r.invDir.y;
			float ty2 = (max.y - r.o.y) * r.invDir.y;
			
			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));
		}
		else if (r.o.y < min.y || r.o.y > max.y) {
			return false;
		}
		
		// 检查z轴
		if (std::abs(r.dir.z) > EPSILON) {
			float tz1 = (min.z - r.o.z) * r.invDir.z;
			float tz2 = (max.z - r.o.z) * r.invDir.z;
			
			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));
		}
		else if (r.o.z < min.z || r.o.z > max.z) {
			return false;
		}
		
		if (tmax < 0 || tmin > tmax) {
			return false;
		}
		
		t = tmin;
		return true;
	}
	
	bool rayAABB(const Ray& r)
	{
		// 独立实现，不计算具体的t值，只判断是否相交
		float tmin = -FLT_MAX;
		float tmax = FLT_MAX;
		
		// 检查x轴
		if (std::abs(r.dir.x) > EPSILON) {
			float tx1 = (min.x - r.o.x) * r.invDir.x;
			float tx2 = (max.x - r.o.x) * r.invDir.x;
			
			tmin = std::max(tmin, std::min(tx1, tx2));
			tmax = std::min(tmax, std::max(tx1, tx2));
		}
		else if (r.o.x < min.x || r.o.x > max.x) {
			return false;
		}
		
		// 检查y轴
		if (std::abs(r.dir.y) > EPSILON) {
			float ty1 = (min.y - r.o.y) * r.invDir.y;
			float ty2 = (max.y - r.o.y) * r.invDir.y;
			
			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));
		}
		else if (r.o.y < min.y || r.o.y > max.y) {
			return false;
		}
		
		// 检查z轴
		if (std::abs(r.dir.z) > EPSILON) {
			float tz1 = (min.z - r.o.z) * r.invDir.z;
			float tz2 = (max.z - r.o.z) * r.invDir.z;
			
			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));
		}
		else if (r.o.z < min.z || r.o.z > max.z) {
			return false;
		}
		
		return tmax >= tmin && tmax > 0;
	}
	// Add code here
	float area()
	{
		// 计算表面积
		Vec3 d = max - min;
		return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
	}
	
	float volume()
	{
		// 计算体积
		Vec3 d = max - min;
		return d.x * d.y * d.z;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};



class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// 添加三角形索引存儲
	std::vector<int> triangleIndices;
	
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	
	// Note there are several options for how to implement the build method. Update this as required
	//void build(std::vector<Triangle>& inputTriangles)
	//{
	//	// Add BVH building code here
	//}
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		
		if (inputTriangles.size() == 0)
			return;
			
		// 计算所有三角形的AABB
		bounds.reset();
		for (int i = 0; i < inputTriangles.size(); i++)
		{
			bounds.extend(inputTriangles[i].vertices[0].p);
			bounds.extend(inputTriangles[i].vertices[1].p);
			bounds.extend(inputTriangles[i].vertices[2].p);
		}
		
		// 如果三角形数量小于阈值，作为叶子节点
		if (inputTriangles.size() <= USE_MAX_TRIANGLES)
		{
			// 存儲三角形在輸出數組中的索引
			int startIndex = outputTriangles.size();
			for (int i = 0; i < inputTriangles.size(); i++)
			{
				outputTriangles.push_back(inputTriangles[i]);
				triangleIndices.push_back(startIndex + i);
			}
			return;
		}
		
		// 使用SAH，但在三角形数量小的時候直接使用中點劃分，加速BVH建構
		if (inputTriangles.size() < 64) {
			// 计算三角形的平均中心点
			Vec3 centroid(0,0,0);
			for (int i = 0; i < inputTriangles.size(); i++) {
				centroid = centroid + inputTriangles[i].centre();
			}
			centroid = centroid * (1.0f / inputTriangles.size());
			
			// 确定最长的轴
			float xSize = bounds.max.x - bounds.min.x;
			float ySize = bounds.max.y - bounds.min.y;
			float zSize = bounds.max.z - bounds.min.z;
			
			int splitAxis = 0;
			if (ySize > xSize) splitAxis = 1;
			if (zSize > xSize && zSize > ySize) splitAxis = 2;
			
			// 根据轴将三角形分为两部分
			std::vector<Triangle> leftTriangles;
			std::vector<Triangle> rightTriangles;
			
			for (int i = 0; i < inputTriangles.size(); i++) {
				Vec3 centre = inputTriangles[i].centre();
				if ((splitAxis == 0 && centre.x < centroid.x) ||
					(splitAxis == 1 && centre.y < centroid.y) ||
					(splitAxis == 2 && centre.z < centroid.z)) {
					leftTriangles.push_back(inputTriangles[i]);
				} else {
					rightTriangles.push_back(inputTriangles[i]);
				}
			}
			
			// 处理特殊情况：所有三角形都在一侧
			if (leftTriangles.size() == 0 || rightTriangles.size() == 0) {
				int mid = inputTriangles.size() / 2;
				leftTriangles.clear();
				rightTriangles.clear();
				
				for (int i = 0; i < inputTriangles.size(); i++) {
					if (i < mid) leftTriangles.push_back(inputTriangles[i]);
					else rightTriangles.push_back(inputTriangles[i]);
				}
			}
			
			// 创建左右子节点并递归构建
			l = new BVHNode();
			r = new BVHNode();
			
			l->build(leftTriangles, outputTriangles);
			r->build(rightTriangles, outputTriangles);
			return;
		}
		
		// 使用SAH
		float bestCost = FLT_MAX;
		int bestAxis = -1;
		int bestBin = -1;
		
		// 遍历三个轴
		for (int axis = 0; axis < 3; axis++)
		{
			
			float minCoord, maxCoord;
			if (axis == 0) { minCoord = bounds.min.x; maxCoord = bounds.max.x; }
			else if (axis == 1) { minCoord = bounds.min.y; maxCoord = bounds.max.y; }
			else { minCoord = bounds.min.z; maxCoord = bounds.max.z; }
			
			float range = maxCoord - minCoord;
			if (range <= 0) continue;  // 跳过扁平轴
			
			// 创建bins
			struct Bin {
				AABB bounds;
				int count = 0;
			};
			
			std::vector<Bin> bins(USE_BUILD_BINS);
			for (int i = 0; i < USE_BUILD_BINS; i++) {
				bins[i].bounds.reset();
			}
			
			// 将三角形分配到对应的bin
			for (int i = 0; i < inputTriangles.size(); i++)
			{
				Vec3 centroid = inputTriangles[i].centre();
				float coord;
				if (axis == 0) coord = centroid.x;
				else if (axis == 1) coord = centroid.y;
				else coord = centroid.z;
				
				int binIndex = std::min(USE_BUILD_BINS - 1, static_cast<int>((coord - minCoord) / range * USE_BUILD_BINS));
				
				bins[binIndex].count++;
				bins[binIndex].bounds.extend(inputTriangles[i].vertices[0].p);
				bins[binIndex].bounds.extend(inputTriangles[i].vertices[1].p);
				bins[binIndex].bounds.extend(inputTriangles[i].vertices[2].p);
			}
			
			// 从左到右计算累积包围盒
			std::vector<AABB> rightBoxes(USE_BUILD_BINS);
			std::vector<int> rightCounts(USE_BUILD_BINS);
			
			rightBoxes[USE_BUILD_BINS - 1] = bins[USE_BUILD_BINS - 1].bounds;
			rightCounts[USE_BUILD_BINS - 1] = bins[USE_BUILD_BINS - 1].count;
			
			for (int i = USE_BUILD_BINS - 2; i >= 0; i--)
			{
				rightBoxes[i] = rightBoxes[i + 1];
				rightBoxes[i].extend(bins[i].bounds.min);
				rightBoxes[i].extend(bins[i].bounds.max);
				rightCounts[i] = rightCounts[i + 1] + bins[i].count;
			}
			
			// 从左向右累积，评估每个分割点
			AABB leftBox;
			int leftCount = 0;
			
			for (int i = 0; i < USE_BUILD_BINS - 1; i++)
			{
				if (bins[i].count == 0) continue;
				
				leftBox.extend(bins[i].bounds.min);
				leftBox.extend(bins[i].bounds.max);
				leftCount += bins[i].count;
				
				if (leftCount == 0 || rightCounts[i + 1] == 0) continue;
				
				// 计算SAH代价
				float leftArea = leftBox.area();
				float rightArea = rightBoxes[i + 1].area();
				float totalArea = bounds.area();
				
				float cost = USE_TRAVERSE_COST + USE_TRIANGLE_COST * (
					(leftArea / totalArea) * leftCount + 
					(rightArea / totalArea) * rightCounts[i + 1]
				);
				
				// 更新最佳分割
				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestBin = i;
				}
			}
		}
		
		// 没有找到好的分割或者分割代价比直接处理所有三角形还高
		float noSplitCost = USE_TRIANGLE_COST * inputTriangles.size();
		
		// 使用纯粹的SAH决策，不强制分割
		if (bestAxis == -1 || bestCost >= noSplitCost)
		{
			// 存儲三角形在輸出數組中的索引
			int startIndex = outputTriangles.size();
			for (int i = 0; i < inputTriangles.size(); i++)
			{
				outputTriangles.push_back(inputTriangles[i]);
				triangleIndices.push_back(startIndex + i);
			}
			return;
		}
		
		// 根据最佳分割分组
		float minCoord, maxCoord;
		if (bestAxis == 0) { minCoord = bounds.min.x; maxCoord = bounds.max.x; }
		else if (bestAxis == 1) { minCoord = bounds.min.y; maxCoord = bounds.max.y; }
		else { minCoord = bounds.min.z; maxCoord = bounds.max.z; }
		
		float range = maxCoord - minCoord;
		float splitPos = minCoord + range * (bestBin + 1) / USE_BUILD_BINS;
		
		std::vector<Triangle> leftTriangles;
		std::vector<Triangle> rightTriangles;
		
		for (int i = 0; i < inputTriangles.size(); i++)
		{
			Vec3 centroid = inputTriangles[i].centre();
			float coord;
			if (bestAxis == 0) coord = centroid.x;
			else if (bestAxis == 1) coord = centroid.y;
			else coord = centroid.z;
			
			if (coord < splitPos)
				leftTriangles.push_back(inputTriangles[i]);
			else
				rightTriangles.push_back(inputTriangles[i]);
		}
		
		// 处理特殊情况：所有三角形都在一侧
		if (leftTriangles.size() == 0 || rightTriangles.size() == 0)
		{
			// 简单地将三角形平分到两边
			leftTriangles.clear();
			rightTriangles.clear();
			
			int mid = inputTriangles.size() / 2;
			for (int i = 0; i < inputTriangles.size(); i++)
			{
				if (i < mid)
					leftTriangles.push_back(inputTriangles[i]);
				else
					rightTriangles.push_back(inputTriangles[i]);
			}
		}
		
		// 创建左右子节点并递归构建
		l = new BVHNode();
		r = new BVHNode();
		
		l->build(leftTriangles, outputTriangles);
		r->build(rightTriangles, outputTriangles);
	}
	
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// 如果射线与当前节点的AABB不相交，直接返回
		if (!bounds.rayAABB(ray))
			return;
		
		// 是叶子节点，遍历所有三角形
		if (l == NULL && r == NULL)
		{
			// 只遍歷當前節點包含的三角形
			for (int i = 0; i < triangleIndices.size(); i++)
			{
				int triIndex = triangleIndices[i];
				float t, u, v;
				if (triangles[triIndex].rayIntersect(ray, t, u, v))
				{
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = triIndex;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			}
			return;
		}
		
		// 计算射线到左右子节点的距离，先访问更近的节点
		float tLeft = FLT_MAX;
		float tRight = FLT_MAX;
		bool hitLeft = (l != NULL) ? l->bounds.rayAABB(ray, tLeft) : false;
		bool hitRight = (r != NULL) ? r->bounds.rayAABB(ray, tRight) : false;
		
		// 根据距离选择遍历顺序
		if (hitLeft && hitRight)
		{
			// 两个子节点都相交，先遍历较近的
			if (tLeft <= tRight)
			{
				l->traverse(ray, triangles, intersection);
				// 如果已经找到交点，且交点距离小于另一个节点的距离，就不用遍历另一个节点了
				if (intersection.t > tRight)
					r->traverse(ray, triangles, intersection);
			}
			else
			{
				r->traverse(ray, triangles, intersection);
				if (intersection.t > tLeft)
					l->traverse(ray, triangles, intersection);
			}
		}
		else
		{
			// 只有一个子节点相交，或者都不相交
			if (hitLeft)
				l->traverse(ray, triangles, intersection);
			if (hitRight)
				r->traverse(ray, triangles, intersection);
		}
	}
	
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// 如果射線與當前節點的AABB不相交，表示沒有遮擋
		float t;
		if (!bounds.rayAABB(ray, t) || t > maxT)
			return true;
		
		// 是叶子节点，遍历所有三角形
		if (l == NULL && r == NULL)
		{
			// 只遍歷當前節點包含的三角形
			for (int i = 0; i < triangleIndices.size(); i++)
			{
				int triIndex = triangleIndices[i];
				float t, u, v;
				if (triangles[triIndex].rayIntersect(ray, t, u, v))
				{
					// 如果相交点在最大距离内，表示有遮挡
					if (t < maxT)
						return false;
				}
			}
			return true;
		}
		
		// 先检查较近的子节点
		float tLeft = FLT_MAX;
		float tRight = FLT_MAX;
		bool hitLeft = (l != NULL) ? l->bounds.rayAABB(ray, tLeft) : false;
		bool hitRight = (r != NULL) ? r->bounds.rayAABB(ray, tRight) : false;
		
		// 根据距离选择遍历顺序
		if (hitLeft && hitRight)
		{
			// 两个子节点都相交，先遍历较近的
			if (tLeft <= tRight)
			{
				// 如果近节点返回false(有遮挡)，直接返回false
				if (!l->traverseVisible(ray, triangles, maxT))
					return false;
				// 否则检查远节点
				return r->traverseVisible(ray, triangles, maxT);
			}
			else
			{
				if (!r->traverseVisible(ray, triangles, maxT))
					return false;
				return l->traverseVisible(ray, triangles, maxT);
			}
		}
		else
		{
			// 只有一个子节点相交，或者都不相交
			if (hitLeft)
				return l->traverseVisible(ray, triangles, maxT);
			if (hitRight)
				return r->traverseVisible(ray, triangles, maxT);
			return true;
		}
	}
	
	// 获取BVH树的深度
	int getDepth() const {
		if (l == NULL && r == NULL) {
			return 1; // 叶子节点
		}
		
		int leftDepth = (l != NULL) ? l->getDepth() : 0;
		int rightDepth = (r != NULL) ? r->getDepth() : 0;
		
		return 1 + ((leftDepth > rightDepth) ? leftDepth : rightDepth);
	}
	
	// 获取BVH树的节点数量
	int getNodeCount() const {
		int count = 1; // 当前节点
		
		if (l != NULL) count += l->getNodeCount();
		if (r != NULL) count += r->getNodeCount();
		
		return count;
	}
};
