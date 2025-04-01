#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	
	// 添加初始化方法
	void init(unsigned int seed)
	{
		generator.seed(seed);
	}
	
	float next()
	{
		return dist(generator);
	}
	
	// 添加静态随机函数，用于光子映射
	static float randomFloat()
	{
		static std::mt19937 staticGen(42);
		static std::uniform_real_distribution<float> staticDist(0.0f, 1.0f);
		return staticDist(staticGen);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// 基于余弦加权的半球采样
		float phi = 2.0f * M_PI * r1;
		float cosTheta = sqrtf(r2);
		float sinTheta = sqrtf(1.0f - r2);
		
		float x = sinTheta * cosf(phi);
		float y = sinTheta * sinf(phi);
		float z = cosTheta;
		
		return Vec3(x, y, z);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// 余弦加权的PDF = cos(theta) / pi
		return wi.z > 0.0f ? wi.z / M_PI : 0.0f;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// 均匀球体采样
		float phi = 2.0f * M_PI * r1;
		float cosTheta = 1.0f - 2.0f * r2;
		float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
		
		float x = sinTheta * cosf(phi);
		float y = sinTheta * sinf(phi);
		float z = cosTheta;
		
		return Vec3(x, y, z);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// 均匀球体的PDF = 1 / (4 * pi)
		return 1.0f / (4.0f * M_PI);
	}
	
	// 平衡启发式(balance heuristic)的MIS权重计算
	static float balanceHeuristic(float pdfA, float pdfB)
	{
		if (pdfA <= 0.0f) return 0.0f;
		return pdfA / (pdfA + pdfB);
	}
	
	// 幂启发式(power heuristic)的MIS权重计算，默认使用beta=2
	static float powerHeuristic(float pdfA, float pdfB, float beta = 2.0f)
	{
		if (pdfA <= 0.0f) return 0.0f;
		float pdfAPow = powf(pdfA, beta);
		float pdfBPow = powf(pdfB, beta);
		return pdfAPow / (pdfAPow + pdfBPow);
	}
};