#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Sampling.h"
#include <vector>
#include <algorithm> // 用于std::lower_bound

#pragma warning( disable : 4244)

// 前向聲明
class BSDF;
class ShadingData;

// 確保能使用use<SceneBounds>()
// SceneBounds在Core.h中定義

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	// 添加重要性采样所需的数据结构
	std::vector<float> marginalCDF; // y方向(行)的累积分布函数
	std::vector<std::vector<float>> conditionalCDF; // 每行x方向的条件累积分布函数
	std::vector<float> rowWeights; // 每行的权重总和
	bool enableImportanceSampling; // 控制是否使用重要性采样

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		enableImportanceSampling = true; // 默认启用重要性采样
		// 预计算基于亮度的CDF用于重要性采样
		buildDistributions();
	}
	
	// 构建用于重要性采样的分布
	void buildDistributions() 
	{
		if (env == nullptr || env->width <= 0 || env->height <= 0) return;
		
		// 调整数据结构大小
		conditionalCDF.resize(env->height);
		marginalCDF.resize(env->height + 1);
		rowWeights.resize(env->height);
		
		// 计算每行的PDF和CDF
		for (int v = 0; v < env->height; ++v) {
			// 该行的像素的权重与sin(theta)成正比
			float sinTheta = sinf(((float)v + 0.5f) / (float)env->height * M_PI);
			
			// 为当前行分配空间
			conditionalCDF[v].resize(env->width + 1);
			conditionalCDF[v][0] = 0;
			
			// 累积该行的亮度值
			float rowWeight = 0;
			for (int u = 0; u < env->width; ++u) {
				Colour pixel = env->texels[v * env->width + u];
				float weight = pixel.Lum() * sinTheta;
				rowWeight += weight;
				conditionalCDF[v][u + 1] = conditionalCDF[v][u] + weight;
			}
			
			// 归一化这一行的CDF
			if (rowWeight > 0) {
				for (int u = 1; u <= env->width; ++u) {
					conditionalCDF[v][u] /= rowWeight;
				}
			}
			
			rowWeights[v] = rowWeight;
			marginalCDF[v + 1] = marginalCDF[v] + rowWeight;
		}
		
		// 归一化纵向的CDF
		if (marginalCDF[env->height] > 0) {
			for (int v = 1; v <= env->height; ++v) {
				marginalCDF[v] /= marginalCDF[env->height];
			}
		}
	}
	
	// 根据均匀随机变量找到CDF中的索引
	int sampleDiscrete(const std::vector<float>& cdf, float u, float& pdf) {
		// 在cdf中找到第一个大于u的位置
		auto it = std::lower_bound(cdf.begin(), cdf.end(), u);
		int index = (int)std::distance(cdf.begin(), it) - 1;
		index = (index < 0) ? 0 : ((index > (int)cdf.size() - 2) ? (int)cdf.size() - 2 : index);
		
		// 计算选择该索引的概率
		pdf = (cdf[index + 1] - cdf[index]);
		return index;
	}
	
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		if (env == nullptr || env->width <= 0 || env->height <= 0 || !enableImportanceSampling || marginalCDF.empty()) {
			// 回退到均匀采样
			Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::uniformSpherePDF(wi);
			reflectedColour = evaluate(shadingData, wi);
			return wi;
		}
		
		// 重要性采样
		float pdfV, pdfU;
		
		// 采样v坐标(行)
		int v = sampleDiscrete(marginalCDF, sampler->next(), pdfV);
		
		// 采样u坐标(列)
		int u = sampleDiscrete(conditionalCDF[v], sampler->next(), pdfU);
		
		// 计算球坐标
		float phi = (u + sampler->next()) / (float)env->width * 2.0f * M_PI;
		float theta = (v + sampler->next()) / (float)env->height * M_PI;
		
		// 转换为笛卡尔坐标
		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);
		
		Vec3 wi(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);
		
		// 计算pdf (注意球面面积元素的jacobian)
		float envPdf = pdfV * pdfU / (2.0f * M_PI * M_PI * sinTheta);
		pdf = envPdf > 0 ? envPdf : SamplingDistributions::uniformSpherePDF(wi);
		
		// 获取环境贴图的颜色
		reflectedColour = evaluate(shadingData, wi);
		
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		if (env == nullptr || env->width <= 0 || env->height <= 0 || !enableImportanceSampling || marginalCDF.empty()) {
			return SamplingDistributions::uniformSpherePDF(wi);
		}
		
		// 计算纹理坐标
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0) phi += 2.0f * M_PI;
		float y_clamped = (wi.y < -1.0f) ? -1.0f : ((wi.y > 1.0f) ? 1.0f : wi.y);
		float theta = acosf(y_clamped);
		
		// 映射到像素坐标，并确保在有效范围内
		int ui = std::min((int)(phi / (2.0f * M_PI) * env->width), env->width - 1);
		int vi = std::min((int)(theta / M_PI * env->height), env->height - 1);
		
		float sinTheta = sinf(theta);
		// 如果sin(theta)接近0，对应球体的极点，返回均匀PDF
		if (sinTheta < 1e-5f) return SamplingDistributions::uniformSpherePDF(wi);
		
		// 从累积分布函数计算该点的PDF
		float rowPdf = (marginalCDF[vi + 1] - marginalCDF[vi]) * env->height;
		float colPdf = 0.0f;
		if (vi < conditionalCDF.size() && ui < env->width) {
			colPdf = (conditionalCDF[vi][ui + 1] - conditionalCDF[vi][ui]) * env->width;
		}
		
		// 计算最终PDF，包括球面面积元素的jacobian
		float pdf = (rowPdf * colPdf) / (2.0f * M_PI * M_PI * sinTheta);
		return (pdf < 0.0f) ? 0.0f : pdf;  // 确保PDF为非负
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		if (env == nullptr || env->width <= 0 || env->height <= 0 || !enableImportanceSampling || marginalCDF.empty()) {
			// 回退到均匀采样
			Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::uniformSpherePDF(wi);
			return wi;
		}
		
		// 使用与sample方法相同的重要性采样技术
		float pdfV, pdfU;
		
		// 采样v坐标(行)
		int v = sampleDiscrete(marginalCDF, sampler->next(), pdfV);
		
		// 采样u坐标(列)
		int u = sampleDiscrete(conditionalCDF[v], sampler->next(), pdfU);
		
		// 计算球坐标
		float phi = (u + sampler->next()) / (float)env->width * 2.0f * M_PI;
		float theta = (v + sampler->next()) / (float)env->height * M_PI;
		
		// 转换为笛卡尔坐标
		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);
		
		Vec3 wi(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);
		
		// 计算pdf (注意球面面积元素的jacobian)
		float envPdf = pdfV * pdfU / (2.0f * M_PI * M_PI * sinTheta);
		pdf = envPdf > 0 ? envPdf : SamplingDistributions::uniformSpherePDF(wi);
		
		return wi;
	}
};