#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
};

class ShadingHelper
{
public:
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Calculate refracted direction
		// Calculate Fresnel (|| and T)
		// return average^2
		return 1.0f;
	}
	static Colour fresnelCondutor(float cosTheta, Colour ior, Colour k)
	{
		// Calculate Fresnel term for Conductors here
		return Colour(1.0f, 1.0f, 1.0f);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		// 使用余弦加权半球采样
		float r1 = sampler->next();
		float r2 = sampler->next();
		
		// 使用余弦半球采样获取本地坐标中的方向
		Vec3 localWi = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		
		// 计算PDF
		pdf = SamplingDistributions::cosineHemispherePDF(localWi);
		
		// 计算反射颜色
		indirect = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		
		// 将本地坐标转换到世界坐标
		Vec3 wi = shadingData.frame.toWorld(localWi);
		
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct evaluation code here
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// 将世界坐标转换回本地坐标
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 如果在背面，PDF为0
		if (localWi.z <= 0.0f) return 0.0f;
		
		// 计算余弦半球PDF
		return SamplingDistributions::cosineHemispherePDF(localWi);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		// 计算完美镜面反射方向
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = Vec3(-wo.x, -wo.y, wo.z); // 镜面反射
		
		// 镜面BSDF的PDF是delta分布
		pdf = 1.0f;
		
		// 反射颜色
		indirect = albedo->sample(shadingData.tu, shadingData.tv);
		
		// 转换回世界坐标
		return shadingData.frame.toWorld(localWi);
	}
	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// 镜面是delta分布，evaluate总是返回0
		// 实际贡献通过sample函数计算
		return Colour(0.0f, 0.0f, 0.0f);
	}
	
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// 镜面是delta分布，PDF无法直接计算
        // 对于delta分布，PDF应该是无穷大
        // 但在实际实现中，我们返回0，因为这个PDF只会在MIS中使用
        // 而MIS会跳过delta分布
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		// 转换入射方向到局部坐标
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		
		// 采样微表面法线
		float r1 = sampler->next();
		float r2 = sampler->next();
		
		// 采样GGX分布获取微表面法线
		float phi = 2.0f * M_PI * r1;
		float cosTheta = sqrtf((1.0f - r2) / (1.0f + (alpha*alpha - 1.0f) * r2));
		float sinTheta = sqrtf(1.0f - cosTheta*cosTheta);
		
		// 微表面法线 (注意：这应该归一化的，但构造方式确保了)
		Vec3 m = Vec3(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
		
		// 计算反射方向
		Vec3 localWi = Vec3(2.0f * Dot(wo, m) * m.x - wo.x,
                           2.0f * Dot(wo, m) * m.y - wo.y,
                           2.0f * Dot(wo, m) * m.z - wo.z);
		
		// 确保反射在正确的半球
		if (localWi.z * wo.z <= 0.0f) {
			// 反射到错误的半球，返回无效采样
			pdf = 0.0f;
			indirect = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 1.0f);
		}
		
		// 计算PDF (微表面法线PDF到反射方向PDF的转换)
		float D = ShadingHelper::Dggx(m, alpha);
		float jacobian = 1.0f / (4.0f * Dot(wo, m));
		pdf = D * cosTheta * jacobian;
		
		// 计算BSDF值
		// F * D * G / (4 * cos(wo) * cos(wi))
		float G = ShadingHelper::Gggx(wo, localWi, alpha);
		Colour F = ShadingHelper::fresnelCondutor(Dot(wo, m), eta, k);
		
		indirect = albedo->sample(shadingData.tu, shadingData.tv) * F * (D * G / (4.0f * wo.z * localWi.z));
		
		// 转换回世界坐标
		return shadingData.frame.toWorld(localWi);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// 转换到局部坐标
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 确保在同一半球
		if (wo.z * localWi.z <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);
		
		// 计算半程向量
		Vec3 h = Vec3(wo.x + localWi.x, wo.y + localWi.y, wo.z + localWi.z);
		float length = sqrtf(h.x*h.x + h.y*h.y + h.z*h.z);
		h = Vec3(h.x / length, h.y / length, h.z / length); // 归一化
		
		// 计算BSDF值
		// F * D * G / (4 * cos(wo) * cos(wi))
		float D = ShadingHelper::Dggx(h, alpha);
		float G = ShadingHelper::Gggx(wo, localWi, alpha);
		Colour F = ShadingHelper::fresnelCondutor(Dot(wo, h), eta, k);
		
		return albedo->sample(shadingData.tu, shadingData.tv) * F * (D * G / (4.0f * wo.z * localWi.z));
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// 转换到局部坐标
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 确保在同一半球
		if (wo.z * localWi.z <= 0.0f) return 0.0f;
		
		// 计算半程向量
		Vec3 h = Vec3(wo.x + localWi.x, wo.y + localWi.y, wo.z + localWi.z);
		float length = sqrtf(h.x*h.x + h.y*h.y + h.z*h.z);
		h = Vec3(h.x / length, h.y / length, h.z / length); // 归一化
		
		// 计算微表面法线的PDF
		float cosTheta = h.z;
		float D = ShadingHelper::Dggx(h, alpha);
		
		// 从微表面法线PDF转换到反射方向PDF
		float jacobian = 1.0f / (4.0f * Dot(wo, h));
		
		return D * cosTheta * jacobian;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		// 转换入射方向到局部坐标
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		
		// 计算入射角余弦值
		float cosTheta = wo.z;
		
		// 计算菲涅尔项，也就是反射的概率
		float F = ShadingHelper::fresnelDielectric(fabsf(cosTheta), intIOR, extIOR);
		
		// 随机决定是反射还是折射
		if (sampler->next() < F)
		{
			// 反射
			Vec3 localWi = Vec3(-wo.x, -wo.y, wo.z); // 完美镜面反射
			pdf = F;  // PDF等于选择反射的概率
			indirect = albedo->sample(shadingData.tu, shadingData.tv);
			return shadingData.frame.toWorld(localWi);
		}
		else
		{
			// 折射
			float eta = cosTheta > 0 ? extIOR/intIOR : intIOR/extIOR;
			float cosThetaT;
			Vec3 localWi;
			
			// 计算折射方向
			if (!refract(wo, eta, cosThetaT, localWi))
			{
				// 全内反射
				localWi = Vec3(-wo.x, -wo.y, wo.z);
		pdf = 1.0f;
			}
			else
			{
				pdf = 1.0f - F;  // PDF等于选择折射的概率
			}
			
			indirect = albedo->sample(shadingData.tu, shadingData.tv);
			return shadingData.frame.toWorld(localWi);
		}
	}
	
	// 帮助函数：计算折射方向
	bool refract(const Vec3& wi, float eta, float& cosThetaT, Vec3& wt)
	{
		float cosThetaI = wi.z;
		float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI*cosThetaI);
		float sin2ThetaT = eta * eta * sin2ThetaI;
		
		// 全内反射检查
		if (sin2ThetaT >= 1.0f) return false;
		
		cosThetaT = sqrtf(1.0f - sin2ThetaT);
		if (cosThetaI > 0)
			wt = Vec3(-eta * wi.x, -eta * wi.y, -cosThetaT);
		else
			wt = Vec3(-eta * wi.x, -eta * wi.y, cosThetaT);
		
		return true;
	}
	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// 玻璃是delta分布，evaluate总是返回0
		return Colour(0.0f, 0.0f, 0.0f);
	}
	
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// 玻璃是delta分布，PDF在采样中已经计算
		// 用于MIS时，我们返回0，因为MIS会跳过delta分布
		return 0.0f;
	}
	
	bool isPureSpecular()
	{
		return true;
	}
	
	bool isTwoSided()
	{
		return false;
	}
	
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		// 转换入射方向到局部坐标
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		
		// 采样微表面法线
		float r1 = sampler->next();
		float r2 = sampler->next();
		
		// 采样GGX分布获取微表面法线
		float phi = 2.0f * M_PI * r1;
		float cosTheta = sqrtf((1.0f - r2) / (1.0f + (alpha*alpha - 1.0f) * r2));
		float sinTheta = sqrtf(1.0f - cosTheta*cosTheta);
		
		// 微表面法线
		Vec3 m = Vec3(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
		
		// 计算菲涅尔项
		float F = ShadingHelper::fresnelDielectric(fabsf(Dot(wo, m)), intIOR, extIOR);
		
		// 计算GGX分布D值
		float D = ShadingHelper::Dggx(m, alpha);
		
		// 随机决定是反射还是折射
		if (sampler->next() < F)
		{
			// 反射
			Vec3 localWi = Vec3(2.0f * Dot(wo, m) * m.x - wo.x,
                              2.0f * Dot(wo, m) * m.y - wo.y,
                              2.0f * Dot(wo, m) * m.z - wo.z);
			
			// 确保反射在正确的半球
			if (localWi.z * wo.z <= 0.0f) {
				pdf = 0.0f;
				indirect = Colour(0.0f, 0.0f, 0.0f);
				return Vec3(0.0f, 0.0f, 1.0f);
			}
			
			// 计算PDF
			float jacobian = 1.0f / (4.0f * Dot(wo, m));
			pdf = D * cosTheta * jacobian * F; // 乘以选择反射的概率
			
			indirect = albedo->sample(shadingData.tu, shadingData.tv);
			return shadingData.frame.toWorld(localWi);
		}
		else
		{
			// 折射
			float eta = (wo.z > 0) ? extIOR/intIOR : intIOR/extIOR;
			float cosThetaT;
			Vec3 localWi;
			
			// 尝试计算折射方向
			bool canRefract = refract(wo, m, eta, localWi);
			if (!canRefract)
			{
				// 全内反射，当作反射处理
				localWi = Vec3(2.0f * Dot(wo, m) * m.x - wo.x,
                            2.0f * Dot(wo, m) * m.y - wo.y,
                            2.0f * Dot(wo, m) * m.z - wo.z);
		pdf = 1.0f;
			}
			else
			{
				// 折射成功
				pdf = D * cosTheta * (1.0f - F); // PDF乘以选择折射的概率
			}
			
			indirect = albedo->sample(shadingData.tu, shadingData.tv);
			return shadingData.frame.toWorld(localWi);
		}
	}
	
	// 帮助函数：计算折射方向
	bool refract(const Vec3& wi, const Vec3& n, float eta, Vec3& wt)
	{
		float cosThetaI = Dot(wi, n);
		float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI*cosThetaI);
		float sin2ThetaT = eta * eta * sin2ThetaI;
		
		// 全内反射检查
		if (sin2ThetaT >= 1.0f) return false;
		
		float cosThetaT = sqrtf(1.0f - sin2ThetaT);
		wt = Vec3(eta * (wi.x - n.x * cosThetaI) - n.x * cosThetaT,
               eta * (wi.y - n.y * cosThetaI) - n.y * cosThetaT,
               eta * (wi.z - n.z * cosThetaI) - n.z * cosThetaT);
		
		return true;
	}
	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// 与GGX + 菲涅尔反射/折射相关的BSDF评估
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 检查是反射还是折射
		bool isReflection = wo.z * localWi.z > 0.0f;
		
		if (isReflection)
		{
			// 计算半程向量
			Vec3 h = Vec3(wo.x + localWi.x, wo.y + localWi.y, wo.z + localWi.z);
			float length = sqrtf(h.x*h.x + h.y*h.y + h.z*h.z);
			h = Vec3(h.x / length, h.y / length, h.z / length); // 归一化
			
			// 计算BSDF
			float D = ShadingHelper::Dggx(h, alpha);
			float G = ShadingHelper::Gggx(wo, localWi, alpha);
			float F = ShadingHelper::fresnelDielectric(fabsf(Dot(wo, h)), intIOR, extIOR);
			
			return albedo->sample(shadingData.tu, shadingData.tv) * F * (D * G / (4.0f * fabsf(wo.z * localWi.z)));
		}
		else
		{
			// 折射情况更复杂，这里简化处理
			return Colour(0.0f, 0.0f, 0.0f);
		}
	}
	
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// 与GGX + 菲涅尔反射/折射相关的PDF
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 检查是反射还是折射
		bool isReflection = wo.z * localWi.z > 0.0f;
		
		if (isReflection)
		{
			// 计算半程向量
			Vec3 h = Vec3(wo.x + localWi.x, wo.y + localWi.y, wo.z + localWi.z);
			float length = sqrtf(h.x*h.x + h.y*h.y + h.z*h.z);
			h = Vec3(h.x / length, h.y / length, h.z / length); // 归一化
			
			// 计算反射PDF
			float cosTheta = h.z;
			float D = ShadingHelper::Dggx(h, alpha);
			float jacobian = 1.0f / (4.0f * Dot(wo, h));
			float F = ShadingHelper::fresnelDielectric(fabsf(Dot(wo, h)), intIOR, extIOR);
			
			return D * cosTheta * jacobian * F; // 乘以选择反射的概率
		}
		else
		{
			// 折射情况更复杂，这里简化处理
			return 0.0f;
		}
	}
	
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		// 使用与漫反射相同的余弦加权半球采样
		float r1 = sampler->next();
		float r2 = sampler->next();
		
		// 使用余弦半球采样获取本地坐标中的方向
		Vec3 localWi = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		
		// 计算PDF
		pdf = SamplingDistributions::cosineHemispherePDF(localWi);
		
		// 计算反射颜色（Oren-Nayar模型）
		indirect = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		
		// 将本地坐标转换到世界坐标
		Vec3 wi = shadingData.frame.toWorld(localWi);
		
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// 将方向转换到局部坐标
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 确保在正确的半球
		if (localWi.z <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);
		
		// 计算Oren-Nayar反射模型
		float sigma2 = sigma * sigma;
		float A = 1.0f - 0.5f * (sigma2 / (sigma2 + 0.33f));
		float B = 0.45f * (sigma2 / (sigma2 + 0.09f));
		
		// 计算角度
		float thetai = acosf(localWi.z);
		float thetao = acosf(wo.z);
		
		// 计算方位角差
		float cosphi = 0.0f;
		if (thetai > 0.0001f && thetao > 0.0001f) {
			cosphi = (localWi.x * wo.x + localWi.y * wo.y) / 
					(sqrtf(localWi.x * localWi.x + localWi.y * localWi.y) * 
					 sqrtf(wo.x * wo.x + wo.y * wo.y));
			cosphi = std::max(-1.0f, std::min(1.0f, cosphi));
		}
		
		// 计算alpha和beta
		float alpha = std::max(thetai, thetao);
		float beta = std::min(thetai, thetao);
		
		// Oren-Nayar反射方程
		float oren_nayar = A + B * std::max(0.0f, cosphi) * sinf(alpha) * tanf(beta);
		
		// 最终反射颜色
		return albedo->sample(shadingData.tu, shadingData.tv) * (oren_nayar / M_PI);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// OrenNayar使用与漫反射相同的采样策略，所以PDF也相同
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 如果在背面，PDF为0
		if (localWi.z <= 0.0f) return 0.0f;
		
		// 计算余弦半球PDF
		return SamplingDistributions::cosineHemispherePDF(localWi);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		// 塑料材质包含漫反射和高光两个成分
		// 需要决定采样哪个成分
		
		// 假设漫反射比例为0.5，高光比例为0.5
		float diffuseRatio = 0.5f;
		
		if (sampler->next() < diffuseRatio)
		{
			// 采样漫反射成分
			float r1 = sampler->next();
			float r2 = sampler->next();
			
			// 使用余弦半球采样
			Vec3 localWi = SamplingDistributions::cosineSampleHemisphere(r1, r2);
			pdf = SamplingDistributions::cosineHemispherePDF(localWi) * diffuseRatio;
			
			// 计算漫反射部分
			indirect = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
			
			return shadingData.frame.toWorld(localWi);
		}
		else
		{
			// 采样高光成分
			Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
			
			// 采样微表面法线
			float r1 = sampler->next();
			float r2 = sampler->next();
			
			// 采样GGX分布
			float phi = 2.0f * M_PI * r1;
			float cosTheta = sqrtf((1.0f - r2) / (1.0f + (alpha*alpha - 1.0f) * r2));
			float sinTheta = sqrtf(1.0f - cosTheta*cosTheta);
			
			Vec3 m = Vec3(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
			
			// 计算反射方向
			Vec3 localWi = Vec3(2.0f * Dot(wo, m) * m.x - wo.x,
                              2.0f * Dot(wo, m) * m.y - wo.y,
                              2.0f * Dot(wo, m) * m.z - wo.z);
			
			// 确保反射在正确的半球
			if (localWi.z <= 0.0f) {
				pdf = 0.0f;
				indirect = Colour(0.0f, 0.0f, 0.0f);
				return Vec3(0.0f, 0.0f, 1.0f);
			}
			
			// 计算高光PDF
			float D = ShadingHelper::Dggx(m, alpha);
			float jacobian = 1.0f / (4.0f * Dot(wo, m));
			pdf = D * cosTheta * jacobian * (1.0f - diffuseRatio);
			
			// 计算菲涅尔项
			float F = ShadingHelper::fresnelDielectric(fabsf(Dot(wo, m)), intIOR, extIOR);
			
			// 计算高光贡献
			float G = ShadingHelper::Gggx(wo, localWi, alpha);
			float result = F * D * G / (4.0f * wo.z * localWi.z);
			indirect = Colour(result, result, result); // 使用相同的RGB值
			
			return shadingData.frame.toWorld(localWi);
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 确保在上半球
		if (localWi.z <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);
		
		// 计算漫反射部分
		Colour diffuse = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		
		// 计算高光部分
		// 计算半程向量
		Vec3 h = Vec3(wo.x + localWi.x, wo.y + localWi.y, wo.z + localWi.z);
		float length = sqrtf(h.x*h.x + h.y*h.y + h.z*h.z);
		h = Vec3(h.x / length, h.y / length, h.z / length); // 归一化
		
		float D = ShadingHelper::Dggx(h, alpha);
		float G = ShadingHelper::Gggx(wo, localWi, alpha);
		float F = ShadingHelper::fresnelDielectric(fabsf(Dot(wo, h)), intIOR, extIOR);
		
		float result = F * D * G / (4.0f * wo.z * localWi.z);
		Colour specular = Colour(result, result, result); // 使用相同的RGB值
		
		// 返回漫反射和高光的组合
		return diffuse + specular;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 localWi = shadingData.frame.toLocal(wi);
		
		// 确保在上半球
		if (localWi.z <= 0.0f) return 0.0f;
		
		// 假设漫反射比例为0.5，高光比例为0.5
		float diffuseRatio = 0.5f;
		
		// 计算漫反射PDF
		float diffusePdf = SamplingDistributions::cosineHemispherePDF(localWi) * diffuseRatio;
		
		// 计算高光PDF
		// 计算半程向量
		Vec3 h = Vec3(wo.x + localWi.x, wo.y + localWi.y, wo.z + localWi.z);
		float length = sqrtf(h.x*h.x + h.y*h.y + h.z*h.z);
		h = Vec3(h.x / length, h.y / length, h.z / length); // 归一化
		
		float D = ShadingHelper::Dggx(h, alpha);
		float cosTheta = h.z;
		float jacobian = 1.0f / (4.0f * Dot(wo, h));
		float specularPdf = D * cosTheta * jacobian * (1.0f - diffuseRatio);
		
		// 返回两种PDF的组合
		return diffusePdf + specularPdf;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& indirect, float& pdf)
	{
		return base->sample(shadingData, sampler, indirect, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};