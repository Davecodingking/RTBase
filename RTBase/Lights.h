#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Sampling.h"
#include <vector>
#include <algorithm> // For std::lower_bound

#pragma warning( disable : 4244)

// Forward declaration
class BSDF;
class ShadingData;

// Ensure use<SceneBounds>() can be used
// SceneBounds is defined in Core.h

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
	// Add data structures needed for importance sampling
	std::vector<float> marginalCDF; // Cumulative distribution function in y direction (rows)
	std::vector<std::vector<float>> conditionalCDF; // Conditional CDF in x direction for each row
	std::vector<float> rowWeights; // Total weight of each row
	bool enableImportanceSampling; // Control whether to use importance sampling
	float brightnessScale; // Environment light brightness scale factor to control overall brightness
	float importanceSamplingStrength; // Importance sampling strength, controls sampling bias towards bright areas
	float minSamplingWeight; // Minimum sampling weight, prevents dark areas from being completely ignored
	bool useCosineWeighting; // Whether to use cosine weighted sampling

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		enableImportanceSampling = true; // Default enable importance sampling
		brightnessScale = 1.0f; // Default brightness scale 1.0, no reduction in environment light intensity
		importanceSamplingStrength = 1.0f; // Default importance sampling strength
		minSamplingWeight = 0.1f; // Default minimum sampling weight
		useCosineWeighting = true; // Default use cosine weighting
		// Precompute luminance-based CDF for importance sampling
		buildDistributions();
	}
	
	// Build distributions for importance sampling
	void buildDistributions() 
	{
		if (env == nullptr || env->width <= 0 || env->height <= 0) return;
		
		// Adjust data structure size
		conditionalCDF.resize(env->height);
		marginalCDF.resize(env->height + 1);
		rowWeights.resize(env->height);
		
		// Calculate PDF and CDF for each row
		for (int v = 0; v < env->height; ++v) {
			// Weight of pixels in this row is proportional to sin(theta)
			float sinTheta = sinf(((float)v + 0.5f) / (float)env->height * M_PI);
			
			// Allocate space for current row
			conditionalCDF[v].resize(env->width + 1);
			conditionalCDF[v][0] = 0;
			
			// Accumulate luminance values for this row
			float rowWeight = 0;
			for (int u = 0; u < env->width; ++u) {
				Colour pixel = env->texels[v * env->width + u];
				float weight = pixel.Lum();
				
				// Apply importance sampling strength
				weight = std::pow(weight, importanceSamplingStrength);
				
				// Apply minimum sampling weight
				weight = std::max(weight, minSamplingWeight);
				
				// Apply cosine weighting
				if (useCosineWeighting) {
					weight *= sinTheta;
				}
				
				rowWeight += weight;
				conditionalCDF[v][u + 1] = conditionalCDF[v][u] + weight;
			}
			
			// Normalize CDF for this row
			if (rowWeight > 0) {
				for (int u = 1; u <= env->width; ++u) {
					conditionalCDF[v][u] /= rowWeight;
				}
			}
			
			rowWeights[v] = rowWeight;
			marginalCDF[v + 1] = marginalCDF[v] + rowWeight;
		}
		
		// Normalize vertical CDF
		if (marginalCDF[env->height] > 0) {
			for (int v = 1; v <= env->height; ++v) {
				marginalCDF[v] /= marginalCDF[env->height];
			}
		}
	}
	
	// Find index in CDF based on uniform random variable
	int sampleDiscrete(const std::vector<float>& cdf, float u, float& pdf) {
		// Find first position in cdf greater than u
		auto it = std::lower_bound(cdf.begin(), cdf.end(), u);
		int index = (int)std::distance(cdf.begin(), it) - 1;
		index = (index < 0) ? 0 : ((index > (int)cdf.size() - 2) ? (int)cdf.size() - 2 : index);
		
		// Calculate probability of selecting this index
		pdf = (cdf[index + 1] - cdf[index]);
		return index;
	}
	
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		if (env == nullptr || env->width <= 0 || env->height <= 0 || !enableImportanceSampling || marginalCDF.empty()) {
			// Fallback to uniform sampling
			Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::uniformSpherePDF(wi);
			reflectedColour = evaluate(shadingData, wi);
			return wi;
		}
		
		// Importance sampling
		float pdfV, pdfU;
		
		// Sample v coordinate (row)
		int v = sampleDiscrete(marginalCDF, sampler->next(), pdfV);
		
		// Sample u coordinate (column)
		int u = sampleDiscrete(conditionalCDF[v], sampler->next(), pdfU);
		
		// Calculate spherical coordinates
		float phi = (u + sampler->next()) / (float)env->width * 2.0f * M_PI;
		float theta = (v + sampler->next()) / (float)env->height * M_PI;
		
		// Convert to Cartesian coordinates
		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);
		
		Vec3 wi(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);
		
		// Calculate pdf (note spherical area element jacobian)
		float envPdf = pdfV * pdfU / (2.0f * M_PI * M_PI * sinTheta);
		pdf = envPdf > 0 ? envPdf : SamplingDistributions::uniformSpherePDF(wi);
		
		// Get environment map color, apply brightness scale
		reflectedColour = evaluate(shadingData, wi);
		
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		// Apply brightness scale factor, but do not automatically adjust brightness
		return env->sample(u, v) * brightnessScale;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		if (env == nullptr || env->width <= 0 || env->height <= 0 || !enableImportanceSampling || marginalCDF.empty()) {
			return SamplingDistributions::uniformSpherePDF(wi);
		}
		
		// Calculate texture coordinates
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0) phi += 2.0f * M_PI;
		float y_clamped = (wi.y < -1.0f) ? -1.0f : ((wi.y > 1.0f) ? 1.0f : wi.y);
		float theta = acosf(y_clamped);
		
		// Map to pixel coordinates and ensure within valid range
		int ui = std::min((int)(phi / (2.0f * M_PI) * env->width), env->width - 1);
		int vi = std::min((int)(theta / M_PI * env->height), env->height - 1);
		
		float sinTheta = sinf(theta);
		// If sin(theta) is close to 0, corresponding to the poles of the sphere, return uniform PDF
		if (sinTheta < 1e-5f) return SamplingDistributions::uniformSpherePDF(wi);
		
		// Calculate PDF from cumulative distribution function
		float rowPdf = (marginalCDF[vi + 1] - marginalCDF[vi]) * env->height;
		float colPdf = 0.0f;
		if (vi < conditionalCDF.size() && ui < env->width) {
			colPdf = (conditionalCDF[vi][ui + 1] - conditionalCDF[vi][ui]) * env->width;
		}
		
		// Calculate final PDF, including spherical area element jacobian
		float pdf = (rowPdf * colPdf) / (2.0f * M_PI * M_PI * sinTheta);
		return (pdf < 0.0f) ? 0.0f : pdf;  // Ensure PDF is non-negative
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
		// Apply brightness scale, but do not automatically adjust brightness
		return total * 4.0f * M_PI * brightnessScale;
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
			// Fallback to uniform sampling
			Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::uniformSpherePDF(wi);
			return wi;
		}
		
		// Use same importance sampling technique as sample method
		float pdfV, pdfU;
		
		// Sample v coordinate (row)
		int v = sampleDiscrete(marginalCDF, sampler->next(), pdfV);
		
		// Sample u coordinate (column)
		int u = sampleDiscrete(conditionalCDF[v], sampler->next(), pdfU);
		
		// Calculate spherical coordinates
		float phi = (u + sampler->next()) / (float)env->width * 2.0f * M_PI;
		float theta = (v + sampler->next()) / (float)env->height * M_PI;
		
		// Convert to Cartesian coordinates
		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);
		
		Vec3 wi(sinTheta * cosPhi, cosTheta, sinTheta * sinPhi);
		
		// Calculate pdf (note spherical area element jacobian)
		float envPdf = pdfV * pdfU / (2.0f * M_PI * M_PI * sinTheta);
		pdf = envPdf > 0 ? envPdf : SamplingDistributions::uniformSpherePDF(wi);
		
		return wi;
	}

	// Set importance sampling strength
	void setImportanceSamplingStrength(float strength) {
		importanceSamplingStrength = strength;
		buildDistributions(); // Rebuild distributions
	}

	// Set minimum sampling weight
	void setMinSamplingWeight(float weight) {
		minSamplingWeight = weight;
		buildDistributions(); // Rebuild distributions
	}

	// Set whether to use cosine weighting
	void setUseCosineWeighting(bool use) {
		useCosineWeighting = use;
		buildDistributions(); // Rebuild distributions
	}
};