#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <iostream>
#include <vector>
#include <algorithm>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	int renderMode = 3; // 默认使用直接光照模式
	int SPP = 20;       // 默認目標樣本數
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 计算直接光照
		Colour L(0.0f, 0.0f, 0.0f);
		
		// 遍历所有光源
		for (int i = 0; i < scene->lights.size(); i++)
		{
			// 获取光源采样信息
			Colour emissionColour;
			float pdf;
			Vec3 lightDirection = scene->lights[i]->sample(shadingData, sampler, emissionColour, pdf);
			
			// 忽略概率为0的采样
			if (pdf <= 0.0f) continue;
			
			// 计算几何项（方向是否在法线同一侧）
			float nDotWi = Dot(shadingData.sNormal, lightDirection);
			if (nDotWi <= 0.0f) continue;  // 光源在背面，忽略
			
			// 计算打到光源的点
			float lightDistance = 1000.0f; // 假设足够远
			Ray shadowRay;
			shadowRay.init(shadingData.x + lightDirection * EPSILON, lightDirection);
			
			// 检查可见性（阴影测试）
			if (!scene->visible(shadingData.x, shadingData.x + lightDirection * lightDistance)) continue;
			
			// 计算BSDF贡献
			Colour f = shadingData.bsdf->evaluate(shadingData, lightDirection);
			
			// 累加贡献
			L = L + f * emissionColour * (nDotWi / pdf);
		}
		
		return L;
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// 路径追踪算法实现
		if (depth > 10) return Colour(0.0f, 0.0f, 0.0f); // 最大深度限制
		
		// 与场景相交
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t == FLT_MAX)
		{
			// 没有相交，返回背景颜色
			ShadingData shadingData;
			shadingData.wo = -r.dir;
			return scene->background->evaluate(shadingData, r.dir);
		}
		
		// 计算着色数据
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		
		// 如果是光源直接返回发光值
		if (shadingData.bsdf->isLight())
		{
			return shadingData.bsdf->emit(shadingData, shadingData.wo);
		}
		
		// 计算直接光照
		Colour directLight = computeDirect(shadingData, sampler);
		
		// 俄罗斯轮盘赌终止
		float continueProb = 0.9f;
		if (depth > 3 && sampler->next() > continueProb)
		{
			return directLight;
		}
		
		// BSDF采样 - 使用BSDF比例采样而非余弦采样
		Colour indirect;
		float pdf;
		Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
		
		// 无效采样
		if (pdf <= 0.0f || (indirect.r <= 0.0f && indirect.g <= 0.0f && indirect.b <= 0.0f)) 
			return directLight;
		
		// 检查方向是否在表面同侧
		float cosTheta = Dot(shadingData.sNormal, wi);
		if (cosTheta <= 0.0f) return directLight;
		
		// 创建新的光线
		Ray nextRay;
		nextRay.init(shadingData.x + wi * EPSILON, wi);
		
		// 递归获取间接光照
		Colour bounceLight = pathTrace(nextRay, pathThroughput, depth + 1, sampler);
		
		// 应用BSDF和几何项 - indirect现在表示BSDF评估结果
		Colour indirectContribution = indirect * bounceLight * (cosTheta / (pdf * continueProb));
		
		// 返回直接光照和间接光照的和
		return directLight + indirectContribution;
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// 计算场景交点
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t == FLT_MAX) 
		{
			// 没有相交，返回背景
			ShadingData shadingData;
			shadingData.wo = -r.dir;
			return scene->background->evaluate(shadingData, r.dir);
		}
		
		// 计算着色数据
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		
		// 如果是光源，直接返回发光值
		if (shadingData.bsdf->isLight())
		{
			return shadingData.bsdf->emit(shadingData, shadingData.wo);
		}
		
		// 计算直接光照，并加入环境光以增加场景亮度
		Colour directLight = computeDirect(shadingData, sampler);
		
		// 增加較多環境光，幫助看清場景
		Colour ambientLight(0.3f, 0.3f, 0.3f);
		Colour albedoColor = shadingData.bsdf->evaluate(shadingData, Vec3(0, 0, 1));
		
		return directLight + albedoColor * ambientLight;
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f); 
	}
	void render()
	{
		// 增加樣本計數但不清空畫面
		film->incrementSPP();
		std::cout << "Adding sample " << film->SPP << "/" << SPP << "..." << std::endl;
		
		// 創建所有像素坐標的向量
		std::vector<std::pair<unsigned int, unsigned int>> pixels;
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				pixels.push_back(std::make_pair(x, y));
			}
		}
		
		// 隨機打亂像素順序，實現漸進式渲染
		std::random_shuffle(pixels.begin(), pixels.end());
		
		// 處理進度計數
		int processed = 0;
		int totalPixels = pixels.size();
		int lastPercent = 0;
		
		// 處理所有像素
		for (const auto& pixel : pixels)
		{
			unsigned int x = pixel.first;
			unsigned int y = pixel.second;
			
			float px = x + 0.5f;
			float py = y + 0.5f;
			Ray ray = scene->camera.generateRay(px, py);

			// 根据需要选择合适的渲染方法
			Colour col;
			
			// 通过renderMode变量选择渲染模式
			switch (renderMode) {
				case 1: // 路径追踪（最慢但最真实）
					{
						Colour pathThroughput(1.0f, 1.0f, 1.0f);
						col = pathTrace(ray, pathThroughput, 0, &samplers[0]);
					}
					break;
					
				case 2: // 直接光照（速度适中）
					col = direct(ray, &samplers[0]);
					break;
					
				case 3: // Albedo（最简单最快）
					col = albedo(ray);
					break;
					
				case 4: // 法线查看（调试模式）
					col = viewNormals(ray);
					break;
					
				default: // 默认使用直接光照
					col = direct(ray, &samplers[0]);
			}
			
			// 累積到現有結果中
			film->splat(px, py, col);
			unsigned char r, g, b;
			film->tonemap(x, y, r, g, b);
			canvas->draw(x, y, r, g, b);
			
			// 每處理一定數量的像素更新一次畫面
			processed++;
			if (processed % 5000 == 0) {
				int percent = (processed * 100) / totalPixels;
				if (percent > lastPercent) {
					lastPercent = percent;
					// 使用\r回車符而不是換行，實現同一行更新
					std::cout << "\rRendering progress: " << percent << "% ";
					std::cout.flush(); // 確保立即輸出
				}
				canvas->present();
			}
		}
		
		// 最後一行空行，使下一條信息換到新行
		std::cout << std::endl;
		std::cout << "Sample " << film->SPP << " complete." << std::endl;
		canvas->present(); // 最終更新畫布
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		film->saveTonemappedPNG(filename, 1.0f);
	}
	
	// 获取当前渲染模式
	int getRenderMode() const {
		return renderMode;
	}
	
	// 设置渲染模式
	void setRenderMode(int mode) {
		if (mode >= 1 && mode <= 4) {
			renderMode = mode;
		}
	}
};