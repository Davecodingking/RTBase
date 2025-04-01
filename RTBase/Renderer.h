#pragma once

// 以下两种包含方式，编译器会选择能找到的那一个
// 实际编译时不会有问题，只是IDE可能会显示错误
//#include "../oidn-2.3.2.x64.windows/include/OpenImageDenoise/oidn.hpp"
#include <OpenImageDenoise/oidn.hpp>

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"

// 移除前向聲明，直接使用完整定義
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "PhotonMapping.h"  // 添加光子映射头文件
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <iostream>
#include <vector>
#include <algorithm> // 确保包含此头文件以使用std::min
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <random>
#include <chrono>
#include <iomanip>
#include <future> // 添加future头文件，用于支持异步任务

// 添加自定义的min函数，避免与std::min冲突
template<typename T>
inline T minimum(T a, T b) {
    return (a < b) ? a : b;
}

// Tile結構體，用於多線程渲染
struct Tile {
	unsigned int startX;
	unsigned int startY;
	unsigned int endX;
	unsigned int endY;
	unsigned int samplesCompleted;
	float varianceThreshold;
	bool completed;
	std::vector<Colour> tileBuffer;
	std::vector<Colour> squaredBuffer; // 用於計算方差
	
	Tile(unsigned int _startX, unsigned int _startY, unsigned int _endX, unsigned int _endY) :
		startX(_startX), startY(_startY), endX(_endX), endY(_endY), 
		samplesCompleted(0), varianceThreshold(0.05f), completed(false) {
		
		unsigned int width = endX - startX;
		unsigned int height = endY - startY;
		tileBuffer.resize(width * height, Colour(0.0f, 0.0f, 0.0f));
		squaredBuffer.resize(width * height, Colour(0.0f, 0.0f, 0.0f));
	}
	
	// 計算tile內像素的方差
	float calculateVariance(unsigned int x, unsigned int y) {
		// 檢查輸入座標是否在tile範圍內
		if (x < startX || x >= endX || y < startY || y >= endY)
			return 0.0f;
			
		unsigned int localX = x - startX;
		unsigned int localY = y - startY;
		unsigned int width = endX - startX;
		
		if (samplesCompleted < 2) return 1.0f; // 至少需要2個樣本才能計算方差
		
		// 确保索引在有效范围内
		if (localX >= width || localY >= (endY - startY))
			return 0.0f;
			
		unsigned int index = localY * width + localX;
		if (index >= tileBuffer.size())
			return 0.0f;
			
		Colour mean = tileBuffer[index] / float(samplesCompleted);
		Colour squaredMean = squaredBuffer[index] / float(samplesCompleted);
		Colour variance = squaredMean - (mean * mean);
		
		// 使用亮度作為方差度量，并确保结果为正数
		float luminance = variance.Lum();
		
		// 檢查NaN
		if (std::isnan(luminance))
			return 0.0f;
			
		return luminance > 0.0f ? luminance : 0.0f;
	}
};

class TileQueue
{
private:
	std::vector<Tile*> tiles;
	std::mutex queueMutex;
	unsigned int currentIndex;
	bool done;
	
public:
	TileQueue() : currentIndex(0), done(false) {}
	
	void reset() {
		std::lock_guard<std::mutex> lock(queueMutex);
		currentIndex = 0;
		done = false;
	}
	
	void setDone() {
		std::lock_guard<std::mutex> lock(queueMutex);
		done = true;
	}
	
	bool isDone() const {
		return done;
	}
	
	void addTiles(const std::vector<Tile*>& newTiles) {
		std::lock_guard<std::mutex> lock(queueMutex);
		tiles.insert(tiles.end(), newTiles.begin(), newTiles.end());
	}
	
	void addTile(Tile* tile) {
		std::lock_guard<std::mutex> lock(queueMutex);
		tiles.push_back(tile);
	}
	
	Tile* getTile() {
		std::lock_guard<std::mutex> lock(queueMutex);
		
		// 如果隊列標記為完成，返回nullptr
		if (done) return nullptr;
		
		// 如果所有tile都已經被處理
		if (currentIndex >= tiles.size()) {
			return nullptr;
		}
		
		// 返回下一個需要處理的tile
		return tiles[currentIndex++];
	}
	
	void clear() {
		std::lock_guard<std::mutex> lock(queueMutex);
		tiles.clear();
		currentIndex = 0;
	}
	
	size_t size() const {
		return tiles.size();
	}
	
	// 獲取處理進度百分比
	float getProgress() const {
		if (tiles.empty()) return 0.0f;
		return static_cast<float>(currentIndex) / static_cast<float>(tiles.size()) * 100.0f;
	}
};

// VPL (虚拟点光源) 数据结构
class VPL {
public:
	Vec3 position;      // VPL位置 (x_i)
	Vec3 normal;        // 表面法线 (n_i)
	ShadingData shadingData; // 包含BSDF (f_r)
	Colour Le;          // VPL辐射度 (L_e(i))
	
	VPL() {}
	
	VPL(Vec3 _position, Vec3 _normal, ShadingData _shadingData, Colour _Le) :
		position(_position), normal(_normal), shadingData(_shadingData), Le(_Le) {}
};

// 添加线程池实现
class ThreadPool {
public:
    ThreadPool(size_t numThreads) : stop(false) {
        for(size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while(true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queueMutex);
                        this->condition.wait(lock, [this] { 
                            return this->stop || !this->tasks.empty(); 
                        });
                        
                        if(this->stop && this->tasks.empty()) {
                            return;
                        }
                        
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
        }
    }
    
    template<class F>
    void enqueue(F&& f) {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if(stop) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            tasks.emplace(std::forward<F>(f));
        }
        condition.notify_one();
    }
    
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();
        for(std::thread &worker : workers) {
            if(worker.joinable()) {
                worker.join();
            }
        }
    }
    
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable condition;
    bool stop;
};

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
	int maxPathDepth = 10; // 路徑追蹤最大深度
	
	// MIS相关控制参数
	bool enableMIS = true; // 控制是否使用MIS
	bool enableImportanceSamplingEnv = true; // 控制是否使用环境光重要性采样
	
	// 新增的多線程和自適應採樣相關變量
	unsigned int tileSize = 64; // 默認tile大小
	std::vector<Tile*> tiles;   // 所有tile的集合
	TileQueue workQueue;        // 工作隊列
	std::atomic<int> activeTiles; // 正在處理的tile數量
	std::mutex filmMutex;       // 保護film的互斥鎖
	bool adaptiveSampling = true; // 是否啟用自適應採樣
	bool multithreaded = true;   // 是否啟用多線程渲染
	float varianceThreshold = 0.05f; // 自適應採樣閾值
	
	// 添加降噪相关变量
	bool enableDenoising = false; // 是否启用降噪
	bool collectAOVs = false;    // 是否收集AOV（仅在启用降噪时有效）
	
	// 即时辐射度相关
	bool enableIR = false;  // 是否启用即时辐射度算法
	std::vector<VPL> vpls;  // 虚拟点光源
	int numVPLs = 1000;     // VPL数量，默认1000个
	
	// 添加光子映射相关变量
	PhotonMapper* photonMapper = nullptr;  // 光子映射器
	bool enablePhotonMapping = false;      // 是否启用光子映射
	bool enableCausticPhotons = true;      // 是否启用焦散光子
	bool enableFinalGathering = true;      // 是否启用最终聚集
	int numGlobalPhotons = 100000;         // 全局光子数量
	int numCausticPhotons = 100000;        // 焦散光子数量
	float photonRadius = 0.5f;             // 光子收集半径
	int finalGatheringSamples = 64;        // 最终聚集的采样数量
	
	// 添加线程池
	ThreadPool* threadPool = nullptr;
	int batchSize = 4; // 每批次渲染的样本数
	int denoiseInterval = 8; // 每渲染多少个样本执行一次降噪
	
	// 設置渲染模式
	void setRenderMode(int mode) {
		renderMode = mode;
		// 如果设置为光子映射模式，自动启用光子映射
		if (mode >= 6 && mode <= 8) {
			setEnablePhotonMapping(true);
		}
	}
	
	// 獲取當前渲染模式
	int getRenderMode() const {
		return renderMode;
	}
	
	// 獲取當前樣本數
	int getSPP() const {
		return SPP;
	}
	
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		// 使用init方法而不是构造函数参数
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellFilter());
		
		// 获取CPU核心数
		SYSTEM_INFO sysinfo;
		GetSystemInfo(&sysinfo);
		numProcs = sysinfo.dwNumberOfProcessors - 1;
		if (numProcs < 1) numProcs = 1;
		
		samplers = new MTRandom[numProcs+1];
		for (int i = 0; i < numProcs + 1; i++) {
			samplers[i].init(i); // 使用init而不是seed
		}
		threads = new std::thread*[numProcs];
		for (int i = 0; i < numProcs; i++) {
			threads[i] = NULL;
		}
		
		// 创建光子映射器
		if (photonMapper == nullptr) {
			photonMapper = new PhotonMapper(scene, numGlobalPhotons, numCausticPhotons, photonRadius);
		}
		
		// 创建线程池
		if (threadPool == nullptr) {
			threadPool = new ThreadPool(numProcs);
		}
		
		std::cout << "Using " << numProcs << " rendering threads." << std::endl;
	}
	
	// 预处理场景信息，提取关键特征以加速渲染
	void preProcessScene() {
		// 如果启用了IR，生成VPLs
		if (enableIR) {
			std::cout << "Generating " << numVPLs << " virtual point lights (VPLs)..." << std::endl;
			auto startTime = std::chrono::high_resolution_clock::now();
			traceVPLs();
			auto endTime = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0f;
			std::cout << "VPL generation complete, time: " << duration << " seconds" << std::endl;
		}
		
		// 如果启用了光子映射，发射光子
		if (enablePhotonMapping && photonMapper) {
			std::cout << "Emitting and tracing photons..." << std::endl;
			auto startTime = std::chrono::high_resolution_clock::now();
			photonMapper->emitPhotons(enableCausticPhotons, true);
			auto endTime = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0f;
			int numGlobal = 0, numCaustic = 0;
			photonMapper->getPhotonMapStats(numGlobal, numCaustic);
			std::cout << "Photon mapping construction complete, time: " << duration << " seconds" << std::endl;
			std::cout << "  Global photons: " << numGlobal << ", Caustic photons: " << numCaustic << std::endl;
		}
	}
	
	void clear()
	{
		film->clear();
		// 清理舊的tiles
		for (auto tile : tiles) {
			delete tile;
		}
		tiles.clear();
		
		// 確保清空畫布
		if (canvas) {
			canvas->clear();
			canvas->present();  // 確保立即呈現清空的畫布
		}
	}
	
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// 提前检查 - 镜面材质不计算直接光照
		if (shadingData.bsdf->isPureSpecular())
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 预先检查是否需要BSDF采样 - 只检查一次条件，避免重复判断
		bool needsBSDFSampling = enableMIS &&  // MIS开关打开
		                         !shadingData.bsdf->isPureSpecular() && // 非纯镜面材质
		                         (scene->background->totalIntegratedPower() > 0.1f || // 有明显的环境光贡献
		                          scene->lights.size() > 1); // 或多个光源
		
		// 计算直接光照
		Colour L(0.0f, 0.0f, 0.0f);
		
		// 第一部分：通过光源采样计算直接光照贡献
		for (int i = 0; i < scene->lights.size(); i++)
		{
			// 获取光源采样信息
			Colour emissionColour;
			float lightPdf;
			Vec3 lightDirection = scene->lights[i]->sample(shadingData, sampler, emissionColour, lightPdf);
			
			// 快速预检查：忽略无效采样
			if (lightPdf <= 0.0f) continue;
			
			// 计算几何项（方向是否在法线同一侧）
			float nDotWi = Dot(shadingData.sNormal, lightDirection);
			if (nDotWi <= 0.0f) continue;  // 光源在背面，忽略
			
			// 计算BSDF贡献
			Colour f = shadingData.bsdf->evaluate(shadingData, lightDirection);
			
			// 确保f不是全黑的
			if (f.r <= 0.0f && f.g <= 0.0f && f.b <= 0.0f) continue;
			
			// 计算光源方向传播的最大距离
			float lightDistance = 1000.0f; // 使用足够大的距离
			
			// 检查可见性（阴影测试）
			if (!scene->visible(shadingData.x, shadingData.x + lightDirection * lightDistance)) continue;
			
			// 简化MIS决策 - 只有在需要BSDF采样的情况下才使用MIS
			// 对于区域光总是使用MIS以提高采样质量
			bool useMisForLight = needsBSDFSampling || scene->lights[i]->isArea();
			
			if (useMisForLight) {
				// 计算BSDF的PDF值，用于MIS权重计算
				float bsdfPdf = shadingData.bsdf->PDF(shadingData, lightDirection);
				
				// 使用幂启发式计算MIS权重
				float weight = SamplingDistributions::powerHeuristic(lightPdf, bsdfPdf);
				
				// 累加贡献 - 应用几何项，PDF和MIS权重
				L = L + f * emissionColour * (nDotWi * weight / lightPdf);
			} else {
				// 对于简单情况，直接使用光源采样
				L = L + f * emissionColour * (nDotWi / lightPdf);
			}
		}
		
		// 第二部分：通过BSDF采样计算直接光照贡献
		// 如果不需要BSDF采样，直接返回
		if (!needsBSDFSampling) {
			return L;
		}
		
		// BSDF采样部分
		Colour bsdfColour;
		float bsdfPdf;
		Vec3 bsdfDirection = shadingData.bsdf->sample(shadingData, sampler, bsdfColour, bsdfPdf);
		
		// 检查采样是否有效
		if (bsdfPdf <= 0.0f || (bsdfColour.r <= 0.0f && bsdfColour.g <= 0.0f && bsdfColour.b <= 0.0f)) {
			return L;
		}
		
		// 检查几何项
		float nDotWi = Dot(shadingData.sNormal, bsdfDirection);
		if (nDotWi <= 0.0f) {
			return L;
		}
		
		// 创建射线 - 使用表面法线方向偏移
		Vec3 offsetOrigin = shadingData.x + shadingData.sNormal * RAY_EPSILON;
		Ray bsdfRay(offsetOrigin, bsdfDirection);
		
		// 检查是否击中光源
		IntersectionData hitInfo = scene->traverse(bsdfRay);
		
		if (hitInfo.t < FLT_MAX)
		{
			// 计算着色数据
			ShadingData lightShadingData = scene->calculateShadingData(hitInfo, bsdfRay);
			
			// 检查是否是光源
			if (lightShadingData.bsdf->isLight())
			{
				// 获取光源发射值
				Colour emissionColour = lightShadingData.bsdf->emit(lightShadingData, -bsdfDirection);
				
				// 快速预检查：忽略黑色光源
				if (emissionColour.r <= 0.0f && emissionColour.g <= 0.0f && emissionColour.b <= 0.0f) {
					return L;
				}
				
				// 找到对应的光源 - 仅检查区域光
				for (int i = 0; i < scene->lights.size(); i++)
				{
					if (scene->lights[i]->isArea())
					{
						// 计算光源的采样PDF
						float lightPdf = scene->lights[i]->PDF(lightShadingData, -bsdfDirection);
						
						// 使用MIS权重
						float weight = SamplingDistributions::powerHeuristic(bsdfPdf, lightPdf);
						
						// 累加贡献
						L = L + bsdfColour * emissionColour * (nDotWi * weight / bsdfPdf);
						break;
					}
				}
			}
		}
		else if (scene->background != nullptr && scene->background->totalIntegratedPower() > 0.1f)
		{
			// 处理环境光/背景光
			Colour emissionColour = scene->background->evaluate(shadingData, bsdfDirection);
			
			// 优化：如果环境光是黑色的，跳过后续计算
			if (emissionColour.r <= 0.0f && emissionColour.g <= 0.0f && emissionColour.b <= 0.0f) {
				return L;
			}
			
			float lightPdf = scene->background->PDF(shadingData, bsdfDirection);
			
			// 防止除零错误
			if (lightPdf > 0.0f) {
				// 使用MIS权重
				float weight = SamplingDistributions::powerHeuristic(bsdfPdf, lightPdf);
				
				// 累加贡献
				L = L + bsdfColour * emissionColour * (nDotWi * weight / bsdfPdf);
			}
		}
		
		// 确保返回值不是NaN
		if (std::isnan(L.r) || std::isnan(L.g) || std::isnan(L.b)) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		return L;
	}
	
	// 计算VPL贡献的间接光照
	Colour computeIndirectVPL(ShadingData& shadingData)
	{
		// 如果没有启用Instant Radiosity或没有VPL，返回黑色
		if (!enableIR || vpls.empty()) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 纯镜面材质不计算VPL贡献（这些将通过路径追踪处理）
		if (shadingData.bsdf->isPureSpecular()) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 用于累积间接光照的变量
		Colour indirect(0.0f, 0.0f, 0.0f);
		
		// 遍历并计算所有VPL的贡献
		for (const VPL& vpl : vpls) {
			// 计算从着色点到VPL的方向
			Vec3 vplDirection = vpl.position - shadingData.x;
			float distance = vplDirection.length();
			
			// 当距离太近时，跳过以避免奇异性
			if (distance < 1e-5f) {
				continue;
			}
			
			// 归一化方向
			vplDirection = vplDirection / distance;
			
			// 检查几何项（方向与法线夹角）
			float nDotL = Dot(shadingData.sNormal, vplDirection);
			float vplNDotL = Dot(vpl.normal, -vplDirection);
			
			// 如果任一角度为负，表示VPL在背面，跳过
			if (nDotL <= 0.0f || vplNDotL <= 0.0f) {
				continue;
			}
			
			// 计算几何项G
			float G = nDotL * vplNDotL / (distance * distance);
			
			// 应用截断以避免奇异性
			if (G > vplClampThreshold) {
				G = vplClampThreshold;
			}
			
			// 检查可见性（阴影测试）
			// 使用表面法线方向偏移，而非射线方向
			Vec3 offsetOrigin = shadingData.x + shadingData.sNormal * RAY_EPSILON;
			Ray shadowRay(offsetOrigin, vplDirection);
			
			if (!scene->visible(shadowRay.o, vpl.position)) {
				continue; // VPL被遮挡
			}
			
			// 计算BRDF项
			Colour brdf = shadingData.bsdf->evaluate(shadingData, vplDirection);
			
			// 累加贡献
			indirect = indirect + brdf * vpl.Le * G;
		}
		
		return indirect;
	}
	
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// 路径追踪算法实现
		if (depth > maxPathDepth) return Colour(0.0f, 0.0f, 0.0f); // 使用可調整的最大深度限制
		
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
		
		// 创建新的光线 - 使用表面法线方向偏移
		Vec3 offsetOrigin = shadingData.x + shadingData.sNormal * RAY_EPSILON;
		Ray nextRay(offsetOrigin, wi);
		
		// 递归获取间接光照
		Colour bounceLight = pathTrace(nextRay, pathThroughput, depth + 1, sampler);
		
		// 应用BSDF和几何项 - indirect现在表示BSDF评估结果
		Colour indirectContribution = indirect * bounceLight * (cosTheta / (pdf * continueProb));
		
		// 計算直接光照和間接光照的和
		Colour result = directLight + indirectContribution;
		
		// 限制極端值，防止白點產生
		const float maxValue = 10.0f; // 設置顏色值的上限
		if (result.r > maxValue) result.r = maxValue;
		if (result.g > maxValue) result.g = maxValue;
		if (result.b > maxValue) result.b = maxValue;
		
		// 返回結果
		return result;
	}
	
	// 计算直接光照和VPL贡献的总照明
	Colour direct(Ray& r, Sampler* sampler)
	{
		// 与场景相交
		IntersectionData hit = scene->traverse(r);
		if (hit.t == FLT_MAX) {
			if (scene->background != nullptr) {
				return scene->background->evaluate(ShadingData(), r.dir);
			}
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 计算交点的着色数据
		ShadingData shadingData = scene->calculateShadingData(hit, r);
		
		// 如果是光源，直接返回发射值
		if (shadingData.bsdf->isLight())
		{
			return shadingData.bsdf->emit(shadingData, -r.dir);
		}
		
		// 计算直接光照
		Colour directLight = computeDirect(shadingData, sampler);
		
		// 如果使用Instant Radiosity，添加VPL贡献的间接光照
		if (enableIR) {
			Colour indirectLight = computeIndirectVPL(shadingData);
			return directLight + indirectLight;
		}
		
		// 不使用IR时只返回直接光照
		return directLight;
	}
	
	// 仅渲染VPL贡献的间接光照（用于可视化和调试）
	Colour irIndirect(Ray& r, Sampler* sampler)
	{
		// 与场景相交
		IntersectionData hit = scene->traverse(r);
		if (hit.t == FLT_MAX) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 计算交点的着色数据
		ShadingData shadingData = scene->calculateShadingData(hit, r);
		
		// 如果是光源，返回黑色（不显示光源）
		if (shadingData.bsdf->isLight())
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 只计算VPL贡献的间接光照
		return computeIndirectVPL(shadingData);
	}
	
	// 可视化VPL位置（用于调试）
	Colour visualizeVPLs(Ray& r, Sampler* sampler)
	{
		// 与场景相交
		IntersectionData hit = scene->traverse(r);
		if (hit.t == FLT_MAX) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 计算交点的着色数据
		ShadingData shadingData = scene->calculateShadingData(hit, r);
		
		// 如果是光源，直接返回发射值
		if (shadingData.bsdf->isLight())
		{
			return shadingData.bsdf->emit(shadingData, -r.dir);
		}
		
		// 检查是否接近任何VPL
		const float vplRadius = 0.05f; // VPL显示半径
		for (const VPL& vpl : vpls) {
			float distance = (shadingData.x - vpl.position).length();
			if (distance < vplRadius) {
				// 在VPL位置显示亮点
				return Colour(5.0f, 0.0f, 0.0f); // 明亮的红色表示VPL位置
			}
		}
		
		// 否则返回灰色场景
		return Colour(0.3f, 0.3f, 0.3f);
	}
	
	// 渲染物体的反照率(Albedo)
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
	
	// 渲染表面法线图以便调试
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
	
	// 初始化tiles
	void initTiles() {
		// 清空之前的tiles
		for (auto tile : tiles) {
			delete tile;
		}
		tiles.clear();
		
		// 設置tile數量
		unsigned int tileCountX = (unsigned int)ceil(scene->camera.width / (float)tileSize);
		unsigned int tileCountY = (unsigned int)ceil(scene->camera.height / (float)tileSize);
		
		// 創建新的tiles
		for (unsigned int y = 0; y < tileCountY; y++) {
			for (unsigned int x = 0; x < tileCountX; x++) {
				// 計算tile邊界
				unsigned int startX = x * tileSize;
				unsigned int startY = y * tileSize;
				unsigned int endX = (startX + tileSize < (unsigned int)scene->camera.width) ? (startX + tileSize) : (unsigned int)scene->camera.width;
				unsigned int endY = (startY + tileSize < (unsigned int)scene->camera.height) ? (startY + tileSize) : (unsigned int)scene->camera.height;
				
				// 創建新的tile並添加到列表中
				Tile* tile = new Tile(startX, startY, endX, endY);
				tile->varianceThreshold = this->varianceThreshold; // 使用成員變數設置閾值
				tiles.push_back(tile);
			}
		}
		
		std::cout << "Created " << tiles.size() << " tiles for rendering." << std::endl;
		
		// 將tiles添加到工作隊列
		workQueue.clear(); // 先清空隊列
		workQueue.addTiles(tiles);
	}
	
	// 使用多線程渲染整個畫面 - 重寫該函數以實現漸進式多線程渲染
	void renderMultithreaded()
	{
		std::cout << "[INFO] Starting multithreaded rendering..." << std::endl;
		std::cout << "[INFO] Configuration: " << SPP << " samples, " 
				  << "Tile size: " << tileSize << "x" << tileSize << ", "
				  << "Image: " << film->width << "x" << film->height << std::endl;
		
		// 清空之前的渲染結果
		film->clear();
		canvas->clear();
		canvas->present();
		
		// 初始化tiles
		initTiles();
		
		// 顯示基本場景信息
		std::cout << "[INFO] Scene: " << scene->triangles.size() << " triangles, "
				  << scene->lights.size() << " lights" << std::endl;
		std::cout << "[INFO] The image will gradually improve as samples complete..." << std::endl;
		
		// 設置線程數量
		unsigned int availableThreads = std::thread::hardware_concurrency();
		unsigned int numThreads = (availableThreads > 1 ? availableThreads - 1 : 1);
		numThreads = (numThreads < (unsigned int)numProcs) ? numThreads : (unsigned int)numProcs;
		
		// 記錄總渲染時間
		auto totalStartTime = std::chrono::high_resolution_clock::now();
		
		// 分階段渲染，每個階段添加一個樣本
		for (int currentSpp = 0; currentSpp < SPP; currentSpp++) {
			// 開始當前樣本
			auto sampleStartTime = std::chrono::high_resolution_clock::now();
			
			// 重置工作隊列，但不清空film
			workQueue.reset();
			
			// 設置正確的活動線程數量
			activeTiles.store(numThreads);
			
			// 啟動線程處理當前樣本
			for (unsigned int i = 0; i < numThreads; i++) {
				threads[i] = new std::thread(&RayTracer::threadWorkSinglePass, this, i, currentSpp + 1);
			}
			
			// 使用主線程來處理顯示更新和進度報告
			bool isPassRendering = true;
			int lastProgress = -1;
			auto lastUpdateTime = std::chrono::high_resolution_clock::now();
			
			// 顯示更新循環
			while (isPassRendering) {
				// 檢查是否所有工作線程都已完成
				if (activeTiles.load() == 0) {
					isPassRendering = false;
				}
				
				// 計算當前進度
				int progress = static_cast<int>(workQueue.getProgress());
				
				// 獲取當前時間
				auto currentTime = std::chrono::high_resolution_clock::now();
				auto timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastUpdateTime).count();
				
				// 定期更新顯示 (每200毫秒) - 降低UI更新頻率，讓更多CPU資源用於渲染
				bool shouldUpdateDisplay = timeSinceLastUpdate > 200;
				
				// 進度有變化或者時間達到時更新進度條
				if ((progress != lastProgress) || shouldUpdateDisplay) {
					// 使用單行進度條更新
					std::cout << "\r" << "Sample " << (currentSpp + 1) << "/" << SPP << " [";
					
					// 進度條寬度30
					int barWidth = 30;
					int pos = barWidth * progress / 100;
					
					for (int i = 0; i < barWidth; ++i) {
						if (i < pos) std::cout << "=";
						else if (i == pos) std::cout << ">";
						else std::cout << " ";
					}
					
					// 計算經過的時間
					auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - sampleStartTime).count() / 1000.0f;
					
					std::cout << "] " << progress << "% (" << std::fixed << std::setprecision(1) << elapsed << "s)";
					std::cout.flush();
					
					lastProgress = progress;
					
					// 如果應該更新顯示，則更新畫面
					if (shouldUpdateDisplay) {
						updateFullDisplay();
						lastUpdateTime = currentTime;
					}
				}
				
				// 短暫休眠，減少CPU佔用 - 使用較長的休眠時間減少主線程的負擔
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			
			// 等待所有線程完成
			for (unsigned int i = 0; i < numThreads; i++) {
				if (threads[i] && threads[i]->joinable()) {
					threads[i]->join();
					delete threads[i];
					threads[i] = nullptr;
				}
			}
			
			// 更新樣本計數
			film->SPP = currentSpp + 1;
			
			// 更新顯示
			updateFullDisplay();
			
			// 計算並顯示樣本完成時間
			auto sampleEndTime = std::chrono::high_resolution_clock::now();
			auto sampleDuration = std::chrono::duration_cast<std::chrono::milliseconds>(sampleEndTime - sampleStartTime).count() / 1000.0f;
			std::cout << "\r" << "Sample " << (currentSpp + 1) << "/" << SPP << " [";
			for (int i = 0; i < 30; ++i) std::cout << "=";
			std::cout << "] 100% (" << std::fixed << std::setprecision(1) << sampleDuration << "s)" << std::endl;
		}
		
		// 計算總時間
		auto totalEndTime = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEndTime - totalStartTime).count() / 1000.0f;
		
		// 最終顯示更新
		updateFullDisplay();
		
		std::cout << std::endl;
		std::cout << "[INFO] Render complete in " << std::fixed << std::setprecision(2) << totalDuration << " seconds!" << std::endl;
		std::cout << "[INFO] Average: " << std::fixed << std::setprecision(2) << (totalDuration / SPP) << " seconds per sample" << std::endl;
	}
	
	// 更新全屏顯示
	void updateFullDisplay() {
		try {
			if (canvas && film) {
				// 更新所有像素
				for (unsigned int y = 0; y < film->height; y++) {
					for (unsigned int x = 0; x < film->width; x++) {
						unsigned char r, g, b;
						film->tonemap(x, y, r, g, b);
						canvas->draw(x, y, r, g, b);
					}
				}
				
				// 顯示更新
				canvas->present();
			}
		} catch (const std::exception& e) {
			std::cout << "[ERROR] Exception during display update: " << e.what() << std::endl;
		} catch (...) {
			std::cout << "[ERROR] Unknown exception during display update!" << std::endl;
		}
	}
	
	// 修改單個線程工作函數，提高多線程性能
	void threadWorkSinglePass(int threadId, int currentSpp) {
		MTRandom& sampler = samplers[threadId];
		bool hasWork = true;
		unsigned int tilesProcessed = 0;
		
		// 每個線程處理一定數量的tiles
		while (hasWork) {
			// 獲取下一個要處理的tile
			Tile* tile = workQueue.getTile();
			
			// 如果沒有更多tile，則退出
			if (!tile) {
				// 不要立即退出，等待一會兒再嘗試獲取新的tile
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				
				tile = workQueue.getTile();
				if (!tile) {
					hasWork = false;
					continue;
				}
			}
			
			// 跳過樣本數已達到當前樣本數的tile
			if (tile->samplesCompleted >= currentSpp) {
				continue;
			}
			
			tilesProcessed++;
			
			// 處理tile中的每個像素
			unsigned int width = tile->endX - tile->startX;
			unsigned int height = tile->endY - tile->startY;
			
			// 創建臨時顏色緩衝區，用於存儲當前tile的結果
			std::vector<Colour> tileColourBuffer(width * height, Colour(0.0f, 0.0f, 0.0f));
			
			// 對tile中的每個像素進行採樣
			for (unsigned int y = tile->startY; y < tile->endY; y++) {
				for (unsigned int x = tile->startX; x < tile->endX; x++) {
					// 計算像素在tile內的索引
					unsigned int localX = x - tile->startX;
					unsigned int localY = y - tile->startY;
					
					// 確保索引有效
					if (localX >= width || localY >= height)
						continue;
						
					unsigned int index = localY * width + localX;
					
					// 檢查索引有效性
					if (index >= tile->tileBuffer.size())
						continue;
					
					// 生成光線
					float px = x + sampler.next();
					float py = y + sampler.next();
					Ray ray = scene->camera.generateRay(px, py);
					
					// 根據渲染模式選擇合適的渲染方法
					Colour pixelColour;
					switch (renderMode) {
						case 0: // 法线可视化
							pixelColour = viewNormals(ray);
							break;
							
						case 1: // 反照率
							pixelColour = albedo(ray);
							break;
							
						case 2: // 可视化VPLs
							pixelColour = visualizeVPLs(ray, &sampler);
							break;
							
						case 3: // Path Tracing
						{
							Colour pathThroughput(1.0f, 1.0f, 1.0f);
							pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							
							// 如果启用了AOV收集，且当前是第一个样本，收集AOV
							if (collectAOVs && currentSpp == 1) {
								// 获取反照率
								Colour albedoColour = albedo(ray);
								
								// 获取法线
								Colour normalColour = viewNormals(ray);
								
								// 保存到本地缓存，稍后再一次性加锁更新
								tileColourBuffer[index] = pixelColour;
								
								// 添加到film中的AOV缓冲区 - AOV仍然需要加锁，但只在第一个样本时进行
								std::lock_guard<std::mutex> lock(filmMutex);
								film->setAlbedo(px, py, albedoColour);
								film->setNormal(px, py, normalColour);
							} else {
								tileColourBuffer[index] = pixelColour;
							}
						}
						break;
						
						case 4: // 直接光照
							pixelColour = directOnly(ray, &sampler);
							
							// 如果启用了AOV收集，且当前是第一个样本，收集AOV
							if (collectAOVs && currentSpp == 1) {
								Colour albedoColour = albedo(ray);
								Colour normalColour = viewNormals(ray);
								
								tileColourBuffer[index] = pixelColour;
								
								std::lock_guard<std::mutex> lock(filmMutex);
								film->setAlbedo(px, py, albedoColour);
								film->setNormal(px, py, normalColour);
							} else {
								tileColourBuffer[index] = pixelColour;
							}
							break;
							
						case 5: // IR全局光照
							if (enableIR) {
								pixelColour = direct(ray, &sampler);
							} else {
								Colour pathThroughput(1.0f, 1.0f, 1.0f);
								pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							}
							
							// 如果启用了AOV收集，且当前是第一个样本，收集AOV
							if (collectAOVs && currentSpp == 1) {
								Colour albedoColour = albedo(ray);
								Colour normalColour = viewNormals(ray);
								
								tileColourBuffer[index] = pixelColour;
								
								std::lock_guard<std::mutex> lock(filmMutex);
								film->setAlbedo(px, py, albedoColour);
								film->setNormal(px, py, normalColour);
							} else {
								tileColourBuffer[index] = pixelColour;
							}
							break;
							
						case 6: // 光子映射
							if (enablePhotonMapping && photonMapper) {
								pixelColour = photonMappingPathTrace(ray, &sampler);
							} else {
								Colour pathThroughput(1.0f, 1.0f, 1.0f);
								pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							}
							
							// 如果启用了AOV收集，且当前是第一个样本，收集AOV
							if (collectAOVs && currentSpp == 1) {
								Colour albedoColour = albedo(ray);
								Colour normalColour = viewNormals(ray);
								
								tileColourBuffer[index] = pixelColour;
								
								std::lock_guard<std::mutex> lock(filmMutex);
								film->setAlbedo(px, py, albedoColour);
								film->setNormal(px, py, normalColour);
							} else {
								tileColourBuffer[index] = pixelColour;
							}
							break;
							
						case 7: // 焦散光子映射
							if (enablePhotonMapping && photonMapper) {
								pixelColour = causticPhotonMapping(ray, &sampler);
							} else {
								Colour pathThroughput(1.0f, 1.0f, 1.0f);
								pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							}
							
							// 如果启用了AOV收集，且当前是第一个样本，收集AOV
							if (collectAOVs && currentSpp == 1) {
								Colour albedoColour = albedo(ray);
								Colour normalColour = viewNormals(ray);
								
								tileColourBuffer[index] = pixelColour;
								
								std::lock_guard<std::mutex> lock(filmMutex);
								film->setAlbedo(px, py, albedoColour);
								film->setNormal(px, py, normalColour);
							} else {
								tileColourBuffer[index] = pixelColour;
							}
							break;
							
						case 8: // 最终聚集光子映射
							if (enablePhotonMapping && photonMapper) {
								
								enableFinalGathering = true;
								pixelColour = photonMappingPathTrace(ray, &sampler);
							} else {
								Colour pathThroughput(1.0f, 1.0f, 1.0f);
								pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							}
							
							// 如果启用了AOV收集，且当前是第一个样本，收集AOV
							if (collectAOVs && currentSpp == 1) {
								Colour albedoColour = albedo(ray);
								Colour normalColour = viewNormals(ray);
								
								tileColourBuffer[index] = pixelColour;
								
								std::lock_guard<std::mutex> lock(filmMutex);
								film->setAlbedo(px, py, albedoColour);
								film->setNormal(px, py, normalColour);
							} else {
								tileColourBuffer[index] = pixelColour;
							}
							break;
							
						default: // 默认使用路径追踪
						{
							Colour pathThroughput(1.0f, 1.0f, 1.0f);
							pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							
							if (collectAOVs && currentSpp == 1) {
								Colour albedoColour = albedo(ray);
								Colour normalColour = viewNormals(ray);
								
								tileColourBuffer[index] = pixelColour;
								
								std::lock_guard<std::mutex> lock(filmMutex);
								film->setAlbedo(px, py, albedoColour);
								film->setNormal(px, py, normalColour);
							} else {
								tileColourBuffer[index] = pixelColour;
							}
						}
						break;
					}
					
					// 處理NaN值
					if (std::isnan(pixelColour.r) || std::isnan(pixelColour.g) || std::isnan(pixelColour.b)) {
						pixelColour = Colour(0.0f, 0.0f, 0.0f);
						tileColourBuffer[index] = pixelColour;
					}
					
					// 更新tile緩衝區
					tile->tileBuffer[index] = tile->tileBuffer[index] + pixelColour;
					
					// 如果啟用了自適應採樣，也更新平方緩衝區
					if (adaptiveSampling) {
						tile->squaredBuffer[index] = tile->squaredBuffer[index] + (pixelColour * pixelColour);
					}
					
					// 限制極端值，防止白點產生和累積
					const float maxValue = 10.0f; // 設置顏色值的上限
					if (pixelColour.r > maxValue) pixelColour.r = maxValue;
					if (pixelColour.g > maxValue) pixelColour.g = maxValue;
					if (pixelColour.b > maxValue) pixelColour.b = maxValue;
				}
			}
			
			// 完成整個tile後，一次性更新film - 顯著減少鎖爭用
			{
				std::lock_guard<std::mutex> lock(filmMutex);
				for (unsigned int y = tile->startY; y < tile->endY; y++) {
					for (unsigned int x = tile->startX; x < tile->endX; x++) {
						unsigned int localX = x - tile->startX;
						unsigned int localY = y - tile->startY;
						
						if (localX >= width || localY >= height)
							continue;
							
						unsigned int index = localY * width + localX;
						
						if (index < tileColourBuffer.size()) {
							film->splat(x + 0.5f, y + 0.5f, tileColourBuffer[index]);
						}
					}
				}
			}
			
			// 增加已完成的樣本數量
			tile->samplesCompleted += 1;
			
			// 移除不必要的休眠，避免性能損失
			// 如果需要避免CPU完全占用，可以使用更高效的方式
		}
		
		// 退出前減少活動線程計數
		activeTiles.fetch_sub(1);
	}
	
	void render()
	{
		// 渲染前初始化优化设置
		preRender();
		
		// 如果没有场景或者film，则返回
		if (!scene || !film) return;
		
		// 清空film中的数据
		clear();
		
		// 根据设置选择渲染方式
		if (multithreaded) {
			renderOptimizedMultithreaded(); // 使用优化后的多线程渲染
		} else {
			renderSingleThreaded(); // 使用单线程渲染
		}
	}
	
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	
	void savePNG(std::string filename)
	{
		film->saveTonemappedPNG(filename);
	}
	
	// 添加保存AOV的方法
	void saveAOVs(std::string prefix)
	{
		film->saveAOV(prefix);
	}
	
	// 设置是否启用降噪
	void setEnableDenoising(bool enable) {
		enableDenoising = enable;
		// 如果启用降噪，则自动收集AOV
		if (enable) {
			collectAOVs = true;
		}
	}
	
	// 设置是否收集AOV
	void setCollectAOVs(bool collect) {
		collectAOVs = collect;
	}
	
	// 使用OIDN执行降噪
	void denoise() {
		if (!enableDenoising) {
			std::cout << "Unable to denoise: Denoising is not enabled." << std::endl;
			return;
		}
		
		if (!film->hasAOVs) {
			std::cout << "Unable to denoise: No AOV data collected. Make sure collectAOVs is set to true." << std::endl;
			return;
		}
		
		std::cout << "Starting denoising process..." << std::endl;
		auto startTime = std::chrono::high_resolution_clock::now();
		
		try {
			// 创建OIDN设备
			oidn::DeviceRef device = oidn::newDevice();
			device.commit();
			
			// 计算图像大小和数据大小
			size_t pixelCount = film->width * film->height;
			size_t dataSize = pixelCount * sizeof(Colour); // Colour是3个float
			
			// 准备输入数据 - 确保用正确的SPP值归一化
			Colour* normalizedColor = new Colour[pixelCount];
			for (unsigned int i = 0; i < pixelCount; i++) {
				// 在输入前预先调整亮度 - 稍微降低亮度，改善去噪后过亮问题
				normalizedColor[i] = film->film[i] / ((float)film->SPP * 1.2f);
			}
			
			// 确保denoisedBuffer已初始化
			if (!film->denoisedBuffer) {
				film->denoisedBuffer = new Colour[pixelCount];
			}
			
			// 创建OIDN设备缓冲区
			oidn::BufferRef colorBuffer = device.newBuffer(dataSize);
			oidn::BufferRef albedoBuffer = device.newBuffer(dataSize);
			oidn::BufferRef normalBuffer = device.newBuffer(dataSize);
			oidn::BufferRef outputBuffer = device.newBuffer(dataSize);
			
			// 复制数据到设备缓冲区
			memcpy(colorBuffer.getData(), normalizedColor, dataSize);
			memcpy(albedoBuffer.getData(), film->albedoBuffer, dataSize);
			memcpy(normalBuffer.getData(), film->normalBuffer, dataSize);
			
			// 创建降噪过滤器（选择"RT"模式，适用于路径追踪）
			oidn::FilterRef filter = device.newFilter("RT");
			
			// 设置输入输出缓冲区 - 使用设备缓冲区而不是直接内存指针
			filter.setImage("color", colorBuffer, oidn::Format::Float3, film->width, film->height);
			filter.setImage("albedo", albedoBuffer, oidn::Format::Float3, film->width, film->height);
			filter.setImage("normal", normalBuffer, oidn::Format::Float3, film->width, film->height);
			filter.setImage("output", outputBuffer, oidn::Format::Float3, film->width, film->height);
			
			// 启用HDR模式但添加参数优化
			filter.set("hdr", true);
			filter.set("cleanAux", true); // 提前清理辅助缓冲区
			
			// 提交配置并执行降噪
			filter.commit();
			filter.execute();
			
			// 检查错误
			const char* errorMessage = nullptr;
			if (device.getError(errorMessage) != oidn::Error::None && errorMessage != nullptr) {
				std::cerr << "OIDN denoising error: " << errorMessage << std::endl;
				delete[] normalizedColor;
				return;
			}
			
			// 将结果从设备缓冲区复制回主机内存
			memcpy(film->denoisedBuffer, outputBuffer.getData(), dataSize);
			
			// 添加亮度调整以解决去噪后过亮问题
			float exposureScale = 0.4f; // 降低亮度
			for (unsigned int i = 0; i < pixelCount; i++) {
				film->denoisedBuffer[i] = film->denoisedBuffer[i] * exposureScale;
			}
			
			// 调试信息：检查输出是否有效
			float maxValue = 0.0f;
			float minValue = 1.0f;
			for (unsigned int i = 0; i < pixelCount; i++) {
				// 使用三目运算符替代std::max和std::min
				maxValue = (film->denoisedBuffer[i].r > maxValue) ? film->denoisedBuffer[i].r : maxValue;
				maxValue = (film->denoisedBuffer[i].g > maxValue) ? film->denoisedBuffer[i].g : maxValue;
				maxValue = (film->denoisedBuffer[i].b > maxValue) ? film->denoisedBuffer[i].b : maxValue;
				
				minValue = (film->denoisedBuffer[i].r < minValue) ? film->denoisedBuffer[i].r : minValue;
				minValue = (film->denoisedBuffer[i].g < minValue) ? film->denoisedBuffer[i].g : minValue;
				minValue = (film->denoisedBuffer[i].b < minValue) ? film->denoisedBuffer[i].b : minValue;
			}
			std::cout << "Denoised result range: [" << minValue << ", " << maxValue << "]" << std::endl;
			
			// 释放临时内存
			delete[] normalizedColor;
			
			auto endTime = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0f;
			std::cout << "Denoising completed in " << duration << " seconds" << std::endl;
		}
		catch (const std::exception& e) {
			std::cerr << "Exception during denoising: " << e.what() << std::endl;
		}
		catch (...) {
			std::cerr << "Unknown exception during denoising" << std::endl;
		}
	}
	
	// 从光源生成VPL集合
	void traceVPLs() {
		// 清空现有VPL
		{
			std::lock_guard<std::mutex> lock(vplMutex);
			vpls.clear();
		}
		
		// 预分配VPL容量以提高性能
		vpls.reserve(numVPLs);
		
		std::cout << "Generating " << numVPLs << " VPLs..." << std::endl;
		
		MTRandom sampler(42); // 使用固定种子以获得确定性结果
		
		// 对每个光源采样并生成VPL
		for (int i = 0; i < numVPLs; i++) {
			// 随机选择一个光源
			float pmf;
			Light* light = scene->sampleLight(&sampler, pmf);
			if (!light) continue;
			
			// 在光源上采样一个点和方向
			float pdfPosition, pdfDirection;
			Vec3 lightPos = light->samplePositionFromLight(&sampler, pdfPosition);
			Vec3 lightDir = light->sampleDirectionFromLight(&sampler, pdfDirection);
			
			// 计算初始光能量
			Colour Le;
			if (light->isArea()) {
				// 第1步：获取光源处的法线
				Vec3 lightNormal = light->normal(ShadingData(lightPos, Vec3(0,0,1)), lightDir);
				
				// 第2步：创建包含正确位置和法线的ShadingData
				ShadingData lightShadingData(lightPos, lightNormal);
				lightShadingData.wo = -lightDir; // 设置出射方向
				
				// 第3步：计算方向与法线的余弦项
				float cosTheta = Dot(lightDir, lightNormal);
				
				// 第4步：根据课件公式计算光源辐射度
				if (cosTheta > 0.0f) {
					// 方向与法线同向，光源可见
					Le = light->evaluate(lightShadingData, -lightDir) * cosTheta;
				} else {
					// 方向与法线反向，光源不可见
					Le = Colour(0.0f, 0.0f, 0.0f);
				}
			} else {
				// 对于点光源或方向光源不需要考虑法线
				ShadingData lightShadingData(lightPos, Vec3(0,0,1));
				lightShadingData.wo = -lightDir;
				Le = light->evaluate(lightShadingData, -lightDir);
			}
			
			// 归一化光能量（考虑PDF和光源选择概率）
			if (pdfPosition > 0.0f && pdfDirection > 0.0f && pmf > 0.0f) {
				Le = Le / (pmf * pdfPosition * pdfDirection * numVPLs);
			} else {
				continue; // 跳过无效采样
			}
			
			// 创建光线并追踪VPL路径 - 使用光源法线方向偏移
			Vec3 lightNormal = light->normal(ShadingData(lightPos, Vec3(0,0,1)), lightDir);
			Vec3 offsetOrigin = lightPos + lightNormal * RAY_EPSILON;
			Ray ray(offsetOrigin, lightDir);
			
			// 追踪VPL路径，从深度0开始
			VPLTracePath(ray, Colour(1.0f, 1.0f, 1.0f), Le, &sampler, 0);
		}
		
		std::cout << "Successfully generated " << vpls.size() << " VPLs" << std::endl;
	}
	
	// 递归追踪VPL路径并存储VPL - 使用参数传递递归深度
	void VPLTracePath(Ray& ray, Colour pathThroughput, Colour Le, Sampler* sampler, int depth = 0) {
		// 最大递归深度检查
		static const int MAX_VPL_DEPTH = 5;
		
		// 递归深度检查
		if (depth > MAX_VPL_DEPTH) {
			return;
		}
		
		// 与场景相交
		IntersectionData hit = scene->traverse(ray);
		if (hit.t == FLT_MAX) {
			return; // 未命中任何物体
		}
		
		// 计算交点的着色数据
		ShadingData shadingData = scene->calculateShadingData(hit, ray);
		
		// 检查是否是镜面材质
		bool isSpecular = shadingData.bsdf->isPureSpecular();
		
		// 如果不是镜面材质，创建并存储VPL
		if (!isSpecular) {
			VPL vpl(shadingData.x, shadingData.sNormal, shadingData, pathThroughput * Le);
			
			// 安全地添加到VPL集合中
			{
				std::lock_guard<std::mutex> lock(vplMutex);
				vpls.push_back(vpl);
			}
		}
		
		// 俄罗斯轮盘赌路径终止
		if (sampler->next() > RR_PROB) {
			return;
		}
		
		// 从着色点采样新方向
		Colour indirect;
		float pdf;
		Vec3 direction = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
		
		// 检查采样有效性
		if (pdf <= 0.0f || (indirect.r <= 0.0f && indirect.g <= 0.0f && indirect.b <= 0.0f)) {
			return;
		}
		
		// 计算新的吞吐量
		float cosTheta = Dot(direction, shadingData.sNormal);
		Colour newThroughput = pathThroughput * indirect * (cosTheta > 0.0f ? cosTheta : 0.0f) / (pdf * RR_PROB);
		
		// 检查吞吐量是否接近零
		if (newThroughput.Lum() < 0.001f) {
			return;
		}
		
		// 创建新光线并继续追踪 - 使用表面法线方向偏移
		Vec3 offsetOrigin = shadingData.x + shadingData.sNormal * RAY_EPSILON;
		Ray newRay(offsetOrigin, direction);
		
		// 递归追踪，深度加1
		VPLTracePath(newRay, newThroughput, Le, sampler, depth + 1);
	}
	
	// 启用/禁用Instant Radiosity
	void setEnableIR(bool enable) {
		enableIR = enable;
		
		// 如果启用IR，生成VPL
		if (enable && scene) {
			traceVPLs();
		}
	}
	
	// 设置VPL数量
	void setNumVPLs(int num) {
		if (num > 0) {
			numVPLs = num;
			// 如果已启用IR，更新VPL
			if (enableIR && scene) {
				traceVPLs();
			}
		}
	}
	
	// 设置VPL强度截断阈值
	void setVPLClampThreshold(float threshold) {
		vplClampThreshold = threshold;
	}
	
	// 设置控制参数的方法
	void setEnableMIS(bool enable) {
		enableMIS = enable;
	}
	
	bool isEnableMIS() const {
		return enableMIS;
	}
	
	void setEnableImportanceSamplingEnv(bool enable) {
		enableImportanceSamplingEnv = enable;
		
		// 更新环境光的重要性采样设置
		if (scene && scene->background) {
			EnvironmentMap* envMap = dynamic_cast<EnvironmentMap*>(scene->background);
			if (envMap) {
				envMap->enableImportanceSampling = enable;
			}
		}
	}
	
	bool isEnableImportanceSamplingEnv() const {
		return enableImportanceSamplingEnv;
	}
	
	// 渲染前的初始化，设置场景中的环境光采样控制
	void preRender() {
		// 如果场景存在，直接设置环境光采样开关
		if (scene && scene->background) {
			EnvironmentMap* envMap = dynamic_cast<EnvironmentMap*>(scene->background);
			if (envMap) {
				// 直接设置而不是在每次采样时判断
				envMap->enableImportanceSampling = enableImportanceSamplingEnv;
			}
		}
	}
	
	// 添加光子映射相关方法
	void setEnablePhotonMapping(bool enable) {
		enablePhotonMapping = enable;
		// 如果启用了光子映射，初始化光子映射器
		if (enable && !photonMapper && scene) {
			photonMapper = new PhotonMapper(scene, numGlobalPhotons, numCausticPhotons, photonRadius);
		}
	}
	
	bool isEnablePhotonMapping() const {
		return enablePhotonMapping;
	}
	
	void setEnableCausticPhotons(bool enable) {
		enableCausticPhotons = enable;
	}
	
	bool isEnableCausticPhotons() const {
		return enableCausticPhotons;
	}
	
	void setEnableFinalGathering(bool enable) {
		enableFinalGathering = enable;
	}
	
	bool isEnableFinalGathering() const {
		return enableFinalGathering;
	}
	
	void setPhotonCounts(int global, int caustic) {
		numGlobalPhotons = global;
		numCausticPhotons = caustic;
		if (photonMapper) {
			photonMapper->setPhotonCounts(global, caustic);
		}
	}
	
	void setPhotonRadius(float radius) {
		photonRadius = radius;
		if (photonMapper) {
			photonMapper->setGatherRadius(radius);
		}
	}
	
	void setFinalGatheringSamples(int samples) {
		finalGatheringSamples = samples;
	}
	
	// 添加使用光子映射的路径追踪方法
	Colour photonMappingPathTrace(Ray& r, Sampler* sampler, int depth = 0) {
		if (depth >= maxPathDepth) return Colour(0, 0, 0);
		
		// 与场景求交
		IntersectionData isect = scene->traverse(r);
		
		// 如果没有交点，返回背景光照
		if (isect.t == FLT_MAX) {
			if (scene->background) {
				return scene->background->evaluate(ShadingData(), -r.dir);
			}
			return Colour(0, 0, 0);
		}
		
		// 计算着色点数据
		ShadingData shadingData = scene->calculateShadingData(isect, r);
		
		// 获取材质
		BSDF* material = scene->materials[scene->triangles[isect.ID].materialIndex];
		shadingData.bsdf = material;
		
		// 如果是光源，直接返回发射值
		if (material->isLight()) {
			return material->emit(shadingData, shadingData.wo);
		}
		
		// 计算直接光照
		Colour directLight = computeDirect(shadingData, sampler);
		
		// 计算间接光照（使用光子映射）
		Colour indirectLight(0, 0, 0);
		
		// 如果材质是漫反射，使用光子映射估计间接光照
		if (material->isDiffuse()) {
			// 确保获取颜色而非灰度信息
			indirectLight = photonMapper->estimateIndirectLight(shadingData, true, true);
			
			// 如果启用了最终聚集，使用最终聚集改善质量
			if (enableFinalGathering) {
				indirectLight = photonMapper->finalGathering(shadingData, finalGatheringSamples);
			}
		} else {
			// 对于镜面材质，使用递归追踪
			float pdf;
			Colour brdf;
			Vec3 bounceDir = material->sample(shadingData, sampler, brdf, pdf);
			
			if (pdf > 0 && (brdf.r > 0.0f || brdf.g > 0.0f || brdf.b > 0.0f)) {
				// 计算反弹方向的贡献
				Ray bounceRay(shadingData.x + shadingData.sNormal * RAY_EPSILON, bounceDir);
				Colour bounceRadiance = photonMappingPathTrace(bounceRay, sampler, depth + 1);
				
				// 计算间接光照贡献 - 确保保留颜色信息
				float cosTheta = Dot(bounceDir, shadingData.sNormal) > 0.0f ? Dot(bounceDir, shadingData.sNormal) : 0.0f;
				indirectLight.r = brdf.r * bounceRadiance.r * cosTheta / pdf;
				indirectLight.g = brdf.g * bounceRadiance.g * cosTheta / pdf;
				indirectLight.b = brdf.b * bounceRadiance.b * cosTheta / pdf;
			}
		}
		
		// 返回直接光照和间接光照的和
		return directLight + indirectLight;
	}
	
	// 添加仅使用焦散光子的渲染模式
	Colour causticPhotonMapping(Ray& r, Sampler* sampler) {
		// 与场景求交
		IntersectionData isect = scene->traverse(r);
		
		// 如果没有交点，返回背景光照
		if (isect.t == FLT_MAX) {
			if (scene->background) {
				return scene->background->evaluate(ShadingData(), -r.dir);
			}
			return Colour(0, 0, 0);
		}
		
		// 计算着色点数据
		ShadingData shadingData = scene->calculateShadingData(isect, r);
		
		// 获取材质
		BSDF* material = scene->materials[scene->triangles[isect.ID].materialIndex];
		shadingData.bsdf = material;
		
		// 如果是光源，直接返回发射值
		if (material->isLight()) {
			return material->emit(shadingData, shadingData.wo);
		}
		
		// 计算直接光照
		Colour directLight = computeDirect(shadingData, sampler);
		
		// 计算焦散光照（使用光子映射）
		Colour causticLight(0, 0, 0);
		
		// 只对漫反射表面使用焦散光子映射
		if (material->isDiffuse()) {
			// 只使用焦散光子 - 确保获取颜色而非灰度信息
			causticLight = photonMapper->estimateIndirectLight(shadingData, true, false);
		}
		
		// 返回直接光照和焦散光照的和
		return directLight + causticLight;
	}
	
	Colour trace(Ray& r, int threadID)
	{
		Sampler* sampler = &samplers[threadID];
		
		// 根据渲染模式选择不同的渲染算法
		switch (renderMode)
		{
		case 0: // 法线可视化
			return viewNormals(r);
		case 1: // 反照率
			return albedo(r);
		case 2: // 可视化VPLs
			if (enableIR) {
				return visualizeVPLs(r, sampler);
			}
			return Colour(0.0f, 0.0f, 0.0f);
		case 3: // Path Tracing
			{
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				return pathTrace(r, pathThroughput, 0, sampler);
			}
		case 4: // 直接照明
			return directOnly(r, sampler);
		case 5: // 使用IR的全局照明
			if (enableIR) {
				return irIndirect(r, sampler);
			}
			{
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				return pathTrace(r, pathThroughput, 0, sampler);
			}
		case 6: // 光子映射
			if (enablePhotonMapping && photonMapper) {
				return photonMappingPathTrace(r, sampler);
			}
			{
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				return pathTrace(r, pathThroughput, 0, sampler);
			}
		case 7: // 仅焦散光子映射
			if (enablePhotonMapping && photonMapper) {
				return causticPhotonMapping(r, sampler);
			}
			{
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				return pathTrace(r, pathThroughput, 0, sampler);
			}
		case 8: // 带最终聚集的光子映射
			if (enablePhotonMapping && photonMapper) {
				enableFinalGathering = true;
				return photonMappingPathTrace(r, sampler);
			}
			{
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				return pathTrace(r, pathThroughput, 0, sampler);
			}
		default:
			{
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				return pathTrace(r, pathThroughput, 0, sampler);
			}
		}
	}
	
	// 添加直接光照渲染函数
	Colour directOnly(Ray& r, Sampler* sampler)
	{
		// 与场景相交
		IntersectionData hit = scene->traverse(r);
		if (hit.t == FLT_MAX) {
			if (scene->background != nullptr) {
				return scene->background->evaluate(ShadingData(), r.dir);
			}
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		// 计算交点的着色数据
		ShadingData shadingData = scene->calculateShadingData(hit, r);
		
		// 如果是光源，直接返回发射值
		if (shadingData.bsdf->isLight())
		{
			return shadingData.bsdf->emit(shadingData, -r.dir);
		}
		
		// 计算直接光照并返回
		return computeDirect(shadingData, sampler);
	}
	
	// 添加类成员变量vplClampThreshold, vplMutex
	std::mutex vplMutex;       // 保护VPL集合的互斥锁
	float vplClampThreshold = 0.1f; // VPL强度截断阈值，用于避免奇异性
	float RR_PROB = 0.8f;      // 俄罗斯轮盘赌概率
	
	Colour estimatePhotonIndirect(const ShadingData& shadingData) {
		if (!photonMapper) return Colour(0.0f, 0.0f, 0.0f);
		
		// 根据渲染模式和启用状态决定使用哪种光子图
		if (renderMode == 7) {
			// 焦散光子映射模式 - 主要关注焦散
			return photonMapper->estimateIndirectLight(shadingData, enableCausticPhotons, false);
		} else {
			// 标准光子映射模式或最终聚集模式
			return photonMapper->estimateIndirectLight(shadingData, enableCausticPhotons, true);
		}
	}
	
	// 新增：逐像素单线程渲染函数 - 不使用tile，直接按像素处理
	void renderSingleThreaded()
	{
		std::cout << "[INFO] Starting single-threaded rendering..." << std::endl;
		std::cout << "[INFO] Configuration: " << SPP << " samples, " 
				  << "Image: " << film->width << "x" << film->height << std::endl;
		
		// 清空之前的渲染结果
		film->clear();
		canvas->clear();
		canvas->present();
		
		// 显示基本场景信息
		std::cout << "[INFO] Scene: " << scene->triangles.size() << " triangles, "
				  << scene->lights.size() << " lights" << std::endl;
		std::cout << "[INFO] The image will gradually improve as samples complete..." << std::endl;
		
		// 使用单个线程的采样器
		MTRandom& sampler = samplers[0];
		
		// 记录总渲染时间
		auto totalStartTime = std::chrono::high_resolution_clock::now();
		
		// 分阶段渲染，每个阶段添加一个样本
		for (int currentSpp = 0; currentSpp < SPP; currentSpp++) {
			// 开始当前样本
			auto sampleStartTime = std::chrono::high_resolution_clock::now();
			
			// 进度变量
			int lastProgress = -1;
			int totalPixels = film->width * film->height;
			int processedPixels = 0;
			
			// 对每个像素渲染一个样本
			for (unsigned int y = 0; y < film->height; y++) {
				for (unsigned int x = 0; x < film->width; x++) {
					// 生成射线
					float px = x + sampler.next();
					float py = y + sampler.next();
					Ray ray = scene->camera.generateRay(px, py);
					
					// 根据渲染模式跟踪射线
					Colour pixelColour = trace(ray, 0);
					
					// 处理NaN值
					if (std::isnan(pixelColour.r) || std::isnan(pixelColour.g) || std::isnan(pixelColour.b)) {
						pixelColour = Colour(0.0f, 0.0f, 0.0f);
					}
					
					// 限制极端值
					const float maxValue = 10.0f;
					if (pixelColour.r > maxValue) pixelColour.r = maxValue;
					if (pixelColour.g > maxValue) pixelColour.g = maxValue;
					if (pixelColour.b > maxValue) pixelColour.b = maxValue;
					
					// 更新film和屏幕
					film->splat(px, py, pixelColour);
					
					// 如果启用了AOV收集
					if (collectAOVs && currentSpp == 0) {
						// 获取反照率
						Colour albedoColour = albedo(ray);
						// 获取法线
						Colour normalColour = viewNormals(ray);
						// 更新AOV
						film->setAlbedo(px, py, albedoColour);
						film->setNormal(px, py, normalColour);
					}
					
					// 立即更新画面 - 实时显示每个像素的变化
					if (x % 8 == 0 && y % 8 == 0) { // 每8个像素更新一次屏幕，减少开销
						unsigned char r, g, b;
						film->tonemap(x, y, r, g, b);
						canvas->draw(x, y, r, g, b);
					}
					
					// 更新进度
					processedPixels++;
					int progress = (processedPixels * 100) / totalPixels;
					
					// 每处理1%的像素更新一次进度条
					if (progress != lastProgress && progress % 1 == 0) {
						auto currentTime = std::chrono::high_resolution_clock::now();
						auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - sampleStartTime).count() / 1000.0f;
						
						std::cout << "\r" << "Sample " << (currentSpp + 1) << "/" << SPP << " [";
						int barWidth = 30;
						int pos = barWidth * progress / 100;
						for (int i = 0; i < barWidth; ++i) {
							if (i < pos) std::cout << "=";
							else if (i == pos) std::cout << ">";
							else std::cout << " ";
						}
						std::cout << "] " << progress << "% (" << std::fixed << std::setprecision(1) << elapsed << "s)";
						std::cout.flush();
						
						lastProgress = progress;
						canvas->present(); // 更新屏幕
					}
				}
			}
			
			// 更新样本计数
			film->SPP = currentSpp + 1;
			
			// 最终更新显示
			for (unsigned int y = 0; y < film->height; y++) {
				for (unsigned int x = 0; x < film->width; x++) {
					unsigned char r, g, b;
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
			canvas->present();
			
			// 计算并显示样本完成时间
			auto sampleEndTime = std::chrono::high_resolution_clock::now();
			auto sampleDuration = std::chrono::duration_cast<std::chrono::milliseconds>(sampleEndTime - sampleStartTime).count() / 1000.0f;
			std::cout << "\r" << "Sample " << (currentSpp + 1) << "/" << SPP << " [";
			for (int i = 0; i < 30; ++i) std::cout << "=";
			std::cout << "] 100% (" << std::fixed << std::setprecision(1) << sampleDuration << "s)" << std::endl;
		}
		
		// 计算总时间
		auto totalEndTime = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEndTime - totalStartTime).count() / 1000.0f;
		
		std::cout << std::endl;
		std::cout << "[INFO] Render complete in " << std::fixed << std::setprecision(2) << totalDuration << " seconds!" << std::endl;
		std::cout << "[INFO] Average: " << std::fixed << std::setprecision(2) << (totalDuration / SPP) << " seconds per sample" << std::endl;
	}
	
	// 优化的tile处理函数 - 处理多个样本，减少同步开销
	void processOptimizedTile(unsigned int tileStartX, unsigned int tileStartY, unsigned int tileEndX, unsigned int tileEndY, int sampleOffset, int sampleCount, int threadId) {
		// 访问线程专用采样器
		MTRandom& sampler = samplers[threadId];
		
		// 计算tile尺寸
		unsigned int tileWidth = tileEndX - tileStartX;
		unsigned int tileHeight = tileEndY - tileStartY;
		
		// 创建批次结果缓冲区
		std::vector<Colour> batchResults(tileWidth * tileHeight, Colour(0.0f, 0.0f, 0.0f));
		
		// 处理这个tile的多个样本
		for (int sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
			// 处理tile中的每个像素
			for (unsigned int y = tileStartY; y < tileEndY; y++) {
				for (unsigned int x = tileStartX; x < tileEndX; x++) {
					// 计算此像素在局部缓冲区中的索引
					unsigned int bufferIdx = (y - tileStartY) * tileWidth + (x - tileStartX);
					
					// 生成光线
					float px = x + sampler.next();
					float py = y + sampler.next();
					Ray ray = scene->camera.generateRay(px, py);
					
					// 跟踪光线
					Colour pixelColour = trace(ray, threadId);
					
					// 处理NaN值
					if (std::isnan(pixelColour.r) || std::isnan(pixelColour.g) || std::isnan(pixelColour.b)) {
						pixelColour = Colour(0.0f, 0.0f, 0.0f);
					}
					
					// 限制极端值
					const float maxValue = 10.0f;
					if (pixelColour.r > maxValue) pixelColour.r = maxValue;
					if (pixelColour.g > maxValue) pixelColour.g = maxValue;
					if (pixelColour.b > maxValue) pixelColour.b = maxValue;
					
					// 累积到批次结果中
					batchResults[bufferIdx] = batchResults[bufferIdx] + pixelColour;
					
					// 收集AOVs（仅针对第一个样本）
					if (collectAOVs && sampleIdx == 0 && sampleOffset == 0) {
						Colour albedoColour = albedo(ray);
						Colour normalColour = viewNormals(ray);
						
						// 使用锁保护AOV更新
						std::lock_guard<std::mutex> lock(filmMutex);
						film->setAlbedo(px, py, albedoColour);
						film->setNormal(px, py, normalColour);
					}
				}
			}
		}
		
		// 批量更新film，减少锁争用
		{
			std::lock_guard<std::mutex> lock(filmMutex);
			for (unsigned int y = tileStartY; y < tileEndY; y++) {
				for (unsigned int x = tileStartX; x < tileEndX; x++) {
					unsigned int bufferIdx = (y - tileStartY) * tileWidth + (x - tileStartX);
					float px = x + 0.5f;
					float py = y + 0.5f;
					film->splat(px, py, batchResults[bufferIdx]);
				}
			}
		}
	}
	
	// 新的优化版多线程渲染函数
	void renderOptimizedMultithreaded() {
		std::cout << "[INFO] Starting optimized multithreaded rendering..." << std::endl;
		std::cout << "[INFO] Configuration: " << SPP << " samples, " 
				  << "Batch size: " << batchSize << ", "
				  << "Tile size: " << tileSize << "x" << tileSize << ", "
				  << "Image: " << film->width << "x" << film->height << std::endl;
		
		// 清空之前的渲染结果
		film->clear();
		canvas->clear();
		canvas->present();
		
		// 计算tile数量
		unsigned int numTilesX = (film->width + tileSize - 1) / tileSize;
		unsigned int numTilesY = (film->height + tileSize - 1) / tileSize;
		unsigned int totalTiles = numTilesX * numTilesY;
		
		// 显示场景信息
		std::cout << "[INFO] Scene: " << scene->triangles.size() << " triangles, "
				  << scene->lights.size() << " lights, "
				  << totalTiles << " tiles" << std::endl;
		std::cout << "[INFO] The image will gradually improve as samples complete..." << std::endl;
		
		// 记录总渲染时间
		auto totalStartTime = std::chrono::high_resolution_clock::now();
		
		// 计算需要多少批次来完成所有样本
		int totalBatches = (SPP + batchSize - 1) / batchSize;
		
		// 批次渲染循环
		for (int batchIdx = 0; batchIdx < totalBatches; batchIdx++) {
			// 计算这批次需要渲染的样本数
			int samplesInBatch = (batchIdx == totalBatches - 1) ? 
								 (SPP - batchIdx * batchSize) : batchSize;
			
			// 开始当前批次计时
			auto batchStartTime = std::chrono::high_resolution_clock::now();
			
			// 进度跟踪
			std::atomic<int> completedTiles(0);
			
			// 提交所有tile任务到线程池
			for (unsigned int tileY = 0; tileY < numTilesY; tileY++) {
				for (unsigned int tileX = 0; tileX < numTilesX; tileX++) {
					// 计算tile边界
					unsigned int startX = tileX * tileSize;
					unsigned int startY = tileY * tileSize;
					unsigned int endX = minimum(startX + tileSize, film->width);
					unsigned int endY = minimum(startY + tileSize, film->height);
					
					// 分配线程ID - 循环分配
					int threadId = (tileY * numTilesX + tileX) % numProcs;
					
					// 提交任务到线程池
					threadPool->enqueue([this, startX, startY, endX, endY, batchIdx, samplesInBatch, threadId, &completedTiles]() {
						// 处理tile
						processOptimizedTile(startX, startY, endX, endY, batchIdx * batchSize, samplesInBatch, threadId);
						
						// 增加完成的tile计数
						completedTiles++;
					});
				}
			}
			
			// 监控进度
			auto lastUpdateTime = std::chrono::high_resolution_clock::now();
			int lastProgress = -1;
			
			// 等待所有tile完成
			while (completedTiles < totalTiles) {
				// 计算进度
				int progress = (completedTiles * 100) / totalTiles;
				
				// 当前时间
				auto currentTime = std::chrono::high_resolution_clock::now();
				auto timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastUpdateTime).count();
				
				// 定期更新显示 (每500毫秒)
				bool shouldUpdateDisplay = timeSinceLastUpdate > 500;
				
				// 如果进度变化或需要更新显示
				if (progress != lastProgress || shouldUpdateDisplay) {
					// 显示进度条
					std::cout << "\r" << "Batch " << (batchIdx + 1) << "/" << totalBatches
							  << " (" << (batchIdx * batchSize + 1) << "-" << minimum((batchIdx + 1) * batchSize, SPP) 
							  << "/" << SPP << " samples) [";
					
					int barWidth = 30;
					int pos = barWidth * progress / 100;
					for (int i = 0; i < barWidth; ++i) {
						if (i < pos) std::cout << "=";
						else if (i == pos) std::cout << ">";
						else std::cout << " ";
					}
					
					// 计算耗时
					auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - batchStartTime).count() / 1000.0f;
					
					std::cout << "] " << progress << "% (" << std::fixed << std::setprecision(1) << elapsed << "s)";
					std::cout.flush();
					
					lastProgress = progress;
					
					// 更新显示
					if (shouldUpdateDisplay) {
						updateFullDisplay();
						lastUpdateTime = currentTime;
					}
				}
				
				// 减轻CPU负担
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
			
			// 批次完成，更新film的样本计数
			film->SPP += samplesInBatch;
			
			// 更新显示
			updateFullDisplay();
			
			// 取消间隔去噪，只在最终渲染完成后去噪
			/*
			bool shouldDenoise = enableDenoising && film->hasAOVs && 
								 (film->SPP >= SPP || (film->SPP % denoiseInterval) == 0);
			
			if (shouldDenoise) {
				std::cout << "\r" << "Performing denoising at " << film->SPP << " samples..." << std::flush;
				denoise();
				
				// 显示降噪后的图像
				for (unsigned int y = 0; y < film->height; y++) {
					for (unsigned int x = 0; x < film->width; x++) {
						unsigned char r, g, b;
						// 从降噪缓冲区中获取颜色
						Colour c = film->denoisedBuffer[y * film->width + x];
						
						// 应用色调映射和伽马校正
						c.r = c.r < 0.0f ? 0.0f : (c.r > 1.0f ? 1.0f : c.r);
						c.g = c.g < 0.0f ? 0.0f : (c.g > 1.0f ? 1.0f : c.g);
						c.b = c.b < 0.0f ? 0.0f : (c.b > 1.0f ? 1.0f : c.b);
						
						r = (unsigned char)(pow(c.r, 1.0f / 2.2f) * 255.0f);
						g = (unsigned char)(pow(c.g, 1.0f / 2.2f) * 255.0f);
						b = (unsigned char)(pow(c.b, 1.0f / 2.2f) * 255.0f);
						
						canvas->draw(x, y, r, g, b);
					}
				}
				canvas->present();
				std::cout << " done!" << std::endl;
			}
			*/
			
			// 计算并显示批次完成时间
			auto batchEndTime = std::chrono::high_resolution_clock::now();
			auto batchDuration = std::chrono::duration_cast<std::chrono::milliseconds>(batchEndTime - batchStartTime).count() / 1000.0f;
			
			std::cout << "\r" << "Batch " << (batchIdx + 1) << "/" << totalBatches 
					  << " (" << (batchIdx * batchSize + 1) << "-" << minimum((batchIdx + 1) * batchSize, SPP) 
					  << "/" << SPP << " samples) [";
			
			for (int i = 0; i < 30; ++i) std::cout << "=";
			std::cout << "] 100% (" << std::fixed << std::setprecision(1) << batchDuration << "s)" << std::endl;
		}
		
		// 计算总时间
		auto totalEndTime = std::chrono::high_resolution_clock::now();
		auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(totalEndTime - totalStartTime).count() / 1000.0f;
		
		// 渲染结束后，执行最终降噪
		if (enableDenoising && film->hasAOVs) {
			std::cout << "Performing final denoising..." << std::flush;
			denoise();
			
			// 显示最终降噪结果
			for (unsigned int y = 0; y < film->height; y++) {
				for (unsigned int x = 0; x < film->width; x++) {
					unsigned char r, g, b;
					Colour c = film->denoisedBuffer[y * film->width + x];
					
					c.r = c.r < 0.0f ? 0.0f : (c.r > 1.0f ? 1.0f : c.r);
					c.g = c.g < 0.0f ? 0.0f : (c.g > 1.0f ? 1.0f : c.g);
					c.b = c.b < 0.0f ? 0.0f : (c.b > 1.0f ? 1.0f : c.b);
					
					r = (unsigned char)(pow(c.r, 1.0f / 2.2f) * 255.0f);
					g = (unsigned char)(pow(c.g, 1.0f / 2.2f) * 255.0f);
					b = (unsigned char)(pow(c.b, 1.0f / 2.2f) * 255.0f);
					
					canvas->draw(x, y, r, g, b);
				}
			}
			canvas->present();
			std::cout << " done!" << std::endl;
		}
		
		std::cout << std::endl;
		std::cout << "[INFO] Render complete in " << std::fixed << std::setprecision(2) << totalDuration << " seconds!" << std::endl;
		std::cout << "[INFO] Average: " << std::fixed << std::setprecision(2) << (totalDuration / totalBatches) << " seconds per batch" << std::endl;
		std::cout << "[INFO] Samples per second: " << std::fixed << std::setprecision(2) 
				  << (film->width * film->height * SPP) / totalDuration << std::endl;
	}
	
	~RayTracer() {
		// 释放线程池
		if (threadPool) {
			delete threadPool;
			threadPool = nullptr;
		}
		
		// ... 现有析构代码 ...
	}

    // 环境光参数控制方法
    void setEnvImportanceSamplingStrength(float strength) {
        if (scene && scene->background) {
            EnvironmentMap* envMap = dynamic_cast<EnvironmentMap*>(scene->background);
            if (envMap) {
                envMap->setImportanceSamplingStrength(strength);
            }
        }
    }

    void setEnvMinSamplingWeight(float weight) {
        if (scene && scene->background) {
            EnvironmentMap* envMap = dynamic_cast<EnvironmentMap*>(scene->background);
            if (envMap) {
                envMap->setMinSamplingWeight(weight);
            }
        }
    }

    void setEnvUseCosineWeighting(bool use) {
        if (scene && scene->background) {
            EnvironmentMap* envMap = dynamic_cast<EnvironmentMap*>(scene->background);
            if (envMap) {
                envMap->setUseCosineWeighting(use);
            }
        }
    }
};