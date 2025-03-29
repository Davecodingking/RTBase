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
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <random>
#include <chrono>

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
	
	// 新增的多線程和自適應採樣相關變量
	unsigned int tileSize = 64; // 默認tile大小
	std::vector<Tile*> tiles;   // 所有tile的集合
	TileQueue workQueue;        // 工作隊列
	std::atomic<int> activeTiles; // 正在處理的tile數量
	std::mutex filmMutex;       // 保護film的互斥鎖
	bool adaptiveSampling = true; // 是否啟用自適應採樣
	bool multithreaded = true;   // 是否啟用多線程渲染
	
	// 設置渲染模式
	void setRenderMode(int mode) {
		renderMode = mode;
	}
	
	// 獲取當前渲染模式
	int getRenderMode() const {
		return renderMode;
	}
	
	// 獲取當前樣本數
	int getSPP() const {
		return film->SPP;
	}
	
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
		// 初始化每個線程的隨機採樣器，使用不同的種子
		for (int i = 0; i < numProcs; ++i) {
			samplers[i] = MTRandom(i + 1); // 使用線程ID+1作為種子
		}
		clear();
		
		// 确保canvas已初始化
		if (canvas) {
			canvas->clear();
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
			
			// 计算光源方向传播的最大距离
			float lightDistance = 1000.0f; // 使用足够大的距离
			
			// 检查可见性（阴影测试）
			Ray shadowRay;
			shadowRay.init(shadingData.x + lightDirection * EPSILON, lightDirection);
			
			// 直接使用可见性测试函数
			if (!scene->visible(shadingData.x, shadingData.x + lightDirection * lightDistance)) continue;
			
			// 计算BSDF贡献
			Colour f = shadingData.bsdf->evaluate(shadingData, lightDirection);
			
			// 确保f不是全黑的
			if (f.r <= 0.0f && f.g <= 0.0f && f.b <= 0.0f) continue;
			
			// 累加贡献 - 应用几何项和PDF
			L = L + f * emissionColour * (nDotWi / pdf);
		}
		
		// 确保返回值不是NaN
		if (std::isnan(L.r) || std::isnan(L.g) || std::isnan(L.b)) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		return L;
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
		
		// 计算直接光照
		Colour directLight = computeDirect(shadingData, sampler);
		
		// 添加环境光以确保场景不会完全黑暗
		Colour ambientLight(0.15f, 0.15f, 0.15f);
		
		// 获取材质的漫反射颜色/albedo
		Colour albedoColor(0.5f, 0.5f, 0.5f); // 默认灰色
		
		// 使用BSDF的evaluate方法在漫反射方向上评估
		Vec3 upVector(0.0f, 1.0f, 0.0f);
		albedoColor = shadingData.bsdf->evaluate(shadingData, upVector);
		
		// 确保返回颜色不为零并且不是NaN
		Colour result = directLight + (albedoColor * ambientLight);
		
		if (std::isnan(result.r) || std::isnan(result.g) || std::isnan(result.b)) {
			return Colour(0.1f, 0.1f, 0.1f); // 返回灰色而不是全黑
		}
		
		// 确保结果至少有最小亮度
		if (result.r <= 0.01f && result.g <= 0.01f && result.b <= 0.01f) {
			return albedoColor * 0.1f; // 返回暗淡的材质颜色
		}
		
		return result;
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
				tile->varianceThreshold = 0.05f; // 提高閾值，減少採樣數
				tiles.push_back(tile);
			}
		}
		
		std::cout << "Created " << tiles.size() << " tiles for rendering." << std::endl;
		
		// 將tiles添加到工作隊列
		workQueue.clear(); // 先清空隊列
		workQueue.addTiles(tiles);
	}
	
	// 線程工作函數，處理工作隊列中的tile
	void threadWork(int threadId) {
		MTRandom& sampler = samplers[threadId];
		bool hasWork = true;
		unsigned int tilesProcessed = 0;
		
		// 每個線程處理一定數量的tiles
		while (hasWork) {
			// 獲取下一個要處理的tile
			Tile* tile = workQueue.getTile();
			
			// 如果沒有更多tile或者隊列標記為完成，則退出
			if (!tile) {
				hasWork = false;
				continue;
			}
			
			// 跳過已完成的tile
			if (tile->completed) {
				continue;
			}
			
			tilesProcessed++;
			
			// 处理tile中的每个像素
			unsigned int initialSamples = 1; // 减少到1个初始样本，加快首帧显示
			unsigned int width = tile->endX - tile->startX;
			unsigned int height = tile->endY - tile->startY;
			
			// 先进行初始采样
			if (tile->samplesCompleted < initialSamples) {
				// 对tile中的每个像素进行采样
				for (unsigned int y = tile->startY; y < tile->endY; y++) {
					for (unsigned int x = tile->startX; x < tile->endX; x++) {
						// 计算像素在tile内的索引
						unsigned int localX = x - tile->startX;
						unsigned int localY = y - tile->startY;
						
						// 确保索引有效
						if (localX >= width || localY >= height)
							continue;
							
						unsigned int index = localY * width + localX;
						
						// 检查索引有效性
						if (index >= tile->tileBuffer.size())
							continue;
						
						// 生成光线
						float px = x + sampler.next();
						float py = y + sampler.next();
						Ray ray = scene->camera.generateRay(px, py);
						
						// 根据渲染模式选择合适的渲染方法
						Colour pixelColour;
						switch (renderMode) {
							case 1: // 路径追踪
							{
								Colour pathThroughput(1.0f, 1.0f, 1.0f);
								pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							}
							break;
							case 2: // 直接光照
								pixelColour = direct(ray, &sampler);
								break;
							case 3: // Albedo
								pixelColour = albedo(ray);
								break;
							case 4: // 法线视图
								pixelColour = viewNormals(ray);
								break;
							default: // 默认使用直接光照
								pixelColour = direct(ray, &sampler);
						}
						
						// 确保颜色值有效
						if (std::isnan(pixelColour.r) || std::isnan(pixelColour.g) || std::isnan(pixelColour.b)) {
							pixelColour = Colour(0.0f, 0.0f, 0.0f);
						}
						
						// 更新tile缓冲区
						tile->tileBuffer[index] = tile->tileBuffer[index] + pixelColour;
						tile->squaredBuffer[index] = tile->squaredBuffer[index] + (pixelColour * pixelColour);
						
						// 更新film，但不直接更新畫面（避免線程衝突）
						{
							std::lock_guard<std::mutex> lock(filmMutex);
							film->splat(px, py, pixelColour);
						}
					}
				}
				
				// 增加已完成的樣本數量
				tile->samplesCompleted += 1;
				
				// 如果未完成初始采样，将tile重新加入队列
				if (tile->samplesCompleted < initialSamples) {
					workQueue.addTile(tile);
				}
				
				// 每處理完一個tile後適當休眠，避免佔用過多CPU
				if (tilesProcessed % 5 == 0) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			}
			else if (adaptiveSampling) {
				// 自适应采样阶段
				bool anyPixelRendered = false;
				
				// 检查每个像素是否需要额外采样
				for (unsigned int y = tile->startY; y < tile->endY; y++) {
					for (unsigned int x = tile->startX; x < tile->endX; x++) {
						// 计算像素在tile内的索引
						unsigned int localX = x - tile->startX;
						unsigned int localY = y - tile->startY;
						
						// 检查索引有效性
						if (localX >= width || localY >= height)
							continue;
						
						// 计算当前像素的方差
						float variance = tile->calculateVariance(x, y);
						
						// 如果方差大于阈值且未达到最大样本数，继续采样
						if (variance > tile->varianceThreshold && tile->samplesCompleted < SPP) {
							// 计算像素在tile内的索引
							unsigned int index = localY * width + localX;
							
							// 检查索引有效性
							if (index >= tile->tileBuffer.size())
								continue;
							
							// 生成光线
							float px = x + sampler.next();
							float py = y + sampler.next();
							Ray ray = scene->camera.generateRay(px, py);
							
							// 根据渲染模式选择合适的渲染方法
							Colour pixelColour;
							switch (renderMode) {
								case 1: // 路径追踪
								{
									Colour pathThroughput(1.0f, 1.0f, 1.0f);
									pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
								}
								break;
								case 2: // 直接光照
									pixelColour = direct(ray, &sampler);
									break;
								case 3: // Albedo
									pixelColour = albedo(ray);
									break;
								case 4: // 法线视图
									pixelColour = viewNormals(ray);
									break;
								default: // 默认使用直接光照
									pixelColour = direct(ray, &sampler);
							}
							
							// 确保颜色值有效
							if (std::isnan(pixelColour.r) || std::isnan(pixelColour.g) || std::isnan(pixelColour.b)) {
								pixelColour = Colour(0.0f, 0.0f, 0.0f);
							}
							
							// 更新tile缓冲区
							tile->tileBuffer[index] = tile->tileBuffer[index] + pixelColour;
							tile->squaredBuffer[index] = tile->squaredBuffer[index] + (pixelColour * pixelColour);
							
							// 更新film，但不直接更新畫面（避免線程衝突）
							{
								std::lock_guard<std::mutex> lock(filmMutex);
								film->splat(px, py, pixelColour);
							}
							
							anyPixelRendered = true;
						}
					}
				}
				
				// 如果有像素被渲染并且未达到最大样本数，将tile重新加入队列
				if (anyPixelRendered && tile->samplesCompleted < SPP) {
					tile->samplesCompleted += 1;
					workQueue.addTile(tile);
				} else {
					// 标记tile为已完成
					tile->completed = true;
				}
				
				// 每處理完一個tile後適當休眠，避免佔用過多CPU
				if (tilesProcessed % 5 == 0) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			} else {
				// 非自适应采样，每个像素添加一个样本
				for (unsigned int y = tile->startY; y < tile->endY; y++) {
					for (unsigned int x = tile->startX; x < tile->endX; x++) {
						// 计算像素在tile内的索引
						unsigned int localX = x - tile->startX;
						unsigned int localY = y - tile->startY;
						
						// 检查索引有效性
						if (localX >= width || localY >= height)
							continue;
							
						unsigned int index = localY * width + localX;
						
						// 检查索引有效性
						if (index >= tile->tileBuffer.size())
							continue;
						
						// 生成光线
						float px = x + sampler.next();
						float py = y + sampler.next();
						Ray ray = scene->camera.generateRay(px, py);
						
						// 根据渲染模式选择合适的渲染方法
						Colour pixelColour;
						switch (renderMode) {
							case 1: // 路径追踪
							{
								Colour pathThroughput(1.0f, 1.0f, 1.0f);
								pixelColour = pathTrace(ray, pathThroughput, 0, &sampler);
							}
							break;
							case 2: // 直接光照
								pixelColour = direct(ray, &sampler);
								break;
							case 3: // Albedo
								pixelColour = albedo(ray);
								break;
							case 4: // 法线视图
								pixelColour = viewNormals(ray);
								break;
							default: // 默认使用直接光照
								pixelColour = direct(ray, &sampler);
						}
						
						// 确保颜色值有效
						if (std::isnan(pixelColour.r) || std::isnan(pixelColour.g) || std::isnan(pixelColour.b)) {
							pixelColour = Colour(0.0f, 0.0f, 0.0f);
						}
						
						// 更新tile缓冲区
						tile->tileBuffer[index] = tile->tileBuffer[index] + pixelColour;
						
						// 更新film，但不直接更新畫面（避免線程衝突）
						{
							std::lock_guard<std::mutex> lock(filmMutex);
							film->splat(px, py, pixelColour);
						}
					}
				}
				
				tile->samplesCompleted += 1;
				
				// 如果未达到总样本数，将tile重新加入队列
				if (tile->samplesCompleted < SPP) {
					workQueue.addTile(tile);
				} else {
					// 标记tile为已完成
					tile->completed = true;
				}
				
				// 每處理完一個tile後適當休眠，避免佔用過多CPU
				if (tilesProcessed % 5 == 0) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			}
		}
		
		// 如果這個線程處理了一些tiles，打印信息
		if (tilesProcessed > 0) {
			std::cout << "Thread " << threadId << " completed processing " << tilesProcessed << " tiles" << std::endl;
		}
		
		// 退出前減少活動線程計數
		activeTiles.fetch_sub(1);
	}
	
	// 使用多線程渲染整個畫面 - 重寫該函數以解決黑屏問題
	void renderMultithreaded() {
		// [DEBUG] Starting multithreaded render function
		std::cout << "[DEBUG] Starting multithreaded rendering..." << std::endl;
		
		// 清空之前的渲染結果
		film->clear();
		canvas->clear();
		canvas->present(); // 立即更新顯示，確保用戶能看到清空的畫面
		
		// 初始化tiles
		std::cout << "[DEBUG] Initializing tiles..." << std::endl;
		initTiles();
		
		// 添加調試輸出，檢查場景是否正確加載
		std::cout << "[DEBUG] Checking scene status..." << std::endl;
		std::cout << "  - Triangle count: " << scene->triangles.size() << std::endl;
		std::cout << "  - Light count: " << scene->lights.size() << std::endl;
		std::cout << "  - Image size: " << film->width << "x" << film->height << std::endl;
		
		// 重置工作隊列並設置計數器
		workQueue.reset();
		film->SPP = 0;
		
		// 使用較少的線程數量，以避免過度並行造成的問題
		unsigned int availableThreads = std::thread::hardware_concurrency();
		// 使用正確的std::min函數調用
		unsigned int numThreads = (availableThreads > 1 ? availableThreads - 1 : 1);
		numThreads = (numThreads < (unsigned int)numProcs) ? numThreads : (unsigned int)numProcs;
		
		std::cout << "[DEBUG] Starting render with " << numThreads << " threads..." << std::endl;
		std::cout << "The image will gradually appear - please be patient..." << std::endl;
		std::cout << "You should see the first results within a few seconds..." << std::endl;
		
		// 記錄開始時間
		auto startTime = std::chrono::high_resolution_clock::now();
		
		// 設置正確的活動線程數量
		activeTiles.store(numThreads);
		
		// 確保線程指針數組初始化正確
		std::cout << "[DEBUG] Launching worker threads..." << std::endl;
		for (unsigned int i = 0; i < numThreads; i++) {
			threads[i] = new std::thread(&RayTracer::threadWork, this, i);
			std::cout << "[DEBUG] Thread " << i << " created." << std::endl;
		}
		
		// 使用主線程來處理顯示更新，避免線程間的Direct3D上下文衝突
		// 這是一個關鍵變更，因為Direct3D通常不是線程安全的
		bool isRendering = true;
		int lastProgress = -1;
		
		std::cout << "[DEBUG] Entering main display loop..." << std::endl;
		
		while (isRendering) {
			// 檢查是否所有工作線程都已完成
			if (activeTiles.load() == 0) {
				isRendering = false;
				std::cout << "[DEBUG] All worker threads have completed." << std::endl;
			}
			
			// 計算當前進度
			int progress = static_cast<int>(workQueue.getProgress());
			
			// 如果進度有變化，則打印
			if (progress != lastProgress && progress % 5 == 0) {
				auto currentTime = std::chrono::high_resolution_clock::now();
				auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
				
				std::cout << "Rendering progress: " << progress << "%, elapsed time: " << elapsed << " seconds" << std::endl;
				lastProgress = progress;
			}
			
			// [DEBUG] Update display
			std::cout << "[DEBUG] Updating display..." << std::endl;
			
			try {
				// 優化顯示更新 - 使用批量更新而不是逐像素更新
				// 這會減少Direct3D的狀態變化，提高性能
				if (canvas && film) {
					// 將film中的數據直接複製到canvas的backBuffer
					unsigned char* backBuffer = canvas->getBackBuffer();
					if (backBuffer) {
						unsigned int pixelCount = film->width * film->height;
						
						// 只更新顯示，每20幀更新一次全畫面，這樣可以減少計算負擔
						static int frameCount = 0;
						frameCount++;
						
						if (frameCount % 20 == 0) {
							// 完整更新
							for (unsigned int i = 0; i < pixelCount; i++) {
								unsigned int x = i % film->width;
								unsigned int y = i / film->width;
								unsigned char r, g, b;
								film->tonemap(x, y, r, g, b);
								canvas->draw(x, y, r, g, b);  // 使用正確版本的draw函數
							}
							std::cout << "[DEBUG] Full display update." << std::endl;
						} else {
							// 部分更新 - 只更新幾個tile的數據
							// 計算當前應該更新的tile
							int tileToUpdate = frameCount % tiles.size();
							if (tileToUpdate < tiles.size()) {
								Tile* tile = tiles[tileToUpdate];
								for (unsigned int y = tile->startY; y < tile->endY; y++) {
									for (unsigned int x = tile->startX; x < tile->endX; x++) {
										if (x < film->width && y < film->height) {
											unsigned char r, g, b;
											film->tonemap(x, y, r, g, b);
											canvas->draw(x, y, r, g, b);
										}
									}
								}
								std::cout << "[DEBUG] Updated tile " << tileToUpdate << " of " << tiles.size() << std::endl;
							}
						}
					} else {
						std::cout << "[ERROR] Canvas backbuffer is null!" << std::endl;
					}
				} else {
					std::cout << "[ERROR] Canvas or film is null!" << std::endl;
				}
				
				// 更新顯示 - 在主線程中調用present而不是在工作線程中
				if (canvas) {
					canvas->present();
					std::cout << "[DEBUG] Display updated successfully." << std::endl;
				} else {
					std::cout << "[ERROR] Canvas is null during present call!" << std::endl;
				}
			} catch (const std::exception& e) {
				std::cout << "[ERROR] Exception during display update: " << e.what() << std::endl;
			} catch (...) {
				std::cout << "[ERROR] Unknown exception during display update!" << std::endl;
			}
			
			// 短暫休眠，減少CPU佔用，同時讓工作線程有更多時間處理渲染
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		
		std::cout << "[DEBUG] Joining worker threads..." << std::endl;
		
		// 等待所有線程完成
		for (unsigned int i = 0; i < numThreads; i++) {
			if (threads[i] && threads[i]->joinable()) {
				std::cout << "[DEBUG] Joining thread " << i << "..." << std::endl;
				threads[i]->join();
				delete threads[i];
				threads[i] = nullptr;
				std::cout << "[DEBUG] Thread " << i << " joined successfully." << std::endl;
			}
		}
		
		std::cout << "[DEBUG] Setting workQueue as done..." << std::endl;
		// 設置工作隊列完成標誌
		workQueue.setDone();
		
		// 記錄結束時間並計算總時間
		auto endTime = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
		
		// 更新film的SPP計數
		film->SPP = SPP;
		
		std::cout << "[DEBUG] Final display update..." << std::endl;
		// 最終更新畫面 - 確保顯示最終結果
		try {
			if (canvas && film) {
				unsigned char* backBuffer = canvas->getBackBuffer();
				if (backBuffer) {
					std::cout << "[DEBUG] Performing final full screen update..." << std::endl;
					// 完整更新所有像素
					for (unsigned int y = 0; y < film->height; y++) {
						for (unsigned int x = 0; x < film->width; x++) {
							unsigned char r, g, b;
							film->tonemap(x, y, r, g, b);
							canvas->draw(x, y, r, g, b);
						}
					}
					
					// 最終一次呈現
					canvas->present();
					std::cout << "[DEBUG] Final display update completed successfully." << std::endl;
				} else {
					std::cout << "[ERROR] Canvas backbuffer is null during final update!" << std::endl;
				}
			} else {
				std::cout << "[ERROR] Canvas or film is null during final update!" << std::endl;
			}
		} catch (const std::exception& e) {
			std::cout << "[ERROR] Exception during final display update: " << e.what() << std::endl;
		} catch (...) {
			std::cout << "[ERROR] Unknown exception during final display update!" << std::endl;
		}
		
		std::cout << "Render complete in " << (duration / 1000.0) << " seconds!" << std::endl;
	}
	
	void render()
	{
		if (multithreaded) {
			renderMultithreaded();
			return;
		}
		
		// 原始的單線程渲染代碼，保持不變
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
		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(pixels.begin(), pixels.end(), g);
		
		// 處理進度計數
		int processed = 0;
		int totalPixels = (int)pixels.size();
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
						break;
					}
					
				case 2: // 直接光照（速度适中）
					col = direct(ray, &samplers[0]);
					break;
					
				case 3: // Albedo（最简单最快）
					col = albedo(ray);
					break;
					
				case 4: // 查看法线（调试用）
					col = viewNormals(ray);
					break;
					
				default: // 默认使用直接光照
					col = direct(ray, &samplers[0]);
					break;
			}
			
			// 添加样本到胶片
			film->splat(px, py, col);
			
			// 立即更新顯示
			unsigned char r, g, b;
			film->tonemap(x, y, r, g, b);
			canvas->draw(x, y, r, g, b);
			
			// 更新处理进度
			processed++;
			int percent = (processed * 100) / totalPixels;
			if (percent > lastPercent) {
				lastPercent = percent;
				std::cout << "\rProgress: " << percent << "%" << std::flush;
				
				// 定期更新畫面
				if (percent % 5 == 0) {
					canvas->present();
				}
			}
		}
		
		// 最終更新畫面
		canvas->present();
		std::cout << "\rProgress: 100%" << std::endl;
	}
	
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	
	void savePNG(std::string filename)
	{
		film->saveTonemappedPNG(filename, 1.0f);
	}
};