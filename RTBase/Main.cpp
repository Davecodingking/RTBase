#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>
#include <iomanip>
#include <algorithm> // 添加algorithm头文件，提供std::max等函数

// 定義在Geometry.h中聲明的外部變量
int USE_MAX_TRIANGLES = MAXNODE_TRIANGLES;
float USE_TRAVERSE_COST = TRAVERSE_COST;
float USE_TRIANGLE_COST = TRIANGLE_COST;
int USE_BUILD_BINS = BUILD_BINS;
float RAY_EPSILON = EPSILON;

void runTests()	
{
	// Add test code here
}

// 辅助函数：绘制分隔线
void printSeparator(int width = 70) {
    std::cout << "+";
    for (int i = 0; i < width - 2; i++) {
        std::cout << "-";
    }
    std::cout << "+" << std::endl;
}

// 辅助函数：绘制表格行
void printRow(const std::string& left, const std::string& right, int width = 70) {
    std::cout << "| " << std::left << std::setw(width/2 - 3) << left 
              << " | " << std::setw(width/2 - 3) << right << " |" << std::endl;
}

// 辅助函数：绘制表格标题
void printHeader(const std::string& title, int width = 70) {
    printSeparator(width);
    std::cout << "| " << std::left << std::setw(width - 4) << title << " |" << std::endl;
    printSeparator(width);
}

// 添加这个函数以获取渲染模式名称
std::string getRenderModeName(int mode) {
	switch (mode) {
		case 1: return "Path Tracing";
		case 2: return "Direct Lighting";
		case 3: return "Albedo Only";
		case 4: return "Normal Visualization";
		default: return "Unknown";
	}
}

int main(int argc, char *argv[])
{
	// Add call to tests if required
	// runTests()
	
	//=================== 參數設置（開始）===================
	// 場景和輸出設置
	std::string sceneName = "Scenes1/bathroom";  // 場景名稱: cornell-box, house 等
	std::string filename = "materials-scene-render.hdr"; // 輸出文件名
	
	//-------------- 渲染質量設置 --------------
	unsigned int SPP = 64;                  // 每像素採樣數
	int renderMode = 1;                     // 渲染模式: 1=路徑追蹤, 2=直接光照, 3=僅顯示顏色, 4=顯示法線
	int maxPathDepth = 4;                  // 路徑追蹤最大反射深度
	bool useMultithread = true;             // 是否啟用多線程渲染
	bool useAdaptiveSampling = true;        // 是否啟用自適應採樣
	int tileSize = 16;                      // 渲染塊大小（較小的塊可以更頻繁地更新畫面）
	float varianceThreshold = 0.001f;        // 自適應採樣閾值 - 較低的值提供更準確的結果但需要更多樣本
	bool enableMIS = false;                  // 是否啟用多重重要性采樣
	bool enableEnvImportanceSampling = true; // 是否啟用環境光重要性采樣
	
	//-------------- 線程設置 --------------
	int threadCount = 0;                    // 渲染線程數 (0=自動使用系統核心數-1)
	
	//-------------- 光線交互設置 --------------
	float rayEpsilon = 0.001f;              // 光線偏移量，防止自相交和陰影痤瘡
	                                        // 太小會導致自相交，太大會導致漏光和錯誤陰影
	
	//-------------- BVH加速結構設置 --------------
	int maxNodeTriangles = 8;               // 每個BVH葉節點最大三角形數
	float traverseCost = 1.0f;              // 遍歷BVH節點的計算成本估計
	float triangleCost = 2.0f;              // 三角形相交測試的計算成本估計
	int buildBins = 16;                     // BVH構建時的空間分割數
	
	// 性能監控變量（無需修改）
	double totalRenderTime = 0.0;
	int frames = 0;
	long long totalRays = 0;
	//=================== 參數設置（結束）===================

	// 處理命令行參數
	if (argc > 1)
	{
		std::unordered_map<std::string, std::string> args;
		for (int i = 1; i < argc; ++i)
		{
			std::string arg = argv[i];
			if (!arg.empty() && arg[0] == '-')
			{
				std::string argName = arg;
				if (i + 1 < argc)
				{
					std::string argValue = argv[++i];
					args[argName] = argValue;
				} else
				{
					std::cerr << "Error: Missing value for argument '" << arg << "'\n";
				}
			} else
			{
				std::cerr << "Warning: Ignoring unexpected argument '" << arg << "'\n";
			}
		}
		for (const auto& pair : args)
		{
			if (pair.first == "-scene")
			{
				sceneName = pair.second;
			}
			if (pair.first == "-outputFilename")
			{
				filename = pair.second;
			}
			if (pair.first == "-SPP")
			{
				SPP = stoi(pair.second);
			}
			if (pair.first == "-renderMode")
			{
				renderMode = stoi(pair.second);
			}
			if (pair.first == "-maxPathDepth")
			{
				maxPathDepth = stoi(pair.second);
			}
			if (pair.first == "-useMultithread")
			{
				useMultithread = (pair.second == "true");
			}
			if (pair.first == "-useAdaptiveSampling")
			{
				useAdaptiveSampling = (pair.second == "true");
			}
			if (pair.first == "-tileSize")
			{
				tileSize = stoi(pair.second);
			}
			if (pair.first == "-threadCount")
			{
				threadCount = stoi(pair.second);
			}
			if (pair.first == "-rayEpsilon")
			{
				rayEpsilon = stof(pair.second);
			}
			if (pair.first == "-varianceThreshold")
			{
				varianceThreshold = stof(pair.second);
			}
			if (pair.first == "-enableMIS")
			{
				enableMIS = (pair.second == "true");
			}
			if (pair.first == "-enableEnvImportanceSampling")
			{
				enableEnvImportanceSampling = (pair.second == "true");
			}
		}
	}
	Scene* scene = loadScene(sceneName);
	if (!scene) {
		std::cerr << "Failed to load scene: " << sceneName << std::endl;
		return -1;
	}
	
	// 注釋掉之前的相機設置代碼，使用場景自帶的相機設置
	/*
	// 手動設置簡單的相機位置用於測試
	std::cout << "原始相機位置: (" << viewcamera.from.x << ", " << viewcamera.from.y << ", " << viewcamera.from.z << ")" << std::endl;
	std::cout << "原始觀察點: (" << viewcamera.to.x << ", " << viewcamera.to.y << ", " << viewcamera.to.z << ")" << std::endl;
	
	// 設置一個簡單的相機位置
	Vec3 newFrom(0.0f, 1.0f, 5.0f);
	Vec3 newTo(0.0f, 0.0f, 0.0f);
	Vec3 newUp(0.0f, 1.0f, 0.0f);
	Matrix newView = Matrix::lookAt(newFrom, newTo, newUp);
	newView = newView.invert();
	scene->camera.updateView(newView);
	
	// 更新viewcamera以便WASD移動能正常工作
	viewcamera.from = newFrom;
	viewcamera.to = newTo;
	viewcamera.up = newUp;
	*/
	
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", 1.0f);
	RayTracer rt;
	rt.init(scene, &canvas);
	
	// 應用上面設置的參數
	rt.setRenderMode(renderMode);
	rt.SPP = SPP;
	rt.maxPathDepth = maxPathDepth;
	rt.multithreaded = useMultithread;
	rt.adaptiveSampling = useAdaptiveSampling;
	rt.tileSize = tileSize;
	rt.varianceThreshold = varianceThreshold; // 設置自適應採樣閾值
	rt.setEnableMIS(enableMIS);
	rt.setEnableImportanceSamplingEnv(enableEnvImportanceSampling);
	
	// 應用線程數設置
	if (threadCount > 0) {
		rt.numProcs = threadCount;  // 設置指定線程數
	}
	
	// 自动调整SPP - 当启用MIS和环境光重要性采样时，可以使用更少的样本获得相同质量
	if (enableMIS && enableEnvImportanceSampling && SPP > 16) {
		int originalSPP = rt.SPP;
		// 简化调整逻辑 - 使用更简单的数学计算
		int newSPP = originalSPP / 2; // 简单直接减半
		if (newSPP < 16) newSPP = 16; // 设置最小值保证质量
		
		rt.SPP = newSPP;
		std::cout << "注意: 由于启用了MIS和环境光重要性采样，自动将SPP从 " << originalSPP 
		          << " 减少到 " << rt.SPP << " (可获得相似质量但速度更快)" << std::endl;
	}
	
	// 設置BVH和光線偏移量參數
	USE_MAX_TRIANGLES = maxNodeTriangles;
	USE_TRAVERSE_COST = traverseCost;
	USE_TRIANGLE_COST = triangleCost;
	USE_BUILD_BINS = buildBins;
	RAY_EPSILON = rayEpsilon;
	
	// 场景初始化完成，打印场景信息
	printHeader("SCENE INFORMATION");
	printRow("Scene Name", sceneName);
	printRow("Resolution", std::to_string((int)scene->camera.width) + " x " + std::to_string((int)scene->camera.height));
	printRow("Total Pixels", std::to_string((int)(scene->camera.width * scene->camera.height)));
	printRow("Triangle Count", std::to_string(scene->triangles.size()));
	printRow("Light Count", std::to_string(scene->lights.size()));
	printRow("Target SPP", std::to_string(SPP));
	printRow("Rendering Mode", getRenderModeName(rt.getRenderMode()));
	printRow("Max Path Depth", std::to_string(rt.maxPathDepth));
	printRow("Output File", filename);
	
	// BVH信息（如果可用）
	printRow("BVH Depth", std::to_string(scene->bvh->getDepth()));
	printRow("BVH Node Count", std::to_string(scene->bvh->getNodeCount()));
	printRow("BVH Max Triangles", std::to_string(maxNodeTriangles));
	
	// 其他渲染參數信息
	printRow("Thread Mode", threadCount == 0 ? "Auto" : std::to_string(threadCount));
	printRow("Ray Epsilon", std::to_string(rayEpsilon));
	printRow("Variance Threshold", std::to_string(varianceThreshold));
	
	printSeparator();
	
	std::cout << "Starting rendering... Press ESC to stop." << std::endl;
	
	// 確保在渲染前畫面已清空
	canvas.clear();
	
	// 打印渲染配置
	std::cout << "Rendering configuration:" << std::endl;
	std::cout << " - Mode: " << getRenderModeName(rt.getRenderMode()) << std::endl;
	std::cout << " - Max Path Depth: " << rt.maxPathDepth << std::endl;
	std::cout << " - Multithreaded: " << (rt.multithreaded ? "Enabled" : "Disabled") << std::endl;
	std::cout << " - Adaptive Sampling: " << (rt.adaptiveSampling ? "Enabled" : "Disabled") << std::endl;
	std::cout << " - Tile Size: " << rt.tileSize << "x" << rt.tileSize << std::endl;
	std::cout << " - Target SPP: " << rt.SPP << std::endl;
	std::cout << " - Enable MIS: " << (rt.isEnableMIS() ? "Enabled" : "Disabled") << std::endl;
	std::cout << " - Enable Environment Importance Sampling: " << (rt.isEnableImportanceSamplingEnv() ? "Enabled" : "Disabled") << std::endl;
	std::cout << std::endl;
	
	bool running = true;
	GamesEngineeringBase::Timer timer;
	GamesEngineeringBase::Timer totalTimer;
	totalTimer.reset();
	
	while (running)
	{
		canvas.checkInput();
		if (canvas.keyPressed(VK_ESCAPE))
		{
			break;
		}
		
		// 檢查是否已經完成所有樣本
		if (rt.film->SPP >= rt.SPP) {
			// 已經達到目標樣本數，不再繼續渲染
			if (frames == 0) {
				// 只在第一次達到目標時保存結果
				std::cout << "Rendering completed with " << rt.film->SPP << " samples." << std::endl;
				// 保存结果
				rt.saveHDR(filename);
				std::cout << "Image saved to " << filename << std::endl;
				
				std::string pngFilename = filename.substr(0, filename.find_last_of('.')) + ".png";
				rt.savePNG(pngFilename);
				std::cout << "PNG image saved to " << pngFilename << std::endl;
				
				frames = 1; // 標記為已完成渲染
			}
		} else {
			// 尚未達到目標樣本數，繼續渲染
			// 啟動渲染計時器
			timer.reset();
			// 繼續渲染下一個樣本
			rt.render();
			// 計算渲染時間
			double renderTime = timer.dt();
			totalRenderTime += renderTime;
			frames++;
			
			std::cout << "Sample " << rt.film->SPP << "/" << rt.SPP << " completed in " << renderTime << " seconds." << std::endl;
			
			// 定期顯示更新的圖像
			canvas.present();
			
			// 如果已達到目標樣本數，保存最終結果
			if (rt.film->SPP >= rt.SPP) {
				// 保存结果
				rt.saveHDR(filename);
				std::cout << "Image saved to " << filename << std::endl;
				
				std::string pngFilename = filename.substr(0, filename.find_last_of('.')) + ".png";
				rt.savePNG(pngFilename);
				std::cout << "PNG image saved to " << pngFilename << std::endl;
			}
		}
		
		// 使用场景相机漫游功能
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward();
			rt.clear();
			canvas.clear();
			frames = 0; // 重置渲染状态，以便重新渲染
		}
		if (canvas.keyPressed('S'))
		{
			viewcamera.back();
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		if (canvas.keyPressed('A'))
		{
			viewcamera.left();
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		if (canvas.keyPressed('D'))
		{
			viewcamera.right();
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		if (canvas.keyPressed('E'))
		{
			viewcamera.flyUp();
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		if (canvas.keyPressed('Q'))
		{
			viewcamera.flyDown();
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		
		// 简单延时以减少CPU使用率
		Sleep(16); // 约60FPS
	}
	return 0;
}