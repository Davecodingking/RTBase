#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>
#include <iomanip>

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
	
	// Initialize default parameters
	std::string sceneName = "Scenes1/bathroom";  // 改用更簡單的bathroom場景
	std::string filename = "bathroom-render.hdr";
	unsigned int SPP = 16; // 進一步減少樣本數
	
	// 追蹤性能
	double totalRenderTime = 0.0;
	int frames = 0;
	long long totalRays = 0; // 声明totalRays变量

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
	// 設置渲染模式為直接光照模式
	rt.setRenderMode(2);
	// 設置目標樣本數
	rt.SPP = SPP;
	bool running = true;
	GamesEngineeringBase::Timer timer;
	GamesEngineeringBase::Timer totalTimer;
	totalTimer.reset();
	
	// 场景初始化完成，打印场景信息
	printHeader("SCENE INFORMATION");
	printRow("Scene Name", sceneName);
	printRow("Resolution", std::to_string((int)scene->camera.width) + " x " + std::to_string((int)scene->camera.height));
	printRow("Total Pixels", std::to_string((int)(scene->camera.width * scene->camera.height)));
	printRow("Triangle Count", std::to_string(scene->triangles.size()));
	printRow("Light Count", std::to_string(scene->lights.size()));
	printRow("Target SPP", std::to_string(SPP));
	printRow("Rendering Mode", getRenderModeName(rt.getRenderMode()));
	printRow("Output File", filename);
	
	// BVH信息（如果可用）
	printRow("BVH Depth", std::to_string(scene->bvh->getDepth()));
	printRow("BVH Node Count", std::to_string(scene->bvh->getNodeCount()));
	
	printSeparator();
	
	std::cout << "Starting rendering... Press ESC to stop." << std::endl;
	
	while (running)
	{
		canvas.checkInput();
		if (canvas.keyPressed(VK_ESCAPE))
		{
			break;
		}
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward();
			rt.clear();
			canvas.clear();
		}
		if (canvas.keyPressed('S'))
		{
			viewcamera.back();
			rt.clear();
			canvas.clear();
		}
		if (canvas.keyPressed('A'))
		{
			viewcamera.left();
			rt.clear();
			canvas.clear();
		}
		if (canvas.keyPressed('D'))
		{
			viewcamera.right();
			rt.clear();
			canvas.clear();
		}
		if (canvas.keyPressed('E'))
		{
			viewcamera.flyUp();
			rt.clear();
			canvas.clear();
		}
		if (canvas.keyPressed('Q'))
		{
			viewcamera.flyDown();
			rt.clear();
			canvas.clear();
		}
		// Time how long a render call takes
		timer.reset();
		rt.render();
		float frameTime = timer.dt();
		totalRenderTime += frameTime;
		frames++;
		
		// 当前SPP
		int currentSPP = rt.getSPP();
		
		// 计算进度和性能指标
		float progress = (float)currentSPP / SPP * 100.0f;
		
		// 计算每秒采样数
		float pixelCount = scene->camera.width * scene->camera.height;
		float samplesPerSecond = (pixelCount * currentSPP) / totalRenderTime;
		
		// 估计光线数量
		long long rayCount = (long long)(pixelCount * currentSPP * 2); // 粗略估计，每个样本平均2条光线
		totalRays = rayCount;
		float raysPerSecond = rayCount / totalRenderTime;
		
		// 简单进度更新（不刷新整个表格）
		std::cout << "SPP: " << currentSPP << "/" << SPP 
			<< " [" << std::fixed << std::setprecision(1) << progress << "%] " 
			<< "Time: " << std::fixed << std::setprecision(2) << totalRenderTime << "s " 
			<< "Frame: " << std::fixed << std::setprecision(2) << frameTime << "s"
			<< std::endl;
		
		if (canvas.keyPressed('P'))
		{
			rt.saveHDR(filename);
		}
		if (canvas.keyPressed('L'))
		{
			size_t pos = filename.find_last_of('.');
			std::string ldrFilename = filename.substr(0, pos) + ".png";
			rt.savePNG(ldrFilename);
		}
		if (currentSPP >= SPP)
		{
			rt.saveHDR(filename);
			
			// 渲染完成，显示最终报告
			double totalTime = totalRenderTime;
			printHeader("RENDERING COMPLETE");
			
			// 时间统计
			printRow("Total Render Time", std::to_string(totalTime) + " seconds");
			printRow("Average Sample Time", std::to_string(totalTime / SPP) + " seconds");
			printRow("Average Frame Time", std::to_string(totalRenderTime / frames) + " seconds");
			
			// 采样统计
			printRow("Total Rays", std::to_string(totalRays));
			printRow("Rays per Second", std::to_string((float)totalRays / totalTime));
			printRow("Total Samples", std::to_string(SPP * (long long)(scene->camera.width * scene->camera.height)));
			
			// 文件信息
			printRow("HDR Output", filename);
			printRow("Preview Image", filename.substr(0, filename.find_last_of('.')) + ".png");
			
			printSeparator();
			
			break;
		}
		canvas.present();
	}
	return 0;
}