#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#include "PhotonMapping.h"  // Add photon mapping header file
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>
#include <iomanip>
#include <algorithm> // Add algorithm header file, providing std::max etc. functions
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <unordered_map>
#include <windows.h>  // Add Windows header file

// Definition of external variables declared in Geometry.h
int USE_MAX_TRIANGLES = MAXNODE_TRIANGLES;
float USE_TRAVERSE_COST = TRAVERSE_COST;
float USE_TRIANGLE_COST = TRIANGLE_COST;
int USE_BUILD_BINS = BUILD_BINS;
float RAY_EPSILON = EPSILON;

void runTests()	
{
	// Add test code here
}

// Helper function: Draw separator line
void printSeparator(int width = 70) {
    std::cout << "+";
    for (int i = 0; i < width - 2; i++) {
        std::cout << "-";
    }
    std::cout << "+" << std::endl;
}

// Helper function: Draw table row
void printRow(const std::string& left, const std::string& right, int width = 70) {
    std::cout << "| " << std::left << std::setw(width/2 - 3) << left 
              << " | " << std::setw(width/2 - 3) << right << " |" << std::endl;
}

// Helper function: Draw table header
void printHeader(const std::string& title, int width = 70) {
    printSeparator(width);
    std::cout << "| " << std::left << std::setw(width - 4) << title << " |" << std::endl;
    printSeparator(width);
}

// Add this function to get render mode name
std::string getRenderModeName(int mode) {
	switch (mode) {
		case 0: return "Normal Visualization";
		case 1: return "Albedo Only";
		case 2: return "VPL Visualization";
		case 3: return "Path Tracing";
		case 4: return "Direct Lighting";
		case 5: return "Instant Radiosity";
		case 6: return "Photon Mapping";
		case 7: return "Caustic Photon Mapping";
		case 8: return "Photon Mapping + Final Gathering";
		default: return "Unknown";
	}
}

// Function to display denoised image
void displayDenoisedImage(RayTracer& rt, GamesEngineeringBase::Window& canvas) {
	canvas.clear();
	// Update screen
	for (unsigned int y = 0; y < rt.film->height; y++) {
		for (unsigned int x = 0; x < rt.film->width; x++) {
			unsigned char r, g, b;
			
			// Use denoised image to calculate color - no need to divide by SPP, already normalized
			Colour c = rt.film->denoisedBuffer[y * rt.film->width + x];
			
			// Apply tone mapping and gamma correction
			c.r = c.r < 0.0f ? 0.0f : (c.r > 1.0f ? 1.0f : c.r);
			c.g = c.g < 0.0f ? 0.0f : (c.g > 1.0f ? 1.0f : c.g);
			c.b = c.b < 0.0f ? 0.0f : (c.b > 1.0f ? 1.0f : c.b);
			
			r = (unsigned char)(pow(c.r, 1.0f / 2.2f) * 255.0f);
			g = (unsigned char)(pow(c.g, 1.0f / 2.2f) * 255.0f);
			b = (unsigned char)(pow(c.b, 1.0f / 2.2f) * 255.0f);
			
			canvas.draw(x, y, r, g, b);
		}
	}
	canvas.present();
}

// Scene preset configuration structure
struct ScenePreset {
    std::string sceneName;
    std::string filename;
    unsigned int SPP;
    int renderMode;
    int maxPathDepth;
    bool useMultithread;
    bool useAdaptiveSampling;
    int batchSize;
    int denoiseInterval;
    unsigned int tileSize;
    float varianceThreshold;
    bool enableMIS;
    bool enableEnvImportanceSampling;
    bool enableDenoising;
    bool saveAOVs;
    bool enableIR;
    int numVPLs;
    float vplClampThreshold;
    int numGlobalPhotons;
    int numCausticPhotons;
    float photonRadius;
    int finalGatheringSamples;
    int threadCount;
    float rayEpsilon;
    int maxNodeTriangles;
    float traverseCost;
    float triangleCost;
    int buildBins;
    // Environment light parameters
    float envBrightnessScale;
    float envImportanceSamplingStrength;
    float envMinSamplingWeight;
    bool envUseCosineWeighting;
};

// Scene preset configuration
const std::vector<ScenePreset> scenePresets = {
    // Cornell Box scene preset - Quick photon mapping
    {
        "cornell-box",
        "cornell-box.hdr",
        256, 3, 3, true, true, 16, 8, 32, 0.1f,  // Reduce SPP to 16, use photon mapping mode, reduce path depth, increase batch processing size
        true, false, true, false, false, 200, 0.2f,  // Enable MIS but reduce VPL count
        50000, 30000, 5.0f, 16, 0, 0.001f,  // Reduce photon count, increase photon radius, reduce final gathering sampling
        8, 1.0f, 2.0f, 16,
        // Environment light parameters
        0.5f, 0.5f, 0.2f, true
    },
    // Bathroom scene preset - Quick instant radiosity
    {
        "Scenes1/bathroom",
        "bathroom.hdr",
        512, 3, 4, true, true, 4, 8, 64, 0.15f,  // Reduce SPP to 8, use IR mode, reduce path depth, increase tile size
        true, true, true, false, true, 200, 0.2f,  // Enable IR but reduce VPL count
        20000, 10000, 8.0f, 8, 0, 0.001f,  
        8, 1.0f, 2.0f, 16,
        // Environment light parameters
        0.3f, 0.5f, 0.2f, true
    },
    // Materials scene preset - Mixed optimization mode
    {
        "MaterialsScene",
        "MaterialsScene.hdr",
        200, 3, 12, true, false, 4, 8, 8, 0.1f,  // Reduce SPP to 12, use caustic photon mapping mode, reduce path depth
        false, false, true, false, false, 200, 0.2f,  // Disable IR, optimize caustic effect
        10000, 30000, 5.0f, 8, 0, 0.001f,  
        8, 1.0f, 2.0f, 16,
        // Environment light parameters
        0.4f, 0.5f, 0.2f, true
    }
};

// Function to apply scene preset
bool applyScenePreset(int presetIndex, std::string& sceneName, std::string& filename, 
                     unsigned int& SPP, int& renderMode, int& maxPathDepth, 
                     bool& useMultithread, bool& useAdaptiveSampling,
                     int& batchSize, int& denoiseInterval, unsigned int& tileSize, 
                     float& varianceThreshold, bool& enableMIS, bool& enableEnvImportanceSampling,
                     bool& enableDenoising, bool& saveAOVs, bool& enableIR,
                     int& numVPLs, float& vplClampThreshold,
                     int& numGlobalPhotons, int& numCausticPhotons, 
                     float& photonRadius, int& finalGatheringSamples,
                     int& threadCount, float& rayEpsilon,
                     int& maxNodeTriangles, float& traverseCost, 
                     float& triangleCost, int& buildBins,
                     float& envBrightnessScale, float& envImportanceSamplingStrength, 
                     float& envMinSamplingWeight, bool& envUseCosineWeighting) {
    
    if (presetIndex < 0 || presetIndex >= (int)scenePresets.size()) {
        return false;
    }
    
    const ScenePreset& preset = scenePresets[presetIndex];
    
    sceneName = preset.sceneName;
    filename = preset.filename;
    SPP = preset.SPP;
    renderMode = preset.renderMode;
    maxPathDepth = preset.maxPathDepth;
    useMultithread = preset.useMultithread;
    useAdaptiveSampling = preset.useAdaptiveSampling;
    batchSize = preset.batchSize;
    denoiseInterval = preset.denoiseInterval;
    tileSize = preset.tileSize;
    varianceThreshold = preset.varianceThreshold;
    enableMIS = preset.enableMIS;
    enableEnvImportanceSampling = preset.enableEnvImportanceSampling;
    enableDenoising = preset.enableDenoising;
    saveAOVs = preset.saveAOVs;
    enableIR = preset.enableIR;
    numVPLs = preset.numVPLs;
    vplClampThreshold = preset.vplClampThreshold;
    numGlobalPhotons = preset.numGlobalPhotons;
    numCausticPhotons = preset.numCausticPhotons;
    photonRadius = preset.photonRadius;
    finalGatheringSamples = preset.finalGatheringSamples;
    threadCount = preset.threadCount;
    rayEpsilon = preset.rayEpsilon;
    maxNodeTriangles = preset.maxNodeTriangles;
    traverseCost = preset.traverseCost;
    triangleCost = preset.triangleCost;
    buildBins = preset.buildBins;
    envBrightnessScale = preset.envBrightnessScale;
    envImportanceSamplingStrength = preset.envImportanceSamplingStrength;
    envMinSamplingWeight = preset.envMinSamplingWeight;
    envUseCosineWeighting = preset.envUseCosineWeighting;
    
    return true;
}

int main(int argc, char *argv[])
{
	// Ensure output directory exists
	std::string outputDir = "RenderOutput/";
	// Use Windows API to create directory
	if (!CreateDirectoryA("RenderOutput", NULL)) {
		DWORD error = GetLastError();
		if (error != ERROR_ALREADY_EXISTS) {
			std::cerr << "Failed to create output directory, error code: " << error << std::endl;
			// Continue execution, files will be saved in current directory
		}
	} else {
		std::cout << "Created output directory: RenderOutput" << std::endl;
	}

	// Add call to tests if required	
	// runTests()
	
	//=================== Parameter Settings (Start) ===================
	// Scene and output settings
	std::string sceneName = "cornell-box";  // Scene name: cornell-box, Scenes1/bathroom, etc.
	std::string filename = "cornell-box.hdr"; // Output filename
	
	//-------------- Rendering Quality Settings --------------
	unsigned int SPP = 68;                  // Samples per pixel
	int renderMode = 3;                     // Render mode: 3=Path Tracing, 4=Direct Light, 6=Photon Mapping, 7=Caustic Photon Mapping
	int maxPathDepth = 3;                   // Maximum path depth for path tracing
	bool useMultithread = true;              // Enable multithreaded rendering
	bool useAdaptiveSampling = true;        // Enable adaptive sampling
	int batchSize = 4;                      // Batch size for optimized multithreading
	int denoiseInterval = 8;                // Interval for denoising in optimized mode
	unsigned int tileSize = 32;             // Rendering tile size (smaller tiles update display more frequently)
	float varianceThreshold = 0.01f;         // Adaptive sampling threshold - lower values provide more accurate results
	bool enableMIS = true;                  // Enable multiple importance sampling
	bool enableEnvImportanceSampling = false; // Enable environment light importance sampling
	bool enableDenoising = true;            // Enable OIDN denoising
	bool saveAOVs = false;                   // Save AOV (normal, albedo) files
	
	//-------------- Instant Radiosity Settings --------------
	bool enableIR = true;                  // Enable Instant Radiosity
	int numVPLs = 200;                      // Number of VPLs
	float vplClampThreshold = 0.2f;         // VPL clamp threshold to prevent singularities
	
	//-------------- Photon Mapping Settings --------------
	int numGlobalPhotons = 50000;           // Number of global photons
	int numCausticPhotons = 30000;          // Number of caustic photons
	float photonRadius = 5.0f;              // Photon gathering radius
	int finalGatheringSamples = 16;         // Final gathering samples
	
	//-------------- Thread Settings --------------
	int threadCount = 0;                    // Number of rendering threads (0=automatically use system cores-1)
	
	//-------------- Ray Interaction Settings --------------
	float rayEpsilon = 0.001f;              // Ray offset to prevent self-intersection and shadow acne
	                                        // Too small will cause self-intersection, too large will cause light leaks and incorrect shadows
	
	//-------------- BVH Acceleration Structure Settings --------------
	int maxNodeTriangles = 8;               // Maximum triangles per BVH leaf node
	float traverseCost = 1.0f;              // Estimated computation cost for traversing BVH nodes
	float triangleCost = 2.0f;              // Estimated computation cost for triangle intersection tests
	int buildBins = 16;                     // Number of spatial subdivisions when building BVH
	
	//-------------- Environment Light Parameters --------------
	float envBrightnessScale = 1.0f;        // Environment light brightness scale factor
	float envImportanceSamplingStrength = 1.0f; // Importance sampling strength
	float envMinSamplingWeight = 0.1f;      // Minimum sampling weight
	bool envUseCosineWeighting = true;      // Cosine weighted sampling
	
	//-------------- Scene Selection Variables --------------
	int selectedScene = 0;                  // Default to first scene
	
	// Performance monitoring variables (no need to modify)
	double totalRenderTime = 0.0;
	int frames = 0;
	long long totalRays = 0;
	//=================== Parameter Settings (End) ===================

	// Command line scene selection functionality
	bool sceneSelectedFromCommandLine = false;
	
	// Parse command line arguments
	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
			std::string arg = argv[i];
			// Find equals sign position
			size_t pos = arg.find('=');
			if (pos != std::string::npos) {
				// Split parameter name and value
				std::pair<std::string, std::string> pair;
				pair.first = arg.substr(0, pos);
				pair.second = arg.substr(pos + 1);
				
				// Match parameter name
				if (pair.first == "-scene") {
					// Find matching scene preset
					for (int j = 0; j < scenePresets.size(); j++) {
						if (scenePresets[j].sceneName == pair.second) {
							selectedScene = j;
							sceneSelectedFromCommandLine = true;
							
							// Apply selected scene preset
							envBrightnessScale = scenePresets[selectedScene].envBrightnessScale;
							envImportanceSamplingStrength = scenePresets[selectedScene].envImportanceSamplingStrength;
							envMinSamplingWeight = scenePresets[selectedScene].envMinSamplingWeight;
							envUseCosineWeighting = scenePresets[selectedScene].envUseCosineWeighting;
							
							applyScenePreset(selectedScene, sceneName, filename, SPP, renderMode, maxPathDepth, 
								useMultithread, useAdaptiveSampling, batchSize, denoiseInterval, tileSize, 
								varianceThreshold, enableMIS, enableEnvImportanceSampling, enableDenoising, 
								saveAOVs, enableIR, numVPLs, vplClampThreshold, numGlobalPhotons, numCausticPhotons, 
								photonRadius, finalGatheringSamples, threadCount, rayEpsilon, maxNodeTriangles, 
								traverseCost, triangleCost, buildBins, envBrightnessScale, envImportanceSamplingStrength, 
								envMinSamplingWeight, envUseCosineWeighting);
								
							std::cout << "Selected scene from command line: " << sceneName << std::endl;
							break;
						}
					}
				} else if (pair.first == "-scenenum") {
					// Directly select scene by index
					int sceneIndex = std::stoi(pair.second);
					if (sceneIndex >= 0 && sceneIndex < scenePresets.size()) {
						selectedScene = sceneIndex;
						sceneSelectedFromCommandLine = true;
						
						// Apply selected scene preset
						envBrightnessScale = scenePresets[selectedScene].envBrightnessScale;
						envImportanceSamplingStrength = scenePresets[selectedScene].envImportanceSamplingStrength;
						envMinSamplingWeight = scenePresets[selectedScene].envMinSamplingWeight;
						envUseCosineWeighting = scenePresets[selectedScene].envUseCosineWeighting;
						
						applyScenePreset(selectedScene, sceneName, filename, SPP, renderMode, maxPathDepth, 
							useMultithread, useAdaptiveSampling, batchSize, denoiseInterval, tileSize, 
							varianceThreshold, enableMIS, enableEnvImportanceSampling, enableDenoising, 
							saveAOVs, enableIR, numVPLs, vplClampThreshold, numGlobalPhotons, numCausticPhotons, 
							photonRadius, finalGatheringSamples, threadCount, rayEpsilon, maxNodeTriangles, 
							traverseCost, triangleCost, buildBins, envBrightnessScale, envImportanceSamplingStrength, 
							envMinSamplingWeight, envUseCosineWeighting);
							
						std::cout << "Selected scene index from command line " << sceneIndex << ": " << sceneName << std::endl;
					}
				} else if (pair.first == "-mode") {
					renderMode = std::stoi(pair.second);
				} else if (pair.first == "-spp") {
					SPP = std::stoi(pair.second);
				} else if (pair.first == "-depth") {
					maxPathDepth = std::stoi(pair.second);
				} else if (pair.first == "-multithread") {
					useMultithread = (pair.second == "true");
				} else if (pair.first == "-adaptive") {
					useAdaptiveSampling = (pair.second == "true");
				} else if (pair.first == "-tilesize") {
					tileSize = std::stoi(pair.second);
				} else if (pair.first == "-variance") {
					varianceThreshold = std::stof(pair.second);
				} else if (pair.first == "-denoise") {
					enableDenoising = (pair.second == "true");
				} else if (pair.first == "-envimportance") {
					enableEnvImportanceSampling = (pair.second == "true");
				} else if (pair.first == "-mis") {
					enableMIS = (pair.second == "true");
				} else if (pair.first == "-ir") {
					enableIR = (pair.second == "true");
				} else if (pair.first == "-vpls") {
					numVPLs = std::stoi(pair.second);
				} else if (pair.first == "-vplclamping") {
					vplClampThreshold = std::stof(pair.second);
				} else if (pair.first == "-batchsize") {
					batchSize = std::stoi(pair.second);
				} else if (pair.first == "-denoiseinterval") {
					denoiseInterval = std::stoi(pair.second);
				} else if (pair.first == "-globalp") {
					numGlobalPhotons = std::stoi(pair.second);
				} else if (pair.first == "-causticp") {
					numCausticPhotons = std::stoi(pair.second);
				} else if (pair.first == "-pradius") {
					photonRadius = std::stof(pair.second);
				} else if (pair.first == "-gather") {
					finalGatheringSamples = std::stoi(pair.second);
				}
			}
		}
	}
	
	// If no scene selected from command line, prompt user to select
	if (!sceneSelectedFromCommandLine) {
		// Scene selection functionality
		selectedScene = 0; // Default to first scene
		std::cout << "Available scenes:" << std::endl;
		for (int i = 0; i < scenePresets.size(); i++) {
			std::cout << i << ": " << scenePresets[i].sceneName << std::endl;
		}
		std::cout << "Input the scene number (0-" << scenePresets.size()-1 << "): ";
		std::cin >> selectedScene;
		
		// Validate input
		if (selectedScene < 0 || selectedScene >= scenePresets.size()) {
			std::cout << "Invalid scene number (0)" << std::endl;
			selectedScene = 0;
		}
		
		// Apply selected scene preset
		envBrightnessScale = scenePresets[selectedScene].envBrightnessScale;
		envImportanceSamplingStrength = scenePresets[selectedScene].envImportanceSamplingStrength;
		envMinSamplingWeight = scenePresets[selectedScene].envMinSamplingWeight;
		envUseCosineWeighting = scenePresets[selectedScene].envUseCosineWeighting;
		
		applyScenePreset(selectedScene, sceneName, filename, SPP, renderMode, maxPathDepth, 
			useMultithread, useAdaptiveSampling, batchSize, denoiseInterval, tileSize, 
			varianceThreshold, enableMIS, enableEnvImportanceSampling, enableDenoising, 
			saveAOVs, enableIR, numVPLs, vplClampThreshold, numGlobalPhotons, numCausticPhotons, 
			photonRadius, finalGatheringSamples, threadCount, rayEpsilon, maxNodeTriangles, 
			traverseCost, triangleCost, buildBins, envBrightnessScale, envImportanceSamplingStrength, 
			envMinSamplingWeight, envUseCosineWeighting);
		
		std::cout << "Selected scene: " << sceneName << std::endl;
	}

	Scene* scene = loadScene(sceneName);
	if (!scene) {
		std::cerr << "Failed to load scene: " << sceneName << std::endl;
		return -1;
	}
	
	// Set environment light parameters
	if (scene->background) {
		EnvironmentMap* envMap = dynamic_cast<EnvironmentMap*>(scene->background);
		if (envMap) {
			envMap->brightnessScale = envBrightnessScale;
			envMap->setImportanceSamplingStrength(envImportanceSamplingStrength);
			envMap->setMinSamplingWeight(envMinSamplingWeight);
			envMap->setUseCosineWeighting(envUseCosineWeighting);
			
			std::cout << "Environment light parameters set:" << std::endl;
			std::cout << "   Brightness scale: " << envBrightnessScale << std::endl;
			std::cout << "   Importance sampling strength: " << envImportanceSamplingStrength << std::endl;
			std::cout << "   Minimum sampling weight: " << envMinSamplingWeight << std::endl;
			std::cout << "   Cosine weighting: " << (envUseCosineWeighting ? "Enabled" : "Disabled") << std::endl;
		}
	}
	
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", 1.0f);
	
	// Add popup prompt to inform user about rendering features
	MessageBoxA(NULL, 
		"                                                                                \n\n"
		"                   HI PROFESSOR :D                 \n\n"
		"THIS RENDERER USES BATCH RENDERING METHOD (4 SAMPLES PER BATCH).\n\n"
		"FIRST BATCH MAY TAKE SOME TIME TO CALCULATE.\n\n"
		"*********************************************\n"
		"NO IMAGE WILL APPEAR until FIRST BATCH COMPLETES.\n"
		"*********************************************\n\n"
		"                  Please wait patiently.                  \n\n"
		"                                                                                ",
		"!!! IMPORTANT NOTICE - PLEASE READ !!!",
		MB_OK | MB_ICONINFORMATION);
	
	RayTracer rt;
	rt.init(scene, &canvas);
	
	// Apply above set parameters
	rt.setRenderMode(renderMode);
	rt.SPP = SPP;
	rt.maxPathDepth = maxPathDepth;
	rt.multithreaded = useMultithread;
	rt.batchSize = batchSize;
	rt.denoiseInterval = denoiseInterval;
	rt.adaptiveSampling = useAdaptiveSampling;
	rt.tileSize = tileSize;
	rt.varianceThreshold = varianceThreshold; // Set adaptive sampling threshold
	rt.setEnableMIS(enableMIS);
	rt.setEnableImportanceSamplingEnv(enableEnvImportanceSampling);
	rt.setEnableDenoising(enableDenoising); // Set whether to enable denoising
	
	// Set IR related parameters
	rt.setEnableIR(enableIR);
	rt.setNumVPLs(numVPLs);
	rt.setVPLClampThreshold(vplClampThreshold);
	
	// Set photon mapping related parameters
	rt.setPhotonCounts(numGlobalPhotons, numCausticPhotons);
	rt.setPhotonRadius(photonRadius);
	rt.setFinalGatheringSamples(finalGatheringSamples);
	
	// Automatically enable corresponding features based on render mode
	if (renderMode >= 6 && renderMode <= 8) {
		// All photon mapping modes enable photon mapping
		rt.setEnablePhotonMapping(true);
		
		// Mode 7 focuses on caustic effect
		bool useCausticPhotons = (renderMode == 6 || renderMode == 7 || renderMode == 8);
		rt.setEnableCausticPhotons(useCausticPhotons);
		
		// Only mode 8 enables final gathering
		bool useFinalGathering = (renderMode == 8);
		rt.setEnableFinalGathering(useFinalGathering);
	} else {
		// Other modes disable photon mapping
		rt.setEnablePhotonMapping(false);
		rt.setEnableCausticPhotons(false);
		rt.setEnableFinalGathering(false);
	}
	
	// Apply thread count settings
	if (threadCount > 0) {
		rt.numProcs = threadCount;  // Set specified thread count
	}
	
	// Automatically adjust SPP - When enabling MIS and environment light importance sampling, fewer samples can be used to achieve the same quality
	if (enableMIS && enableEnvImportanceSampling && SPP > 16) {
		int originalSPP = rt.SPP;
		// Simplified adjustment logic - Use simpler mathematical calculation
		int newSPP = originalSPP / 2; // Simple direct halving
		if (newSPP < 16) newSPP = 16; // Set minimum value to ensure quality
		
		rt.SPP = newSPP;
		std::cout << "Note: Due to enabling MIS and environment importance sampling, automatically reduced SPP from " << originalSPP 
		          << " to " << rt.SPP << " (similar quality but faster)" << std::endl;
	}
	
	// Set BVH and ray epsilon parameters
	USE_MAX_TRIANGLES = maxNodeTriangles;
	USE_TRAVERSE_COST = traverseCost;
	USE_TRIANGLE_COST = triangleCost;
	USE_BUILD_BINS = buildBins;
	RAY_EPSILON = rayEpsilon;
	
	// Scene initialization completed, print scene information
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
	
	// BVH information (if available)
	printRow("BVH Depth", std::to_string(scene->bvh->getDepth()));
	printRow("BVH Node Count", std::to_string(scene->bvh->getNodeCount()));
	printRow("BVH Max Triangles", std::to_string(maxNodeTriangles));
	
	// Other rendering parameter information
	printRow("Thread Mode", threadCount == 0 ? "Auto" : std::to_string(threadCount));
	printRow("Ray Epsilon", std::to_string(rayEpsilon));
	printRow("Variance Threshold", std::to_string(varianceThreshold));
	
	printSeparator();
	
	std::cout << "Starting rendering... Press ESC to stop." << std::endl;
	
	// Ensure screen is cleared before rendering
	canvas.clear();
	
	// Print rendering parameters
	std::cout << "[Rendering Configuration]" << std::endl;
	std::cout << " - Mode: " << getRenderModeName(rt.getRenderMode()) << " (Mode " << rt.getRenderMode() << ")" << std::endl;
	std::cout << " - SPP: " << rt.getSPP() << std::endl;
	std::cout << " - Max Path Depth: " << rt.maxPathDepth << std::endl;
	std::cout << " - Multithreading: " << (rt.multithreaded ? "Enabled" : "Disabled") << std::endl;
	std::cout << " - Batch Size: " << rt.batchSize << std::endl;
	std::cout << " - Denoise Interval: " << rt.denoiseInterval << " samples" << std::endl;
	if (rt.multithreaded) {
		std::cout << " - Tile Size: " << rt.tileSize << "x" << rt.tileSize << std::endl;
	}
	std::cout << " - Adaptive Sampling: " << (rt.adaptiveSampling ? "Enabled" : "Disabled") << std::endl;
	if (rt.adaptiveSampling) {
		std::cout << " - Variance Threshold: " << rt.varianceThreshold << std::endl;
	}
	std::cout << " - MIS: " << (rt.isEnableMIS() ? "Enabled" : "Disabled") << std::endl;
	std::cout << " - Environment Importance Sampling: " << (rt.isEnableImportanceSamplingEnv() ? "Enabled" : "Disabled") << std::endl;
	std::cout << " - Denoising: " << (rt.enableDenoising ? "Enabled" : "Disabled") << std::endl;
	
	// IR information
	std::cout << " - Enable IR: " << (enableIR ? "Enabled" : "Disabled") << std::endl;
	if (enableIR) {
		std::cout << " - Num VPLs: " << numVPLs << std::endl;
		std::cout << " - VPL Clamp Threshold: " << vplClampThreshold << std::endl;
	}
	
	// Photon mapping information
	if (renderMode >= 6 && renderMode <= 8) {
		std::cout << " - Photon Mapping: Enabled (Mode " << renderMode << ")" << std::endl;
		
		if (renderMode == 6) {
			std::cout << " - Mode: Standard Photon Mapping" << std::endl;
		} else if (renderMode == 7) {
			std::cout << " - Mode: Caustic Photon Mapping" << std::endl;
		} else if (renderMode == 8) {
			std::cout << " - Mode: Photon Mapping with Final Gathering" << std::endl;
		}
		
		std::cout << " - Global Photons: " << numGlobalPhotons << std::endl;
		
		if (renderMode == 6 || renderMode == 7 || renderMode == 8) {
			std::cout << " - Caustic Photons: " << numCausticPhotons << std::endl;
		}
		
		std::cout << " - Photon Radius: " << photonRadius << std::endl;
		
		if (renderMode == 8) {
			std::cout << " - Final Gathering Samples: " << finalGatheringSamples << std::endl;
		}
	} else {
		std::cout << " - Photon Mapping: Disabled" << std::endl;
	}
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
		
		// Check if all samples have been completed
		if (rt.film->SPP >= rt.SPP) {
			// Target sample count reached, stop rendering
			if (frames == 0) {
				// Only save results the first time we reach the target
				std::cout << "Rendering completed with " << rt.film->SPP << " samples." << std::endl;
				// Save results
				std::string fullFilename = outputDir + filename;
				rt.saveHDR(fullFilename);
				std::cout << "Image saved to " << fullFilename << std::endl;
				
				std::string baseFilename = filename.substr(0, filename.find_last_of('.'));
				std::string pngFilename = outputDir + baseFilename + ".png";
				rt.savePNG(pngFilename);
				std::cout << "PNG image saved to " << pngFilename << std::endl;
				
				// If denoising is enabled, execute denoising
				if (enableDenoising) {
					rt.denoise();
					
					// Display denoised image
					if (rt.film->hasAOVs) {
						displayDenoisedImage(rt, canvas);
						
						// Save denoised image
						std::string denoisedHDRFilename = outputDir + baseFilename + "_denoised.hdr";
						stbi_write_hdr(denoisedHDRFilename.c_str(), rt.film->width, rt.film->height, 3, (float*)rt.film->denoisedBuffer);
						std::cout << "Denoised HDR image saved to " << denoisedHDRFilename << std::endl;
						
						std::string denoisedPNGFilename = outputDir + baseFilename + "_denoised.png";
						rt.film->saveTonemappedPNG(denoisedPNGFilename, 1.0f, rt.film->denoisedBuffer);
						std::cout << "Denoised PNG image saved to " << denoisedPNGFilename << std::endl;
					}
				}
				
				// If saveAOVs is enabled, save AOV images
				if (saveAOVs) {
					rt.saveAOVs(outputDir + baseFilename);
					std::cout << "AOV images saved with prefix " << outputDir + baseFilename << std::endl;
				}
				
				frames = 1; // Mark rendering as completed
			}
		} else {
			// Target sample count not yet reached, continue rendering
			// Start rendering timer
			timer.reset();
			// Render next sample
			rt.render();
			// Calculate rendering time
			double renderTime = timer.dt();
			totalRenderTime += renderTime;
			frames++;
			
			std::cout << "Sample " << rt.film->SPP << "/" << rt.SPP << " completed in " << renderTime << " seconds." << std::endl;
			
			// Periodically display updated image
			canvas.present();
			
			// If target sample count reached, save final results
			if (rt.film->SPP >= rt.SPP) {
				// Save results
				std::string fullFilename = outputDir + filename;
				rt.saveHDR(fullFilename);
				std::cout << "Image saved to " << fullFilename << std::endl;
				
				std::string baseFilename = filename.substr(0, filename.find_last_of('.'));
				std::string pngFilename = outputDir + baseFilename + ".png";
				rt.savePNG(pngFilename);
				std::cout << "PNG image saved to " << pngFilename << std::endl;
				
				// If denoising is enabled, execute denoising
				if (enableDenoising) {
					rt.denoise();
					
					// Display denoised image
					if (rt.film->hasAOVs) {
						displayDenoisedImage(rt, canvas);
						
						// Save denoised image
						std::string denoisedHDRFilename = outputDir + baseFilename + "_denoised.hdr";
						stbi_write_hdr(denoisedHDRFilename.c_str(), rt.film->width, rt.film->height, 3, (float*)rt.film->denoisedBuffer);
						std::cout << "Denoised HDR image saved to " << denoisedHDRFilename << std::endl;
						
						std::string denoisedPNGFilename = outputDir + baseFilename + "_denoised.png";
						rt.film->saveTonemappedPNG(denoisedPNGFilename, 1.0f, rt.film->denoisedBuffer);
						std::cout << "Denoised PNG image saved to " << denoisedPNGFilename << std::endl;
					}
				}
				
				// If saveAOVs is enabled, save AOV images
				if (saveAOVs) {
					rt.saveAOVs(outputDir + baseFilename);
					std::cout << "AOV images saved with prefix " << outputDir + baseFilename << std::endl;
				}
			}
		}
		
		// Scene camera roaming feature
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward();
			rt.clear();
			canvas.clear();
			frames = 0; // Reset rendering state to start over
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
		
		// Add photon mapping render mode switch shortcut keys
		if (canvas.keyPressed('6'))
		{
			rt.setRenderMode(6); // Standard photon mapping
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		if (canvas.keyPressed('7'))
		{
			rt.setRenderMode(7); // Caustic photon mapping
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		if (canvas.keyPressed('8'))
		{
			rt.setRenderMode(8); // Photon mapping with final gathering
			rt.clear();
			canvas.clear();
			frames = 0;
		}
		
		// Simple delay to reduce CPU usage
		Sleep(16); // Approximately 60 FPS
	}

	// Display complete help information
	std::cout << "Command Line Usage Examples:" << std::endl;
	std::cout << "  RTBase.exe -scene Scenes1/bathroom -SPP 64 -renderMode 1" << std::endl;
	std::cout << "  RTBase.exe -scene cornell-box -SPP 128 -renderMode 2 -useMultithread true -enableMIS true" << std::endl;
	std::cout << "  RTBase.exe -scene MaterialsScene -SPP 256 -renderMode 1 -enableDenoising true -maxPathDepth 5" << std::endl;
	std::cout << "  RTBase.exe -scene cornell-box -SPP 64 -renderMode 5 -numVPLs 1000 -vplClampThreshold 0.1" << std::endl;
	std::cout << "  RTBase.exe -scene Scenes1/bathroom -SPP 32 -renderMode 6 -numGlobalPhotons 500000" << std::endl;
	std::cout << "  RTBase.exe -scene cornell-box -SPP 16 -renderMode 7 -numCausticPhotons 500000" << std::endl;
	std::cout << "  RTBase.exe -scene cornell-box -SPP 16 -renderMode 8 -finalGatheringSamples 128" << std::endl;
	std::cout << std::endl;
	
	// Add help information about scene preset and environment light parameter control
	std::cout << "Scene Preset System Usage Instructions:" << std::endl;
	std::cout << "  1. Command line select scene: RTBase.exe -scene <scene name>" << std::endl;
	std::cout << "  2. Command line select scene by index: RTBase.exe -scenenum <scene index>" << std::endl;
	std::cout << "  3. If no scene specified, runtime will prompt to select scene" << std::endl;
	std::cout << std::endl;
	
	std::cout << "Preset Scene List:" << std::endl;
	for (int i = 0; i < scenePresets.size(); i++) {
		std::cout << "  " << i << ": " << scenePresets[i].sceneName << std::endl;
		std::cout << "     - Brightness Scale: " << scenePresets[i].envBrightnessScale << std::endl;
		std::cout << "     - Importance Sampling Strength: " << scenePresets[i].envImportanceSamplingStrength << std::endl;
		std::cout << "     - Minimum Sampling Weight: " << scenePresets[i].envMinSamplingWeight << std::endl;
		std::cout << "     - Cosine Weighting: " << (scenePresets[i].envUseCosineWeighting ? "Enabled" : "Disabled") << std::endl;
		std::cout << "     - Recommended Render Mode: " << scenePresets[i].renderMode << " (" << getRenderModeName(scenePresets[i].renderMode) << ")" << std::endl;
		std::cout << "     - Recommended Sampling Count: " << scenePresets[i].SPP << std::endl;
		std::cout << std::endl;
	}

	return 0;
}