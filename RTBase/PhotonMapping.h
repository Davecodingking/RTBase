#pragma once

#include "Core.h"

// Add helper methods for Colour type
inline bool isBlack(const Colour& c) {
    return c.r <= 0.0f && c.g <= 0.0f && c.b <= 0.0f;
}

inline float maxComponent(const Colour& c) {
    return std::max(c.r, std::max(c.g, c.b));
}

#include "Geometry.h"
#include "Sampling.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include <vector>
#include <algorithm>
#include <functional>
#include <memory>
#include <iostream>
#include <mutex>

// Photon structure definition
struct Photon {
    Vec3 position;    // Photon position
    Vec3 direction;   // Incident direction (normalized)
    Colour power;     // Photon power/energy (RGB)
    bool isCaustic;   // Whether it's a caustic photon (passed through at least one specular surface)
    
    Photon() : position(0,0,0), direction(0,0,1), power(0,0,0), isCaustic(false) {}
    
    Photon(const Vec3& pos, const Vec3& dir, const Colour& pow, bool caustic = false)
        : position(pos), direction(dir), power(pow), isCaustic(caustic) {}
};

// KD-Tree node
class KDNode {
public:
    Photon photon;
    KDNode* left;
    KDNode* right;
    int axis; // Split axis (0=x, 1=y, 2=z)
    
    KDNode() : left(nullptr), right(nullptr), axis(0) {}
    ~KDNode() {
        delete left;
        delete right;
    }
};

// KD-Tree implementation
class KDTree {
private:
    KDNode* root;
    int photonCount;
    
    // Recursively build KD-Tree
    KDNode* buildRecursive(std::vector<Photon>& photons, int start, int end, int depth) {
        if (start >= end) return nullptr;
        
        // Choose split axis (x=0, y=1, z=2)
        int axis = depth % 3;
        
        // Sort photons based on current axis
        int mid = (start + end) / 2;
        std::nth_element(photons.begin() + start, 
                          photons.begin() + mid,
                          photons.begin() + end,
                          [axis](const Photon& a, const Photon& b) {
                              return a.position.coords[axis] < b.position.coords[axis];
                          });
        
        // Create current node
        KDNode* node = new KDNode();
        node->photon = photons[mid];
        node->axis = axis;
        
        // Recursively build left and right subtrees
        node->left = buildRecursive(photons, start, mid, depth + 1);
        node->right = buildRecursive(photons, mid + 1, end, depth + 1);
        
        return node;
    }
    
    // Recursively query photons within given radius
    void queryRadiusRecursive(KDNode* node, const Vec3& point, float radius, float& squaredRadius, 
                             std::vector<std::pair<Photon, float>>& result) const {
        if (!node) return;
        
        // Calculate distance from current photon to query point
        Vec3 diff = node->photon.position - point;
        float squaredDist = diff.dot(diff);
        
        // If within radius, add to result set
        if (squaredDist < squaredRadius) {
            result.push_back(std::make_pair(node->photon, squaredDist));
        }
        
        // Determine if we need to traverse both subtrees
        float distOnAxis = point.coords[node->axis] - node->photon.position.coords[node->axis];
        float squaredDistOnAxis = distOnAxis * distOnAxis;
        
        // Prioritize traversal of subtree more likely to contain results
        KDNode* first = (distOnAxis <= 0) ? node->left : node->right;
        KDNode* second = (distOnAxis <= 0) ? node->right : node->left;
        
        // Recursively search subtree that might contain results
        queryRadiusRecursive(first, point, radius, squaredRadius, result);
        
        // Check if we need to search the other subtree
        if (squaredDistOnAxis < squaredRadius) {
            queryRadiusRecursive(second, point, radius, squaredRadius, result);
        }
    }
    
public:
    KDTree() : root(nullptr), photonCount(0) {}
    
    ~KDTree() {
        delete root;
    }
    
    // Build KD-Tree
    void build(std::vector<Photon>& photons) {
        delete root; // Clear old tree
        root = nullptr;
        
        if (photons.empty()) return;
        
        photonCount = photons.size();
        root = buildRecursive(photons, 0, photons.size(), 0);
    }
    
    // Query photons within given radius
    std::vector<std::pair<Photon, float>> queryRadius(const Vec3& point, float radius) const {
        std::vector<std::pair<Photon, float>> result;
        float squaredRadius = radius * radius;
        queryRadiusRecursive(root, point, radius, squaredRadius, result);
        return result;
    }
    
    // Return the number of stored photons
    int size() const {
        return photonCount;
    }
};

// Main Photon Mapping class
class PhotonMapper {
private:
    std::vector<Photon> globalPhotons;    // Store global photons
    std::vector<Photon> causticPhotons;   // Store caustic photons
    KDTree globalPhotonMap;               // KD-Tree for global photons
    KDTree causticPhotonMap;              // KD-Tree for caustic photons
    int numGlobalPhotons;                 // Number of global photons
    int numCausticPhotons;                // Number of caustic photons
    float gatherRadius;                   // Photon gathering radius
    Scene* scene;                         // Scene pointer
    std::mutex causticMutex;              // Mutex for caustic photons
    std::mutex globalMutex;               // Mutex for global photons
    // Random number generator
    MTRandom rng;
    // Maximum depth
    int maxDepth = 10;
    // Camera reference
    Camera camera;
    
public:
    // Keep the original constructor interface
    PhotonMapper(Scene* _scene, int _numGlobalPhotons = 100000, int _numCausticPhotons = 100000, 
                float _gatherRadius = 1.5f) 
        : scene(_scene), numGlobalPhotons(_numGlobalPhotons), 
          numCausticPhotons(_numCausticPhotons), gatherRadius(_gatherRadius) 
    {
        // Default camera initialization
        // We don't use camera in this constructor, but keep it for compatibility
    }

    // New constructor interface with camera
    PhotonMapper(Scene* _scene, Camera _camera, int _numGlobalPhotons = 100000, int _numCausticPhotons = 100000, 
                float _gatherRadius = 1.5f) 
        : scene(_scene), camera(_camera), numGlobalPhotons(_numGlobalPhotons), 
          numCausticPhotons(_numCausticPhotons), gatherRadius(_gatherRadius) {}
    
    // Emit and trace photons
    void emitPhotons(bool emitCausticPhotons = true, bool emitGlobalPhotons = true) {
        if (emitGlobalPhotons) {
            globalPhotons.clear();
            // Reserve space for global photons
            globalPhotons.reserve(numGlobalPhotons);
        }
        
        if (emitCausticPhotons) {
            causticPhotons.clear();
            // Reserve space for caustic photons
            causticPhotons.reserve(numCausticPhotons);
        }
        
        // Emit photons from all light sources
        emitPhotonsFromLights(emitCausticPhotons, emitGlobalPhotons);
        
        // Build KD trees
        if (emitGlobalPhotons) {
            globalPhotonMap.build(globalPhotons);
        }
        
        if (emitCausticPhotons) {
            causticPhotonMap.build(causticPhotons);
        }
        
        std::cout << "Launched " << globalPhotons.size() << " global photons and " 
                 << causticPhotons.size() << " caustic photons" << std::endl;
    }
    
    // Original version of emitPhotonsFromLights function - for compatibility with old code
    void emitPhotonsFromLights(bool emitCausticPhotons = true, bool emitGlobalPhotons = true) {
        // Total power of all lights
        float totalPower = 0.0f;
        for (auto light : scene->lights) {
            totalPower += light->totalIntegratedPower();
        }
        
        if (totalPower <= 0.0f) {
            std::cout << "WARNING: No light power detected in the scene!" << std::endl;
            return;
        }
        
        std::cout << "Emitting photons with total light power: " << totalPower << std::endl;
        
        // Sample and emit global photons
        if (emitGlobalPhotons) {
            for (int i = 0; i < numGlobalPhotons; i++) {
                // Select a light source
                float lightSelectProb = 1.0f / scene->lights.size();
                int lightIdx = std::min((int)(MTRandom::randomFloat() * scene->lights.size()), 
                                    (int)(scene->lights.size() - 1));
                Light* light = scene->lights[lightIdx];
                
                // Emit photon
                float pdfPos = 0.0f;
                float pdfDir = 0.0f;
                Vec3 pos = light->samplePositionFromLight(&rng, pdfPos);
                Vec3 dir = light->sampleDirectionFromLight(&rng, pdfDir);
                
                // Calculate photon power
                Colour power = Colour(0.0f, 0.0f, 0.0f);
                if (light->isArea()) {
                    // Area light - keep full RGB color information
                    AreaLight* areaLight = static_cast<AreaLight*>(light);
                    power.r = areaLight->emission.r * (totalPower / numGlobalPhotons) * 20.0f;
                    power.g = areaLight->emission.g * (totalPower / numGlobalPhotons) * 20.0f;
                    power.b = areaLight->emission.b * (totalPower / numGlobalPhotons) * 20.0f;
                } else {
                    ShadingData dummyData;
                    Colour lightEmission = light->evaluate(dummyData, -dir);
                    // keep full RGB color information
                    power.r = lightEmission.r * (totalPower / numGlobalPhotons) * 20.0f;
                    power.g = lightEmission.g * (totalPower / numGlobalPhotons) * 20.0f;
                    power.b = lightEmission.b * (totalPower / numGlobalPhotons) * 20.0f;
                }
                
                // Consider light PDF
                if (pdfPos > 0.0f && pdfDir > 0.0f) {
                    power.r = power.r / (pdfPos * pdfDir * lightSelectProb);
                    power.g = power.g / (pdfPos * pdfDir * lightSelectProb);
                    power.b = power.b / (pdfPos * pdfDir * lightSelectProb);
                }
                
                // Debug first photon power
                if (i == 0) {
                    std::cout << "First global photon power: " 
                              << power.r << ", " << power.g << ", " << power.b 
                              << " (pdf pos: " << pdfPos << ", pdf dir: " << pdfDir << ")" << std::endl;
                }
                
                // Add photon to list
                Ray ray(pos, dir);
                tracePhotonToList(ray, power, false, 0, globalPhotons, false);
            }
        }
        
        // Sample and emit caustic photons
        if (emitCausticPhotons) {
            for (int i = 0; i < numCausticPhotons; i++) {
                // Select a light source
                float lightSelectProb = 1.0f / scene->lights.size();
                int lightIdx = std::min((int)(MTRandom::randomFloat() * scene->lights.size()), 
                                    (int)(scene->lights.size() - 1));
                Light* light = scene->lights[lightIdx];
                
                // Emit photon
                float pdfPos = 0.0f;
                float pdfDir = 0.0f;
                Vec3 pos = light->samplePositionFromLight(&rng, pdfPos);
                Vec3 dir = light->sampleDirectionFromLight(&rng, pdfDir);
                
                // Calculate photon power
                Colour power = Colour(0.0f, 0.0f, 0.0f);
                if (light->isArea()) {
                    // Area light - keep full RGB color information
                    AreaLight* areaLight = static_cast<AreaLight*>(light);
                    power.r = areaLight->emission.r * (totalPower / numCausticPhotons) * 20.0f;
                    power.g = areaLight->emission.g * (totalPower / numCausticPhotons) * 20.0f;
                    power.b = areaLight->emission.b * (totalPower / numCausticPhotons) * 20.0f;
                } else {
                    ShadingData dummyData;
                    Colour lightEmission = light->evaluate(dummyData, -dir);
                    // keep full RGB color information
                    power.r = lightEmission.r * (totalPower / numCausticPhotons) * 20.0f;
                    power.g = lightEmission.g * (totalPower / numCausticPhotons) * 20.0f;
                    power.b = lightEmission.b * (totalPower / numCausticPhotons) * 20.0f;
                }
                
                // Consider light PDF
                if (pdfPos > 0.0f && pdfDir > 0.0f) {
                    power.r = power.r / (pdfPos * pdfDir * lightSelectProb);
                    power.g = power.g / (pdfPos * pdfDir * lightSelectProb);
                    power.b = power.b / (pdfPos * pdfDir * lightSelectProb);
                }
                
                // Debug first photon power
                if (i == 0) {
                    std::cout << "First caustic photon power: " 
                              << power.r << ", " << power.g << ", " << power.b 
                              << " (pdf pos: " << pdfPos << ", pdf dir: " << pdfDir << ")" << std::endl;
                }
                
                // Add photon to list
                Ray ray(pos, dir);
                tracePhotonToList(ray, power, true, 0, causticPhotons, true);
            }
        }
    }
    
    // Modify photon tracing algorithm to add photons to list instead of directly storing them in member variables
    void tracePhotonToList(const Ray& ray, const Colour& power, bool causticPath, int depth, 
                      std::vector<Photon>& photonList, bool causticOnly) {
        // Ensure photon energy is zero when exiting early
        if (power.r <= 0.0f && power.g <= 0.0f && power.b <= 0.0f) return;
        
        // Reach maximum depth, stop tracing
        if (depth >= maxDepth) return;
        
        // Add photon energy monitoring
        static bool debugEnergyPrinted = false;
        if (!debugEnergyPrinted && (power.r > 0.1f || power.g > 0.1f || power.b > 0.1f)) {
            std::cout << "Photon power at depth " << depth << ": " 
                      << power.r << ", " << power.g << ", " << power.b << std::endl;
            debugEnergyPrinted = true;
        }
        
        // Intersect with scene
        IntersectionData isect = scene->traverse(ray);
        
        // If no hit, photon disappears
        if (isect.t == FLT_MAX) return;
        
        // Calculate shading data - fix Ray parameter type issue
        Ray rayNonConst = ray; // Create non-const copy
        ShadingData shadingData = scene->calculateShadingData(isect, rayNonConst);
        
        // Get material
        BSDF* material = scene->materials[scene->triangles[isect.ID].materialIndex];
        shadingData.bsdf = material;
        
        // If light source, ignore (photon emitted from light, should not hit light again)
        if (material->isLight()) return;
        
        // Check if it's a specular material - fix isSpecular method missing issue
        // Use other method to check if it's a specular material, like !isDiffuse()
        bool isSpecular = !material->isDiffuse();
        
        // Mark current photon's state (caustic path)
        bool newCausticPath = causticPath;
        if (isSpecular && !causticPath) {
            // First time from diffuse to specular, mark as caustic path
            newCausticPath = true;
        } else if (!isSpecular && causticPath) {
            // Caustic photon hits diffuse surface, store photon
            if (!causticOnly || causticPath) {
                // Keep full color information when creating photon
                Photon photon(shadingData.x, -ray.dir, power, causticPath);
                
                // Relax photon storage condition
                photonList.push_back(photon);
            }
            
            // Caustic path ends (caustic photon stored)
            newCausticPath = false;
        }
        
        // If it's a diffuse surface, store global photon (non-caustic path)
        if (!isSpecular && !causticPath && !causticOnly) {
            // Keep full color information when creating photon
            Photon photon(shadingData.x, -ray.dir, power, false);
            
            // Relax photon storage condition
            photonList.push_back(photon);
        }
        
        // Russian roulette decision to continue propagation - increase termination probability
        float luminance = 0.2126f * power.r + 0.7152f * power.g + 0.0722f * power.b;
        float surviveProb = std::min(0.6f, luminance); // Reduce survival probability, speed up calculation (original 0.8f)
        
        if (MTRandom::randomFloat() > surviveProb) {
            return;  // Photon absorbed
        }
        
        // Simplified subsequent photon tracing - reduce survival probability for large depth
        if (depth > 2) {
            if (MTRandom::randomFloat() > 0.5f) {
                return; // Further reduce depth large photon
            }
        }
        
        // Sample BSDF to decide new direction
        Vec3 wi;
        Colour brdf;
        float pdf;
        
        // Fix BSDF::sample call, use correct interface
        // Use member variable rng as sampler instead of null
        wi = material->sample(shadingData, &rng, brdf, pdf);
        
        // Invalid direction or zero BRDF, photon terminates
        if (pdf <= 0.0f || (brdf.r <= 0.0f && brdf.g <= 0.0f && brdf.b <= 0.0f)) {
            return;
        }
        
        // Calculate new photon power - ensure separate processing for RGB
        Colour newPower;
        newPower.r = power.r * brdf.r / (pdf * surviveProb);
        newPower.g = power.g * brdf.g / (pdf * surviveProb);
        newPower.b = power.b * brdf.b / (pdf * surviveProb);
        
        // Recursive photon tracing
        Ray newRay(shadingData.x + shadingData.sNormal * RAY_EPSILON, wi);
        tracePhotonToList(newRay, newPower, newCausticPath, depth + 1, photonList, causticOnly);
    }
    
    // Estimate indirect lighting
    Colour estimateIndirectLight(const ShadingData& shadingData, bool useCaustic = true, bool useGlobal = true) const {
        Colour result(0.0f, 0.0f, 0.0f);
        
        // Gather global photon lighting
        if (useGlobal) {
            result = result + estimatePhotonDensity(shadingData, globalPhotonMap);
        }
        
        // Gather caustic photon lighting
        if (useCaustic) {
            result = result + estimatePhotonDensity(shadingData, causticPhotonMap);
        }
        
        return result;
    }
    
    // Estimate photon density at given point using kernel function
    Colour estimatePhotonDensity(const ShadingData& shadingData, const KDTree& photonMap) const {
        // Query photons within radius
        auto photonsWithDist = photonMap.queryRadius(shadingData.x, gatherRadius);
        
        // Add debug information
        static bool debugInfoPrinted = false;
        if (!debugInfoPrinted && !photonsWithDist.empty()) {
            std::cout << "Photon density: " << photonsWithDist.size() << " photons" << std::endl;
            std::cout << "First photon power: " 
                      << photonsWithDist[0].first.power.r << ", "
                      << photonsWithDist[0].first.power.g << ", "
                      << photonsWithDist[0].first.power.b << std::endl;
            std::cout << "Using gathering radius: " << gatherRadius << std::endl;
            debugInfoPrinted = true;
        }
        
        if (photonsWithDist.empty()) return Colour(0.0f, 0.0f, 0.0f);
        
        // Optimize: limit maximum photon count to speed up calculation
        const int maxPhotonsToConsider = 100; // Limit used maximum photon count, speed up estimation
        int numPhotonsToUse = std::min((int)photonsWithDist.size(), maxPhotonsToConsider);
        
        Colour sum(0.0f, 0.0f, 0.0f);
        
        for (int i = 0; i < numPhotonsToUse; i++) {
            const Photon& photon = photonsWithDist[i].first;
            float squaredDist = photonsWithDist[i].second;
            
            // Improved kernel function: use Gaussian kernel function, provide smoother photon density estimation
            float alpha = 0.918f; // Reduce parameter to expand influence range (original 1.818f)
            float kernelValue = std::exp(-alpha * squaredDist / (gatherRadius * gatherRadius));
            
            kernelValue = std::max(0.0f, kernelValue);
            
            // Only consider hemisphere in normal direction
            Vec3 photonDir = photon.direction;  // Note: Already the reverse of incident direction
            if (Dot(photonDir, shadingData.sNormal) > 0.0f) {
                // Calculate BSDF - simplify calculation to speed up performance
                Colour bsdf = shadingData.bsdf->evaluate(shadingData, photonDir);
                
                // Modify: ensure separate component processing for color calculation
                float rContrib = photon.power.r * bsdf.r * kernelValue;
                float gContrib = photon.power.g * bsdf.g * kernelValue;
                float bContrib = photon.power.b * bsdf.b * kernelValue;
                
                sum.r += rContrib;
                sum.g += gContrib;
                sum.b += bContrib;
            }
        }
        
        // Modify normalization factor: use correct photon density formula
        float area = M_PI * gatherRadius * gatherRadius;
        int numPhotons = photonsWithDist.size(); // Use actual photon count for normalization
        
        // Avoid division by zero
        if (numPhotons == 0) return Colour(0.0f, 0.0f, 0.0f);
        
        // Modify: use more accurate normalization formula, but increase photon brightness factor
        float scale = 1.0f / (area * maxPhotonsToConsider);
        
        // Adjust visibility - increase brightness factor
        scale *= 100.0f; // Larger brightness factor to ensure photon mapping visibility (original 50.0f)
        
        // Gamma correction for result, increase brightness perception
        Colour result(sum.r * scale, sum.g * scale, sum.b * scale);
        
        // Ensure result is not black
        if (result.r < 0.001f && result.g < 0.001f && result.b < 0.001f) {
            // Provide minimum brightness for near-black result
            float minValue = 0.001f;
            result.r = std::max(result.r, minValue * sum.r > 0 ? 1.0f : 0.0f);
            result.g = std::max(result.g, minValue * sum.g > 0 ? 1.0f : 0.0f);
            result.b = std::max(result.b, minValue * sum.b > 0 ? 1.0f : 0.0f);
        }
        
        return result;
    }
    
    // Final Gathering optimization - optimize final gathering
    Colour finalGathering(const ShadingData& shadingData, int numSamples = 64) const {
        // Optimize: reduce sample count to speed up calculation
        numSamples = std::min(numSamples, 32); // Limit maximum sample count
        
        Colour sum(0.0f, 0.0f, 0.0f);
        
        // Build local coordinate system
        Frame frame;
        frame.fromVector(shadingData.sNormal);
        
        // Sample multiple directions and send secondary rays
        for (int i = 0; i < numSamples; i++) {
            // Use cosine-weighted hemisphere sampling
            float r1 = MTRandom::randomFloat();
            float r2 = MTRandom::randomFloat();
            float phi = 2.0f * M_PI * r1;
            float cosTheta = sqrtf(r2);
            float sinTheta = sqrtf(1.0f - r2);
            
            // Calculate direction in local space
            Vec3 localDir(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
            
            // Transform to world space
            Vec3 worldDir = frame.toWorld(localDir);
            
            // Create secondary ray
            Ray secondaryRay(shadingData.x + worldDir * EPSILON, worldDir);
            
            // Intersect with scene
            IntersectionData isect = scene->traverse(secondaryRay);
            
            // If hit an object
            if (isect.t != FLT_MAX) {
                // Calculate shading data at hit point - fix Ray parameter type issue
                Ray rayNonConst = secondaryRay; // Create non-const copy
                ShadingData secondaryShadingData = scene->calculateShadingData(isect, rayNonConst);
                
                // Get material
                BSDF* material = scene->materials[scene->triangles[isect.ID].materialIndex];
                secondaryShadingData.bsdf = material;
                
                // If diffuse surface, estimate lighting using photon mapping
                if (material->isDiffuse()) {
                    // Query global and caustic photon contributions - simplify only use one photon type to speed up performance
                    Colour indirect;
                    if (i % 2 == 0) {
                        // For even samples, use global photon
                        indirect = estimatePhotonDensity(secondaryShadingData, globalPhotonMap);
                    } else {
                        // For odd samples, use caustic photon
                        indirect = estimatePhotonDensity(secondaryShadingData, causticPhotonMap);
                    }
                    
                    // Calculate BRDF component
                    Colour brdf = shadingData.bsdf->evaluate(shadingData, worldDir);
                    
                    // Combine results, keep color information
                    sum.r += brdf.r * indirect.r * cosTheta;
                    sum.g += brdf.g * indirect.g * cosTheta;
                    sum.b += brdf.b * indirect.b * cosTheta;
                } 
                // If it's a light source, also consider its contribution
                else if (material->isLight()) {
                    Colour emission = material->emit(secondaryShadingData, -worldDir);
                    Colour brdf = shadingData.bsdf->evaluate(shadingData, worldDir);
                    
                    // Directly add light source contribution
                    sum.r += brdf.r * emission.r * cosTheta;
                    sum.g += brdf.g * emission.g * cosTheta;
                    sum.b += brdf.b * emission.b * cosTheta;
                }
            }
        }
        
        // PDF is cosTheta / PI, divide by sampling PDF and multiply by 2π
        float scale = M_PI / (float)numSamples;
        
        // Increase brightness factor to ensure visibility
        scale *= 2.0f;
        
        return Colour(sum.r * scale, sum.g * scale, sum.b * scale);
    }
    
    // Get photon map statistics
    void getPhotonMapStats(int& numGlobal, int& numCaustic) const {
        numGlobal = globalPhotons.size();
        numCaustic = causticPhotons.size();
    }
    
    // Set photon gathering radius
    void setGatherRadius(float radius) {
        gatherRadius = radius;
    }
    
    // Set photon counts
    void setPhotonCounts(int global, int caustic) {
        numGlobalPhotons = global;
        numCausticPhotons = caustic;
    }

    // Direct lighting function - fix interface
    Colour directLighting(const Scene& scene, const ShadingData& shadingData) const {
        Colour result(0.0f, 0.0f, 0.0f);
        
        // Iterate through all lights
        for (auto light : scene.lights) {
            // Use correct light sampling interface
            Vec3 lightDir;
            float lightPdf;
            
            // Use correct sample interface
            Colour lightIntensity = light->evaluate(shadingData, lightDir);
            lightPdf = 1.0f;  // Simplified version, actual should calculate based on light type
            
            if (lightPdf > 0.0f && !isBlack(lightIntensity)) {
                // Calculate shadow ray
                Ray shadowRay(shadingData.x + shadingData.sNormal * RAY_EPSILON, lightDir);
                
                // Check shadow - use non-const version
                Scene* nonConstScene = const_cast<Scene*>(&scene);
                IntersectionData isect = nonConstScene->traverse(shadowRay);
                
                // Simplified light visibility check - just check if outside scene
                bool isVisible = (isect.t == FLT_MAX);
                
                // If visible
                if (isVisible) {
                    // Calculate BSDF
                    Colour brdf = shadingData.bsdf->evaluate(shadingData, lightDir);
                    
                    // Calculate contribution
                    float cosTheta = std::abs(Dot(lightDir, shadingData.sNormal));
                    Colour contrib = brdf * lightIntensity * cosTheta / lightPdf;
                    
                    result = result + contrib;
                }
            }
        }
        
        return result;
    }
    
    // Fixed version of photonMappingPathTrace function - adapt to const Scene&
    Colour photonMappingPathTrace(const Scene& scene, Ray& ray, int depth = 0, bool directLightOnly = false) {
        if (depth > maxDepth) return Colour(0.0f, 0.0f, 0.0f);
        
        // Use traverse non-const version
        Scene* nonConstScene = const_cast<Scene*>(&scene);
        IntersectionData isect = nonConstScene->traverse(ray);
        
        if (isect.t == FLT_MAX) {
            // Environment light processing
            bool hasEnvLight = false;
            
            // Check if there's environment light
            for (auto light : scene.lights) {
                if (!light->isArea()) {  // Assume non-area light is environment light
                    ShadingData dummyData;
                    Colour emission = light->evaluate(dummyData, ray.dir);
                    // Ensure return correct color
                    return emission;
                }
            }
            
            return Colour(0.0f, 0.0f, 0.0f);
        }
        
        // Generate shading data
        ShadingData shadingData = nonConstScene->calculateShadingData(isect, ray);
        shadingData.bsdf = scene.materials[scene.triangles[isect.ID].materialIndex];
        
        // Check if it's a light-emitting object
        if (shadingData.bsdf->isLight()) {
            // Return light source emission strength - directly use emit instead of manual lookup
            return shadingData.bsdf->emit(shadingData, -ray.dir);
        }
        
        // Calculate direct lighting (always calculate)
        Colour directLight = directLighting(scene, shadingData);
        
        // If direct lighting is zero and it's direct lighting mode, return zero
        if (directLightOnly) {
            return directLight;
        }
        
        // Use photon map to estimate indirect lighting (especially caustic)
        Colour indirect(0.0f, 0.0f, 0.0f);
        
        // Get global photon contribution from indirect lighting
        if (globalPhotonMap.size() > 0) {
            Colour globalContrib = estimatePhotonDensity(shadingData, globalPhotonMap);
            indirect.r += globalContrib.r;
            indirect.g += globalContrib.g;
            indirect.b += globalContrib.b;
        }
        
        // Get photon contribution from caustic effect
        if (causticPhotonMap.size() > 0) {
            // Apply additional enhancement factor to caustic effect
            Colour causticContrib = estimatePhotonDensity(shadingData, causticPhotonMap);
            indirect.r += causticContrib.r * 2.0f; // Amplify caustic effect
            indirect.g += causticContrib.g * 2.0f;
            indirect.b += causticContrib.b * 2.0f;
        }
        
        // Random decision to continue tracing path
        if (depth < 3 || MTRandom::randomFloat() < 0.5f) {
            // Random sample BSDF
            Vec3 wi;
            Colour brdf;
            float pdf;
            
            // Sample BSDF - fix interface call
            Sampler* sampler = nullptr;
            wi = shadingData.bsdf->sample(shadingData, sampler, brdf, pdf);
            
            if (pdf > 0.0f && (brdf.r > 0.0f || brdf.g > 0.0f || brdf.b > 0.0f)) {
                Ray newRay(shadingData.x + shadingData.sNormal * RAY_EPSILON, wi);
                
                // Recursive tracing path
                float continuationProb = (depth < 3) ? 1.0f : 0.5f;
                Colour pathContribution = photonMappingPathTrace(scene, newRay, depth + 1);
                
                // Calculate BSDF-based indirect lighting contribution (separate processing for each color component)
                float cosTheta = std::abs(Dot(wi, shadingData.sNormal));
                float factor = cosTheta / (pdf * continuationProb);
                
                // Calculate RGB channels separately
                float indirectR = brdf.r * pathContribution.r * factor;
                float indirectG = brdf.g * pathContribution.g * factor;
                float indirectB = brdf.b * pathContribution.b * factor;
                
                // Prevent over brightness
                const float maxValue = 100.0f;
                if (indirectR > 0.0f || indirectG > 0.0f || indirectB > 0.0f) {
                    if (indirectR < maxValue && indirectG < maxValue && indirectB < maxValue) {
                        // Additional enhanced indirect lighting, easier to see
                        indirect.r += indirectR * 0.5f;
                        indirect.g += indirectG * 0.5f;
                        indirect.b += indirectB * 0.5f;
                    }
                }
            }
        }
        
        // Output debug information
        static bool debugPrinted = false;
        if (!debugPrinted && (directLight.r > 0.0f || directLight.g > 0.0f || directLight.b > 0.0f)) {
            std::cout << "Direct lighting example: " << directLight.r << ", " << directLight.g << ", " << directLight.b << std::endl;
            debugPrinted = true;
        }
        
        static bool indirectDebugPrinted = false;
        if (!indirectDebugPrinted && (indirect.r > 0.0f || indirect.g > 0.0f || indirect.b > 0.0f)) {
            std::cout << "Indirect lighting example: " << indirect.r << ", " << indirect.g << ", " << indirect.b << std::endl;
            indirectDebugPrinted = true;
        }
        
        // Combine direct and indirect lighting
        return Colour(
            directLight.r + indirect.r,
            directLight.g + indirect.g,
            directLight.b + indirect.b
        );
    }

    // New version of emitPhotonsFromLights function - for new implementation
    KDTree emitPhotonsFromLights(const Scene& scene, int numPhotons, bool causticOnly = false) {
        std::vector<Photon> photons;
        photons.reserve(numPhotons);
        
        // Total power of all lights
        float totalPower = 0.0f;
        for (auto light : scene.lights) {
            totalPower += light->totalIntegratedPower();
        }
        
        if (totalPower <= 0.0f) {
            KDTree emptyTree;
            return emptyTree;
        }
        
        // Sample and emit photons
        for (int i = 0; i < numPhotons; i++) {
            // Select a light source
            float lightSelectProb = 1.0f / scene.lights.size();
            int lightIdx = std::min((int)(MTRandom::randomFloat() * scene.lights.size()), 
                               (int)(scene.lights.size() - 1));
            Light* light = scene.lights[lightIdx];
            
            // Emit photon
            float pdfPos = 0.0f;
            float pdfDir = 0.0f;
            Vec3 pos = light->samplePositionFromLight(nullptr, pdfPos);
            Vec3 dir = light->sampleDirectionFromLight(nullptr, pdfDir);
            
            // Calculate photon power
            Colour power = Colour(0.0f, 0.0f, 0.0f);
            if (light->isArea()) {
                // Area light - keep full RGB color information
                AreaLight* areaLight = static_cast<AreaLight*>(light);
                power.r = areaLight->emission.r * (totalPower / numPhotons) * 20.0f;
                power.g = areaLight->emission.g * (totalPower / numPhotons) * 20.0f;
                power.b = areaLight->emission.b * (totalPower / numPhotons) * 20.0f;
            } else {
                // Environment light or other types
                ShadingData dummyData;
                Colour lightEmission = light->evaluate(dummyData, -dir);
                // keep full RGB color information
                power.r = lightEmission.r * (totalPower / numPhotons) * 20.0f;
                power.g = lightEmission.g * (totalPower / numPhotons) * 20.0f;
                power.b = lightEmission.b * (totalPower / numPhotons) * 20.0f;
            }
            
            // Consider light PDF
            if (pdfPos > 0.0f && pdfDir > 0.0f) {
                power.r = power.r / (pdfPos * pdfDir * lightSelectProb);
                power.g = power.g / (pdfPos * pdfDir * lightSelectProb);
                power.b = power.b / (pdfPos * pdfDir * lightSelectProb);
            }
            
            // Add photon to list
            Ray ray(pos, dir);
            // Use scene pointer version of tracePhotonToList
            Scene* nonConstScene = const_cast<Scene*>(&scene);  // Necessary, because we need non-const scene
            tracePhotonToSceneList(ray, power, false, 0, photons, causticOnly, nonConstScene);
        }
        
        // Build KD tree and return
        KDTree photonMap;
        photonMap.build(photons);
        return photonMap;
    }
    
    // Special version, accept non-member Scene's tracePhotonToList
    void tracePhotonToSceneList(const Ray& ray, const Colour& power, bool causticPath, int depth, 
                      std::vector<Photon>& photonList, bool causticOnly, Scene* scenePtr) {
        // Ensure photon energy is zero when exiting early
        if (power.r <= 0.0f && power.g <= 0.0f && power.b <= 0.0f) return;
        
        // Reach maximum depth, stop tracing
        if (depth >= maxDepth) return;
        
        // Intersect with scene
        IntersectionData isect = scenePtr->traverse(ray);
        
        // If no hit, photon disappears
        if (isect.t == FLT_MAX) return;
        
        // Calculate shading data - fix Ray parameter type issue
        Ray rayNonConst = ray; // Create non-const copy
        ShadingData shadingData = scenePtr->calculateShadingData(isect, rayNonConst);
        
        // Get material
        BSDF* material = scenePtr->materials[scenePtr->triangles[isect.ID].materialIndex];
        shadingData.bsdf = material;
        
        // If light source, ignore (photon emitted from light, should not hit light again)
        if (material->isLight()) return;
        
        // Check if it's a specular material - fix isSpecular method missing issue
        // Use other method to check if it's a specular material, like !isDiffuse()
        bool isSpecular = !material->isDiffuse();
        
        // Mark current photon's state (caustic path)
        bool newCausticPath = causticPath;
        if (isSpecular && !causticPath) {
            // First time from diffuse to specular, mark as caustic path
            newCausticPath = true;
        } else if (!isSpecular && causticPath) {
            // Caustic photon hits diffuse surface, store photon
            if (!causticOnly || causticPath) {
                // Keep full color information when creating photon
                Photon photon(shadingData.x, -ray.dir, power, causticPath);
                
                // Relax photon storage condition
                photonList.push_back(photon);
            }
            
            // Caustic path ends (caustic photon stored)
            newCausticPath = false;
        }
        
        // If it's a diffuse surface, store global photon (non-caustic path)
        if (!isSpecular && !causticPath && !causticOnly) {
            // Keep full color information when creating photon
            Photon photon(shadingData.x, -ray.dir, power, false);
            
            // Relax photon storage condition
            photonList.push_back(photon);
        }
        
        // Russian roulette decision to continue propagation
        float luminance = 0.2126f * power.r + 0.7152f * power.g + 0.0722f * power.b;
        float surviveProb = std::min(0.8f, luminance);
        
        if (MTRandom::randomFloat() > surviveProb) {
            return;  // Photon absorbed
        }
        
        // Sample BSDF to decide new direction
        Vec3 wi;
        Colour brdf;
        float pdf;
        
        // Fix BSDF::sample call, use correct interface
        Sampler* sampler = nullptr;
        wi = material->sample(shadingData, &rng, brdf, pdf);
        
        // Invalid direction or zero BRDF, photon terminates
        if (pdf <= 0.0f || (brdf.r <= 0.0f && brdf.g <= 0.0f && brdf.b <= 0.0f)) {
            return;
        }
        
        // Calculate new photon power - ensure separate processing for RGB
        Colour newPower;
        newPower.r = power.r * brdf.r / (pdf * surviveProb);
        newPower.g = power.g * brdf.g / (pdf * surviveProb);
        newPower.b = power.b * brdf.b / (pdf * surviveProb);
        
        // Recursive tracing photon
        Ray newRay(shadingData.x + shadingData.sNormal * RAY_EPSILON, wi);
        tracePhotonToSceneList(newRay, newPower, newCausticPath, depth + 1, photonList, causticOnly, scenePtr);
    }

    // Modified version of causticPhotonMapping function
    void causticPhotonMapping(const Scene& scene, int width, int height, Colour* film) {
        // Increase iteration count, more iterations can accumulate more photon sample
        int maxIterations = 50;  // Originally 20, increased to 50
        
        for (int iter = 0; iter < maxIterations; iter++) {
            int numPhotons = 100000;  // Photons emitted per iteration
            std::cout << "Emitting photons [Iteration " << iter + 1 << "/" << maxIterations << "]: " << numPhotons << " photons" << std::endl;
            
            // Use new method to get photons
            KDTree localCausticPhotonMap = emitPhotonsFromLights(scene, numPhotons, true);
            std::cout << "Stored photons: " << localCausticPhotonMap.size() << " photons" << std::endl;
            
            // Use first bounce photons for caustics
            #pragma omp parallel for
            for (int i = 0; i < width * height; i++) {
                int x = i % width;
                int y = i / width;
                
                // Use multiple samples to reduce noise
                const int samplesPerPixel = 4;  // Originally 1, increased to 4
                Colour pixelColor(0.0f, 0.0f, 0.0f);
                
                for (int s = 0; s < samplesPerPixel; s++) {
                    // Add jitter to reduce aliasing
                    float u = (x + MTRandom::randomFloat()) / float(width);
                    float v = (y + MTRandom::randomFloat()) / float(height);
                    
                    // Compatibility handling, we don't use camera member variable here
                    Ray ray;
                    if (camera.width > 0) {
                        ray = camera.generateRay(u, v);
                    } else {
                        // Assume a fixed camera position
                        Vec3 origin(0, 0, 5);
                        Vec3 dir(u - 0.5f, v - 0.5f, -1.0f);
                        dir.normalize();
                        ray = Ray(origin, dir);
                    }
                    
                    // Use traverse instead of intersect
                    Scene* nonConstScene = const_cast<Scene*>(&scene);
                    IntersectionData isect = nonConstScene->traverse(ray);
                    
                    if (isect.t != FLT_MAX) {
                        // Correct to use calculateShadingData
                        ShadingData shadingData = nonConstScene->calculateShadingData(isect, ray);
                        
                        // Get material
                        shadingData.bsdf = scene.materials[scene.triangles[isect.ID].materialIndex];
                        
                        // Use only direct lighting and photon mapping indirect lighting
                        Colour directLight = directLighting(scene, shadingData);
                        Colour indirectLight = estimatePhotonDensity(shadingData, localCausticPhotonMap);
                        
                        // Combine direct and indirect lighting
                        Colour combinedColor = directLight + indirectLight;
                        pixelColor = pixelColor + combinedColor;
                    }
                }
                
                // Average multiple samples
                pixelColor = pixelColor / float(samplesPerPixel);
                
                // Accumulate to film buffer - use iteration count for weighted average
                if (iter == 0) {
                    film[i] = pixelColor;
                } else {
                    float weight = 1.0f / float(iter + 1);
                    film[i] = film[i] * (1.0f - weight) + pixelColor * weight;
                }
            }
            
            // Update display after each iteration
            std::cout << "Completed iteration " << iter + 1 << "/" << maxIterations << std::endl;
        }
    }
}; 