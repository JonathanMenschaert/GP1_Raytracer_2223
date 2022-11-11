//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

#include <iostream>
#include <thread>

#include <future> //async stuff
#include <ppl.h> //parallel stuff

using namespace dae;

//#define ASYNC
#define PARALLEL_FOR

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
	m_AspectRatio = m_Width / static_cast<float>(m_Height);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();
	const uint32_t numPixels = m_Width * m_Height;
	camera.CalculateCameraToWorld();

#if defined(ASYNC)
	//async
	const uint32_t numCores{ std::thread::hardware_concurrency() };
	std::vector<std::future<void>> async_futures{};
	const uint32_t numPixelsPerTask{ numPixels / numCores };
	uint32_t numUnnassignedPixels = { numPixels % numCores };
	uint32_t currPixelIndex{};

	//Create Tasks
	for (uint32_t coreId{}; coreId < numCores; ++coreId)
	{
		uint32_t taskSize{ numPixelsPerTask };
		if (numUnnassignedPixels > 0)
		{
			++taskSize;
			--numUnnassignedPixels;
		}
		async_futures.push_back(
			std::async(std::launch::async, [=,this] 
				{
					const uint32_t pixelIndexEnd{ currPixelIndex + taskSize };
					for (uint32_t pixelIndex{ currPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex)
					{
						RenderPixel(pScene, pixelIndex, m_AspectRatio, camera, lights, materials);
					}
				}
			)
		);
		currPixelIndex += taskSize;
	}

	//Wait for all tasks;
	for (const std::future<void>& f : async_futures)
	{
		f.wait();
	}

#elif (defined(PARALLEL_FOR))
	//parallel
	concurrency::parallel_for(0u, numPixels, [=, this](int i)
		{
			RenderPixel(pScene, i, m_AspectRatio, camera, lights, materials);
		}
	);
#else
	//synchronous
	for (uint32_t i{}; i < numPixels; ++i)
	{
		RenderPixel(pScene, i, m_AspectRatio, camera, lights, materials);
	}
#endif
	

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, float aspectRatio, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{

	//Calculate pixel locations and pixel centers
	const int px = pixelIndex % m_Width;
	const int py = pixelIndex / m_Width;
	
	const float cx = (2.f * ((px + 0.5f) / m_Width) - 1) * m_AspectRatio * camera.fov;
	const float cy = (1.f - (2.f * (py + 0.5f) / m_Height)) * camera.fov;


	//Calculate view direction & ray
	Vector3 viewDirection{ camera.cameraToWorld.TransformVector(cx, cy, 1) };
	viewDirection.Normalize();
	const Ray viewRay{ camera.origin,  viewDirection };

	//Attempt to hit an object with the calculated ray
	HitRecord closestHit{};
	pScene->GetClosestHit(viewRay, closestHit);

	float shadowFactor{ 1.f };
	ColorRGB finalColor{};

	if (closestHit.didHit)
	{
		//offset from the hit origin to prevent the object from incorrectly shadowing itself.
		const Vector3 originOffset{ closestHit.origin + closestHit.normal * 0.0001f };

		for (const auto& light : lights)
		{
			Vector3 lightDirection{ LightUtils::GetDirectionToLight(light, originOffset) };
			const float magnitude{ lightDirection.Normalize() };

			if (m_ShadowsEnabled)
			{
				//Attempt to hit an object between the the hit origin and the light.
				const Ray shadowRay{ originOffset, lightDirection, 0.0001f, magnitude };				
				if (pScene->DoesHit(shadowRay)) 
				{
					shadowFactor *= 0.95f;
					continue;
				}
			}			

			//Calculate the color of the pixel based on the lighting mode
			switch (m_CurrentLightingMode)
			{
			case LightingMode::Combined:
			{
				const float observedArea{ std::max(Vector3::Dot(closestHit.normal, lightDirection), 0.f) };
				const ColorRGB radiance{ LightUtils::GetRadiance(light, closestHit.origin) };
				const ColorRGB brdf{ materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -viewDirection) };
				finalColor += observedArea * radiance * brdf;
				break;
			}
			case LightingMode::ObservedArea:
			{
				const float observedArea{ std::max(Vector3::Dot(closestHit.normal, lightDirection), 0.f) };
				finalColor += ColorRGB{ observedArea, observedArea, observedArea };
				break;
			}
			case LightingMode::Radiance:
			{
				finalColor += LightUtils::GetRadiance(light, closestHit.origin);
				break;
			}
			case LightingMode::BRDF:
			{
				finalColor += materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, -viewDirection);
				break;
			}
			}
		}
		finalColor *= shadowFactor;
	}
	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));	
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	m_CurrentLightingMode = static_cast<LightingMode>((static_cast<int>(m_CurrentLightingMode) + 1) % 
		static_cast<int>(LightingMode::Count));
}
