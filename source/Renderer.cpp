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

using namespace dae;

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
	const Matrix cameraToWorld = camera.CalculateCameraToWorld();

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			float cx = (2.f * ((px + 0.5f) / m_Width) - 1) * m_AspectRatio * camera.fov;
			float cy = (1.f - (2.f * (py + 0.5f) / m_Height)) * camera.fov;
			Vector3 rayDirection{cameraToWorld.TransformVector(cx, cy, 1)};
			rayDirection.Normalize();
			Ray viewRay{ camera.origin,  rayDirection };
			ColorRGB finalColor{};
			HitRecord closestHit{};
			pScene->GetClosestHit(viewRay, closestHit);
			if (closestHit.didHit)
			{
				//finalColor = materials[closestHit.materialIndex]->Shade();
				for (size_t idx{}; idx < lights.size(); ++idx)
				{
					Vector3 lightDirection{ LightUtils::GetDirectionToLight(lights[idx], closestHit.origin) };
					float magnitude{ lightDirection.Normalize() };
					Ray lightRay{ closestHit.origin, lightDirection, 0.0001f, magnitude };
					if (pScene->DoesHit(lightRay) && m_ShadowsEnabled) continue;

					float observedArea{ Vector3::Dot(closestHit.normal, lightDirection) };
					if (observedArea < 0 ) continue;					
							
					switch (m_CurrentLightingMode)
					{
					case LightingMode::Combined:
						finalColor += LightUtils::GetRadiance(lights[idx], closestHit.origin) *
							materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, rayDirection) * observedArea;
						break;
					case LightingMode::ObservedArea:
						finalColor += {observedArea, observedArea, observedArea};
						break;
					case LightingMode::Radiance:
						finalColor += LightUtils::GetRadiance(lights[idx], closestHit.origin);
						break;
					case LightingMode::BDRF:
						finalColor += materials[closestHit.materialIndex]->Shade(closestHit, lightDirection, rayDirection);
						break;
					default:
						break;
					}


				}
			}
			//Update Color in Buffer
			finalColor.MaxToOne();

			m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
				static_cast<uint8_t>(finalColor.r * 255),
				static_cast<uint8_t>(finalColor.g * 255),
				static_cast<uint8_t>(finalColor.b * 255));
		}
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::Update(dae::Timer* pTimer)
{
	const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
	if (pKeyboardState[SDL_SCANCODE_F3])
	{
		CycleLightingMode();
	}
	if (pKeyboardState[SDL_SCANCODE_F2])
	{
		ToggleShadows();
	}
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
