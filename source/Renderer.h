#pragma once

#include <cstdint>
#include <iostream>
#include <vector>
struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Scene;
	class Timer;
	class Material;
	struct Camera;
	struct Light;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer() = default;

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Render(Scene* pScene) const;
		void RenderPixel(Scene* pScene, uint32_t pixelIndex, float aspectRatio, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const;

		bool SaveBufferToImage() const;

		void CycleLightingMode();
		void ToggleShadows() { 
			m_ShadowsEnabled = !m_ShadowsEnabled; 
		}

	private:

		enum class LightingMode
		{
			ObservedArea,
			Radiance,
			BRDF,
			Combined,
			//Define modes above
			Count
		};
		LightingMode m_CurrentLightingMode{ LightingMode::Combined };
		bool m_ShadowsEnabled{ true };
		bool m_F3Pressed{ false };
		bool m_F2Pressed{ false };

		SDL_Window* m_pWindow{};

		SDL_Surface* m_pBuffer{};
		uint32_t* m_pBufferPixels{};

		int m_Width{};
		int m_Height{};
		float m_AspectRatio{ };
	};
}
