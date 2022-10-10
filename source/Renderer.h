#pragma once

#include <cstdint>
#include <iostream>

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Scene;
	class Timer;

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
		void Update(dae::Timer* pTimer);

		bool SaveBufferToImage() const;

		void CycleLightingMode();
		void ToggleShadows() { 
			m_ShadowsEnabled = !m_ShadowsEnabled; 
			std::cout << "shadows toggled" << "\n";
		}

	private:

		enum class LightingMode
		{
			ObservedArea,
			Radiance,
			BDRF,
			Combined,
			//Define modes above
			Count
		};
		LightingMode m_CurrentLightingMode{ LightingMode::Combined };
		bool m_ShadowsEnabled{ true };

		SDL_Window* m_pWindow{};

		SDL_Surface* m_pBuffer{};
		uint32_t* m_pBufferPixels{};

		int m_Width{};
		int m_Height{};
		float m_AspectRatio{ };
	};
}
