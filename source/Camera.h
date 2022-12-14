#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

#include <iostream>

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}			
		{
		}


		Vector3 origin{};
		float fovAngle{90.f};
		float fov{};

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		bool forwardChanged = true;

		float totalPitch{0.f};
		float totalYaw{0.f};

		const float minFov {10.f};
		const float maxFov {175.f};

		Matrix cameraToWorld{};

		//Return calculated rotationmatrix. Recalculate if the forward vector changed
		const Matrix& CalculateCameraToWorld()
		{
			if (forwardChanged)
			{
				right = Vector3::Cross(Vector3::UnitY, forward).Normalized();
				up = Vector3::Cross(forward, right).Normalized();
				cameraToWorld = { right, up, forward, origin };
				forwardChanged = false;
			}
			return cameraToWorld;
		}

		void SetCameraFOV(float degrees)
		{
			fovAngle = std::max(minFov, std::min(degrees, maxFov));
			fov = tanf(fovAngle * TO_RADIANS / 2.f);
		}

		void CalculateForwardVector()
		{
			const Matrix finalRotation = Matrix::CreateRotation({ totalPitch, totalYaw, 0.f });
			forward = finalRotation.TransformVector(Vector3::UnitZ);
			forwardChanged = true;
		}

		void Update(Timer* pTimer)
		{
			//Set constants
			const float deltaTime = pTimer->GetElapsed();
			const float linearSpeed{ 4.f };
			const float rotationSpeed{ 15.f };			

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			const bool isShiftPressed{ pKeyboardState[SDL_SCANCODE_LSHIFT] || pKeyboardState[SDL_SCANCODE_RSHIFT] };			
			const float shiftModifier{ 4.f * isShiftPressed + 1.f * !isShiftPressed };			
			const float speedModifier{ deltaTime * linearSpeed * shiftModifier };

			const bool isForwardsPressed{ pKeyboardState[SDL_SCANCODE_W] || pKeyboardState[SDL_SCANCODE_UP] };
			const bool isBackwardsPressed{ pKeyboardState[SDL_SCANCODE_S] || pKeyboardState[SDL_SCANCODE_DOWN] };
			const bool isRightPressed{ pKeyboardState[SDL_SCANCODE_D] || pKeyboardState[SDL_SCANCODE_RIGHT] };
			const bool isLeftPressed{ pKeyboardState[SDL_SCANCODE_A] || pKeyboardState[SDL_SCANCODE_LEFT] };
			origin += forward * speedModifier * isForwardsPressed;
			origin += forward * -speedModifier * isBackwardsPressed;
			origin += right * speedModifier * isRightPressed;
			origin += right * -speedModifier * isLeftPressed;

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);
			const float rotationModifier{ deltaTime * rotationSpeed * shiftModifier };			
			
			//Calculate rotation & movement on mouse movement
			if (mouseY != 0.f || mouseX != 0.f)
			{
				origin += forward * speedModifier * (mouseState == SDL_BUTTON_LMASK) * static_cast<float>(mouseY);
				origin += Vector3::UnitY * speedModifier * (mouseState == (SDL_BUTTON_RMASK | SDL_BUTTON_LMASK)) * static_cast<float>(mouseY);
				totalPitch -= static_cast<float>(mouseY) * TO_RADIANS * (mouseState == SDL_BUTTON_RMASK) * rotationModifier;
				totalYaw += static_cast<float>(mouseX) * TO_RADIANS *
					(mouseState & SDL_BUTTON_LMASK || mouseState & SDL_BUTTON_RMASK) * rotationModifier;
				CalculateForwardVector();
			}
		}
	};
}

