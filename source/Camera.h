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

		//Change to support reference
		Matrix CalculateCameraToWorld()
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
			fovAngle = degrees;
			fov = tanf(degrees * TO_RADIANS / 2.f);
		}

		void CalculateForwardVector()
		{
			const Matrix finalRotation = Matrix::CreateRotation({ totalPitch, totalYaw, 0.f });
			forward = finalRotation.TransformVector(Vector3::UnitZ);
			forward.Normalize();
			forwardChanged = true;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();
			const float linearSpeed{ 4.f };
			const float rotationSpeed{ 10.f };
			float shiftModifier{ 1.f };
			Vector3 displacement{};

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			if (pKeyboardState[SDL_SCANCODE_LSHIFT] || pKeyboardState[SDL_SCANCODE_RSHIFT])
			{
				shiftModifier = 4.f;
			}

			if (pKeyboardState[SDL_SCANCODE_LEFT])
			{
				if (fovAngle > minFov)
				{
					SetCameraFOV(fovAngle - 1);
				}
			}
			else if (pKeyboardState[SDL_SCANCODE_RIGHT])
			{
				if (fovAngle < maxFov)
				{
					SetCameraFOV(fovAngle + 1);
				}
			}
			float speedModifier{ deltaTime * linearSpeed * shiftModifier };

			origin += forward * speedModifier * pKeyboardState[SDL_SCANCODE_W];
			origin += forward * -speedModifier * pKeyboardState[SDL_SCANCODE_S];
			origin += right * speedModifier * pKeyboardState[SDL_SCANCODE_D];
			origin += right * -speedModifier * pKeyboardState[SDL_SCANCODE_A];

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);
			float rotationModifier{ deltaTime * rotationSpeed * shiftModifier };			
			
			if (mouseY != 0.f || mouseX != 0.f)
			{
				origin += forward * speedModifier * (mouseState == SDL_BUTTON_LMASK) * static_cast<float>(mouseY);
				origin += Vector3::UnitY * speedModifier * (mouseState == (SDL_BUTTON_RMASK | SDL_BUTTON_LMASK)) * static_cast<float>(mouseY);
				totalPitch += static_cast<float>(mouseY) * TO_RADIANS * (mouseState == SDL_BUTTON_RMASK) * rotationModifier;
				totalYaw += static_cast<float>(-mouseX) * TO_RADIANS *
					(mouseState & SDL_BUTTON_LMASK || mouseState & SDL_BUTTON_RMASK) * rotationModifier;
				CalculateForwardVector();
			}
		}
	};
}

/*if ((mouseState & SDL_BUTTON(1)) )
			{

				if (mouseY != 0.f)
				{
					if (mouseState & SDL_BUTTON(3))
					{
						origin.y += mouseY * deltaTime * linearSpeed * shiftModifier;
					}
					else
					{
						float modifier{ speedModifier * mouseY };
						displacement.x += forward.x * modifier;
						displacement.y += forward.y * modifier;
						displacement.z += forward.z * modifier;
					}
				}
				if (mouseX != 0.f) {
					totalYaw -= mouseX * deltaTime * TO_RADIANS * rotationSpeed * shiftModifier;
					CalculateForwardVector();
				}
			}
			else if ((mouseState & SDL_BUTTON(3)))
			{
				totalPitch += mouseY * deltaTime * TO_RADIANS * rotationSpeed * shiftModifier;
				totalYaw -= mouseX * deltaTime * TO_RADIANS * rotationSpeed * shiftModifier;
				CalculateForwardVector();
			}*/
