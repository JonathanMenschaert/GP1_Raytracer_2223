#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 originVector{ ray.origin - sphere.origin };
			//at� + bt + c = 0
			const float a{ Vector3::Dot(ray.direction, ray.direction) };
			const float b{ Vector3::Dot(2 * ray.direction, originVector) };
			const float c{ Vector3::Dot(originVector, originVector) - Square(sphere.radius)};
			
			const float discriminant = Square(b) - 4 * a * c;

			if (discriminant > 0) 
			{
				const float discriminantSqrt{ sqrtf(discriminant) };
				float t{ (-b - discriminantSqrt) / (2 * a) };
				if (t < ray.min)
				{
					t = (-b + discriminantSqrt) / (2 * a);
					if (t < ray.min)
					{
						return false;
					}
				}
				if (t < ray.max)
				{
					if (!ignoreHitRecord)
					{
						hitRecord.materialIndex = sphere.materialIndex;
						hitRecord.t = t;
						hitRecord.didHit = true;
						hitRecord.origin = ray.origin + hitRecord.t * ray.direction;
						hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
					}
					return true;
				}
			}
			return false;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const float t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / Vector3::Dot(ray.direction, plane.normal);
			if (t >= ray.min && t < ray.max)
			{
				if (!ignoreHitRecord)
				{
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.t = t;
					hitRecord.didHit = true;
					hitRecord.normal = plane.normal;
					hitRecord.origin = ray.origin + hitRecord.t * ray.direction;
				}
				return true;
			}
			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const float cullDot{ Vector3::Dot(triangle.normal, ray.direction) };
			if (std::abs(cullDot) < FLT_EPSILON) return false;
			switch (triangle.cullMode)
			{
			case TriangleCullMode::FrontFaceCulling:
			{
				
				const float cullResult{ cullDot * !ignoreHitRecord - cullDot * ignoreHitRecord };
				if (cullResult < 0.f) return false;
			}
			break;
			case TriangleCullMode::BackFaceCulling:
			{
				const float cullResult{ cullDot * !ignoreHitRecord - cullDot * ignoreHitRecord };
				if (cullResult > 0.f) return false;
			}
			break;
			case TriangleCullMode::NoCulling:
				break;
			default:
				break;
			}

			const Vector3 center{ (triangle.v0 + triangle.v1 + triangle.v2) / 3.f };
			const Vector3 l{ center - ray.origin };
			const float t{ Vector3::Dot(l, triangle.normal) / Vector3::Dot(ray.direction, triangle.normal) };
			if (t < ray.min || t > ray.max) return false;

			const Vector3 intersectionPoint{ ray.origin + t * ray.direction };

			Vector3 edge{ triangle.v1 - triangle.v0 };
			Vector3 pointToSide{ intersectionPoint - triangle.v0 };

			if (Vector3::Dot(triangle.normal, Vector3::Cross(edge, pointToSide)) < 0) return false;

			edge = triangle.v2 - triangle.v1;
			pointToSide = intersectionPoint - triangle.v1;

			if (Vector3::Dot(triangle.normal, Vector3::Cross(edge, pointToSide)) < 0) return false;

			edge = triangle.v0 - triangle.v2;
			pointToSide = intersectionPoint - triangle.v2;

			if (Vector3::Dot(triangle.normal, Vector3::Cross(edge, pointToSide)) < 0) return false;
			
			if (!ignoreHitRecord)
			{
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.t = t;
				hitRecord.didHit = true;
				hitRecord.normal = triangle.normal;
				hitRecord.origin = intersectionPoint;
			}

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const size_t idxIncr{ 3 };
			HitRecord closestHit{};
			bool didHit{ };
			Triangle triangle{};
			triangle.materialIndex = mesh.materialIndex;
			triangle.cullMode = mesh.cullMode;
			for (size_t idx{}; idx < mesh.indices.size(); idx += idxIncr)
			{
				size_t startIdx{ idx };
				triangle.v0 = mesh.transformedPositions[static_cast<size_t>(mesh.indices[startIdx])];
				triangle.v1 = mesh.transformedPositions[static_cast<size_t>(mesh.indices[++startIdx])];
				triangle.v2 = mesh.transformedPositions[static_cast<size_t>(mesh.indices[++startIdx])];
				triangle.normal = mesh.transformedNormals[idx / idxIncr];
				
				
				if (HitTest_Triangle(triangle, ray, closestHit, ignoreHitRecord)) 
				{
					if (ignoreHitRecord) return true;
					if (closestHit.t < hitRecord.t)
					{
						hitRecord = closestHit;
					}
					didHit = true;
				}				
			}
			return didHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			switch (light.type)
			{
			case LightType::Point:
				return { light.origin - origin };
			case LightType::Directional:
				return {  FLT_MAX * (light.origin - origin) };
			default:
				return Vector3{};
			}
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			switch (light.type)
			{
			case LightType::Point:
				//Ergb = lightcolor * intensity / (light origin - target)�
				return { light.color * (light.intensity / (light.origin - target).SqrMagnitude())};
			case LightType::Directional:
				return { light.color * light.intensity};
			default:
				return ColorRGB{};
			}
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}