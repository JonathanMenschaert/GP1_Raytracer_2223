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
			//at² + bt + c = 0
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
						hitRecord.normal = (hitRecord.origin - sphere.origin);
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
			if (abs(cullDot) < FLT_EPSILON) return false;
			TriangleCullMode cullMode = triangle.cullMode;
			if (ignoreHitRecord)
			{
				switch (cullMode)
				{
				case TriangleCullMode::FrontFaceCulling:
					cullMode = TriangleCullMode::BackFaceCulling;
					break;
				case TriangleCullMode::BackFaceCulling:
					cullMode = TriangleCullMode::FrontFaceCulling;
					break;
				}
			}

			switch (cullMode)
			{
			case TriangleCullMode::FrontFaceCulling:
				if (cullDot < 0)
					return false;
				break;
			case TriangleCullMode::BackFaceCulling:
				if (cullDot > 0)
					return false;
				break;
			}

			//Source: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
			Vector3 edge1{ triangle.v1 - triangle.v0 };
			Vector3 edge2{ triangle.v2 - triangle.v0 };
			Vector3 h{ Vector3::Cross(ray.direction, edge2) };

			float a{ Vector3::Dot(edge1, h) };
			if (abs(a) < FLT_EPSILON) return false;

			float aInverse{ 1.f / a };
			Vector3 s{ ray.origin - triangle.v0 };
			float u{ aInverse * Vector3::Dot(s, h) };
			if (u < 0.f || u > 1.f) return false;

			Vector3 q{ Vector3::Cross(s, edge1) };
			float v{ aInverse * Vector3::Dot(ray.direction, q) };
			if (v < 0.f || (u + v) > 1.f) return false;

			float t{ aInverse * Vector3::Dot(edge2, q) };
			if (t < ray.min || t >= ray.max) return false;

			Vector3 intersectionPoint{ ray.origin + t * ray.direction };

			/*const Vector3 center{ (triangle.v0 + triangle.v1 + triangle.v2) / 3.f };
			const Vector3 vectorToPlane{ center - ray.origin };
			const float t{ Vector3::Dot(vectorToPlane, triangle.normal) / cullDot };

			if (t < ray.min || t >= ray.max) return false;

			const Vector3 intersectionPoint{ ray.origin + t * ray.direction };

			if (Vector3::Dot(triangle.normal, Vector3::Cross(triangle.v1 - triangle.v0, intersectionPoint - triangle.v0)) < 0) return false;


			if (Vector3::Dot(triangle.normal, Vector3::Cross(triangle.v2 - triangle.v1, intersectionPoint - triangle.v1)) < 0) return false;


			if (Vector3::Dot(triangle.normal, Vector3::Cross(triangle.v0 - triangle.v2, intersectionPoint - triangle.v2)) < 0) return false;*/

			
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

		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			float tx1 = (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x;
			float tx2 = (mesh.transformedMaxAABB.x - ray.origin.x) / ray.direction.x;

			float tMin = std::min(tx1, tx2);
			float tMax = std::max(tx1, tx2);

			float ty1 = (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y;
			float ty2 = (mesh.transformedMaxAABB.y - ray.origin.y) / ray.direction.y;

			tMin = std::max(tMin, std::min(ty1, ty2));
			tMax = std::min(tMax, std::max(ty1, ty2));

			float tz1 = (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z;
			float tz2 = (mesh.transformedMaxAABB.z - ray.origin.z) / ray.direction.z;

			tMin = std::max(tMin, std::min(tz1, tz2));
			tMax = std::min(tMax, std::max(tz1, tz2));

			return tMax > 0 && tMax >= tMin;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			if (!SlabTest_TriangleMesh(mesh, ray))
			{
				return false;
			}
			HitRecord closestHit{};
			bool didHit{ };
			Triangle triangle{};
			triangle.materialIndex = mesh.materialIndex;
			triangle.cullMode = mesh.cullMode;
			for (int idx{}; idx < mesh.indices.size(); idx += 3)
			{
				triangle.v0 = mesh.transformedPositions[mesh.indices[idx]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[idx + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[idx + 2]];
				triangle.normal = mesh.transformedNormals[idx / 3];
				
				
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
			}
			return Vector3{};
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			switch (light.type)
			{
			case LightType::Point:
				//Ergb = lightcolor * intensity / (light origin - target)²
				return { light.color * (light.intensity / (light.origin - target).SqrMagnitude())};
			case LightType::Directional:
				return { light.color * light.intensity};
			}
			return ColorRGB{};
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