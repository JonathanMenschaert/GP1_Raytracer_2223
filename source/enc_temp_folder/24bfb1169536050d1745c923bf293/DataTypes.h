#pragma once
#include <cassert>

#include "Math.h"
#include "vector"
#include <iostream>

#define BVH
#define USE_BINS
namespace dae
{
#pragma region GEOMETRY
	struct Sphere
	{
		Vector3 origin{};
		float radius{};

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin{};
		Vector3 normal{};

		unsigned char materialIndex{ 0 };
	};

	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	enum class Axis
	{
		axisX,
		axisY,
		axisZ
	};

	struct BVHNode
	{
		Vector3 minAABB{ Vector3::MaxVector };
		Vector3 maxAABB{ Vector3::MinVector };
		unsigned int firstIdx;
		unsigned int idxCount;
		unsigned int leftNode;
		bool IsLeaf() 
		{
			return idxCount > 0;
		};
	};

	struct AABB
	{
		Vector3 minAABB{ Vector3::MaxVector };
		Vector3 maxAABB{ Vector3::MinVector };
		void Grow(const Vector3& point)
		{
			minAABB = Vector3::Min(minAABB, point);
			maxAABB = Vector3::Max(maxAABB, point);
		}

		void Grow(const AABB& aabb)
		{
			minAABB = Vector3::Min(minAABB, aabb.minAABB);
			maxAABB = Vector3::Max(maxAABB, aabb.maxAABB);
		}

		float Area()
		{
			Vector3 aabb{ maxAABB - minAABB };
			return aabb.x * aabb.y + aabb.y * aabb.z + aabb.z * aabb.x;
		}
	};

	struct Bin
	{
		AABB bounds{};
		unsigned int idxCount{};
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal):
			v0{_v0}, v1{_v1}, v2{_v2}, normal{_normal.Normalized()}{}

		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}

		Vector3 v0{};
		Vector3 v1{};
		Vector3 v2{};

		Vector3 normal{};

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};

	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode):
		positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}

		~TriangleMesh()
		{
			delete[] pBVHNodes;
		}

		std::vector<Vector3> positions{};
		std::vector<Vector3> normals{};
		std::vector<int> indices{};
		unsigned char materialIndex{};

		TriangleCullMode cullMode{TriangleCullMode::BackFaceCulling};

		Matrix rotationTransform{};
		Matrix translationTransform{};
		Matrix scaleTransform{};

		Vector3 minAABB;
		Vector3 maxAABB;
		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;

		BVHNode* pBVHNodes{};
		unsigned int rootNodeIdx{};
		unsigned int nodesUsed{1};
		const unsigned int leafSize{3 * 3 - 1};

		std::vector<Vector3> transformedPositions{};
		std::vector<Vector3> transformedNormals{};

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.push_back(triangle.v0);
			positions.push_back(triangle.v1);
			positions.push_back(triangle.v2);

			indices.push_back(startIndex);
			indices.push_back(++startIndex);
			indices.push_back(++startIndex);

			normals.push_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			normals.reserve(indices.size() / 3);
			const size_t idxIncr{ 3 };
			for (size_t idx{}; idx < indices.size(); idx += idxIncr)
			{
				size_t startIdx{ idx };
				const Vector3& v0{ positions[static_cast<size_t>(indices[startIdx])] };
				const Vector3& v1{ positions[static_cast<size_t>(indices[++startIdx])] };
				const Vector3& v2{ positions[static_cast<size_t>(indices[++startIdx])] };

				Vector3 edgeA{ v1 - v0 };
				Vector3 edgeB{ v2 - v0 };

				normals.emplace_back(Vector3::Cross(edgeA, edgeB).Normalized());
			}
			
		}

		void UpdateTransforms()
		{
			//Calculate Final Transform 
			const auto finalTransform = scaleTransform * rotationTransform * translationTransform;			


			//Transform Positions (positions > transformedPositions)
			transformedPositions.clear();
			transformedPositions.reserve(positions.size());
			for (auto& pos : positions)
			{
				transformedPositions.emplace_back(finalTransform.TransformPoint(pos));
			}

			//Transform Normals (normals > transformedNormals)
			transformedNormals.clear();
			transformedNormals.reserve(normals.size());
			for (auto& normal : normals)
			{
				transformedNormals.emplace_back(finalTransform.TransformVector(normal).Normalized());
			}			
#ifdef BVH
			BuildBVH();
#else
			UpdateTransformedAABB(finalTransform);
#endif
		}

		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];
				for (auto& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			//AABB update: be careful -> transform the 8 vertices of the aabb
			//and calculate new min and max.
			Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
			Vector3 tMaxAABB = tMinAABB;
			//xmax, ymin, zmin
			Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//xmax, ymin, zmax
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//xmin, ymin, zmax
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//xmin, ymax, zmin
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//xmax, ymax, zmin
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//xmax, ymax, zmax
			tAABB = finalTransform.TransformPoint(maxAABB);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			//xmin, ymax, zmax
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}

		//BVH algoritme taken from: https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
		//Including part 2 & 3
		void BuildBVH()
		{			
			BVHNode& root{ pBVHNodes[rootNodeIdx] };
			

			root.leftNode = 0;
			root.firstIdx = 0;
			root.idxCount = static_cast<unsigned int>(indices.size());

			//Reset nodesUsed to 1 to take into account the root node
			nodesUsed = 1;

			UpdateNodeBounds(rootNodeIdx);
			Subdivide(rootNodeIdx);
		}

		void UpdateNodeBounds(unsigned int nodeIdx)
		{
			BVHNode& node{ pBVHNodes[nodeIdx] };
			node.minAABB = Vector3::MaxVector;
			node.maxAABB = Vector3::MinVector;
			for (unsigned int i{ node.firstIdx }; i < (node.firstIdx + node.idxCount); ++i)
			{
				node.minAABB = Vector3::Min(node.minAABB, transformedPositions[indices[i]]);
				node.maxAABB = Vector3::Max(node.maxAABB, transformedPositions[indices[i]]);
			}
		}

		void Subdivide(unsigned int nodeIdx)
		{
			//Terminate Recursion if necessary
			BVHNode& node = pBVHNodes[nodeIdx];
			if (node.idxCount <= 8) return;

			//Determine split axis
#ifdef USE_BINS
			int axis{ 0 };
			float splitPos{};
			const float splitCost{ FindBestSplitPlane(node, axis, splitPos)};
			const float noSplitCost{ CalculateNodeCost(node) };
			if (splitCost >= noSplitCost) return;
#else
			Vector3 extent{ node.maxAABB - node.minAABB };
			int axis{ 0 };
			if (extent.y > extent.x) axis = 1;
			if (extent.z > extent[axis]) axis = 2;
			float splitPos{ node.minAABB[axis] + extent[axis] * 0.5f };
#endif
			//Partitioning
			int i{ static_cast<int>(node.firstIdx) };
			int j{ i + static_cast<int>(node.idxCount) - 1 };
			while (i <= j)
			{
				Vector3 centroid{ (transformedPositions[indices[i]] + transformedPositions[indices[i + 1]] + transformedPositions[indices[i + 2]]) * 0.3333f };
				if (centroid[axis] < splitPos)
				{
					i += 3;
				}
				else
				{
					std::swap(normals[i * 0.3334], normals[(j - 2) * 0.3334]);
					std::swap(transformedNormals[i / 3], transformedNormals[(j - 2) * 0.3334]);

					std::swap(indices[i], indices[j - 2]);
					std::swap(indices[i + 1], indices[j - 1]);
					std::swap(indices[i + 2], indices[j]);
					j -= 3;
				}
			}

			int leftCount{ i - static_cast<int>(node.firstIdx) };
			if (leftCount == 0 || leftCount == node.idxCount)
			{
				return;
			}

			int leftNodeIdx = nodesUsed++;
			int rightNodeIdx = nodesUsed++;

			node.leftNode = leftNodeIdx;
			pBVHNodes[leftNodeIdx].firstIdx = node.firstIdx;
			pBVHNodes[leftNodeIdx].idxCount = leftCount;
			pBVHNodes[rightNodeIdx].firstIdx = static_cast<unsigned int>(i);
			pBVHNodes[rightNodeIdx].idxCount = node.idxCount - leftCount;
			node.idxCount = 0;
			UpdateNodeBounds(leftNodeIdx);
			UpdateNodeBounds(rightNodeIdx);

			Subdivide(leftNodeIdx);
			Subdivide(rightNodeIdx);
		}

		float CalculateNodeCost(const BVHNode& node)
		{
			Vector3 extent { node.maxAABB - node.minAABB };
			float area{ extent.x * extent.y + extent.y * extent.z + extent.z * extent.x };
			return node.idxCount * area;
		}

		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos)
		{
			float bestCost{ FLT_MAX };
			for (int axisIdx{}; axisIdx < 3; ++axisIdx)
			{
				float minBounds{ FLT_MAX };
				float maxBounds{ FLT_MIN };

				for (unsigned int idx{}; idx < node.idxCount; idx += 3)
				{
					const unsigned int idxOffset{ node.firstIdx + idx };
					Vector3 centroid{
						(transformedPositions[indices[idxOffset]] + 
						transformedPositions[indices[idxOffset + 1]] + 
							transformedPositions[indices[idxOffset + 2]]) * 0.3333f
					};

					minBounds = std::min(minBounds, centroid[axisIdx]);
					maxBounds = std::max(maxBounds, centroid[axisIdx]);
				}
				const float boundsDifference{ maxBounds - minBounds };
				if (abs(boundsDifference) < FLT_EPSILON) continue;

				//Populate bins
				const int amountOfBins{ 8 };
				Bin bins[amountOfBins];
				float scale = amountOfBins / boundsDifference;
				const int amountOfPlaneBins{ amountOfBins - 1 };
				for (unsigned int idx{}; idx < node.idxCount; idx += 3)
				{
					const unsigned int idxOffset{ node.firstIdx + idx };
					const Vector3& v0{ transformedPositions[indices[idxOffset]] };
					const Vector3& v1{ transformedPositions[indices[idxOffset + 1]] };
					const Vector3& v2{ transformedPositions[indices[idxOffset + 2]] };
					const Vector3 centroid{ (v0 + v1 + v2) * 0.3333f };

					const int binIdx{ std::min(amountOfPlaneBins, static_cast<int>((centroid[axisIdx] - minBounds) * scale)) };
					bins[binIdx].idxCount += 3;
					bins[binIdx].bounds.Grow(v0);
					bins[binIdx].bounds.Grow(v1);
					bins[binIdx].bounds.Grow(v2);
				}

				//Gather data for binAmount - 1 planes for binAmount planes
				

				float leftArea[amountOfPlaneBins];
				float rightArea[amountOfPlaneBins];
				int leftCount[amountOfPlaneBins];
				int rightCount[amountOfPlaneBins];
				int leftSum{};
				int rightSum{};
				AABB leftBox;
				AABB rightBox;

				for (int i{}; i < amountOfPlaneBins; ++i)
				{
					//Left side
					leftSum += bins[i].idxCount;
					leftCount[i] = leftSum;
					leftBox.Grow(bins[i].bounds);
					leftArea[i] = leftBox.Area();

					//Right side
					rightSum += bins[amountOfPlaneBins - i].idxCount;
					rightCount[amountOfPlaneBins - i - 1] = rightSum;
					rightBox.Grow(bins[amountOfPlaneBins - i].bounds);
					rightArea[amountOfPlaneBins - i - 1] = rightBox.Area();
				}

				//calculate SAH cost for the binAmount - 1 planes
				scale = boundsDifference / amountOfBins;
				for (int i{}; i < amountOfPlaneBins; ++i)
				{
					const float planeCost{ leftCount[i] * leftArea[i] + rightCount[i] + rightArea[i] };
					if (planeCost < bestCost)
					{
						axis = axisIdx;
						splitPos = minBounds + scale * (i + 1);
						bestCost = planeCost;
					}
				}
			}
			return bestCost;
		}

		float EvalulateSAH(BVHNode& node, int axis, float pos)
		{
			AABB leftBox{};
			AABB rightBox{};
			int leftCount{};
			int rightCount{};
			for (unsigned int idx{}; idx < node.idxCount; idx += 3)
			{
				const unsigned int idxOffset{ node.firstIdx + idx };
				const Vector3& v0{ transformedPositions[indices[idxOffset]] };
				const Vector3& v1{ transformedPositions[indices[idxOffset + 1]] };
				const Vector3& v2{ transformedPositions[indices[idxOffset + 2]] };
				const Vector3 centroid{ (v0 + v1 + v2) / 3.f };

				if (centroid[axis] > pos)
				{
					++leftCount;
					leftBox.Grow(v0);
					leftBox.Grow(v1);
					leftBox.Grow(v2);
				}
				else
				{
					++rightCount;
					rightBox.Grow(v0);
					rightBox.Grow(v1);
					rightBox.Grow(v2);
				}
			}

			const float cost{ leftCount * leftBox.Area() + rightCount * rightBox.Area() };
			return cost > 0 ? cost : FLT_MAX;
		}
	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Vector3 origin{};
		Vector3 direction{};
		Vector3 inversedDir{};

		float min{ 0.0001f };
		float max{ FLT_MAX };

		Ray(const Vector3& origin, const Vector3& dir)
			: origin{ origin }
			, direction{ dir }
			, inversedDir{ 1.f / dir.x, 1.f / dir.y, 1.f / dir.z }
		{
		}

		Ray(const Vector3& origin, const Vector3& dir, float minDistance, float maxDistance)
			: origin{origin}
			, direction{dir}
			, inversedDir{1.f / dir.x, 1.f / dir.y, 1.f / dir.z}
			, min {minDistance}
			, max {maxDistance}
		{
		}
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}