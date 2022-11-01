#pragma once
#include <cassert>

#include "Math.h"
#include "vector"
#include <iostream>

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
		Vector3 minAABB{};
		Vector3 maxAABB{};

		std::vector<int> posIndices{};

		BVHNode* leftNode {nullptr};
		BVHNode* rightNode{nullptr};

		~BVHNode()
		{
			delete leftNode;
			delete rightNode;
		}

		void CalculateAABB(std::vector<int>& indices, std::vector<Vector3>& positions, int start, int end)
		{
			if (indices.size() > 0)
			{
				minAABB = positions[indices[start]];
				maxAABB = positions[indices[start]];
				for (size_t i{ static_cast<size_t>(start) }; i < static_cast<size_t>(end); ++i)
				{
					Vector3 pos{ positions[indices[i]] };
					minAABB = Vector3::Min(pos, minAABB);
					maxAABB = Vector3::Max(pos, maxAABB);
				}
			}
		}		
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
			delete pBVHNode;
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

		BVHNode* pBVHNode = nullptr;
		const int leafSize{4};

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
				const Vector3 v0{ positions[static_cast<size_t>(indices[startIdx])] };
				const Vector3 v1{ positions[static_cast<size_t>(indices[++startIdx])] };
				const Vector3 v2{ positions[static_cast<size_t>(indices[++startIdx])] };

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


			UpdateTransformedAABB(finalTransform);			
			UpdateBVH(*pBVHNode, 0, static_cast<int>(indices.size()));
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

		int GetPartitionIndex(int axis, std::vector<Vector3>& centroids, int centroidOffset, int start, int end)
		{

			int centroidStartIdx{ (start - centroidOffset) / 3 };
			int rangeIdx{ end - start - 1};
			int pivotIdx{ centroidStartIdx + rangeIdx / 6};
			float pivotVal{ centroids[pivotIdx][axis] };

			int centroidEndIdx{centroidStartIdx + std::min(static_cast<int>(centroids.size() - 1), rangeIdx / 3)};

			while (centroidStartIdx <= centroidEndIdx)
			{
				while (centroids[centroidStartIdx][axis] < pivotVal)
				{
					++centroidStartIdx;
				}
				while (centroids[centroidEndIdx][axis] > pivotVal)
				{
					--centroidEndIdx;
				}
				if (centroidStartIdx <= centroidEndIdx)
				{
					//Swap centroids
					std::swap(centroids[centroidStartIdx], centroids[centroidEndIdx]);
					//Swap positions
					size_t startOffset{ static_cast<size_t>(centroidStartIdx * 3 )};
					size_t endOffset{ static_cast<size_t>(centroidEndIdx * 3) };
					std::swap(indices[startOffset], indices[endOffset]);
					std::swap(indices[startOffset + 1], indices[endOffset + 1]);
					std::swap(indices[startOffset + 2], indices[endOffset + 2]);
					std::swap(normals[startOffset / 3], normals[endOffset / 3]);
					std::swap(transformedNormals[startOffset / 3], transformedNormals[endOffset / 3]);

					++centroidStartIdx;
					--centroidEndIdx;
				}
			}
			return start + centroidStartIdx * 3;
		}

		void QuicksortBVH(int axis, std::vector<Vector3>& centroids, int centroidOffset, int start, int end)
		{
			if (start < end - 3)
			{
				int pivot{ GetPartitionIndex(axis, centroids, centroidOffset, start, end) };
				int pivotOffset{ pivot - 1 };
				
				QuicksortBVH(axis, centroids, start, start, pivotOffset);				
				QuicksortBVH(axis, centroids, start, pivot, end);
			}
		}

		int SortOnAxis(Axis axis, int start, int end)
		{
			//Calculate centroids of the triangles between the start and end index of indices
			std::vector<Vector3> centroids{};
			int deltaIdx{ end - start };
			centroids.clear();
			centroids.reserve((deltaIdx) / 3);
			for (size_t i{static_cast<size_t>(start)}; i < end - 3; i += 3)
			{
				Vector3 centroid{ (transformedPositions[indices[i]] + positions[indices[i + 1]] + positions[indices[i + 2]]) / 3};
				centroids.emplace_back(centroid);
			}

			//Quicksort algorithm to sort the values along the chosen axis
			QuicksortBVH(static_cast<int>(axis), centroids, start, start, end - 3);

			//Return the index to split on. Multiply by 3 to fit the indices vector
			int splitIdx{ static_cast<int>(3 * centroids.size() / 2.f) - 1};
			return splitIdx;
		}

		void UpdateBVH(BVHNode& node, int start, int end)
		{
			//Calculate bounding box of positions between start and end
			node.CalculateAABB(indices, transformedPositions, start, end);			

			int test{ end - start };
			if ((end - start) > (3 * leafSize))
			{
				//Determine axis the algorithm needs to use
				float axisX{ node.maxAABB.x - node.minAABB.x };
				float axisY{ node.maxAABB.y - node.minAABB.y };
				float axisZ{ node.maxAABB.z - node.minAABB.z };

				float axisLongest{ std::max(axisX, std::max(axisY, axisZ)) };
				Axis axis{ Axis::axisX };
				if (abs(axisLongest - axisY) <= FLT_EPSILON)
				{
					axis = Axis::axisY;
				}
				else if (axisLongest == axisZ)
				{
					axis = Axis::axisZ;
				}
				
				int splitIdx{ SortOnAxis(axis, start, end) };

				//Init nodes if they don't exist yet
				if (!node.leftNode) node.leftNode = new BVHNode{};
				if (!node.rightNode) node.rightNode = new BVHNode{};	
				UpdateBVH(*node.leftNode, start, start + splitIdx - 1);
				UpdateBVH(*node.rightNode, start + splitIdx, end);
			}
			else
			{
				node.posIndices.clear();
				node.posIndices.reserve(static_cast<size_t>(end - start));
				for (int i{ start }; i < end; ++i)
				{
					node.posIndices.emplace_back(indices[static_cast<size_t>(i)]);
				}
			}
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

		float min{ 0.0001f };
		float max{ FLT_MAX };
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