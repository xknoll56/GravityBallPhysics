#pragma once
#include "GBInclude.h"
#include <vector>
#include <map>
#include <algorithm>

struct GBCell
{
	GBVector3 anchor;
	float cellSize;
	GBVector3 gridOrigin;
	int gridIndex;
	bool isOccupied = false;

	GBCell(GBVector3 origin, int gridIndex)
		: anchor{ 0, 0, 0 }, cellSize(1.0f), gridIndex(gridIndex)
	{
	}

	GBCell(const GBVector3& anchorPoint, float size, int gridIndex)
		: anchor(anchorPoint), cellSize(size), gridIndex(gridIndex)
	{
	}

	GBVector3 getCenter() const
	{
		return GBVector3(
			anchor.x + cellSize * 0.5f,
			anchor.y + cellSize * 0.5f,
			anchor.z + cellSize * 0.5f
		);
	}

	GBAABB toAABB() const
	{
		GBVector3 halfExtents(cellSize * 0.5f, cellSize * 0.5f, cellSize * 0.5f);
		GBVector3 center = getCenter();
		return GBAABB(center, halfExtents);
	}
	std::vector<GBBody*> bodies;
	std::vector<GBStaticGeometry*> staticGeometry;
};
struct GBGrid
{
	GBVector3 origin;       // Origin point of the grid
	float cellSize;     // Size of each cell in the grid
	int cellsX;            // Number of cells along the X axis
	int cellsY;            // Number of cells along the Y axis
	int cellsZ;            // Number of cells along the Z axis
	std::vector<GBCell> cells;
	GBAABB aabb;
	std::vector<GBBody*> bodies;
	int index = 0;
	std::vector<GBCell*> occupiedCells;
	std::vector<GBStaticGeometry*> staticGeometry;

	//std::vector<GBCollider> colliders;
	// ----------------------------
	// Default constructor
	// 10x10x10 grid, size 1.0f, origin at (0,0,0)
	// ----------------------------
	GBGrid()
		: origin(0, 0, 0)
		, cellSize(1.0f)
		, cellsX(20)
		, cellsY(20)
		, cellsZ(10)
		, index(0)
	{
		cells.reserve(cellsX * cellsY * cellsZ);

		for (int x = 0; x < cellsX; ++x)
		{
			for (int y = 0; y < cellsY; ++y)
			{
				for (int z = 0; z < cellsZ; ++z)
				{
					GBVector3 cellAnchor(
						origin.x + x * cellSize,
						origin.y + y * cellSize,
						origin.z + z * cellSize
					);

					cells.emplace_back(cellAnchor, cellSize, index);
				}
			}
		}
		aabb = toAABB();
	}


	GBGrid(const GBVector3& originPoint, float cellSize, int numCellsX, int numCellsY, int numCellsZ, int index)
		: origin(originPoint), cellSize(cellSize), cellsX(numCellsX), cellsY(numCellsY), cellsZ(numCellsZ), index(index)
	{
		cells.reserve(cellsX * cellsY * cellsZ);
		for (int x = 0; x < cellsX; ++x)
		{
			for (int y = 0; y < cellsY; ++y)
			{
				for (int z = 0; z < cellsZ; ++z)
				{
					GBVector3 cellAnchor = GBVector3(
						origin.x + x * cellSize,
						origin.y + y * cellSize,
						origin.z + z * cellSize
					);
					cells.emplace_back(cellAnchor, cellSize, index);
				}
			}
		}
		aabb = toAABB();
	}

	int positionToIndex(GBVector3 position)
	{
		GBVector3 clamped = (position - origin) / cellSize;
		clamped.truncate();
		int clampedX = clamped.x;
		int clampedY = clamped.y;
		int clampedZ = clamped.z;
		if (clampedX >= 0 && clampedX < cellsX &&
			clampedY >= 0 && clampedY < cellsY &&
			clampedZ >= 0 && clampedZ < cellsZ)
			return clampedX * cellsY * cellsZ + clampedY * cellsZ + clampedZ;
		else
			return -1;
	}

	void setOccupied(GBVector3 cellPosition, bool value = true)
	{
		int ind = positionToIndex(cellPosition);
		if (ind >= 0 && ind < cells.size())
		{
			cells[ind].isOccupied = value;
			if (value)
			{
				if (std::find(occupiedCells.begin(), occupiedCells.end(), &cells[ind]) == occupiedCells.end())
					occupiedCells.push_back(&cells[ind]);
			}
			else
			{
				auto it = std::find(occupiedCells.begin(), occupiedCells.end(), &cells[ind]);
				if(it != occupiedCells.end())
					occupiedCells.push_back(&cells[ind]);
			}
		}
	}

	void sampleGrid(GBAABB sampleAABB, std::vector<GBCell*>& outCells, bool checkForOccupied = false)
	{
		if (!aabb.intersects(sampleAABB)) return;

		GBVector3 minP = (sampleAABB.low() - origin) / cellSize;
		GBVector3 maxP = (sampleAABB.high() - origin) / cellSize;

		minP.truncate();
		maxP.truncate();

		int i0 = std::max(0, (int)minP.x);
		int j0 = std::max(0, (int)minP.y);
		int k0 = std::max(0, (int)minP.z);

		int i1 = std::min(cellsX - 1, (int)maxP.x);
		int j1 = std::min(cellsY - 1, (int)maxP.y);
		int k1 = std::min(cellsZ - 1, (int)maxP.z);

		for (int i = i0; i <= i1; ++i)
			for (int j = j0; j <= j1; ++j)
				for (int k = k0; k <= k1; ++k)
				{
					GBCell* pCell = &cells[i * cellsY * cellsZ + j * cellsZ + k];
					if (!checkForOccupied || pCell->isOccupied)
						outCells.push_back(pCell);
				}
	}

	void sampleBodies(GBAABB sampleAABB, std::vector<GBBody*>& outBodies)
	{
		std::vector<GBCell*> sampled;
		sampleGrid(sampleAABB, sampled);
		for (GBCell* pCell : sampled)
		{
			for (GBBody* pBody : pCell->bodies)
			{
				if (std::find(outBodies.begin(), outBodies.end(), pBody) == outBodies.end())
					outBodies.push_back(pBody);
			}
		}
	}

	GBAABB toAABB() const
	{
		GBVector3 halfExtents(
			(cellsX * cellSize) * 0.5f,
			(cellsY * cellSize) * 0.5f,
			(cellsZ * cellSize) * 0.5f
		);
		GBVector3 center = GBVector3(
			origin.x + halfExtents.x,
			origin.y + halfExtents.y,
			origin.z + halfExtents.z
		);
		return GBAABB(center, halfExtents);
	}

	void insertBody(GBBody& body)
	{
		GBAABB sample = body.aabb;
		std::vector<GBCell*> overlappingCells;
		sampleGrid(sample, overlappingCells); // now we get pointers
		for (GBCell* cell : overlappingCells)
		{
			cell->bodies.push_back(&body);
			body.occupiedCells.push_back(cell);
		}
		if (std::find(bodies.begin(), bodies.end(), &body) == bodies.end())
			bodies.push_back(&body);
	}

	void removeBody(GBBody& body)
	{
		for (GBCell* cell : body.occupiedCells)
		{
			// Remove all occurrences of the collider pointer in this cell
			cell->bodies.erase(
				std::remove(cell->bodies.begin(), cell->bodies.end(), (GBBody*)&body),
				cell->bodies.end()
			);
		}
		body.occupiedCells.clear();
		auto it = std::find(bodies.begin(), bodies.end(), &body);
		if (it != bodies.end())
			bodies.erase(it);
	}

	void moveBody(GBBody& body)
	{
		removeBody(body);
		insertBody(body);
	}

	void insertStaticGeometry(const GBAABB& sample, GBStaticGeometry& geometry)
	{
		//GBAABB sample;
		//switch (geometry.type)
		//{
		//case GBStaticGeometryType::QUAD:
		//	break;
		//case GBStaticGeometryType::TRIANGLE:
		//	GBTriangle* pTriangle = (GBTriangle*)&geometry;
		//	sample = pTriangle->toAABB();
		//	break;
		//}
		std::vector<GBCell*> overlappingCells;
		sampleGrid(sample, overlappingCells); // now we get pointers
		for (GBCell* cell : overlappingCells)
		{
			cell->staticGeometry.push_back(&geometry);
			geometry.occupiedCells.push_back(cell);
		}
		if (std::find(staticGeometry.begin(), staticGeometry.end(), &geometry) == staticGeometry.end())
			staticGeometry.push_back(&geometry);
	}

	void sampleStaticGeometry(const GBAABB& sampleAABB, std::vector<GBStaticGeometry*>& outGeometry)
	{
		std::vector<GBCell*> overlappingCells;
		sampleGrid(sampleAABB, overlappingCells); // now we get pointers
		for (GBCell* cell : overlappingCells)
		{
			for (GBStaticGeometry* sg : cell->staticGeometry)
			{
				if (std::find(outGeometry.begin(), outGeometry.end(), sg) == outGeometry.end())
				{
					outGeometry.push_back(sg);
				}
			}
		}
	}

	void removeGeometry(GBStaticGeometry& geometry)
	{
		for (GBCell* cell : geometry.occupiedCells)
		{
			// Remove all occurrences of the collider pointer in this cell
			cell->staticGeometry.erase(
				std::remove(cell->staticGeometry.begin(), cell->staticGeometry.end(), (GBStaticGeometry*)&geometry),
				cell->staticGeometry.end()
			);
		}
		geometry.occupiedCells.clear();
		auto it = std::find(staticGeometry.begin(), staticGeometry.end(), &geometry);
		if (it != staticGeometry.end())
			staticGeometry.erase(it);
	}

	bool raycast(
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = 1e30f)
	{
		GBContact gridHit;
		if (!GBManifoldGeneration::GBRaycastAABB(toAABB(), rayOrigin, rayDir, gridHit, maxDistance))
			return false;

		// Starting point (clamp to entry)
		GBVector3 p = gridHit.position;
		int ix = (int)((p.x - origin.x) / cellSize);
		int iy = (int)((p.y - origin.y) / cellSize);
		int iz = (int)((p.z - origin.z) / cellSize);

		int stepX = rayDir.x > 0 ? 1 : -1;
		int stepY = rayDir.y > 0 ? 1 : -1;
		int stepZ = rayDir.z > 0 ? 1 : -1;

		auto inv = GBVector3(
			rayDir.x != 0 ? 1.0f / rayDir.x : 1e30f,
			rayDir.y != 0 ? 1.0f / rayDir.y : 1e30f,
			rayDir.z != 0 ? 1.0f / rayDir.z : 1e30f
		);

		bool isZero[3];
		isZero[0] = rayDir.x == 0.0f ? true : false;
		isZero[1] = rayDir.y == 0.0f ? true : false;
		isZero[2] = rayDir.z == 0.0f ? true : false;

		auto nextBoundary = [&](int i, float o, float d)
			{
				return origin.x + (i + (d > 0)) * cellSize;
			};

		float tMaxX = (nextBoundary(ix, rayOrigin.x, rayDir.x) - rayOrigin.x) * inv.x;
		float tMaxY = (origin.y + (iy + (rayDir.y > 0)) * cellSize - rayOrigin.y) * inv.y;
		float tMaxZ = (origin.z + (iz + (rayDir.z > 0)) * cellSize - rayOrigin.z) * inv.z;

		float tDeltaX = cellSize * fabs(inv.x);
		float tDeltaY = cellSize * fabs(inv.y);
		float tDeltaZ = cellSize * fabs(inv.z);

		bool hit = false;
		float closestT = maxDistance;

		if (iz == cellsZ && !isZero[2] && rayDir.z < 0.0f)
			iz--;
		if (iy == cellsY && !isZero[1] && rayDir.y < 0.0f)
			iy--;
		if (ix == cellsX && !isZero[0] && rayDir.x < 0.0f)
			ix--;

		while (ix >= 0 && ix < cellsX &&
			iy >= 0 && iy < cellsY &&
			iz >= 0 && iz < cellsZ)
		{
			int ind = ix * cellsY * cellsZ + iy * cellsZ + iz;
			GBCell& cell = cells[ind];

			// 🔥 TEST CELL CONTENTS HERE 🔥
			if (cell.isOccupied)
			{
				if (GBManifoldGeneration::GBRaycastAABB(cell.toAABB(), rayOrigin, rayDir, outContact, maxDistance))
				{
					return true;
				}
			}
			else
			{
				if (cell.bodies.size() > 0 && GBManifoldGeneration::GBRaycastBodiesVector(rayOrigin, rayDir, cell.bodies, outContact))
				{
					return true;
				}
			}

			if ((tMaxX < tMaxY && tMaxX < tMaxZ) && !isZero[0]) {
				ix += stepX; tMaxX += tDeltaX;
			}
			else if ((tMaxY < tMaxZ || isZero[0]) && !isZero[1]) {  // <-- add fallback
				iy += stepY; tMaxY += tDeltaY;
			}
			else if (!isZero[2]) {
				iz += stepZ; tMaxZ += tDeltaZ;
			}
			else {
				// fallback if all smaller axes are zero (degenerate ray)
				if (!isZero[0]) { ix += stepX; tMaxX += tDeltaX; }
				else if (!isZero[1]) { iy += stepY; tMaxY += tDeltaY; }
				else if (!isZero[2]) { iz += stepZ; tMaxZ += tDeltaZ; }
				else break; // truly zero-length ray
			}

			if (GBMin(tMaxX, GBMin(tMaxY, tMaxZ)) > closestT)
				break;
		}

		return false;
	}

};

struct GBGridMap
{
	GBVector3 origin = { -50.0f, -50.0f, -25.0f };       // Origin point of the grid
	float cellSize = 1.0f;     // Size of each cell in the grid
	int cellsX = 10;            // Number of cells along the X axis
	int cellsY = 10;            // Number of cells along the Y axis
	int cellsZ = 10;            // Number of cells along the Z axis
	int gridsX = 10;
	int gridsY = 10;
	int gridsZ = 5;

	std::vector<int> occupiedGridIndices;
	std::map<int, GBGrid> grids;

	GBGridMap(GBVector3 origin = GBVector3(-50.0f, -50.0f, -25.0f), float cellSize = 1.0f, int cellsX = 10, int cellsY = 10, int cellsZ = 10, int gridsX = 10, int gridsY = 10, int gridsZ = 5) :
		origin(origin), cellSize(cellSize), cellsX(cellsX), cellsY(cellsY), cellsZ(cellsZ), gridsX(gridsX), gridsY(gridsY), gridsZ(gridsZ)
	{

	}

	const GBGridMap operator=(const GBGridMap& other)
	{
		this->origin = other.origin;
		cellSize = other.cellSize;
		cellsX = other.cellsX;
		cellsY = other.cellsY;
		cellsZ = other.cellsZ;
		gridsX = other.gridsX;
		gridsY = other.gridsY;
		gridsZ = other.gridsZ;
		occupiedGridIndices = other.occupiedGridIndices;
		grids = other.grids;
	}


	int gridCount() const
	{
		return gridsX * gridsY * gridsZ;
	}

	GBVector3 gridExtents()
	{
		return GBVector3(cellsX * cellSize, cellsY * cellSize, cellsZ * cellSize);
	}

	GBVector3 gridOriginFromIndices(int x, int y, int z)
	{
		GBVector3 extents = gridExtents();
		return origin + GBVector3(extents.x * x, extents.y * y, extents.z * z);
	}

	int gridIndex(int x, int y, int z)
	{
		if (x >= 0 && x < gridsX &&
			y >= 0 && y < gridsY &&
			z >= 0 && z < gridsZ)
		{
			return x * gridsY * gridsZ + y * gridsZ + z;
		}
		else
		{
			return -1;
		}
	}

	std::vector<int> sampleGridIndices(const GBAABB& sampleAABB)
	{
		std::vector<int> indices;
		GBVector3 minPoint = sampleAABB.low() - origin;
		GBVector3 extents = gridExtents();
		minPoint.x /= extents.x;
		minPoint.y /= extents.y;
		minPoint.z /= extents.z;
		GBVector3 maxPoint = sampleAABB.high() - origin;
		maxPoint.x /= extents.x;
		maxPoint.y /= extents.y;
		maxPoint.z /= extents.z;
		minPoint.truncate();
		maxPoint.truncate();

		int lengthX = maxPoint.x - minPoint.x;
		int lengthY = maxPoint.y - minPoint.y;
		int lengthZ = maxPoint.z - minPoint.z;

		for (int i = (int)minPoint.x; i <= (int)(minPoint.x + lengthX); i++)
		{
			for (int j = (int)minPoint.y; j <= (int)(minPoint.y + lengthY); j++)
			{
				for (int k = (int)minPoint.z; k <= (int)(minPoint.z + lengthZ); k++)
				{

					int index = gridIndex(i, j, k);
					if (index >= 0)
					{
						indices.push_back(index);
					}
				}
			}
		}

		//GBVector3 highIndices = pointToIndices(sampleAABB.high());
		//int index = gridIndex(highIndices.x, highIndices.y, highIndices.z);
		//if (std::find(indices.begin(), indices.end(), index) == indices.end())
		//	indices.push_back(index);

		return indices;
	}

	GBVector3 pointToIndices(GBVector3 point)
	{
		GBVector3 local = point - origin;
		GBVector3 extents = gridExtents();
		local.x /= extents.x;
		local.y /= extents.y;
		local.z /= extents.z;
		local.truncate();
		return local;
	}

	bool validIndex(int index) const
	{
		return index >= 0 && index < gridCount();
	}

	void setOccupied(GBVector3 cellPosition, bool value = true)
	{
		GBVector3 indices = pointToIndices(cellPosition);
		int ind = gridIndex(indices.x, indices.y, indices.z);
		GBVector3 index3D = pointToIndices(cellPosition);
		if (validIndex(ind))
		{
			if (grids.find(ind) == grids.end())
			{
				grids[ind] = GBGrid(gridOriginFromIndices(index3D.x, index3D.y, index3D.z), cellSize, cellsX, cellsY, cellsZ, ind);
				if (std::find(occupiedGridIndices.begin(), occupiedGridIndices.end(), ind) == occupiedGridIndices.end())
				{
					occupiedGridIndices.push_back(ind);
				}
			}
			grids[ind].setOccupied(cellPosition, value);
		}
	}


	GBAABB toAABB()
	{
		GBVector3 half = gridExtents();
		half.x *= gridsX;
		half.y *= gridsY;
		half.z *= gridsZ;
		half *= 0.5f;
		return GBAABB(origin + half, half);
	}

	std::vector<GBVector3> sampleGrid3DIndices(const GBAABB& sampleAABB)
	{
		std::vector<GBVector3> indices;

		GBVector3 minPoint = sampleAABB.low() - origin;
		GBVector3 extents = gridExtents();
		minPoint.x /= extents.x;
		minPoint.y /= extents.y;
		minPoint.z /= extents.z;
		GBVector3 maxPoint = sampleAABB.high() - origin;
		maxPoint.x /= extents.x;
		maxPoint.y /= extents.y;
		maxPoint.z /= extents.z;
		minPoint.truncate();
		maxPoint.truncate();

		int lengthX = maxPoint.x - minPoint.x;
		int lengthY = maxPoint.y - minPoint.y;
		int lengthZ = maxPoint.z - minPoint.z;

		for (int i = (int)minPoint.x; i <= (int)(minPoint.x + lengthX); i++)
		{
			for (int j = (int)minPoint.y; j <= (int)(minPoint.y + lengthY); j++)
			{
				for (int k = (int)minPoint.z; k <= (int)(minPoint.z + lengthZ); k++)
				{

					int index = gridIndex(i, j, k);
					if (index >= 0)
					{
						indices.push_back(GBVector3(i, j, k));
					}
				}
			}
		}
		//GBVector3 highIndex = pointToIndices(sampleAABB.high());
		//if (std::find(indices.begin(), indices.end(), highIndex) == indices.end())
		//	indices.push_back(highIndex);

		return indices;
	}


	void sampleMap(GBAABB sampleAABB, std::vector<GBGrid*>& outGrids, bool doCreateIfNotFound = false)
	{
		if (doCreateIfNotFound)
		{
			std::vector<GBVector3> indices3D = sampleGrid3DIndices(sampleAABB);
			for (GBVector3 index3D : indices3D)
			{
				int index = gridIndex(index3D.x, index3D.y, index3D.z);
				if (std::find(occupiedGridIndices.begin(), occupiedGridIndices.end(), index) == occupiedGridIndices.end())
				{
					grids[index] = GBGrid(gridOriginFromIndices(index3D.x, index3D.y, index3D.z), cellSize, cellsX, cellsY, cellsZ, index);
					occupiedGridIndices.push_back(index);
				}
			}
		}
		else
		{
			std::vector<int> indices = sampleGridIndices(sampleAABB);
			for (int index : indices)
			{
				if (grids.find(index) != grids.end())
					outGrids.push_back(&grids[index]);
			}
		}
	}
	
	void sampleCells(GBAABB sampleAABB, std::vector<GBCell*>& outCells, bool checkForOccupied = false)
	{
		std::vector<GBGrid*> sampled;
		sampleMap(sampleAABB, sampled);
		for (GBGrid* pGrid : sampled)
		{
			pGrid->sampleGrid(sampleAABB, outCells, checkForOccupied);
		}
	}

	void sampleBodies(GBAABB sampleAABB, std::vector<GBBody*>& outBodies, GBVector3 localOrigin = GBVector3::zero())
	{
		std::vector<GBGrid*> sampled;
		sampleAABB.center += localOrigin;
		sampleMap(sampleAABB, sampled);
		for (GBGrid* pGrid : sampled)
		{
			std::vector<GBBody*> bodies;
			pGrid->sampleBodies(sampleAABB, bodies);
			for (GBBody* body : bodies)
			{
				if (std::find(outBodies.begin(), outBodies.end(), body) == outBodies.end() && !body->ignoreSample)
				{
					outBodies.push_back(body);
				}
			}
		}
	}


	void sampleStaticGeometry(GBAABB sampleAABB, std::vector<GBStaticGeometry*>& outGeometries)
	{
		std::vector<GBGrid*> occupiedGrids;
		sampleMap(sampleAABB, occupiedGrids);
		for (GBGrid* pGrid : occupiedGrids)
			pGrid->sampleStaticGeometry(sampleAABB, outGeometries);
	}


	void insertBody(GBBody& body)
	{
		std::vector<GBVector3> indices = sampleGrid3DIndices(body.aabb);
		for (GBVector3 index3D : indices)
		{
			int singleIndex = gridIndex(index3D.x, index3D.y, index3D.z);
			if (singleIndex >= 0)
			{
				if (grids.find(singleIndex) == grids.end())
				{
					grids[singleIndex] = GBGrid(gridOriginFromIndices(index3D.x, index3D.y, index3D.z), cellSize, cellsX, cellsY, cellsZ, singleIndex);
				}
				grids[singleIndex].insertBody(body);
				if (std::find(occupiedGridIndices.begin(), occupiedGridIndices.end(), singleIndex) == occupiedGridIndices.end())
					occupiedGridIndices.push_back(singleIndex);
			}
		}
	}

	void insertStaticGeometry(GBStaticGeometry& geometry)
	{
		GBAABB sample;
		switch (geometry.type)
		{
		case GBStaticGeometryType::QUAD:
			break;
		case GBStaticGeometryType::TRIANGLE:
			GBTriangle* pTriangle = (GBTriangle*)&geometry;
			sample = pTriangle->toAABB();
			break;
		}
		std::vector<GBGrid*> occupiedGrids;
		sampleMap(sample, occupiedGrids, true);
		for (GBGrid* pGrid : occupiedGrids)
		{
			pGrid->insertStaticGeometry(sample, geometry);
		}
	}

	void removeBody(GBBody& body)
	{
		for (GBCell* cell : body.occupiedCells)
		{
			// Remove all occurrences of the collider pointer in this cell
			cell->bodies.erase(
				std::remove(cell->bodies.begin(), cell->bodies.end(), (GBBody*)&body),
				cell->bodies.end()
			);
			auto it = std::find(grids[cell->gridIndex].bodies.begin(), grids[cell->gridIndex].bodies.end(), &body);
			if (it != grids[cell->gridIndex].bodies.end())
				grids[cell->gridIndex].bodies.erase(it);
		}
		body.occupiedCells.clear();
	}

	void removeStaticGeometry(GBStaticGeometry& geometry)
	{
		GBAABB sample;
		switch (geometry.type)
		{
		case GBStaticGeometryType::QUAD:
			break;
		case GBStaticGeometryType::TRIANGLE:
			GBTriangle* pTriangle = (GBTriangle*)&geometry;
			sample = pTriangle->toAABB();
			break;
		}
		std::vector<GBGrid*> occupiedGrids;
		sampleMap(sample, occupiedGrids);
		for (GBGrid* pGrid : occupiedGrids)
		{
			pGrid->removeGeometry(geometry);
		}
	}

	void moveBody(GBBody& body)
	{
		removeBody(body);
		insertBody(body);
	}


	bool raycast(
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance)
	{
		// First, check if ray hits the whole map AABB
		GBAABB mapAABB = toAABB();
		GBContact entryContact;
		if (!GBManifoldGeneration::GBRaycastAABB(mapAABB, rayOrigin, rayDir, entryContact, maxDistance))
			return false;

		// Compute starting point in grid-map coordinates
		GBVector3 p = entryContact.position;

		GBVector3 extents = gridExtents();
		int gx = (int)((p.x - origin.x) / extents.x);
		int gy = (int)((p.y - origin.y) / extents.y);
		int gz = (int)((p.z - origin.z) / extents.z);

		int stepX = rayDir.x > 0 ? 1 : -1;
		int stepY = rayDir.y > 0 ? 1 : -1;
		int stepZ = rayDir.z > 0 ? 1 : -1;

		GBVector3 inv(
			rayDir.x != 0 ? 1.0f / rayDir.x : 1e30f,
			rayDir.y != 0 ? 1.0f / rayDir.y : 1e30f,
			rayDir.z != 0 ? 1.0f / rayDir.z : 1e30f
		);

		bool isZero[3] = { rayDir.x == 0, rayDir.y == 0, rayDir.z == 0 };

		auto nextBoundary = [&](int i, float o, float d, float gridSize, float originCoord)
			{
				return originCoord + (i + (d > 0)) * gridSize;
			};

		float tMaxX = (nextBoundary(gx, rayOrigin.x, rayDir.x, extents.x, origin.x) - rayOrigin.x) * inv.x;
		float tMaxY = (nextBoundary(gy, rayOrigin.y, rayDir.y, extents.y, origin.y) - rayOrigin.y) * inv.y;
		float tMaxZ = (nextBoundary(gz, rayOrigin.z, rayDir.z, extents.z, origin.z) - rayOrigin.z) * inv.z;

		float tDeltaX = extents.x * fabs(inv.x);
		float tDeltaY = extents.y * fabs(inv.y);
		float tDeltaZ = extents.z * fabs(inv.z);

		float closestT = maxDistance;

		// Correct for boundary entry
		if (gx == gridsX && !isZero[0] && rayDir.x < 0.0f) gx--;
		if (gy == gridsY && !isZero[1] && rayDir.y < 0.0f) gy--;
		if (gz == gridsZ && !isZero[2] && rayDir.z < 0.0f) gz--;

		while (gx >= 0 && gx < gridsX &&
			gy >= 0 && gy < gridsY &&
			gz >= 0 && gz < gridsZ)
		{
			int gridIdx = gridIndex(gx, gy, gz);
			auto it = grids.find(gridIdx);
			if (it != grids.end())
			{
				GBGrid& grid = it->second;
				GBContact cellContact;
				if (grid.raycast(rayOrigin, rayDir, cellContact, maxDistance))
				{
					outContact = cellContact;
					return true;
				}
			}

			// Step to next grid along ray
			if ((tMaxX < tMaxY && tMaxX < tMaxZ) && !isZero[0]) {
				gx += stepX; tMaxX += tDeltaX;
			}
			else if ((tMaxY < tMaxZ || isZero[0]) && !isZero[1]) {
				gy += stepY; tMaxY += tDeltaY;
			}
			else if (!isZero[2]) {
				gz += stepZ; tMaxZ += tDeltaZ;
			}
			else {
				if (!isZero[0]) { gx += stepX; tMaxX += tDeltaX; }
				else if (!isZero[1]) { gy += stepY; tMaxY += tDeltaY; }
				else if (!isZero[2]) { gz += stepZ; tMaxZ += tDeltaZ; }
				else break;
			}

			if (GBMin(tMaxX, GBMin(tMaxY, tMaxZ)) > closestT)
				break;
		}

		return false;
	}
};