#pragma once
#include "GBInclude.h"
#include <vector>
#include <map>
#include <algorithm>

struct GBGrid;

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

	std::vector<GBStaticGeometry*> staticGeometry;
	std::vector<GBGrid*> grids;
	std::vector<GBCollider*> colliders;
};

enum GBGridType
{
	NORMAL = 0,
	TERRAIN = 1
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
	int index = 0;
	std::vector<GBCell*> occupiedCells;
	std::vector<GBStaticGeometry*> staticGeometry;
	std::vector<GBGrid*> grids;
	GBGridType type = GBGridType::NORMAL;

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
		, type(GBGridType::NORMAL)
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


	GBGrid(const GBVector3& originPoint, float cellSize, int numCellsX, int numCellsY, int numCellsZ, int index, GBGridType type = GBGridType::NORMAL)
		: origin(originPoint), cellSize(cellSize), cellsX(numCellsX), cellsY(numCellsY), cellsZ(numCellsZ), index(index), type(type)
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

	void sampleColliders(GBAABB sampleAABB, std::vector<GBCollider*>& outCols)
	{
		std::vector<GBCell*> sampled;
		sampleGrid(sampleAABB, sampled);
		for (GBCell* pCell : sampled)
		{
			for (GBCollider* pCol: pCell->colliders)
			{
				if (std::find(outCols.begin(), outCols.end(), pCol) == outCols.end())
					outCols.push_back(pCol);
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

	void updateAABB()
	{
		aabb = toAABB();
	}

	void insertCollider(GBCollider& col)
	{
		GBAABB sample = col.aabb;
		std::vector<GBCell*> overlappingCells;
		sampleGrid(sample, overlappingCells); // now we get pointers
		for (GBCell* cell : overlappingCells)
		{
			cell->colliders.push_back(&col);
			col.occupiedCells.push_back(cell);
		}
	}

	void removeCollider(GBCollider& col)
	{
		for (GBCell* cell : col.occupiedCells)
		{
			// Remove all occurrences of the collider pointer in this cell
			cell->colliders.erase(
				std::remove(cell->colliders.begin(), cell->colliders.end(), (GBCollider*)&col),
				cell->colliders.end()
			);
		}
		col.occupiedCells.clear();
	}

	void moveCollider(GBCollider& col)
	{
		removeCollider(col);
		insertCollider(col);
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
			GBSATCollisionData data;
			GBAABB cellAABB = cell->toAABB();
			GBBoxCollider temp = GBBoxCollider::boxFromAABB(cellAABB);
			if (GBManifoldGeneration::GBCollisionBoxTriangleSAT(temp, *(GBTriangle*)&geometry, data))
			{
				cell->staticGeometry.push_back(&geometry);
				geometry.occupiedCells.push_back(cell);
			}
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

	void insertTerrainGrid(GBGrid& grid)
	{
		if (grid.type == GBGridType::TERRAIN)
		{
			GBAABB sampleAABB = grid.aabb;
			std::vector<GBCell*> overlappingCells;
			sampleGrid(sampleAABB, overlappingCells);
			for (GBCell* cell : overlappingCells)
			{
				// Grids inside grids are just for terrains for now, so the cells need to know that a grid is contained,
				// But the grid does not need to know where all the containing cells are...
				// Rather than using occupied cells for voxels we use them here to track the occupied cells
				cell->grids.push_back(&grid);
				grid.occupiedCells.push_back(cell);
			}
			if (std::find(grids.begin(), grids.end(), &grid) == grids.end())
				grids.push_back(&grid);
		}
	}

	void sampleTerrainGrids(const GBAABB& sampleAABB, std::vector<GBGrid*>& sampledGrids)
	{
		std::vector<GBCell*> overlappingCells;
		sampleGrid(sampleAABB, overlappingCells); // now we get pointers
		for (GBCell* cell : overlappingCells)
		{
			for (GBGrid* pGrid : cell->grids)
			{
				if (pGrid->type == GBGridType::TERRAIN)
				{
					if (std::find(sampledGrids.begin(), sampledGrids.end(), pGrid) == sampledGrids.end())
					{
						sampledGrids.push_back(pGrid);
					}
				}
			}
		}
	}

	void removeTerrainGrid(GBGrid& grid)
	{
		if (grid.type == GBGridType::TERRAIN)
		{
			for (GBCell* cell : grid.occupiedCells)
			{
				// Remove all occurrences of the terrain pointer in this cell
				cell->grids.erase(
					std::remove(cell->grids.begin(), cell->grids.end(), (GBGrid*)&grid),
					cell->grids.end()
				);
			}
			grid.occupiedCells.clear();
			auto it = std::find(grids.begin(), grids.end(), &grid);
			if (it != grids.end())
				grids.erase(it);
		}
	}

	bool raycast(
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		GBContact& outContact,
		float maxDistance = 1e30f,
		unsigned int mask = 0xFFFFFFFF)
	{
		GBVector3 dir = rayDir.normalized();

		GBContact gridHit;

		const static GBVector3 skin = { 0.0005f, 0.0005f , 0.0005f };
		GBAABB gridAABB = toAABB();
		gridAABB.halfExtents += skin;
		
		if (!GBManifoldGeneration::GBRaycastAABB(
			gridAABB,
			rayOrigin,
			dir,
			gridHit,
			maxDistance))
		{
			return false;
		}

		// DDA starts from entry point
		GBVector3 p = gridHit.position;

		// IMPORTANT: floorf, not truncation
		int ix = (int)floorf((p.x - origin.x) / cellSize);
		int iy = (int)floorf((p.y - origin.y) / cellSize);
		int iz = (int)floorf((p.z - origin.z) / cellSize);

		// Clamp edge cases
		ix = GBClamp(ix, 0, cellsX - 1);
		iy = GBClamp(iy, 0, cellsY - 1);
		iz = GBClamp(iz, 0, cellsZ - 1);

		int stepX = (dir.x >= 0.0f) ? 1 : -1;
		int stepY = (dir.y >= 0.0f) ? 1 : -1;
		int stepZ = (dir.z >= 0.0f) ? 1 : -1;

		const float INF = std::numeric_limits<float>::infinity();

		float invX = (dir.x != 0.0f) ? 1.0f / dir.x : INF;
		float invY = (dir.y != 0.0f) ? 1.0f / dir.y : INF;
		float invZ = (dir.z != 0.0f) ? 1.0f / dir.z : INF;

		auto nextBoundary = [](int cell, int step, float gridOrigin, float cellSize)
			{
				return gridOrigin + (cell + (step > 0 ? 1 : 0)) * cellSize;
			};

		float nextX = nextBoundary(ix, stepX, origin.x, cellSize);
		float nextY = nextBoundary(iy, stepY, origin.y, cellSize);
		float nextZ = nextBoundary(iz, stepZ, origin.z, cellSize);

		// IMPORTANT:
		// relative to ENTRY POINT p
		float tMaxX = (dir.x != 0.0f)
			? (nextX - p.x) * invX
			: INF;

		float tMaxY = (dir.y != 0.0f)
			? (nextY - p.y) * invY
			: INF;

		float tMaxZ = (dir.z != 0.0f)
			? (nextZ - p.z) * invZ
			: INF;

		float tDeltaX = (dir.x != 0.0f)
			? fabsf(cellSize * invX)
			: INF;

		float tDeltaY = (dir.y != 0.0f)
			? fabsf(cellSize * invY)
			: INF;

		float tDeltaZ = (dir.z != 0.0f)
			? fabsf(cellSize * invZ)
			: INF;

		float traveledT = 0.0f;
		float closestT = maxDistance;
		bool didHit = false;

		while (
			ix >= 0 && ix < cellsX &&
			iy >= 0 && iy < cellsY &&
			iz >= 0 && iz < cellsZ)
		{
			int ind = ix * cellsY * cellsZ +
				iy * cellsZ +
				iz;

			GBCell& cell = cells[ind];

			GBContact test;
			// Test contents
			{
				bool hitBody = false;
				if (!cell.colliders.empty())
				{
					if (GBManifoldGeneration::GBRaycastColliderVector(
						rayOrigin,
						dir,
						cell.colliders,
						test,
						mask))
					{
						if (test.penetrationDepth < closestT)
						{
							didHit = true;
							outContact = test;
							closestT = test.penetrationDepth;
						}
					}
				}

				//Test static geometry
				bool hitStatic = false;
				if (!cell.staticGeometry.empty())
				{
					
					if (GBManifoldGeneration::GBRaycastStaticGeometryVector(rayOrigin, dir, cell.staticGeometry,
						test, mask))
					{
						if (test.penetrationDepth < closestT)
						{
							didHit = true;
							outContact = test;
							closestT = test.penetrationDepth;
						}
					}
				}

			}

			// Canonical DDA stepping
			if (tMaxX < tMaxY)
			{
				if (tMaxX < tMaxZ)
				{
					ix += stepX;
					traveledT = tMaxX;
					tMaxX += tDeltaX;
				}
				else
				{
					iz += stepZ;
					traveledT = tMaxZ;
					tMaxZ += tDeltaZ;
				}
			}
			else
			{
				if (tMaxY < tMaxZ)
				{
					iy += stepY;
					traveledT = tMaxY;
					tMaxY += tDeltaY;
				}
				else
				{
					iz += stepZ;
					traveledT = tMaxZ;
					tMaxZ += tDeltaZ;
				}
			}

			if (traveledT > closestT)
				break;
		}

		return didHit;
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

	GBGridMap(GBVector3 origin = GBVector3(-50.0f, -50.0f, -25.0f), float cellSize = 1.0f, int cellsX = 20, int cellsY = 20, int cellsZ = 20, int gridsX = 10, int gridsY = 10, int gridsZ = 5) :
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
				outGrids.push_back(&grids[index]);
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

	void sampleColliders(GBAABB sampleAABB, std::vector<GBCollider*>& outCols, GBVector3 localOrigin = GBVector3::zero())
	{
		std::vector<GBGrid*> sampled;
		sampleAABB.center += localOrigin;
		sampleMap(sampleAABB, sampled);
		for (GBGrid* pGrid : sampled)
		{
			pGrid->sampleColliders(sampleAABB, outCols);
		}
	}


	void sampleStaticGeometry(GBAABB sampleAABB, std::vector<GBStaticGeometry*>& outGeometries)
	{
		std::vector<GBGrid*> occupiedGrids;
		sampleMap(sampleAABB, occupiedGrids);
		for (GBGrid* pGrid : occupiedGrids)
			pGrid->sampleStaticGeometry(sampleAABB, outGeometries);
	}


	void sampleTerrainGrids(GBAABB sampleAABB, std::vector<GBGrid*>& outGrids)
	{
		std::vector<GBGrid*> occupiedGrids;
		sampleMap(sampleAABB, occupiedGrids);
		for (GBGrid* pGrid : occupiedGrids)
		{
			pGrid->sampleTerrainGrids(sampleAABB, outGrids);
		}
	}

	void sampleTerrainGridStaticGeometry(GBAABB sampleAABB, std::vector<GBStaticGeometry*>& outGeometries)
	{
		std::vector<GBGrid*> occupiedGrids;
		sampleMap(sampleAABB, occupiedGrids);
		std::vector<GBGrid*> terrainGrids;
		for (GBGrid* pGrid : occupiedGrids)
		{
			pGrid->sampleTerrainGrids(sampleAABB, terrainGrids);
		}

		for (GBGrid* terrainGrid : terrainGrids)
		{
			terrainGrid->sampleStaticGeometry(sampleAABB, outGeometries);
		}
	}


	void insertBody(GBBody& body)
	{
		for (GBCollider* pCol : body.colliders)
		{
			std::vector<GBVector3> indices = sampleGrid3DIndices(pCol->aabb);
			for (GBVector3 index3D : indices)
			{
				int singleIndex = gridIndex(index3D.x, index3D.y, index3D.z);
				if (singleIndex >= 0)
				{
					if (grids.find(singleIndex) == grids.end())
					{
						grids[singleIndex] = GBGrid(gridOriginFromIndices(index3D.x, index3D.y, index3D.z), cellSize, cellsX, cellsY, cellsZ, singleIndex);
					}
					grids[singleIndex].insertCollider(*pCol);
					if (std::find(occupiedGridIndices.begin(), occupiedGridIndices.end(), singleIndex) == occupiedGridIndices.end())
						occupiedGridIndices.push_back(singleIndex);
				}
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
			sample.halfExtents = GBMax(sample.halfExtents, { 0.05f, 0.05f, 0.05f });
			break;
		}
		std::vector<GBGrid*> occupiedGrids;
		sampleMap(sample, occupiedGrids, true);
		for (GBGrid* pGrid : occupiedGrids)
		{
			pGrid->insertStaticGeometry(sample, geometry);
		}
	}

	void insertTerrainGrid(GBGrid& grid)
	{
		if (grid.type == GBGridType::TERRAIN)
		{
			std::vector<GBGrid*> occupiedGrids;
			sampleMap(grid.aabb, occupiedGrids, true);
			for (GBGrid* pGrid : occupiedGrids)
			{
				pGrid->insertTerrainGrid(grid);
			}
		}
	}

	void removeTerrainGrid(GBGrid& grid)
	{
		if (grid.type == GBGridType::TERRAIN)
		{
			std::vector<GBGrid*> occupiedGrids;
			sampleMap(grid.aabb, occupiedGrids, false);
			for (GBGrid* pGrid : occupiedGrids)
			{
				pGrid->removeTerrainGrid(grid);
			}
		}
	}

	void removeBody(GBBody& body)
	{
		for (GBCollider* pCol : body.colliders)
		{
			for (GBCell* cell : pCol->occupiedCells)
			{
				// Remove all occurrences of the collider pointer in this cell
				cell->colliders.erase(
					std::remove(cell->colliders.begin(), cell->colliders.end(), pCol),
					cell->colliders.end()
				);
			}
			pCol->occupiedCells.clear();
		}
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
			std::vector<GBGrid*> terrainGrids;
			
			pGrid->sampleTerrainGrids(sample, terrainGrids);
			if (terrainGrids.size() > 0)
			{
				for (GBGrid* pTerrainGrid : terrainGrids)
				{
					pTerrainGrid->removeGeometry(geometry);
				}
			}
			else
			{
				pGrid->removeGeometry(geometry);
			}
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
		float maxDistance = FLT_MAX,
		unsigned int mask = 0xFFFFFFFF)
	{

		// Normalize ray
		GBVector3 dir = rayDir.normalized();

		// Check ray against whole map first
		GBAABB mapAABB = toAABB();

		GBContact entryContact;
		if (!GBManifoldGeneration::GBRaycastAABB(
			mapAABB,
			rayOrigin,
			dir,
			entryContact,
			maxDistance))
		{
			return false;
		}

		// DDA traversal begins from map entry point
		GBVector3 p = entryContact.position;

		GBVector3 extents = gridExtents();

		// IMPORTANT: use floorf, not truncation
		int gx = (int)floorf((p.x - origin.x) / extents.x);
		int gy = (int)floorf((p.y - origin.y) / extents.y);
		int gz = (int)floorf((p.z - origin.z) / extents.z);

		// Clamp to valid range in case entry point lies exactly on boundary
		gx = GBClamp(gx, 0, gridsX - 1);
		gy = GBClamp(gy, 0, gridsY - 1);
		gz = GBClamp(gz, 0, gridsZ - 1);

		int stepX = (dir.x >= 0.0f) ? 1 : -1;
		int stepY = (dir.y >= 0.0f) ? 1 : -1;
		int stepZ = (dir.z >= 0.0f) ? 1 : -1;

		const float INF = std::numeric_limits<float>::infinity();

		float invX = (dir.x != 0.0f) ? 1.0f / dir.x : INF;
		float invY = (dir.y != 0.0f) ? 1.0f / dir.y : INF;
		float invZ = (dir.z != 0.0f) ? 1.0f / dir.z : INF;

		auto nextBoundary = [](int cell, int step, float gridOrigin, float cellSize)
			{
				return gridOrigin + (cell + (step > 0 ? 1 : 0)) * cellSize;
			};

		float nextX = nextBoundary(gx, stepX, origin.x, extents.x);
		float nextY = nextBoundary(gy, stepY, origin.y, extents.y);
		float nextZ = nextBoundary(gz, stepZ, origin.z, extents.z);

		// IMPORTANT:
		// tMax is relative to traversal start point p,
		// NOT original ray origin
		float tMaxX = (dir.x != 0.0f)
			? (nextX - p.x) * invX
			: INF;

		float tMaxY = (dir.y != 0.0f)
			? (nextY - p.y) * invY
			: INF;

		float tMaxZ = (dir.z != 0.0f)
			? (nextZ - p.z) * invZ
			: INF;

		float tDeltaX = (dir.x != 0.0f)
			? fabsf(extents.x * invX)
			: INF;

		float tDeltaY = (dir.y != 0.0f)
			? fabsf(extents.y * invY)
			: INF;

		float tDeltaZ = (dir.z != 0.0f)
			? fabsf(extents.z * invZ)
			: INF;

		float traveledT = 0.0f;
		float closestT = maxDistance;
		bool didHit = false;
		std::unordered_set<GBGrid*> terrainGrids;

		while (
			gx >= 0 && gx < gridsX &&
			gy >= 0 && gy < gridsY &&
			gz >= 0 && gz < gridsZ)
		{
			int gridIdx = gridIndex(gx, gy, gz);

			auto it = grids.find(gridIdx);
			if (it != grids.end())
			{
				GBGrid& grid = it->second;
				GBContact test;
				if (grid.raycast(rayOrigin, rayDir, test, maxDistance, mask))
				{
					if (test.penetrationDepth < closestT)
					{
						didHit = true;
						outContact = test;
						closestT = test.penetrationDepth;
					}
				}


				for (GBGrid* terrainGrid : grid.grids)
				{
					auto [_, inserted] = terrainGrids.insert(terrainGrid);

					if (inserted)
					{
						GBContact terrainTest;

						if (terrainGrid->raycast(
							rayOrigin,
							rayDir,
							terrainTest,
							maxDistance,
							mask))
						{
							if (terrainTest.penetrationDepth < closestT)
							{
								didHit = true;
								outContact = terrainTest;
								closestT = terrainTest.penetrationDepth;
							}
						}
					}
				}
			}

			// Step along smallest boundary distance
			if (tMaxX < tMaxY)
			{
				if (tMaxX < tMaxZ)
				{
					gx += stepX;
					traveledT = tMaxX;
					tMaxX += tDeltaX;
				}
				else
				{
					gz += stepZ;
					traveledT = tMaxZ;
					tMaxZ += tDeltaZ;
				}
			}
			else
			{
				if (tMaxY < tMaxZ)
				{
					gy += stepY;
					traveledT = tMaxY;
					tMaxY += tDeltaY;
				}
				else
				{
					gz += stepZ;
					traveledT = tMaxZ;
					tMaxZ += tDeltaZ;
				}
			}

			if (traveledT > closestT)
				break;
		}

		return didHit;
	}

	std::vector<int> raycastGridIndices(
		const GBVector3& rayOrigin,
		const GBVector3& rayDir,
		float maxDistance = FLT_MAX,
		unsigned int mask = 0xFFFFFFFF)
	{
		std::vector<int> indices;

		// Normalize ray
		GBVector3 dir = rayDir.normalized();

		// Check ray against whole map first
		GBAABB mapAABB = toAABB();

		GBContact entryContact;
		if (!GBManifoldGeneration::GBRaycastAABB(
			mapAABB,
			rayOrigin,
			dir,
			entryContact,
			maxDistance))
		{
			return indices;
		}

		// DDA traversal begins from map entry point
		GBVector3 p = entryContact.position;

		GBVector3 extents = gridExtents();

		// IMPORTANT: use floorf, not truncation
		int gx = (int)floorf((p.x - origin.x) / extents.x);
		int gy = (int)floorf((p.y - origin.y) / extents.y);
		int gz = (int)floorf((p.z - origin.z) / extents.z);

		// Clamp to valid range in case entry point lies exactly on boundary
		gx = GBClamp(gx, 0, gridsX - 1);
		gy = GBClamp(gy, 0, gridsY - 1);
		gz = GBClamp(gz, 0, gridsZ - 1);

		int stepX = (dir.x >= 0.0f) ? 1 : -1;
		int stepY = (dir.y >= 0.0f) ? 1 : -1;
		int stepZ = (dir.z >= 0.0f) ? 1 : -1;

		const float INF = std::numeric_limits<float>::infinity();

		float invX = (dir.x != 0.0f) ? 1.0f / dir.x : INF;
		float invY = (dir.y != 0.0f) ? 1.0f / dir.y : INF;
		float invZ = (dir.z != 0.0f) ? 1.0f / dir.z : INF;

		auto nextBoundary = [](int cell, int step, float gridOrigin, float sz)
			{
				return gridOrigin + (cell + (step > 0 ? 1 : 0)) * sz;
			};

		float nextX = nextBoundary(gx, stepX, origin.x, extents.x);
		float nextY = nextBoundary(gy, stepY, origin.y, extents.y);
		float nextZ = nextBoundary(gz, stepZ, origin.z, extents.z);

		// IMPORTANT:
		// tMax is relative to traversal start point p,
		// NOT original ray origin
		float tMaxX = (dir.x != 0.0f)
			? (nextX - p.x) * invX
			: INF;

		float tMaxY = (dir.y != 0.0f)
			? (nextY - p.y) * invY
			: INF;

		float tMaxZ = (dir.z != 0.0f)
			? (nextZ - p.z) * invZ
			: INF;

		float tDeltaX = (dir.x != 0.0f)
			? fabsf(extents.x * invX)
			: INF;

		float tDeltaY = (dir.y != 0.0f)
			? fabsf(extents.y * invY)
			: INF;

		float tDeltaZ = (dir.z != 0.0f)
			? fabsf(extents.z * invZ)
			: INF;

		float traveledT = 0.0f;

		while (
			gx >= 0 && gx < gridsX &&
			gy >= 0 && gy < gridsY &&
			gz >= 0 && gz < gridsZ)
		{
			int gridIdx = gridIndex(gx, gy, gz);

			indices.push_back(gridIdx);

			// Step along smallest boundary distance
			if (tMaxX < tMaxY)
			{
				if (tMaxX < tMaxZ)
				{
					gx += stepX;
					traveledT = tMaxX;
					tMaxX += tDeltaX;
				}
				else
				{
					gz += stepZ;
					traveledT = tMaxZ;
					tMaxZ += tDeltaZ;
				}
			}
			else
			{
				if (tMaxY < tMaxZ)
				{
					gy += stepY;
					traveledT = tMaxY;
					tMaxY += tDeltaY;
				}
				else
				{
					gz += stepZ;
					traveledT = tMaxZ;
					tMaxZ += tDeltaZ;
				}
			}

			if (traveledT > maxDistance)
				break;
		}

		return indices;
	}
};