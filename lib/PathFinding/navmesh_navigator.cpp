/* navmesh_navigator.cpp - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <stdio.h>

#include "rendersystem.h"		// FileExists
#include "navmesh_navigator.h"
#include "navmesh_common.h"

namespace lighthouse2 {

#define DETOUR_ERROR(X, ...) return NavMeshError(X, __VA_ARGS__)
#define DETOUR_LOG(...) NavMeshError(NMSUCCESS, __VA_ARGS__)

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::FindPath                                                 |
//  |  Finds the shortest path from start to end. *path* and *pathCount* are      |
//  |  the output. *maxCount* specifies the maximum path length in nodes.   LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindPath(float3 start, float3 end, dtPolyRef* path, int* pathCount, int maxCount)
{
	dtPolyRef startRef, endRef;
	m_errorCode = FindNearestPoly(start, &startRef);
	m_errorCode = FindNearestPoly(end, &endRef);
	if (m_errorCode) return m_errorCode;
	float startl[3] = { start.x, start.y, start.z };
	float endl[3] = { end.x, end.y, end.z };
	dtStatus err = m_query->findPath(startRef, endRef, startl, endl, &m_filter, path, pathCount, maxCount);
	if (dtStatusFailed(err))
		DETOUR_ERROR(NMDETOUR, "Could find a path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
			start.x, start.y, start.z, end.x, end.y, end.z);
	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::FindPath                                                 |
//  |  Finds the shortest path from start to end.                                 |
//  |  *path* is the resulting vector of world positions to follow.               |
//  |  *distToEnd* is the error from the last pos to the end, 0.0 when reachable. |
//  |  *maxCount* specifies the maximum path length in nodes.               LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindPath(float3 start, float3 end, std::vector<float3>& path, float& distToEnd, int maxCount)
{
	// Prepare pathfinding
	dtPolyRef startRef, endRef;
	m_errorCode = FindNearestPoly(start, &startRef);
	m_errorCode = FindNearestPoly(end, &endRef);
	if (m_errorCode) return m_errorCode;
	float startl[3] = { start.x, start.y, start.z };
	float endl[3] = { end.x, end.y, end.z };
	dtPolyRef* polyPath = (dtPolyRef*)malloc(sizeof(dtPolyRef)*maxCount);
	int pathCount;

	// Calculate path
	dtStatus err = m_query->findPath(startRef, endRef, startl, endl, &m_filter, polyPath, &pathCount, maxCount);
	if (dtStatusFailed(err))
		DETOUR_ERROR(NMDETOUR, "Could find a path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
			start.x, start.y, start.z, end.x, end.y, end.z);

	// Convert polygon refs to positions
	if (!path.empty()) path.clear();
	path.reserve(pathCount + 1);
	float iterPos[3] = { start.x, start.y, start.z };
	for (int i = 0; i < pathCount; i++)
	{
		dtStatus err = m_query->closestPointOnPoly(polyPath[i], iterPos, iterPos, 0);
		if (dtStatusFailed(err))
			DETOUR_ERROR(NMDETOUR, "Closest point on poly '%i' to (%.2f, %.2f, %.2f) could not be found",
				polyPath[i], iterPos[0], iterPos[1], iterPos[2]);
		path.push_back({ iterPos[0], iterPos[1], iterPos[2] });
	}
	
	// Add last position
	if (polyPath[pathCount - 1] == endRef) // reachable
	{
		path.push_back(end);
		distToEnd = 0;
	}
	else // unreachable
	{
		dtStatus err = m_query->closestPointOnPoly(polyPath[pathCount - 1], endl, iterPos, 0);
		if (dtStatusFailed(err))
			DETOUR_ERROR(NMDETOUR, "Closest point on poly '%i' to (%.2f, %.2f, %.2f) could not be found",
				polyPath[pathCount - 1], iterPos[0], iterPos[1], iterPos[2]);
		float3 lastPathPos = { iterPos[0], iterPos[1], iterPos[2] };
		path.push_back(lastPathPos);
		distToEnd = length(end - lastPathPos);
	}

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::FindNearestPointOnPoly                                   |
//  |  Finds the nearest pos on the specified *polyID* from the given position.   |
//  |  *closest* is the output.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindClosestPointOnPoly(dtPolyRef polyID, float3 pos, float3* closest, bool* posOverPoly)
{
	float posl[3] = { pos.x, pos.y, pos.z }, closestl[3];
	dtStatus err = m_query->closestPointOnPoly(polyID, posl, closestl, posOverPoly);
	if (dtStatusFailed(err))
		DETOUR_ERROR(NMDETOUR, "Closest point on poly '%i' to (%.2f, %.2f, %.2f) could not be found",
			polyID, pos.x, pos.y, pos.z);
	closest->x = closestl[0];
	closest->y = closestl[1];
	closest->z = closestl[2];
	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::FindNearestPoly                                          |
//  |  Finds the polygon closest to the specified position. *polyID* and          |
//  |  *polyPos* are the ouput. The position on the polygon is optional.    LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindNearestPoly(float3 pos, dtPolyRef* polyID, float3* polyPos) const
{
	float posl[3] = { pos.x, pos.y, pos.z };
	float polyPosl[3];
	dtStatus status = m_query->findNearestPoly(posl, m_polyFindExtention, &m_filter, polyID, polyPosl);
	if (dtStatusFailed(status))
		DETOUR_ERROR(NMDETOUR, "Couldn't find the nearest poly to (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);
	polyPos = &make_float3(polyPosl[0], polyPosl[1], polyPosl[2]);
	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::Clean                                                    |
//  |  Frees memory and restores default values.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshNavigator::Clean()
{
	m_errorCode = NMSUCCESS;
	if (m_navmesh && m_owner) dtFreeNavMesh(m_navmesh);
	m_navmesh = 0;
	dtFreeNavMeshQuery(m_query);
	if (m_query) m_navmesh = 0;
	m_filter = dtQueryFilter();
}



// Definitions required for NavMesh serialization
static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;
struct NavMeshSetHeader { int magic, version, numTiles; dtNavMeshParams params; };
struct NavMeshTileHeader { dtTileRef tileRef; int dataSize; };

//  +-----------------------------------------------------------------------------+
//  |  Serialize                                                                  |
//  |  Writes the navmesh to storage for future use.                        LH2'19|
//  +-----------------------------------------------------------------------------+
int SerializeNavMesh(const char* dir, const char* ID, dtNavMesh* navmesh)
{
	// Opening navmesh file for writing
	char filename[128];
	sprintf_s(filename, "%s%s.navmesh", dir, ID);
	const dtNavMesh* navMesh = navmesh;
	if (!navMesh) DETOUR_ERROR(NMINPUT & NMDETOUR, "Can't serialize '%s', dtNavMesh is nullpointer", ID);
	FILE* fp;
	fopen_s(&fp, filename, "wb");
	if (!fp) DETOUR_ERROR(NMIO, "Filename '%s' can't be opened", filename);

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < navMesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = navMesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, navMesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < navMesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = navMesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = navMesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}
	fclose(fp);

	DETOUR_LOG("NavMesh '%s' saved as '%s'", ID, filename);
	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  DeserializeNavMesh                                                         |
//  |  Loads a serialized NavMesh from storage and checks for errors.       LH2'19|
//  +-----------------------------------------------------------------------------+
int DeserializeNavMesh(const char* dir, const char* ID, dtNavMesh* navmesh)
{
	// Opening file
	char filename[128];
	sprintf_s(filename, "%s%s.navmesh", dir, ID);
	if (!FileExists(filename))
		DETOUR_ERROR(NMIO, "NavMesh file '%s' does not exist", filename);
	FILE* fp;
	fopen_s(&fp, filename, "rb");
	if (!fp)
		DETOUR_ERROR(NMIO, "NavMesh file '%s' could not be opened", filename);

	// Reading header
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		DETOUR_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		DETOUR_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		DETOUR_ERROR(NMIO, "NavMesh file '%s' has the wrong navmesh set version", filename);
	}

	// Initializing navmesh with header info
	navmesh = dtAllocNavMesh();
	if (!navmesh)
	{
		fclose(fp);
		DETOUR_ERROR(NMDETOUR & NMALLOCATION, "NavMesh for '%s' could not be allocated", ID);
	}
	dtStatus status = navmesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		dtFreeNavMesh(navmesh);
		DETOUR_ERROR(NMDETOUR & NMCREATION, "NavMesh for '%s' failed to initialize", ID);
	}

	// Reading tiles
	for (int i = 0; i < header.numTiles; ++i)
	{
		// Reading tile header
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			dtFreeNavMesh(navmesh);
			DETOUR_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
		}
		if (!tileHeader.tileRef || !tileHeader.dataSize) break;

		// Reading tile data
		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			dtFreeNavMesh(navmesh);
			fclose(fp);
			DETOUR_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
		}
		navmesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	fclose(fp);

	DETOUR_LOG("NavMesh '%s' loaded from '%s'", ID, filename);
	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  CreateNavMeshQuery                                                         |
//  |  Creates NavMesQuery from a navmesh and checks for errors.            LH2'19|
//  +-----------------------------------------------------------------------------+
int GetNavMeshQuery(dtNavMesh* navmesh, dtNavMeshQuery** query)
{
	*query = dtAllocNavMeshQuery();
	if (!*query)
	{
		DETOUR_ERROR(NMDETOUR & NMALLOCATION, "NavMesh Query could not be allocated");
	}
	dtStatus status = (*query)->init(navmesh, DETOUR_MAX_NAVMESH_NODES);
	if (dtStatusFailed(status))
	{
		dtFreeNavMeshQuery(*query);
		DETOUR_ERROR(NMDETOUR & NMALLOCATION, "Could not init Detour navmesh query");
	}
	return NMSUCCESS;
}

} // namespace Lighthouse2

// EOF