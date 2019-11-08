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
//  |  NavMeshNavigator::FindPathConstSize                                        |
//  |  Wrapper for the Detour findPath function. Finds a path of PathNodes.       |
//  |  *path* and *count* are the output (both preallocated).		     		  |
//  |  *reachable* specifies whether the end poly matches the last path poly.     |
//  |  *maxCount* specifies the maximum path length in nodes.               LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindPathConstSize(float3 start, float3 end, PathNode* path, int& count, bool& reachable, int maxCount)
{
	if (!path) DETOUR_ERROR(NMDETOUR & NMINPUT, "Pathfinding failed: *path* is a nullpointer");

	// Resolve positions into navmesh polygons
	dtPolyRef startRef, endRef;
	float3 firstPos, endPos; // first/last pos on poly
	m_errorCode = FindNearestPoly(start, startRef, firstPos);
	m_errorCode = FindNearestPoly(end, endRef, endPos);
	if (m_errorCode) return m_errorCode;

	// Add the start pos
	maxCount--;
	m_errorCode = FindClosestPointOnPoly(startRef, start, firstPos);
	if (m_errorCode) return m_errorCode;
	path[0] = PathNode{ firstPos, GetPoly(startRef) };

	// When start & end are on the same poly
	if (startRef == endRef)
	{
		m_errorCode = FindClosestPointOnPoly(endRef, end, endPos);
		if (m_errorCode) return m_errorCode;
		path[1] = PathNode{ endPos, GetPoly(endRef) };
		count = 2;
		reachable = true;
		return NMSUCCESS;
	}

	// Calculate path
	dtPolyRef* polyPath = (dtPolyRef*)malloc(sizeof(dtPolyRef)*maxCount);
	dtStatus err = m_query->findPath(startRef, endRef, (float*)&start, (float*)&end, &m_filter, polyPath, &count, maxCount);
	if (dtStatusFailed(err))
	{
		free(polyPath);
		DETOUR_ERROR(NMDETOUR, "Couldn't find a path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
			start.x, start.y, start.z, end.x, end.y, end.z);
	}

	// Converting to PathNodes
	float3 iterPos = firstPos;
	for (int i = 0; i < count; i++)
	{
		m_errorCode = FindClosestPointOnPoly(polyPath[i], iterPos, iterPos);
		if (m_errorCode) { free(polyPath); return m_errorCode; }
		path[i + 1] = PathNode{ iterPos, GetPoly(polyPath[i]) };
	}

	// Finding the closest valid point to the target
	bool pathComplete = (count < maxCount); // means there's room for endPos
	if (pathComplete && (endRef == polyPath[count - 1])) // complete & reachable
	{
		m_errorCode = FindClosestPointOnPoly(endRef, end, endPos);
		if (m_errorCode) { free(polyPath); return m_errorCode; }
		path[count + 1] = PathNode{ endPos, GetPoly(endRef) };
		count++;
		reachable = true;
	}
	else if (pathComplete) // path ended, poly unreachable
	{
		m_errorCode = FindClosestPointOnPoly(polyPath[count - 1], end, endPos);
		if (m_errorCode) { free(polyPath); return m_errorCode; }
		path[count + 1] = PathNode{ endPos, GetPoly(polyPath[count - 1]) };
		count++;
		reachable = false;
	}
	else // path incomplete, reachability possible
	{
		reachable = true;
	}
	free(polyPath);

	count++; // to include firstPos

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::FindPath                                                 |
//  |  Finds the shortest path from start to end.                                 |
//  |  *path* is std::vector of world positions to be filled.                     |
//  |  *distToEnd* is the error between the last position and the target.         |
//  |  *maxCount* specifies the number of allocated elements.               LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindPath(float3 start, float3 end, std::vector<PathNode>& path, bool& reachable, int maxCount)
{
	path.resize(maxCount);
	int pathCount;
	m_errorCode = FindPathConstSize(start, end, path.data(), pathCount, reachable, maxCount);
	if (m_errorCode) return m_errorCode;
	path.resize(pathCount);
	path.shrink_to_fit();
	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::FindNearestPointOnPoly                                   |
//  |  Finds the nearest pos on the specified *polyID* from the given position.   |
//  |  *closest* is the output.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindClosestPointOnPoly(dtPolyRef polyID, float3 pos, float3& closest, bool* posOverPoly)
{
	dtStatus err = m_query->closestPointOnPoly(polyID, (float*)&pos, (float*)&closest, posOverPoly);
	if (dtStatusFailed(err))
		DETOUR_ERROR(NMDETOUR, "Closest point on poly '%i' to (%.2f, %.2f, %.2f) could not be found",
			polyID, pos.x, pos.y, pos.z);
	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::FindNearestPoly                                          |
//  |  Finds the polygon closest to the specified position. *polyID* and          |
//  |  *polyPos* are the ouput. The position on the polygon is optional.    LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshNavigator::FindNearestPoly(float3 pos, dtPolyRef& polyID, float3& polyPos) const
{
	dtStatus status = m_query->findNearestPoly((float*)&pos, m_polyFindExtention, &m_filter, &polyID, (float*)&polyPos);
	if (dtStatusFailed(status))
		DETOUR_ERROR(NMDETOUR, "Couldn't find the nearest poly to (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);
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

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator::GetPoly                                                  |
//  |  Returns a polygon pointer given its detour reference.                LH2'19|
//  +-----------------------------------------------------------------------------+
const dtPoly* NavMeshNavigator::GetPoly(dtPolyRef ref) const
{
	const dtMeshTile* tile; const dtPoly* poly;
	if (!dtStatusFailed(m_navmesh->getTileAndPolyByRef(ref, &tile, &poly)))
		return poly;
	else return 0;
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
int DeserializeNavMesh(const char* dir, const char* ID, dtNavMesh*& navmesh)
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