/* navmesh_navigator.h - Copyright 2019 Utrecht University

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

#pragma once

#include <vector>

//#ifdef PATHFINDINGBUILD
	#include "DetourNavMeshQuery.h" // dtNavMesh, dtNavMeshQuery, dtQueryFilter
//#endif

#include "system.h"	// float3
#include "navmesh_common.h"		// NavMeshError, DETOUR_MAX_NAVMESH_NODES, NMINPUT

namespace lighthouse2 {

int SerializeNavMesh(const char* dir, const char* ID, const dtNavMesh* navmesh);
int DeserializeNavMesh(const char* dir, const char* ID, dtNavMesh*& navmesh);
int GetNavMeshQuery(dtNavMesh* navmesh, dtNavMeshQuery** query);

static const dtQueryFilter s_filter; // default empty filter

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator                                                           |
//  |  A wrapper class for the Detour pathfinding functionality.            LH2'19|
//  +-----------------------------------------------------------------------------+
class NavMeshNavigator
{
public:

	// constructor/destructor
	NavMeshNavigator(const char* dir, const char* ID) : m_ID(ID) { m_errorCode = Load(dir, ID); };
	NavMeshNavigator(dtNavMesh* navmesh, const char* ID="NO_ID") : m_ID(ID)
	{
		if (!navmesh) m_errorCode = NavMeshError(NMINPUT,
			"NavMeshNavigator was initialized with a nullpointer navmesh");
		else
		{
			m_navmesh = navmesh;
			m_errorCode = GetNavMeshQuery(m_navmesh, &m_query);
			m_owner = false;
		}
	}
	~NavMeshNavigator() { Clean(); };

	int Load(const char* dir, const char* ID)
	{
		m_errorCode = DeserializeNavMesh(dir, ID, m_navmesh);
		if (m_errorCode) return m_errorCode;
		m_errorCode = GetNavMeshQuery(m_navmesh, &m_query);
		m_owner = true;
		return m_errorCode;
	}

	struct PathNode { float3 pos; const dtPoly* poly; }; // poly is nullptr if not on a poly

	int FindNearestPoly(float3 pos, dtPolyRef& polyID, float3& polyPos) const;
	int FindClosestPointOnPoly(dtPolyRef polyID, float3 pos, float3& nearestPoint, bool* posOverPoly=0);
	int FindPathConstSize(float3 start, float3 end, PathNode* path, int& count, bool& reachable, int maxCount=64, const dtQueryFilter* filter=&s_filter);
	int FindPathConstSize_Legacy(float3 start, float3 end, PathNode* path, int& count, bool& reachable, int maxCount = 64, const dtQueryFilter* filter = &s_filter);
	int FindPath(float3 start, float3 end, std::vector<PathNode>& path, bool& reachable, int maxCount=64);
	void Clean();

	inline const dtNavMesh* GetDetourMesh() const { return m_navmesh; };
	inline const dtPoly* GetPoly(dtPolyRef ref) const;
	inline const char* GetID() const { return m_ID.c_str(); };

protected:
	std::string m_ID;
	int m_errorCode;		 // error code of the last error
	bool m_owner;			 // owner of the dtNavMesh
	dtNavMesh* m_navmesh = 0;
	dtNavMeshQuery* m_query = 0;
	float m_polyFindExtention[3] = { 5.0f, 5.0f, 5.0f };	// Half the search area for FindNearestPoly calls
};

} // namespace Lighthouse2

// EOF