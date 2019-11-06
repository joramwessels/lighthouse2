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

#include "system.h" // float3
#include "navmesh_common.h"

namespace lighthouse2 {

int SerializeNavMesh(const char* dir, const char* ID, dtNavMesh* navmesh);
int DeserializeNavMesh(const char* dir, const char* ID, dtNavMesh* navmesh);
int GetNavMeshQuery(dtNavMesh* navmesh, dtNavMeshQuery** query);

//  +-----------------------------------------------------------------------------+
//  |  NavMeshNavigator                                                           |
//  |  A wrapper class for the Detour pathfinding functionality.            LH2'19|
//  +-----------------------------------------------------------------------------+
class NavMeshNavigator
{
public:

	// constructor/destructor
	NavMeshNavigator(const char* dir, const char* ID) { m_errorCode = Load(dir, ID); };
	NavMeshNavigator(dtNavMesh* navmesh)
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
	int FindNearestPoly(float3 pos, dtPolyRef* polyID, float3* polyPos = 0) const;
	int FindClosestPointOnPoly(dtPolyRef polyID, float3 pos, float3* nearestPoint, bool* posOverPoly=0);
	int FindPath(float3 start, float3 end, dtPolyRef* path, int& pathCount, bool& reachable, int maxCount=64);
	int FindPath(float3 start, float3 end, float3* path, int maxCount, int* count=0, float* distToEnd=0);
	int FindPath(float3 start, float3 end, std::vector<float3>& path, float* distToEnd=0, int maxCount=64);
	void Clean();

	const dtPoly* GetPoly(dtPolyRef ref) const;

protected:
	int m_errorCode;		 // error code of the last error
	bool m_owner;			 // owner of the dtNavMesh
	dtNavMesh* m_navmesh = 0;
	dtNavMeshQuery* m_query = 0;
	dtQueryFilter m_filter;  // TODO: is this agent specific? or a navmesh variable
	float m_polyFindExtention[3] = { 5.0f, 5.0f, 5.0f };	// Half the search area for FindNearestPoly calls
};

} // namespace Lighthouse2

// EOF