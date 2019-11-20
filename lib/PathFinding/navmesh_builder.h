/* navmesh_builder.h - Copyright 2019 Utrecht University
   
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

//#ifdef PATHFINDINGBUILD
	#include "Recast.h" // rcContext, rcHeightfield, rcPolyMesh, etc.
//#endif

#include "rendersystem.h"	   // HostScene, HostMesh, HostTri, float3, int3, FileExists
#include "navmesh_common.h"    // NavMeshStatus, NavMeshConfig
#include "navmesh_navigator.h" // NavMeshNavigator, SerializeNavMesh, DeserializeNavMesh

namespace lighthouse2 {

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder                                                             |
//  |  A contex in which navmeshes can be build, edited, and saved.         LH2'19|
//  +-----------------------------------------------------------------------------+
class NavMeshBuilder
{
public:

	// constructor / destructor
	NavMeshBuilder(const char* dir);
	~NavMeshBuilder() { Cleanup(); };

	NavMeshStatus Build(HostScene* scene);
	NavMeshStatus Serialize() { return Serialize(m_dir, m_config.m_id.c_str()); };
	NavMeshStatus Deserialize() { return Deserialize(m_dir, m_config.m_id.c_str()); };
	void Cleanup();

	void AddOffMeshConnection(float3 v0, float3 v1, float radius, bool unidirectional);
	void ApplyChanges() { if (m_pmesh && m_dmesh) CreateDetourData(); };

	void SetConfig(NavMeshConfig config) { m_config = config; };
	void SetID(const char* id) { m_config.m_id = id; };

	bool IsClean() const { if (m_navMesh) return false; else return true; };
	bool HasIntermediateResults() const { return (m_pmesh && m_dmesh); };
	const char* GetDir() const { return m_dir; };
	NavMeshConfig* GetConfig() { return &m_config; };
	dtNavMesh* GetMesh() const { return m_navMesh; };
	NavMeshStatus GetStatus() { return m_status; };
	NavMeshNavigator* GetNavigator() const
	{
		NavMeshNavigator* nmn = new NavMeshNavigator(*m_navMesh, m_config.m_id.c_str());
		nmn->SetFlagAndAreaMappings(m_config.m_flags, m_config.m_areas);
		return nmn;
	};

protected:

	// Input
	const char* m_dir;
	rcContext* m_ctx;				// Recast context for logging
	NavMeshConfig m_config;			// NavMesh generation configurations

	// Generated in Build()
	unsigned char* m_triareas;
	rcHeightfield* m_heightField;	// The first voxel mold
	rcCompactHeightfield* m_chf;	// The compact height field
	rcContourSet* m_cset;			// The area contours
	rcPolyMesh* m_pmesh;			// The polygon mesh
	rcPolyMeshDetail* m_dmesh;		// The detailed polygon mesh
	dtNavMesh* m_navMesh;			// The final navmesh as used by Detour
	NavMeshStatus m_status;

	// Runtime additions
	std::vector<float3> m_offMeshVerts; // (v0, v1) * nConnections
	std::vector<float> m_offMeshRadii;
	std::vector<unsigned short> m_offMeshFlags;
	std::vector<unsigned char> m_offMeshAreas;
	std::vector<unsigned int> m_offMeshUserIDs;
	std::vector<unsigned char> m_offMeshDirection;

	// Build functions
	int RasterizePolygonSoup(const int vert_count, const float* verts, const int tri_count, const int* tris);
	int FilterWalkableSurfaces();
	int PartitionWalkableSurface();
	int ExtractContours();
	int BuildPolygonMesh();
	int CreateDetailMesh();
	int CreateDetourData();

	int Serialize(const char* dir, const char* ID);
	int Deserialize(const char* dir, const char* ID);
};

// Area Types // TODO: remove
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED = 0x10,	// Disabled polygon
	SAMPLE_POLYFLAGS_ALL = 0xffff		// All abilities.
};

} // namespace lighthouse2

// EOF