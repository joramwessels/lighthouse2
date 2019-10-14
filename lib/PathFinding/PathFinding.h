/* pathfinding.h - Copyright 2019 Utrecht University
   
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

#include <string.h> // memset
#include <vector>	// vector

#include "tinyxml2.h" // configuration saving and -loading

#include "Recast.h"				  // navmesh generation
#include "RecastDump.h"			  // duLogBuildTimes
#include "DetourNavMesh.h"		  // Detour navmesh
#include "DetourNavMeshBuilder.h" // Detour navmesh generation
#include "DetourNavMeshQuery.h"	  // Detour query system

#include "rendersystem.h" // HostScene, HostMesh, HostTri, float3, int3

namespace lighthouse2 {

//  +-------------------------------------------------------------------------------------------------------+
//  |  SamplePartitionType                                                                                  |
//  |  The heightfield is partitioned so that a simple algorithm can triangulate the walkable areas.	    |
//  |  There are 3 martitioning methods, each with some pros and cons:										|
//  |  1) Watershed partitioning																			|
//  |    - the classic Recast partitioning																	|
//  |    - creates the nicest tessellation																	|
//  |    - usually slowest																					|
//  |    - partitions the heightfield into nice regions without holes or overlaps							|
//  |    - the are some corner cases where this method creates produces holes and overlaps					|
//  |       - holes may appear when a small obstacle is close to large open area (triangulation won't fail) |
//  |       - overlaps may occur on narrow spiral corridors (i.e stairs) and triangulation may fail         |
//  |    * generally the best choice if you precompute the nacmesh, use this if you have large open areas	|
//  |  2) Monotone partioning																				|
//  |    - fastest																							|
//  |    - partitions the heightfield into regions without holes and overlaps (guaranteed)					|
//  |    - creates long thin polygons, which sometimes causes paths with detours							|
//  |    * use this if you want fast navmesh generation														|
//  |  3) Layer partitoining																				|
//  |    - quite fast																						|
//  |    - partitions the heighfield into non-overlapping regions											|
//  |    - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)		|
//  |    - produces better triangles than monotone partitioning												|
//  |    - does not have the corner cases of watershed partitioning											|
//  |    - can be slow and create a bit ugly tessellation (still better than monotone)						|
//  |      if you have large open areas with small obstacles (not a problem if you use tiles)				|
//  |    * good choice to use for tiled navmesh with medium and small sized tiles					  LH2'19|
//  +-------------------------------------------------------------------------------------------------------+
enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS,
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig                                                              |
//  |  Contains all settings regarding the navmesh generation.              LH2'19|
//  +-----------------------------------------------------------------------------+
struct NavMeshConfig
{
	int m_width, m_height, m_tileSize, m_borderSize;		 // Automatically computed
	float m_cs, m_ch;										 // Voxel cell size and -height
	float3 m_bmin, m_bmax;									 // AABB navmesh restraints
	float m_walkableSlopeAngle;								 // In degrees
	int m_walkableHeight, m_walkableClimb, m_walkableRadius; // In voxels
	int m_maxEdgeLen;
	float m_maxSimplificationError;
	int m_minRegionArea, m_mergeRegionArea, m_maxVertsPerPoly; // maxVertsPerPoly should not exceed 6
	float m_detailSampleDist, m_detailSampleMaxError;

	SamplePartitionType m_partitionType; // The partitioning method
	bool m_keepInterResults;			 // Whether to keep intermediate results
	bool m_filterLowHangingObstacles;	 // Whether to filter for low obstacles
	bool m_filterLedgeSpans;			 // Whether to filter for ledges
	bool m_filterWalkableLowHeightSpans; // Whether to filter for low ceilings
	bool m_printBuildStats;				 // Whether to print detailed build statistics
	const char* m_id;					 // Name of the navmesh instance

	NavMeshConfig();

	void SetCellSize(float width, float height) { m_cs = width; m_ch = height; };
	void SetAABB(float3 min, float3 max) { m_bmin = min; m_bmax = max; }; // if AABB is not 3D, input mesh is used
	void SetAgentInfo(float maxWalkableAngle, int minWalkableHeight,
		int maxClimbableHeight, int minWalkableRadius);
	void SetPolySettings(int maxEdgeLen, float maxSimplificationError,
		int minRegionArea, int minMergedRegionArea, int maxVertPerPoly);
	void SetDetailPolySettings(float sampleDist, float maxSimplificationError);
	void NavMeshConfig::SetPartitionType(SamplePartitionType type) { m_partitionType = type; };
	void SetKeepInterResults(bool keep) { m_keepInterResults = keep; };
	void SetSurfaceFilterSettings(bool lowHangingObstacles,
		bool ledgeSpans, bool WalkableLowHeightSpans);
	void SetPrintBuildStats(bool print) { m_printBuildStats = print; };
	void SetID(const char* ID) { m_id = ID; };

	void ScaleSettings(float scale);
	void Save(const char* filename) const;
	void Load(const char* filename);
};

typedef float TimeVal;
class BuildContext : public rcContext
{
	TimeVal m_startTime[RC_MAX_TIMERS];
	TimeVal m_accTime[RC_MAX_TIMERS];

	static const int MAX_MESSAGES = 1000;
	const char* m_messages[MAX_MESSAGES];
	int m_messageCount;
	static const int TEXT_POOL_SIZE = 8000;
	char m_textPool[TEXT_POOL_SIZE];
	int m_textPoolSize;
	Timer timer;

public:
	BuildContext() : m_messageCount(0), m_textPoolSize(0)
	{
		memset(m_messages, 0, sizeof(char*) * MAX_MESSAGES);
		resetTimers();
	};

	/// Dumps the log to stdout.
	void dumpLog(const char* format, ...);
	/// Returns number of log messages.
	int getLogCount() const { return m_messageCount; };
	/// Returns log message text.
	const char* getLogText(const int i) const { return m_messages[i] + 1; };

protected:
	/// Virtual functions for custom implementations.
	///@{
	virtual void doResetLog() { m_messageCount = 0; m_textPoolSize = 0; };
	virtual void doLog(const rcLogCategory category, const char* msg, const int len);
	virtual void doResetTimers() { for (int i = 0; i < RC_MAX_TIMERS; ++i) m_accTime[i] = -1; };
	virtual void doStartTimer(const rcTimerLabel label) { m_startTime[label] = timer.elapsed(); };
	virtual void doStopTimer(const rcTimerLabel label);
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const { return m_accTime[label] * 1000000.0f; };
	///@}
};

//  +-----------------------------------------------------------------------------+
//  |  NavMesh                                                                    |
//  |  NavMesh class definition.                                            LH2'19|
//  +-----------------------------------------------------------------------------+
class NavMeshBuilder
{
public:
	// constructor / destructor
	NavMeshBuilder(const char* dir) : m_dir(dir)
	{
		m_ctx = new BuildContext();
	};
	~NavMeshBuilder() { Cleanup(); };

	void Build(HostScene* scene);
	void Serialize() const { Serialize(m_dir, m_config.m_id); };
	void Deserialize() { Deserialize(m_dir, m_config.m_id); };
	void SaveAsMesh() const { SaveAsMesh(m_dir, m_config.m_id); };
	void Cleanup();
	void DumpLog() const { ((BuildContext*)m_ctx)->dumpLog(""); };

	void SetConfig(NavMeshConfig config) { m_config = config; };
	void SetID(const char* id) { m_config.m_id = id; };

	NavMeshConfig* GetConfig() { return &m_config; };
	dtNavMesh* GetMesh() { return m_navMesh; };
	dtNavMeshQuery* GetQuery() { return m_navQuery; };

protected:

	// Input
	const char* m_dir;
	rcContext* m_ctx;				// Recast context for logging
	NavMeshConfig m_config;			// NavMesh generation configurations
	const char* m_matFile = "navmesh";

	// Generated in Build()
	unsigned char* m_triareas;
	rcHeightfield* m_heightField;	// The first voxel mold
	rcCompactHeightfield* m_chf;	// The compact height field
	rcContourSet* m_cset;			// The area contours
	rcPolyMesh* m_pmesh;			// The polygon mesh
	rcPolyMeshDetail* m_dmesh;		// The detailed polygon mesh
	dtNavMesh* m_navMesh;			// The final navmesh as used by Detour
	dtNavMeshQuery* m_navQuery;		// The navmesh query system

	// Build functions
	void RasterizePolygonSoup(const int vert_count, const float* verts, const int tri_count, const int* tris);
	void FilterWalkableSurfaces();
	void PartitionWalkableSurface();
	void ExtractContours();
	void BuildPolygonMesh();
	void CreateDetailMesh();
	void CreateDetourData();

	void WriteMaterialFile(const char* dir) const;
	void WriteTileToMesh(const dtMeshTile* tile, FILE* file) const;
	void SaveAsMesh(const char* dir, const char* ID) const;
	void Serialize(const char* dir, const char* ID) const;
	void Deserialize(const char* dir, const char* ID);
};

// Area Types // TODO make accessible to app
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
	SAMPLE_POLYFLAGS_DISABLED = 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL = 0xffff	// All abilities.
};

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

} // namespace lighthouse2

// EOF