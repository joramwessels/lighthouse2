/* navmesh_builder.cpp - Copyright 2019 Utrecht University
   
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

#include <vector>	// vector

#include "Recast.h"
#include "RecastDump.h"			  // duLogBuildTimes
#include "DetourNavMeshBuilder.h" // dtNavMeshCreateParams, dtCreateNavMeshData

#include "navmesh_builder.h"
#include "navmesh_navigator.h"
#include "buildcontext.h"	   // BuildContext

//#define RECAST_ERROR(X, Y, ...) return Error(RC_LOG_ERROR, X, (std::string("AI ERROR: ") + Y).c_str(), __VA_ARGS__)
#define RECAST_ERROR(X, ...) return NavMeshError(&m_errorCode, X, __VA_ARGS__)
#define RECAST_LOG(...) NavMeshError(NMSUCCESS, __VA_ARGS__)

namespace lighthouse2 {

//  +-----------------------------------------------------------------------------+
//  |  GetMinMaxBounds                                                            |
//  |  Determines the AABB bounds of the entire input mesh.                 LH2'19|
//  +-----------------------------------------------------------------------------+
	void GetMinMaxBounds(std::vector<float3>* data, float3* min, float3* max)
	{
		*min = make_float3(0.0f);
		*max = make_float3(0.0f);
		for (std::vector<float3>::iterator it = data->begin(); it != data->end(); ++it)
		{
			if (it->x < min->x) min->x = it->x;
			if (it->y < min->y) min->y = it->y;
			if (it->z < min->z) min->z = it->z;
			if (it->x > max->x) max->x = it->x;
			if (it->y > max->y) max->y = it->y;
			if (it->z > max->z) max->z = it->z;
		}
	}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::NavMeshBuilder                                             |
//  |  Constructor                                                          LH2'19|
//  +-----------------------------------------------------------------------------+
NavMeshBuilder::NavMeshBuilder(const char* dir) : m_dir(dir)
{
	m_ctx = new BuildContext();
	m_triareas = 0;
	m_heightField = 0;
	m_chf = 0;
	m_cset = 0;
	m_pmesh = 0;
	m_dmesh = 0;
	m_navMesh = 0;
	m_errorCode = 0;
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Build                                                      |
//  |  Builds a navmesh for the given scene.                                LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::Build(HostScene* scene)
{
	if (!scene || !&scene->meshes)
		RECAST_ERROR(NMRECAST & NMINPUT, "buildNavigation: Input mesh is not specified.");

	printf("generating navmesh... ");
	
	// Extracting meshes
	const std::vector<HostMesh*>* meshes = &scene->meshes;
	int meshCount = (const int)meshes->size();
	std::vector<float3> vertices;
	std::vector<int3> triangles;
	for (int i = 0; i < meshCount; i++)
		for (int j = 0; j < (*meshes)[i]->triangles.size(); j++)
		{
			vertices.push_back((*meshes)[i]->triangles[j].vertex0);
			vertices.push_back((*meshes)[i]->triangles[j].vertex1);
			vertices.push_back((*meshes)[i]->triangles[j].vertex2);
			triangles.push_back(make_int3(
				(i * meshCount + j) * 3 + 0,
				(i * meshCount + j) * 3 + 1,
				(i * meshCount + j) * 3 + 2
			));
		}

	// Initializing bounds
	if (m_config.m_bmin.x == m_config.m_bmax.x ||
		m_config.m_bmin.y == m_config.m_bmax.y ||
		m_config.m_bmin.z == m_config.m_bmax.z)
		GetMinMaxBounds(&vertices, &m_config.m_bmin, &m_config.m_bmax);
	rcCalcGridSize(
		(const float*)&m_config.m_bmin,
		(const float*)&m_config.m_bmax,
		m_config.m_cs,
		&m_config.m_width,
		&m_config.m_height
	);

	// Initializing timer
	m_ctx->resetTimers();
	m_ctx->startTimer(RC_TIMER_TOTAL);
	if (m_config.m_printBuildStats)
	{
		m_ctx->log(RC_LOG_PROGRESS, "===   NavMesh build stats for   %s", m_config.m_id);
		m_ctx->log(RC_LOG_PROGRESS, " - Voxel grid: %d x %d cells", m_config.m_width, m_config.m_height);
		m_ctx->log(RC_LOG_PROGRESS, " - Input mesh: %.1fK verts, %.1fK tris",
			vertices.size() / 1000.0f, triangles.size() / 1000.0f);
	}

	// NavMesh generation
	RasterizePolygonSoup(
		(const int)vertices.size()*3, (float*)vertices.data(),
		(const int)triangles.size(), (int*)triangles.data()
	);
	if (!m_config.m_keepInterResults) { delete[] m_triareas; m_triareas = 0; }
	FilterWalkableSurfaces();
	PartitionWalkableSurface();
	if (!m_config.m_keepInterResults) { rcFreeHeightField(m_heightField); m_heightField = 0; }
	ExtractContours();
	BuildPolygonMesh();
	CreateDetailMesh();
	if (!m_config.m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}
	CreateDetourData();

	// Log performance stats
	m_ctx->stopTimer(RC_TIMER_TOTAL);
	if (!m_errorCode) // short single-line duration log
		printf("%.3fms", m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f);
	if (m_config.m_printBuildStats && !m_errorCode) // calculating detailed multi-line duration log
		duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	if (!m_errorCode) // single-line mesh info
		m_ctx->log(RC_LOG_PROGRESS, "   '%s' polymesh: %d vertices  %d polygons",
		m_config.m_id, m_pmesh->nverts, m_pmesh->npolys);
	if (m_errorCode) Cleanup();

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::RasterizePolygonSoup                                       |
//  |  Takes a triangle soup and rasterizes all walkable triangles based          |
//  |  on their slope. Results in a height map (aka voxel mold).            LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::RasterizePolygonSoup(const int vert_count, const float* verts, const int tri_count, const int* tris)
{
	if (m_errorCode) return NMINPUT;

	// Allocate voxel heightfield where we rasterize our input data to.
	m_heightField = rcAllocHeightfield();
	if (!m_heightField)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'solid'.");

	if (!rcCreateHeightfield(m_ctx, *m_heightField, m_config.m_width, m_config.m_height,
		(const float*)&m_config.m_bmin, (const float*)&m_config.m_bmax, m_config.m_cs, m_config.m_ch))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not create solid heightfield.");

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas = new unsigned char[tri_count];
	if (!m_triareas)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'm_triareas' (%d).", tri_count);

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	memset(m_triareas, 0, tri_count * sizeof(unsigned char));
	rcMarkWalkableTriangles(m_ctx, m_config.m_walkableSlopeAngle, verts, vert_count, tris, tri_count, m_triareas);
	if (!rcRasterizeTriangles(m_ctx, verts, vert_count, tris,
		m_triareas, tri_count, *m_heightField, m_config.m_walkableClimb))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not rasterize triangles.");

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::FilterWalkableSurfaces                                     |
//  |  Filters the correctly angled surfaces for height restrictions.       LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::FilterWalkableSurfaces()
{
	if (m_errorCode) return NMINPUT;

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_config.m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_config.m_walkableClimb, *m_heightField);
	if (m_config.m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_config.m_walkableHeight, m_config.m_walkableClimb, *m_heightField);
	if (m_config.m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_config.m_walkableHeight, *m_heightField);

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::PartitionWalkableSurface                                   |
//  |  Transforms the heightfield into a compact height field, connects           |
//  |  neightboring walkable surfaces, erodes all surfaces by the agent           |
//  |  radius and partitions them into regions.                             LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::PartitionWalkableSurface()
{
	if (m_errorCode) return NMINPUT;

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'chf'.");

	if (!rcBuildCompactHeightfield(m_ctx, m_config.m_walkableHeight, m_config.m_walkableClimb, *m_heightField, *m_chf))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not build compact data.");

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_config.m_walkableRadius, *m_chf))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not erode.");

	//// (Optional) Mark areas.
	//const ConvexVolume* vols = m_geom->getConvexVolumes();
	//for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
	//	rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	if (m_config.m_partitionType == NavMeshConfig::SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_ctx, *m_chf))
			RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not build distance field.");

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_ctx, *m_chf, 0, m_config.m_minRegionArea, m_config.m_mergeRegionArea))
			RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not build watershed regions.");
	}
	else if (m_config.m_partitionType == NavMeshConfig::SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_config.m_minRegionArea, m_config.m_mergeRegionArea))
			RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not build monotone regions.");
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_config.m_minRegionArea))
			RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not build layer regions.");
	}

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::ExtractContours                                            |
//  |  Extracts contours from the compact height field.                     LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::ExtractContours()
{
	if (m_errorCode) return NMINPUT;
	m_cset = rcAllocContourSet();
	if (!m_cset)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'cset'.");

	if (!rcBuildContours(m_ctx, *m_chf, m_config.m_maxSimplificationError, m_config.m_maxEdgeLen, *m_cset))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not create contours.");

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::BuildPolygonMesh                                           |
//  |  Transforms the contours into a polygon mesh.                         LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::BuildPolygonMesh()
{
	if (m_errorCode) return NMINPUT;
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'pmesh'.");

	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_config.m_maxVertsPerPoly, *m_pmesh))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not triangulate contours.");

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::CreateDetailMesh                                           |
//  |  Creates the detailed polygon mesh.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::CreateDetailMesh()
{
	if (m_errorCode) return NMINPUT;
	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'pmdtl'.");

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_config.m_detailSampleDist, m_config.m_detailSampleMaxError, *m_dmesh))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not build detail mesh.");

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::CreateDetourData                                           |
//  |  Creates Detour navmesh from the two poly meshes.                     LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::CreateDetourData()
{
	if (m_errorCode) return NMINPUT;

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if (m_config.m_maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}


		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		params.offMeshConVerts = 0;//m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = 0;//m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = 0;//m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = 0;//m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = 0;//m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = 0;//m_geom->getOffMeshConnectionId();
		params.offMeshConCount = 0;//m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_config.m_walkableHeight;
		params.walkableRadius = m_config.m_walkableRadius;
		params.walkableClimb = m_config.m_walkableClimb;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_config.m_cs;
		params.ch = m_config.m_ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
			RECAST_ERROR(NMDETOUR & NMCREATION, "Could not build Detour navmesh.");

		m_navMesh = dtAllocNavMesh();
		if (!m_navMesh)
		{
			dtFree(navData);
			RECAST_ERROR(NMDETOUR & NMALLOCATION, "Could not allocate Detour navmesh");
		}

		dtStatus status;

		status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			RECAST_ERROR(NMDETOUR & NMCREATION, "Could not init Detour navmesh");
		}

		//m_navQuery = dtAllocNavMeshQuery();
		//status = m_navQuery->init(m_navMesh, DETOUR_MAX_NAVMESH_NODES);
		//if (dtStatusFailed(status))
		//	RECAST_ERROR(NMDETOUR & NMCREATION, "Could not init Detour navmesh query");
	}

	return NMSUCCESS;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Serialize                                                  |
//  |  Writes the navmesh to storage for future use.                        LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::Serialize(const char* dir, const char* ID)
{
	if (m_errorCode) return NMINPUT;

	// Saving config file
	char configfile[128];
	sprintf_s(configfile, "%s%s.config", dir, ID);
	m_config.Save(configfile);

	// Saving dtNavMesh
	m_errorCode = SerializeNavMesh(dir, ID, m_navMesh);
	if (m_errorCode) return m_errorCode;

	return RECAST_LOG("NavMesh build '%s' successfully saved", ID);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Deserialize                                                |
//  |  Reads a previously stored navmesh from storage.                      LH2'19|
//  +-----------------------------------------------------------------------------+
int NavMeshBuilder::Deserialize(const char* dir, const char* ID)
{
	if (m_errorCode) return NMINPUT;
	Cleanup();

	// Loading config file
	char configfile[128];
	sprintf_s(configfile, "%s%s.config", dir, ID);
	m_config.Load(configfile);
	m_config.m_id = ID; // strings aren't loaded correctly

	// Loading dtNavMesh and dtNavMeshQuery
	m_errorCode = DeserializeNavMesh(dir, ID, m_navMesh);
	if (m_errorCode) return m_errorCode;
	//m_errorCode = GetNavMeshQuery(m_navMesh, m_navQuery);
	//if (m_errorCode) return m_errorCode;

	return RECAST_LOG("NavMesh build '%s' successfully loaded", ID);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Cleanup                                                    |
//  |  Ensures all navmesh memory allocations are deleted.                  LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::Cleanup()
{
	if (m_triareas) delete[] m_triareas;
	m_triareas = 0;
	if (m_heightField) rcFreeHeightField(m_heightField);
	m_heightField = 0;
	if (m_chf) rcFreeCompactHeightfield(m_chf);
	m_chf = 0;
	if (m_cset) rcFreeContourSet(m_cset);
	m_cset = 0;
	if (m_pmesh) rcFreePolyMesh(m_pmesh);
	m_pmesh = 0;
	if (m_dmesh) rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = 0;
	if (m_navMesh) dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::DumpLog                                                    |
//  |  Dumps all logged information to stdout.                              LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::DumpLog() const
{
	((BuildContext*)m_ctx)->dumpLog("\n");
};

} // namespace lighthouse2

// EOF