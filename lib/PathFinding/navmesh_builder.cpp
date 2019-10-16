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

#include "navmesh_builder.h"

#define RECAST_ERROR(X, Y, ...) return Error(RC_LOG_ERROR, X, (std::string("AI ERROR: ") + Y).c_str(), __VA_ARGS__)
#define RECAST_LOG(...) Error(RC_LOG_PROGRESS, NMSUCCESS, __VA_ARGS__)

#define DETOUR_MAX_NAVMESH_NODES 2048

namespace lighthouse2 {

//  +-----------------------------------------------------------------------------+
//  |  GetMinMaxBounds                                                   |
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
//  |  NavMeshConfig::NavMeshConfig                                               |
//  |  Initializes using default configurations.                            LH2'19|
//  +-----------------------------------------------------------------------------+
NavMeshConfig::NavMeshConfig()
{
	m_width = 0;
	m_height = 0;
	m_tileSize = 0;
	m_borderSize = 0;
	m_cs = 1.0f;
	m_ch = 1.0f;
	m_bmin = make_float3(0.0f);
	m_bmax = make_float3(0.0f);

	m_walkableSlopeAngle = 40.0f;
	m_walkableHeight = 10;
	m_walkableClimb = 2;
	m_walkableRadius = 3;

	m_maxEdgeLen = 20;
	m_maxSimplificationError = 2.5f;
	m_minRegionArea = 12;
	m_mergeRegionArea = 25;
	m_maxVertsPerPoly = 6;
	m_detailSampleDist = 10.0f;
	m_detailSampleMaxError = 2.0f;

	m_partitionType = SAMPLE_PARTITION_WATERSHED;
	m_keepInterResults = false;
	m_filterLowHangingObstacles = true;
	m_filterLedgeSpans = true;
	m_filterWalkableLowHeightSpans = true;
	m_id = "default_ID";
	m_printBuildStats = false;
	m_printImmediately = false;
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig::SetAgentInfo                                                |
//  |  Sets all configurations regarding the agent.                         LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshConfig::SetAgentInfo(float angle, int height, int climb, int radius)
{
	m_walkableSlopeAngle = angle;
	m_walkableHeight = height;
	m_walkableClimb = climb;
	m_walkableRadius = radius;
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig::SetPolySettings                                             |
//  |  Sets all configurations regarding the polygon mesh.                  LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshConfig::SetPolySettings(int maxEdgeLen, float maxSimplificationError,
	int minRegionArea, int minMergedRegionArea, int maxVertPerPoly)
{
	m_maxEdgeLen = maxEdgeLen;
	m_maxSimplificationError = maxSimplificationError;
	m_minRegionArea = minRegionArea;
	m_mergeRegionArea = minMergedRegionArea;
	m_maxVertsPerPoly = maxVertPerPoly;
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig::SetDetailPolySettings                                       |
//  |  Sets all configurations regarding the detail polygon mesh.           LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshConfig::SetDetailPolySettings(float sampleDist, float maxSimplificationError)
{
	m_detailSampleDist = sampleDist;
	m_maxSimplificationError = maxSimplificationError;
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig::SetSurfaceFilterSettings                                    |
//  |  Sets what to filter the walkable surfaces for.                       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshConfig::SetSurfaceFilterSettings(bool lowHangingObstacles,
	bool ledgeSpans, bool WalkableLowHeightSpans)
{
	m_filterLowHangingObstacles = lowHangingObstacles;
	m_filterLedgeSpans = ledgeSpans;
	m_filterWalkableLowHeightSpans = WalkableLowHeightSpans;
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig::ScaleSettings                                               |
//  |  Easily scales all relevant settings when the scene itself is scaled. LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshConfig::ScaleSettings(float scale)
{
	m_cs /= scale; m_ch /= scale;
	m_walkableHeight /= scale; m_walkableClimb /= scale; m_walkableRadius /= scale;
	m_maxEdgeLen /= scale; m_maxSimplificationError /= scale;
	m_minRegionArea /= (scale*scale); m_mergeRegionArea /= (scale*scale);
	m_detailSampleDist /= scale; m_maxSimplificationError /= scale;
};

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig::Save                                                        |
//  |  Writes the configurations to storage as an XML.                      LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshConfig::Save(const char* filename) const
{
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLNode* root = doc.NewElement("configurations");
	tinyxml2::XMLNode* bmin = doc.NewElement("bmin");
	tinyxml2::XMLNode* bmax = doc.NewElement("bmax");
	doc.InsertFirstChild(root);

	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("width")))->SetText(m_width);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("height")))->SetText(m_height);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("tileSize")))->SetText(m_tileSize);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("borderSize")))->SetText(m_borderSize);

	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("cs")))->SetText(m_cs);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("ch")))->SetText(m_ch);
	root->InsertEndChild(bmin);
	((tinyxml2::XMLElement*)bmin->InsertEndChild(doc.NewElement("x")))->SetText(m_bmin.x);
	((tinyxml2::XMLElement*)bmin->InsertEndChild(doc.NewElement("y")))->SetText(m_bmin.y);
	((tinyxml2::XMLElement*)bmin->InsertEndChild(doc.NewElement("z")))->SetText(m_bmin.z);
	root->InsertEndChild(bmax);
	((tinyxml2::XMLElement*)bmax->InsertEndChild(doc.NewElement("x")))->SetText(m_bmax.x);
	((tinyxml2::XMLElement*)bmax->InsertEndChild(doc.NewElement("y")))->SetText(m_bmax.y);
	((tinyxml2::XMLElement*)bmax->InsertEndChild(doc.NewElement("z")))->SetText(m_bmax.z);

	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("walkableSlopeAngle")))->SetText(m_walkableSlopeAngle);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("walkableClimb")))->SetText(m_walkableClimb);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("walkableHeight")))->SetText(m_walkableHeight);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("walkableRadius")))->SetText(m_walkableRadius);

	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("maxEdgeLen")))->SetText(m_maxEdgeLen);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("maxSimplificationError")))->SetText(m_maxSimplificationError);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("minRegionArea")))->SetText(m_minRegionArea);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("mergeRegionArea")))->SetText(m_mergeRegionArea);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("maxVertsPerPoly")))->SetText(m_maxVertsPerPoly);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("detailSampleDist")))->SetText(m_detailSampleDist);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("detailSampleMaxError")))->SetText(m_detailSampleMaxError);

	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("partitionType")))->SetText(m_partitionType);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("keepInterResults")))->SetText(m_keepInterResults);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("filterLowHangingObstacles")))->SetText(m_filterLowHangingObstacles);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("filterLedgeSpans")))->SetText(m_filterLedgeSpans);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("filterWalkableLowHeightSpans")))->SetText(m_filterWalkableLowHeightSpans);

	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("printBuildStats")))->SetText(m_printBuildStats);
	((tinyxml2::XMLElement*)root->InsertEndChild(doc.NewElement("ID")))->SetText(m_id);

	doc.SaveFile(filename);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshConfig::Load                                                        |
//  |  Loads an XML configurations file from storage.                       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshConfig::Load(const char* filename)
{
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError result = doc.LoadFile(filename);
	if (result != tinyxml2::XML_SUCCESS) return;
	tinyxml2::XMLNode* root = doc.FirstChildElement("configurations");
	if (root == nullptr) return;

	if (root->FirstChildElement( "width" )) root->FirstChildElement("width")->QueryIntText(&m_width);
	if (root->FirstChildElement( "height" )) root->FirstChildElement("height")->QueryIntText(&m_height);
	if (root->FirstChildElement( "tileSize" )) root->FirstChildElement("tileSize")->QueryIntText(&m_tileSize);
	if (root->FirstChildElement( "borderSize" )) root->FirstChildElement("borderSize")->QueryIntText(&m_borderSize);

	if (root->FirstChildElement( "cs" )) root->FirstChildElement("cs")->QueryFloatText(&m_cs);
	if (root->FirstChildElement( "ch" )) root->FirstChildElement("ch")->QueryFloatText(&m_ch);
	if (root->FirstChildElement( "bmin" )) root->FirstChildElement("bmin")->FirstChildElement("x")->QueryFloatText(&m_bmin.x);
	if (root->FirstChildElement( "bmin" )) root->FirstChildElement("bmin")->FirstChildElement("y")->QueryFloatText(&m_bmin.y);
	if (root->FirstChildElement( "bmin" )) root->FirstChildElement("bmin")->FirstChildElement("z")->QueryFloatText(&m_bmin.z);
	if (root->FirstChildElement( "bmax" )) root->FirstChildElement("bmax")->FirstChildElement("x")->QueryFloatText(&m_bmax.x);
	if (root->FirstChildElement( "bmax" )) root->FirstChildElement("bmax")->FirstChildElement("y")->QueryFloatText(&m_bmax.y);
	if (root->FirstChildElement( "bmax" )) root->FirstChildElement("bmax")->FirstChildElement("z")->QueryFloatText(&m_bmax.z);

	if (root->FirstChildElement( "walkableSlopeAngle" )) root->FirstChildElement("walkableSlopeAngle")->QueryFloatText(&m_walkableSlopeAngle);
	if (root->FirstChildElement( "walkableHeight" )) root->FirstChildElement("walkableHeight")->QueryIntText(&m_walkableHeight);
	if (root->FirstChildElement( "walkableClimb" )) root->FirstChildElement("walkableClimb")->QueryIntText(&m_walkableClimb);
	if (root->FirstChildElement( "walkableRadius" )) root->FirstChildElement("walkableRadius")->QueryIntText(&m_walkableRadius);

	if (root->FirstChildElement( "maxEdgeLen" )) root->FirstChildElement("maxEdgeLen")->QueryIntText(&m_maxEdgeLen);
	if (root->FirstChildElement( "maxSimplificationError" )) root->FirstChildElement("maxSimplificationError")->QueryFloatText(&m_maxSimplificationError);
	if (root->FirstChildElement( "minRegionArea" )) root->FirstChildElement("minRegionArea")->QueryIntText(&m_minRegionArea);
	if (root->FirstChildElement( "mergeRegionArea" )) root->FirstChildElement("mergeRegionArea")->QueryIntText(&m_mergeRegionArea);
	if (root->FirstChildElement( "maxVertsPerPoly" )) root->FirstChildElement("maxVertsPerPoly")->QueryIntText(&m_maxVertsPerPoly);
	if (root->FirstChildElement( "detailSampleDist" )) root->FirstChildElement("detailSampleDist")->QueryFloatText(&m_detailSampleDist);
	if (root->FirstChildElement( "detailSampleMaxError" )) root->FirstChildElement("detailSampleMaxError")->QueryFloatText(&m_detailSampleMaxError);

	if (root->FirstChildElement( "partitionType" )) root->FirstChildElement("partitionType")->QueryIntText((int*)&m_partitionType);
	if (root->FirstChildElement( "keepInterResults" )) root->FirstChildElement("keepInterResults")->QueryBoolText(&m_keepInterResults);
	if (root->FirstChildElement( "filterLowHangingObstacles" )) root->FirstChildElement("filterLowHangingObstacles")->QueryBoolText(&m_filterLowHangingObstacles);
	if (root->FirstChildElement( "filterLedgeSpans" )) root->FirstChildElement("filterLedgeSpans")->QueryBoolText(&m_filterLedgeSpans);
	if (root->FirstChildElement( "filterWalkableLowHeightSpans" )) root->FirstChildElement("filterWalkableLowHeightSpans")->QueryBoolText(&m_filterWalkableLowHeightSpans);

	if (root->FirstChildElement( "printBuildStats" )) root->FirstChildElement("printBuildStats")->QueryBoolText(&m_printBuildStats);
	if (root->FirstChildElement( "ID" )) m_id = root->FirstChildElement("ID")->FirstChild()->Value();
}





//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Build                                                      |
//  |  Builds a navmesh for the given scene.                                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::Build(HostScene* scene)
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
		RECAST_LOG("===   NavMesh build stats for   %s", m_config.m_id);
		RECAST_LOG(" - Voxel grid: %d x %d cells", m_config.m_width, m_config.m_height);
		RECAST_LOG(" - Input mesh: %.1fK verts, %.1fK tris",
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
		RECAST_LOG("   '%s' polymesh: %d vertices  %d polygons",
		m_config.m_id, m_pmesh->nverts, m_pmesh->npolys);
	if (m_errorCode) Cleanup();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::RasterizePolygonSoup                                       |
//  |  Takes a triangle soup and rasterizes all walkable triangles based          |
//  |  on their slope. Results in a height map (aka voxel mold).            LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::RasterizePolygonSoup(const int vert_count, const float* verts, const int tri_count, const int* tris)
{
	if (m_errorCode) return;

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
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::FilterWalkableSurfaces                                     |
//  |  Filters the correctly angled surfaces for height restrictions.       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::FilterWalkableSurfaces()
{
	if (m_errorCode) return;

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_config.m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_config.m_walkableClimb, *m_heightField);
	if (m_config.m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_config.m_walkableHeight, m_config.m_walkableClimb, *m_heightField);
	if (m_config.m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_config.m_walkableHeight, *m_heightField);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::PartitionWalkableSurface                                   |
//  |  Transforms the heightfield into a compact height field, connects           |
//  |  neightboring walkable surfaces, erodes all surfaces by the agent           |
//  |  radius and partitions them into regions.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::PartitionWalkableSurface()
{
	if (m_errorCode) return;

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
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::ExtractContours                                            |
//  |  Extracts contours from the compact height field.                     LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::ExtractContours()
{
	if (m_errorCode) return;
	m_cset = rcAllocContourSet();
	if (!m_cset)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'cset'.");

	if (!rcBuildContours(m_ctx, *m_chf, m_config.m_maxSimplificationError, m_config.m_maxEdgeLen, *m_cset))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not create contours.");
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::BuildPolygonMesh                                           |
//  |  Transforms the contours into a polygon mesh.                         LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::BuildPolygonMesh()
{
	if (m_errorCode) return;
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'pmesh'.");

	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_config.m_maxVertsPerPoly, *m_pmesh))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not triangulate contours.");
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::CreateDetailMesh                                           |
//  |  Creates the detailed polygon mesh.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::CreateDetailMesh()
{
	if (m_errorCode) return;
	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
		RECAST_ERROR(NMRECAST & NMALLOCATION, "buildNavigation: Out of memory 'pmdtl'.");

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_config.m_detailSampleDist, m_config.m_detailSampleMaxError, *m_dmesh))
		RECAST_ERROR(NMRECAST & NMCREATION, "buildNavigation: Could not build detail mesh.");
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::CreateDetourData                                           |
//  |  Creates Detour navmesh from the two poly meshes.                     LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::CreateDetourData()
{
	if (m_errorCode) return;

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

		m_navQuery = dtAllocNavMeshQuery();
		status = m_navQuery->init(m_navMesh, DETOUR_MAX_NAVMESH_NODES);
		if (dtStatusFailed(status))
			RECAST_ERROR(NMDETOUR & NMCREATION, "Could not init Detour navmesh query");
	}
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Save                                                       |
//  |  Writes the navmesh to storage for future use.                        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::Serialize(const char* dir, const char* ID)
{
	if (m_errorCode) return;

	// Saving config file
	char configfile[128];
	sprintf_s(configfile, "%s%s.config", dir, ID);
	m_config.Save(configfile);

	// Opening navmesh file for writing
	char filename[128];
	sprintf_s(filename, "%s%s.navmesh", dir, ID);
	const dtNavMesh* navMesh = m_navMesh;
	if (!navMesh) RECAST_ERROR(NMDETOUR & NMINPUT, "Can't serialize '%s', dtNavMesh is nullpointer", ID);
	FILE* fp;
	fopen_s(&fp, filename, "wb");
	if (!fp) RECAST_ERROR(NMIO, "Filename '%s' can't be opened", filename);
	
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
	RECAST_LOG("NavMesh build saved as '%s'", filename);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Load                                                       |
//  |  Reads a previously stored navmesh from storage.                      LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::Deserialize(const char* dir, const char* ID)
{
	if (m_errorCode) return;
	Cleanup();

	// Loading config file
	char configfile[128];
	sprintf_s(configfile, "%s%s.config", dir, ID);
	m_config.Load(configfile);
	m_config.m_id = ID; // strings aren't loaded correctly
	
	// Opening file
	char filename[128];
	sprintf_s(filename, "%s%s.navmesh", dir, ID);
	if (!FileExists(filename))
		RECAST_ERROR(NMIO, "NavMesh file '%s' does not exist", filename);
	FILE* fp;
	fopen_s(&fp, filename, "rb");
	if (!fp)
		RECAST_ERROR(NMIO, "NavMesh file '%s' could not be opened", filename);
	
	// Reading header
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		RECAST_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		RECAST_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		RECAST_ERROR(NMIO, "NavMesh file '%s' has the wrong navmesh set version", filename);
	}
	
	// Initializing navmesh with header info
	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		fclose(fp);
		RECAST_ERROR(NMDETOUR & NMALLOCATION, "NavMesh for '%s' could not be allocated", ID);
	}
	dtStatus status = m_navMesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		RECAST_ERROR(NMDETOUR & NMCREATION, "NavMesh for '%s' failed to initialize", ID);
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
			RECAST_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
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
			fclose(fp);
			RECAST_ERROR(NMIO, "NavMesh file '%s' is corrupted", filename);
		}
		m_navMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}
	fclose(fp);

	// Creating navMeshQuery
	m_navQuery = dtAllocNavMeshQuery();
	if (!m_navQuery)
	{
		dtFreeNavMesh(m_navMesh);
		RECAST_ERROR(NMDETOUR & NMALLOCATION, "NavMesh Query for '%s' could not be allocated", ID);
	}
	status = m_navQuery->init(m_navMesh, DETOUR_MAX_NAVMESH_NODES);
	if (dtStatusFailed(status))
	{
		dtFreeNavMesh(m_navMesh);
		RECAST_ERROR(NMDETOUR & NMCREATION, "Could not init Detour navmesh query for navMesh '%s'", ID);
	}
	RECAST_LOG("NavMesh '%s' loaded", ID);
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
	if (m_navQuery) dtFreeNavMeshQuery(m_navQuery);
	m_navQuery = 0;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshBuilder::Error                                                      |
//  |  Handles errors, logging, and error code maintenance.                 LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshBuilder::Error(rcLogCategory level, int code, const char* format, ...)
{
	if (code) m_errorCode = code;

	if (m_config.m_printImmediately)
	{
		printf("\n");
		va_list ap;
		__crt_va_start(ap, format);
		vprintf(format, ap);
		__crt_va_end(ap);
		if (code) printf("\n"); // No more logs after an error
	}
	else
	{
		va_list ap;
		__crt_va_start(ap, format);
		m_ctx->log(level, format, ap);
		__crt_va_end(ap);
	}
}





//  +-----------------------------------------------------------------------------+
//  |  BuildContext::doLog                                                        |
//  |  Virtual function called by rcContext::log                                 |
//  |  Adds messages to the queue.                                          LH2'19|
//  +-----------------------------------------------------------------------------+
void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (!len) return;
	if (m_messageCount >= MAX_MESSAGES)
		return;
	char* dst = &m_textPool[m_textPoolSize];
	int n = TEXT_POOL_SIZE - m_textPoolSize;
	if (n < 2)
		return;
	char* cat = dst;
	char* text = dst + 1;
	const int maxtext = n - 1;
	// Store category
	*cat = (char)category;
	// Store message
	const int count = rcMin(len + 1, maxtext);
	memcpy(text, msg, count);
	text[count - 1] = '\0';
	m_textPoolSize += 1 + count;
	m_messages[m_messageCount++] = dst;
}

//  +-----------------------------------------------------------------------------+
//  |  BuildContext::doStopTimer                                                  |
//  |  Virtual function called by rcContext::stopTimer                            |
//  |  Stops the timer and calculates the passed time.                      LH2'19|
//  +-----------------------------------------------------------------------------+
void BuildContext::doStopTimer(const rcTimerLabel label)
{
	const float endTime = timer.elapsed();
	const float deltaTime = endTime - m_startTime[label];
	if (m_accTime[label] == -1)
		m_accTime[label] = deltaTime;
	else
		m_accTime[label] += deltaTime;
}

//  +-----------------------------------------------------------------------------+
//  |  BuildContext::dumpLog                                                      |
//  |  Prints all logged messages to stdout.                                LH2'19|
//  +-----------------------------------------------------------------------------+
void BuildContext::dumpLog(const char* format, ...)
{
	// Print header.
	va_list ap;
	__crt_va_start(ap, format);
	vprintf(format, ap);
	__crt_va_end(ap);

	// Print messages
	const int TAB_STOPS[4] = { 28, 36, 44, 52 };
	for (int i = 0; i < m_messageCount; ++i)
	{
		const char* msg = m_messages[i] + 1;
		int n = 0;
		while (*msg)
		{
			if (*msg == '\t')
			{
				int count = 1;
				for (int j = 0; j < 4; ++j)
					if (n < TAB_STOPS[j])
					{
						count = TAB_STOPS[j] - n;
						break;
					}
				while (--count) { putchar(' '); n++; }
			}
			else
			{
				putchar(*msg);
				n++;
			}
			msg++;
		}
		putchar('\n');
	}

	resetLog();
}


} // namespace lighthouse2

// EOF