/* main_ui.h - Copyright 2019 Utrecht University

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

#include "anttweakbar.h"
#include "navmesh_builder.h"
#include "navmesh_navigator.h"
#include "navmesh_assets.h"

namespace AI_UI {

static TwBar* settings = 0, *navmesh = 0;
static NavMeshAssets* navMeshAssets = 0;
static bool leftClickLastFrame = false;
static bool rightClickLastFrame = false;

// Settings bar
static float mraysincl = 0, mraysexcl = 0;
enum MeshName { INPUT, NAVMESH, NODE, EDGE, AGENT };
static std::string meshName;
static int probMeshID = -1, probeInstID = -1, probeTriID = -1;
static float3 probedPos;
static float3 pathStart, pathEnd;

// Navmesh bar
static NavMeshConfig ui_nm_config;
static std::string ui_nm_id;
static bool ui_nm_errorcode = false;

// Path Finding bar


// Forward declarations
void InitFPSPrinter();
void PrintFPS(float deltaTime);
void RefreshSettings();
void RefreshNavMesh();


//  +-----------------------------------------------------------------------------+
//  |  RefreshUI                                                                  |
//  |  AntTweakBar.                                                         LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshUI()
{
	RefreshSettings();
	RefreshNavMesh();
}

//  +-----------------------------------------------------------------------------+
//  |  InitGUI                                                                    |
//  |  Prepares a basic user interface.                                     LH2'19|
//  +-----------------------------------------------------------------------------+
void InitGUI()
{
	// Init AntTweakBar
	TwInit( TW_OPENGL_CORE, NULL );
	settings = TwNewBar( "Settings" );
	navmesh = TwNewBar( "NavMesh" );
	RefreshUI();

	navMeshAssets = new NavMeshAssets(renderer, "data\\ai\\");

	InitFPSPrinter();
}

//  +-----------------------------------------------------------------------------+
//  |  DrawNavMesh                                                                |
//  |  Draws the GL aspects of the navmesh.                                 LH2'19|
//  +-----------------------------------------------------------------------------+
void DrawNavMesh()
{
	navMeshAssets->PlotPath(pathStart);
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL BuildNavMesh                                                       |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL BuildNavMesh(void *data)
{
	// Set configurations
	ui_nm_errorcode = NMSUCCESS;
	NavMeshConfig* config = navMeshBuilder->GetConfig();
	*config = ui_nm_config;
	config->m_id = ui_nm_id.c_str();

	// Build new mesh
	navMeshBuilder->Cleanup();
	navMeshBuilder->Build(renderer->GetScene());
	navMeshBuilder->DumpLog();
	ui_nm_errorcode = (bool)navMeshBuilder->GetError();
	if (ui_nm_errorcode) return;
	navMeshAssets->ReplaceMesh(navMeshBuilder);
	navMeshNavigator = navMeshBuilder->GetNavigator(); // TODO: move?

	ui_nm_config = *config;
	ui_nm_id = config->m_id;
	camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL SaveNavMesh                                                        |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL SaveNavMesh(void *data)
{
	// Set configurations
	ui_nm_errorcode = NMSUCCESS;
	NavMeshConfig* config = navMeshBuilder->GetConfig();
	*config = ui_nm_config;
	config->m_id = ui_nm_id.c_str();

	navMeshBuilder->Serialize();
	navMeshBuilder->DumpLog();
	ui_nm_errorcode = (bool)navMeshBuilder->GetError();
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL LoadNavMesh                                                        |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL LoadNavMesh(void *data)
{
	// Set configurations
	ui_nm_errorcode = NMSUCCESS;
	NavMeshConfig* config = navMeshBuilder->GetConfig();
	*config = ui_nm_config;
	config->m_id = ui_nm_id.c_str();

	// Load mesh
	navMeshBuilder->Deserialize();
	navMeshBuilder->DumpLog();
	ui_nm_errorcode = (bool)navMeshBuilder->GetError();
	if (ui_nm_errorcode) return;
	navMeshAssets->ReplaceMesh(navMeshBuilder);
	navMeshNavigator = navMeshBuilder->GetNavigator(); // TODO: move?

	ui_nm_config = *config;
	ui_nm_id = config->m_id;
	camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshSettings                                                            |
//  |  Defines the layout of the 'Settings' bar.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshSettings()
{
	TwDefine(" Settings size='200 400' color='50 120 50' alpha=220");
	TwDefine(" Settings help='LightHouse2 data' ");
	TwDefine(" Settings resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" Settings position='20 20' ");
	int opened = 1, closed = 0;
	TwStructMember float3Members[] = {
		{ "x", TW_TYPE_FLOAT, offsetof(float3, x), "" },
		{ "y", TW_TYPE_FLOAT, offsetof(float3, y), "" },
		{ "z", TW_TYPE_FLOAT, offsetof(float3, z), "" }
	};
	TwType float3Type = TwDefineStruct("float3", float3Members, 3, sizeof(float3), NULL, NULL);

	// create collapsed statistics block
	TwAddVarRO(settings, "rays", TW_TYPE_UINT32, &coreStats.totalRays, " group='statistics'");
	TwAddVarRO(settings, "build time", TW_TYPE_FLOAT, &coreStats.bvhBuildTime, " group='statistics'");
	TwAddVarRO(settings, "render time", TW_TYPE_FLOAT, &coreStats.renderTime, " group='statistics'");
	TwAddVarRO(settings, "shade time", TW_TYPE_FLOAT, &coreStats.shadeTime, " group='statistics'");
	TwAddVarRO(settings, "mrays inc", TW_TYPE_FLOAT, &mraysincl, " group='statistics'");
	TwAddVarRO(settings, "mrays ex", TW_TYPE_FLOAT, &mraysexcl, " group='statistics'");
	TwSetParam(settings, "statistics", "opened", TW_PARAM_INT32, 1, &closed);

	// create collapsed renderer block
	TwAddVarRW(settings, "epsilon", TW_TYPE_FLOAT, &renderer->GetSettings()->geometryEpsilon, "group='renderer'");
	TwAddVarRW(settings, "maxDirect", TW_TYPE_FLOAT, &renderer->GetSettings()->filterDirectClamp, "group='renderer' min=1 max=50 step=0.5");
	TwAddVarRW(settings, "maxIndirect", TW_TYPE_FLOAT, &renderer->GetSettings()->filterIndirectClamp, "group='renderer' min=1 max=50 step=0.5");
	TwSetParam(settings, "renderer", "opened", TW_PARAM_INT32, 1, &closed);

	// create collapsed camera block
	TwAddVarRO(settings, "position", float3Type, &renderer->GetCamera()->position, "group='camera'");
	TwAddVarRO(settings, "direction", float3Type, &renderer->GetCamera()->direction, "group='camera'");
	TwAddVarRW(settings, "FOV", TW_TYPE_FLOAT, &renderer->GetCamera()->FOV, "group='camera' min=10 max=99 step=1");
	TwAddVarRW(settings, "focaldist", TW_TYPE_FLOAT, &renderer->GetCamera()->focalDistance, "group='camera' min=0.1 max=100 step=0.01");
	TwAddVarRW(settings, "aperture", TW_TYPE_FLOAT, &renderer->GetCamera()->aperture, "group='camera' min=0 max=1 step=0.001");
	TwAddVarRW(settings, "brightness", TW_TYPE_FLOAT, &renderer->GetCamera()->brightness, "group='camera' min=-1 max=1 step=0.01");
	TwAddVarRW(settings, "contrast", TW_TYPE_FLOAT, &renderer->GetCamera()->contrast, "group='camera' min=-1 max=1 step=0.01");
	TwAddVarRW(settings, "clampValue", TW_TYPE_FLOAT, &renderer->GetCamera()->clampValue, "group='camera' min=1 max=100 step=1");
	TwSetParam(settings, "camera", "opened", TW_PARAM_INT32, 1, &closed);

	// create opened probing block
	TwAddVarRO(settings, "Mesh", TW_TYPE_STDSTRING, &meshName, "group='probing'");
	TwAddVarRO(settings, "Mesh ID", TW_TYPE_INT32, &probMeshID, "group='probing'");
	TwAddVarRO(settings, "Inst ID", TW_TYPE_INT32, &probeInstID, "group='probing'");
	TwAddVarRO(settings, "Tri ID", TW_TYPE_INT32, &probeTriID, "group='probing'");
	TwAddVarRO(settings, "Pos", float3Type, &probedPos, "group='probing'");
	TwAddVarRO(settings, "Start", float3Type, &pathStart, "group='probing'");
	TwAddVarRO(settings, "End", float3Type, &pathEnd, "group='probing'");
	TwSetParam(settings, "probing", "opened", TW_PARAM_INT32, 1, &closed);
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshMenu                                                                |
//  |  Defines the layout of the 'NavMesh' bar.                                LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshNavMesh()
{
	TwDefine(" NavMesh size='280 400' color='50 120 50' alpha=220");
	TwDefine(" NavMesh help='NavMesh generation options' ");
	TwDefine(" NavMesh resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" NavMesh position='1300 20' ");
	int opened = 1, closed = 0;
	TwEnumVal partitionEV[] = {
		{ NavMeshConfig::SAMPLE_PARTITION_WATERSHED, "Watershed" },
		{ NavMeshConfig::SAMPLE_PARTITION_MONOTONE, "Monotone" },
		{ NavMeshConfig::SAMPLE_PARTITION_LAYERS, "Layers" }
	};
	TwType PartitionType = TwDefineEnum("PartitionType", partitionEV, 3);
	TwStructMember float3Members[] = {
		{ "x", TW_TYPE_FLOAT, offsetof(float3, x), "" },
		{ "y", TW_TYPE_FLOAT, offsetof(float3, y), "" },
		{ "z", TW_TYPE_FLOAT, offsetof(float3, z), "" }
	};
	TwType float3Type = TwDefineStruct("AABB", float3Members, 3, sizeof(float3), NULL, NULL);

	// create voxelgrid block
	TwAddVarRW(navmesh, "AABB min", float3Type, &ui_nm_config.m_bmin, " group='voxelgrid'");
	TwAddVarRW(navmesh, "AABB max", float3Type, &ui_nm_config.m_bmax, " group='voxelgrid'");
	TwAddVarRW(navmesh, "cell size", TW_TYPE_FLOAT, &ui_nm_config.m_cs, " group='voxelgrid' min=0");
	TwAddVarRW(navmesh, "cell height", TW_TYPE_FLOAT, &ui_nm_config.m_ch, " group='voxelgrid' min=0");
	TwSetParam(navmesh, "voxelgrid", "opened", TW_PARAM_INT32, 1, &closed);

	// create agent block
	TwAddVarRW(navmesh, "max slope", TW_TYPE_FLOAT, &ui_nm_config.m_walkableSlopeAngle, " group='agent' min=0 max=90");
	TwAddVarRW(navmesh, "min height", TW_TYPE_INT32, &ui_nm_config.m_walkableHeight, " group='agent' min=1");
	TwAddVarRW(navmesh, "max climb", TW_TYPE_INT32, &ui_nm_config.m_walkableClimb, " group='agent' min=0");
	TwAddVarRW(navmesh, "min radius", TW_TYPE_INT32, &ui_nm_config.m_walkableRadius, " group='agent' min=1");
	TwSetParam(navmesh, "agent", "opened", TW_PARAM_INT32, 1, &closed);

	// create filtering block
	TwAddVarRW(navmesh, "low hanging obstacles", TW_TYPE_BOOL8, &ui_nm_config.m_filterLowHangingObstacles, " group='filtering'");
	TwAddVarRW(navmesh, "ledge spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterLedgeSpans, " group='filtering'");
	TwAddVarRW(navmesh, "low height spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterWalkableLowHeightSpans, " group='filtering'");
	TwSetParam(navmesh, "filtering", "opened", TW_PARAM_INT32, 1, &closed);

	// create partitioning block
	TwAddVarRW(navmesh, "partition type", PartitionType, &ui_nm_config.m_partitionType, " group='partitioning'");
	TwAddVarRW(navmesh, "min region area", TW_TYPE_INT32, &ui_nm_config.m_minRegionArea, " group='partitioning' min=0");
	TwAddVarRW(navmesh, "min merged region", TW_TYPE_INT32, &ui_nm_config.m_mergeRegionArea, " group='partitioning' min=0");
	TwSetParam(navmesh, "partitioning", "opened", TW_PARAM_INT32, 1, &closed);

	// create rasterization block
	TwAddVarRW(navmesh, "max edge length", TW_TYPE_INT32, &ui_nm_config.m_maxEdgeLen, " group='poly mesh' min=0");
	TwAddVarRW(navmesh, "max simpl err", TW_TYPE_FLOAT, &ui_nm_config.m_maxSimplificationError, " group='poly mesh' min=0");
	TwAddVarRW(navmesh, "max verts per poly", TW_TYPE_INT32, &ui_nm_config.m_maxVertsPerPoly, " group='poly mesh' min=3 max=6");
	TwAddVarRW(navmesh, "detail sample dist", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleDist, " group='poly mesh'");
	TwAddVarRW(navmesh, "detail max err", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleMaxError, " group='poly mesh' min=0 help='een heleboelnie tinterresa nteinformatie'");
	TwSetParam(navmesh, "poly mesh", "opened", TW_PARAM_INT32, 1, &closed);

	// create output block
	TwAddVarRW(navmesh, "NavMesh ID", TW_TYPE_STDSTRING, &ui_nm_id, " group='output'");
	TwAddVarRW(navmesh, "Print build stats", TW_TYPE_BOOL8, &ui_nm_config.m_printBuildStats, " group='output'");
	TwAddSeparator(navmesh, "menuseparator0", "group='output'");
	TwAddVarRO(navmesh, "error code", TW_TYPE_BOOL8, &ui_nm_errorcode, " group='output' true='ERROR' false=''");
	TwAddSeparator(navmesh, "menuseparator1", "group='output'");
	TwAddButton(navmesh, "Build", BuildNavMesh, NULL, " label='Build' ");
	TwAddButton(navmesh, "Save", SaveNavMesh, NULL, " label='Save' ");
	TwAddSeparator(navmesh, "menuseparator2", "group='output'");
	TwAddButton(navmesh, "Load", LoadNavMesh, NULL, " label='Load' ");
	TwAddSeparator(navmesh, "menuseparator3", "group='output'");
	TwSetParam(navmesh, "output", "opened", TW_PARAM_INT32, 1, &opened);
}

//  +-----------------------------------------------------------------------------+
//  |  HandleInput                                                                |
//  |  Process user input.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
bool HandleInput(float frameTime)
{
	if (!hasFocus) return false;

	// handle keyboard input
	float translateSpeed = (GetAsyncKeyState(VK_SHIFT) ? 15.0f : 5.0f) * frameTime, rotateSpeed = 2.5f * frameTime;
	bool changed = false;
	Camera* camera = renderer->GetCamera();
	if (GetAsyncKeyState('A')) { changed = true; camera->TranslateRelative(make_float3(-translateSpeed, 0, 0)); }
	if (GetAsyncKeyState('D')) { changed = true; camera->TranslateRelative(make_float3(translateSpeed, 0, 0)); }
	if (GetAsyncKeyState('W')) { changed = true; camera->TranslateRelative(make_float3(0, 0, translateSpeed)); }
	if (GetAsyncKeyState('S')) { changed = true; camera->TranslateRelative(make_float3(0, 0, -translateSpeed)); }
	if (GetAsyncKeyState('R')) { changed = true; camera->TranslateRelative(make_float3(0, translateSpeed, 0)); }
	if (GetAsyncKeyState('F')) { changed = true; camera->TranslateRelative(make_float3(0, -translateSpeed, 0)); }
	if (GetAsyncKeyState('B')) changed = true; // force restart
	if (GetAsyncKeyState(VK_UP)) { changed = true; camera->TranslateTarget(make_float3(0, -rotateSpeed, 0)); }
	if (GetAsyncKeyState(VK_DOWN)) { changed = true; camera->TranslateTarget(make_float3(0, rotateSpeed, 0)); }
	if (GetAsyncKeyState(VK_LEFT)) { changed = true; camera->TranslateTarget(make_float3(-rotateSpeed, 0, 0)); }
	if (GetAsyncKeyState(VK_RIGHT)) { changed = true; camera->TranslateTarget(make_float3(rotateSpeed, 0, 0)); }

	// Probing results are one frame delayed due to camera refresh
	if (leftClickLastFrame || rightClickLastFrame)
	{
		// DEBUG: crashes when probed distance is 0.0f
		if (coreStats.probedDist <= 0.0f)
		{
			printf("\tPROBED DIST == 0.0f ");
			return changed;
		}

		// Identify probed instance
		probeInstID = coreStats.probedInstid;
		probeTriID = coreStats.probedTriid;
		probMeshID = renderer->GetScene()->nodes[probeInstID]->meshID;
		meshName = renderer->GetScene()->meshes[probMeshID]->name;

		// Get 3D probe position
		ViewPyramid p = camera->GetView();
		float3 unitRight = (p.p2 - p.p1) / scrwidth;
		float3 unitDown = (p.p3 - p.p1) / scrheight;
		float3 pixelLoc = p.p1 + probeCoords.x * unitRight + probeCoords.y * unitDown;
		probedPos = camera->position + normalize(pixelLoc - camera->position) * coreStats.probedDist;

		// Apply new probe position
		if (leftClickLastFrame && navMeshAssets->isNavMesh(probMeshID))
		{
			pathStart = probedPos;
			navMeshAssets->pathStart = new float3(probedPos);
			if (navMeshAssets->pathEnd) // if both start and end are set
				navMeshAssets->UpdatePath(navMeshNavigator, pathStart, pathEnd);
			//navMeshAssets->PlaceAgent(probedPos); // DEBUG
		}
		if (rightClickLastFrame && navMeshAssets->isNavMesh(probMeshID))
		{
			pathEnd = probedPos;
			navMeshAssets->pathEnd = new float3(probedPos);
			if (navMeshAssets->pathStart) // if both start and end are set
				navMeshAssets->UpdatePath(navMeshNavigator, pathStart, pathEnd);
		}
		if (coreStats.probedDist < 1000) // prevents scene from going invisible
			camera->focalDistance = coreStats.probedDist;

		// Reset click delay and update focal distance
		leftClickLastFrame = rightClickLastFrame = false;
		changed = true;
	}

	// process button click
	if ((leftClicked || rightClicked) && GetAsyncKeyState(VK_LSHIFT))
	{
		if (leftClicked) leftClickLastFrame = true;
		if (rightClicked) rightClickLastFrame = true;
		leftClicked = rightClicked = false;
		changed = true; // probing requires a camera refresh
	}

	// let the main loop know if the camera should update
	return changed;
}

} // namespace AI_UI

// EOF