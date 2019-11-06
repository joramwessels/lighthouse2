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
#include "navmesh_shader.h"

namespace AI_UI {

static NavMeshShader* navMeshShader = 0;
static TwBar* settingsBar = 0, *editBar = 0, *debugBar;
static bool leftClickLastFrame = false, rightClickLastFrame = false;
static bool ctrlClickLastFrame = false, shiftClickLastFrame = false, altClickLastFrame = false;

// GUI mode
enum GUIMODE {EDIT, DEBUG};
static GUIMODE guiMode = EDIT;
static GUIMODE editMode = EDIT, debugMode = DEBUG; // AntTweakBar callback needs smth to point to
int alphaActive = 220, alphaPassive = 80;

// Settings bar
static float mraysincl = 0, mraysexcl = 0;
static std::string meshName;
static int probMeshID = -1, probeInstID = -1, probeTriID = -1;
static float3 probedPos;

// Edit bar
static NavMeshConfig ui_nm_config;
static std::string ui_nm_id;
static bool ui_nm_errorcode = false;

// Debug bar
static std::vector<float3> path;
static float distToEnd;
static float3 *pathStart = 0, *pathEnd = 0;
static Agent* agent;
static const float3 *agentPos, *agentDir, *agentTarget;
static mat4 agentScale;

// Forward declarations
void InitFPSPrinter();
void PrintFPS(float deltaTime);
void OnSelectAgent(Agent* agent);

//  +-----------------------------------------------------------------------------+
//  |  RemoveNavmeshAssets                                                        |
//  |  Removes all navmesh assets from the scene.                           LH2'19|
//  +-----------------------------------------------------------------------------+
void RemoveNavmeshAssets()
{
	OnSelectAgent(0);
	rigidBodies.Clean();
	navMeshAgents.Clean();
	navMeshShader->Clean();
	navMeshBuilder->Cleanup();
	if (navMeshNavigator) delete navMeshNavigator;
	navMeshNavigator = 0;
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
	RemoveNavmeshAssets();
	navMeshBuilder->Build(renderer->GetScene());
	navMeshBuilder->DumpLog();
	ui_nm_errorcode = (bool)navMeshBuilder->GetError();
	if (ui_nm_errorcode) return;

	// Get navigator
	if (navMeshNavigator) delete navMeshNavigator;
	navMeshNavigator = navMeshBuilder->GetNavigator();
	navMeshShader->UpdateMesh(navMeshNavigator);

	// Updating new config data
	ui_nm_config = *config;
	//ui_nm_id = config->m_id;
	float radius = config->m_walkableRadius * config->m_cs; // voxels to world units
	float height = config->m_walkableHeight * config->m_ch; // voxels to world units
	agentScale = mat4::Scale(make_float3(radius * 2, height, radius * 2));

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
	RemoveNavmeshAssets();
	navMeshBuilder->Deserialize();
	navMeshBuilder->DumpLog();
	ui_nm_errorcode = (bool)navMeshBuilder->GetError();
	if (ui_nm_errorcode) return;

	// Get navigator
	if (navMeshNavigator) delete navMeshNavigator;
	navMeshNavigator = navMeshBuilder->GetNavigator();
	navMeshShader->UpdateMesh(navMeshNavigator);

	// Updating new config data
	ui_nm_config = *config;
	//ui_nm_id = config->m_id;
	float radius = config->m_walkableRadius * config->m_cs; // voxels to world units
	float height = config->m_walkableHeight * config->m_ch; // voxels to world units
	agentScale = mat4::Scale(make_float3(radius * 2, height, radius * 2));

	camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL CleanNavMesh                                                       |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL CleanNavMesh(void *data)
{
	// Load mesh
	RemoveNavmeshAssets();
	camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL SwitchGUIMode                                                      |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL SwitchGUIMode(void *data)
{
	guiMode = *((GUIMODE*)data);
	if (guiMode == EDIT)
	{
		TwSetParam(editBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaActive);
		TwSetParam(debugBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
	}
	else if (guiMode == DEBUG)
	{
		TwSetParam(editBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
		TwSetParam(debugBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaActive);
	}
	navMeshShader->RemoveAllAgents();
	navMeshAgents.Clean();
	navMeshShader->Deselect();
	agent = 0;
	navMeshShader->SetPath(0, 0);
	navMeshShader->SetPathStart(0);
	navMeshShader->SetPathEnd(0);
	pathStart = pathEnd = 0;
}

void TW_CALL CopyStdStringToClient(std::string& dststr, const std::string& srcstr)
{
	// Copy the content of souceString handled by the AntTweakBar library to destinationClientString handled by your application
	dststr = srcstr;
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshSettings                                                            |
//  |  Defines the layout of the 'Settings' bar.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshSettingsBar()
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
	TwAddVarRO(settingsBar, "rays", TW_TYPE_UINT32, &coreStats.totalRays, " group='statistics'");
	TwAddVarRO(settingsBar, "build time", TW_TYPE_FLOAT, &coreStats.bvhBuildTime, " group='statistics'");
	TwAddVarRO(settingsBar, "render time", TW_TYPE_FLOAT, &coreStats.renderTime, " group='statistics'");
	TwAddVarRO(settingsBar, "shade time", TW_TYPE_FLOAT, &coreStats.shadeTime, " group='statistics'");
	TwAddVarRO(settingsBar, "mrays inc", TW_TYPE_FLOAT, &mraysincl, " group='statistics'");
	TwAddVarRO(settingsBar, "mrays ex", TW_TYPE_FLOAT, &mraysexcl, " group='statistics'");
	TwSetParam(settingsBar, "statistics", "opened", TW_PARAM_INT32, 1, &closed);

	// create collapsed renderer block
	TwAddVarRW(settingsBar, "epsilon", TW_TYPE_FLOAT, &renderer->GetSettings()->geometryEpsilon, "group='renderer'");
	TwAddVarRW(settingsBar, "maxDirect", TW_TYPE_FLOAT, &renderer->GetSettings()->filterDirectClamp, "group='renderer' min=1 max=50 step=0.5");
	TwAddVarRW(settingsBar, "maxIndirect", TW_TYPE_FLOAT, &renderer->GetSettings()->filterIndirectClamp, "group='renderer' min=1 max=50 step=0.5");
	TwSetParam(settingsBar, "renderer", "opened", TW_PARAM_INT32, 1, &closed);

	// create collapsed camera block
	TwAddVarRO(settingsBar, "position", float3Type, &renderer->GetCamera()->position, "group='camera'");
	TwAddVarRO(settingsBar, "direction", float3Type, &renderer->GetCamera()->direction, "group='camera'");
	TwAddVarRW(settingsBar, "FOV", TW_TYPE_FLOAT, &renderer->GetCamera()->FOV, "group='camera' min=10 max=99 step=1");
	TwAddVarRW(settingsBar, "focaldist", TW_TYPE_FLOAT, &renderer->GetCamera()->focalDistance, "group='camera' min=0.1 max=100 step=0.01");
	TwAddVarRW(settingsBar, "aperture", TW_TYPE_FLOAT, &renderer->GetCamera()->aperture, "group='camera' min=0 max=1 step=0.001");
	TwAddVarRW(settingsBar, "brightness", TW_TYPE_FLOAT, &renderer->GetCamera()->brightness, "group='camera' min=-1 max=1 step=0.01");
	TwAddVarRW(settingsBar, "contrast", TW_TYPE_FLOAT, &renderer->GetCamera()->contrast, "group='camera' min=-1 max=1 step=0.01");
	TwAddVarRW(settingsBar, "clampValue", TW_TYPE_FLOAT, &renderer->GetCamera()->clampValue, "group='camera' min=1 max=100 step=1");
	TwSetParam(settingsBar, "camera", "opened", TW_PARAM_INT32, 1, &closed);

	// create opened probing block
	TwAddVarRO(settingsBar, "Mesh", TW_TYPE_STDSTRING, &meshName, "group='probing'");
	TwAddVarRO(settingsBar, "Mesh ID", TW_TYPE_INT32, &probMeshID, "group='probing'");
	TwAddVarRO(settingsBar, "Inst ID", TW_TYPE_INT32, &probeInstID, "group='probing'");
	TwAddVarRO(settingsBar, "Tri ID", TW_TYPE_INT32, &probeTriID, "group='probing'");
	TwAddVarRO(settingsBar, "Pos", float3Type, &probedPos, "group='probing'");
	TwSetParam(settingsBar, "probing", "opened", TW_PARAM_INT32, 1, &closed);
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshMenu                                                                |
//  |  Defines the layout of the 'NavMesh' bar.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshEditBar()
{
	TwDefine(" NavMesh size='280 400' color='50 120 50' alpha=220");
	TwDefine(" NavMesh help='NavMesh generation options' ");
	TwDefine(" NavMesh resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" NavMesh position='1300 20' ");
	TwCopyStdStringToClientFunc(CopyStdStringToClient);
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

	TwAddButton(editBar, "Activate EDIT mode", SwitchGUIMode, &editMode, " label='Switch to EDIT mode' ");
	TwAddSeparator(editBar, "editactivateseparator", "");

	// create voxelgrid block
	TwAddVarRW(editBar, "AABB min", float3Type, &ui_nm_config.m_bmin, " group='voxelgrid'");
	TwAddVarRW(editBar, "AABB max", float3Type, &ui_nm_config.m_bmax, " group='voxelgrid'");
	TwAddVarRW(editBar, "cell size", TW_TYPE_FLOAT, &ui_nm_config.m_cs, " group='voxelgrid' min=0");
	TwAddVarRW(editBar, "cell height", TW_TYPE_FLOAT, &ui_nm_config.m_ch, " group='voxelgrid' min=0");
	TwSetParam(editBar, "voxelgrid", "opened", TW_PARAM_INT32, 1, &closed);

	// create agent block
	TwAddVarRW(editBar, "max slope", TW_TYPE_FLOAT, &ui_nm_config.m_walkableSlopeAngle, " group='agent' min=0 max=90");
	TwAddVarRW(editBar, "min height", TW_TYPE_INT32, &ui_nm_config.m_walkableHeight, " group='agent' min=1");
	TwAddVarRW(editBar, "max climb", TW_TYPE_INT32, &ui_nm_config.m_walkableClimb, " group='agent' min=0");
	TwAddVarRW(editBar, "min radius", TW_TYPE_INT32, &ui_nm_config.m_walkableRadius, " group='agent' min=1");
	TwSetParam(editBar, "agent", "opened", TW_PARAM_INT32, 1, &closed);

	// create filtering block
	TwAddVarRW(editBar, "low hanging obstacles", TW_TYPE_BOOL8, &ui_nm_config.m_filterLowHangingObstacles, " group='filtering'");
	TwAddVarRW(editBar, "ledge spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterLedgeSpans, " group='filtering'");
	TwAddVarRW(editBar, "low height spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterWalkableLowHeightSpans, " group='filtering'");
	TwSetParam(editBar, "filtering", "opened", TW_PARAM_INT32, 1, &closed);

	// create partitioning block
	TwAddVarRW(editBar, "partition type", PartitionType, &ui_nm_config.m_partitionType, " group='partitioning'");
	TwAddVarRW(editBar, "min region area", TW_TYPE_INT32, &ui_nm_config.m_minRegionArea, " group='partitioning' min=0");
	TwAddVarRW(editBar, "min merged region", TW_TYPE_INT32, &ui_nm_config.m_mergeRegionArea, " group='partitioning' min=0");
	TwSetParam(editBar, "partitioning", "opened", TW_PARAM_INT32, 1, &closed);

	// create rasterization block
	TwAddVarRW(editBar, "max edge length", TW_TYPE_INT32, &ui_nm_config.m_maxEdgeLen, " group='poly mesh' min=0");
	TwAddVarRW(editBar, "max simpl err", TW_TYPE_FLOAT, &ui_nm_config.m_maxSimplificationError, " group='poly mesh' min=0");
	TwAddVarRW(editBar, "max verts per poly", TW_TYPE_INT32, &ui_nm_config.m_maxVertsPerPoly, " group='poly mesh' min=3 max=6");
	TwAddVarRW(editBar, "detail sample dist", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleDist, " group='poly mesh'");
	TwAddVarRW(editBar, "detail max err", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleMaxError, " group='poly mesh' min=0 help='een heleboelnie tinterresa nteinformatie'");
	TwSetParam(editBar, "poly mesh", "opened", TW_PARAM_INT32, 1, &closed);

	// create output block
	TwAddVarRW(editBar, "NavMesh ID", TW_TYPE_STDSTRING, &ui_nm_id, " group='output'");
	TwAddVarRW(editBar, "Print build stats", TW_TYPE_BOOL8, &ui_nm_config.m_printBuildStats, " group='output'");
	TwAddSeparator(editBar, "menuseparator0", "group='output'");
	TwAddVarRO(editBar, "error code", TW_TYPE_BOOL8, &ui_nm_errorcode, " group='output' true='ERROR' false=''");
	TwAddSeparator(editBar, "menuseparator1", "group='output'");
	TwAddButton(editBar, "Build", BuildNavMesh, NULL, " label='Build' ");
	TwAddButton(editBar, "Save", SaveNavMesh, NULL, " label='Save' ");
	TwAddSeparator(editBar, "menuseparator2", "group='output'");
	TwAddButton(editBar, "Load", LoadNavMesh, NULL, " label='Load' ");
	TwAddSeparator(editBar, "menuseparator3", "group='output'");
	TwAddButton(editBar, "Clean", CleanNavMesh, NULL, " label='Clean' ");
	TwAddSeparator(editBar, "menuseparator4", "group='output'");
	TwSetParam(editBar, "output", "opened", TW_PARAM_INT32, 1, &opened);
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshMenu                                                                |
//  |  Defines the layout of the 'NavMesh' bar.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshDebugBar()
{
	TwDefine(" Navigation size='280 400' color='50 120 50' alpha=80");
	TwDefine(" Navigation help='Pathfinding options' ");
	TwDefine(" Navigation resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" Navigation position='1300 440' ");
	int opened = 1, closed = 0;
	TwStructMember float3Members[] = {
		{ "x", TW_TYPE_FLOAT, offsetof(float3, x), "" },
		{ "y", TW_TYPE_FLOAT, offsetof(float3, y), "" },
		{ "z", TW_TYPE_FLOAT, offsetof(float3, z), "" }
	};
	TwType float3Type = TwDefineStruct("pos", float3Members, 3, sizeof(float3), NULL, NULL);

	TwAddButton(debugBar, "Activate DEBUG mode", SwitchGUIMode, &debugMode, " label='Switch to DEBUG mode' ");
	TwAddSeparator(debugBar, "debugactivateseparator", "");

	// create path block
	TwAddVarRO(debugBar, "Start", float3Type, &pathStart, "group='path'");
	TwAddVarRO(debugBar, "End", float3Type, &pathEnd, "group='path'");
	TwAddVarRO(debugBar, "Distance to end", TW_TYPE_FLOAT, &distToEnd, "group='path'");
	TwSetParam(debugBar, "path", "opened", TW_PARAM_INT32, 1, &opened);

	// create agent block
	TwAddVarRO(debugBar, "Selected", TW_TYPE_BOOL32, &agent, "group='agent'");
	TwAddVarRO(debugBar, "Pos", float3Type, &agentPos, "group='agent'");
	TwAddVarRO(debugBar, "Dir", float3Type, &agentDir, "group='agent'");
	TwAddVarRO(debugBar, "Target", TW_TYPE_FLOAT, &agentTarget, "group='agent'");
	TwSetParam(debugBar, "agent", "opened", TW_PARAM_INT32, 1, &opened);
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshUI                                                                  |
//  |  AntTweakBar.                                                         LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshUI()
{
	RefreshSettingsBar();
	RefreshEditBar();
	RefreshDebugBar();
}

//  +-----------------------------------------------------------------------------+
//  |  InitGUI                                                                    |
//  |  Prepares a basic user interface.                                     LH2'19|
//  +-----------------------------------------------------------------------------+
void InitGUI()
{
	// Init AntTweakBar
	TwInit(TW_OPENGL_CORE, NULL);
	settingsBar = TwNewBar("Settings");
	editBar = TwNewBar("NavMesh");
	debugBar = TwNewBar("Navigation");
	RefreshUI();

	navMeshShader = new NavMeshShader(renderer, "data\\ai\\");

	InitFPSPrinter();
}

//  +-----------------------------------------------------------------------------+
//  |  OnSelectAgent                                                              |
//  |  Handles selecting/deselecting an agent. Deselect by passing 0.       LH2'19|
//  +-----------------------------------------------------------------------------+
void OnSelectAgent(Agent* a_agent)
{
	agent = a_agent;
	pathStart = 0;
	if (agent) // select
	{
		pathEnd = agent->GetTarget();
		navMeshShader->SetPathStart(0);
		navMeshShader->SetPathEnd(pathEnd);
		navMeshShader->SetPath(agent->GetPos(), agent->GetPath());
		agentPos = agent->GetPos();
		agentDir = agent->GetDir();
		agentTarget = agent->GetTarget();
	}
	else // deselect
	{
		pathEnd = 0;
		navMeshShader->SetPath(0, 0);
		navMeshShader->SetPathStart(0);
		navMeshShader->SetPathEnd(0);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  HandleMouseInputEditMode                                                   |
//  |  Process mouse input when in EDIT mode.                               LH2'19|
//  +-----------------------------------------------------------------------------+
void HandleMouseInputEditMode()
{
	// Instance selecting (SHIFT) (L-CLICK)
	if (leftClickLastFrame && shiftClickLastFrame)
	{
		if (navMeshShader->isNavMesh(probMeshID))
			OnSelectAgent(0); // Deselect the agent
		if (navMeshShader->isVert(probMeshID))
			navMeshShader->SelectVert(probeInstID);
		else if (navMeshShader->isEdge(probMeshID))
			navMeshShader->SelectEdge(probeInstID);
		else if (navMeshShader->isPoly(probMeshID))
			navMeshShader->SelectPoly(probedPos, navMeshNavigator);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  HandleMouseInputDebugMode                                                  |
//  |  Process mouse input when in DEBUG mode.                              LH2'19|
//  +-----------------------------------------------------------------------------+
void HandleMouseInputDebugMode()
{
	// Agent placement (SHIFT) (R-CLICK)
	if (rightClickLastFrame && shiftClickLastFrame)
	{
		if (navMeshShader->isPoly(probMeshID))
		{
			RigidBody* rb = rigidBodies.AddRB(agentScale, mat4::Identity(), mat4::Translate(probedPos));
			Agent* agent = navMeshAgents.AddAgent(navMeshNavigator, rb);
			navMeshShader->AddAgentToScene(agent);
		}
	}

	// Agent selecting (SHIFT) (L-CLICK)
	if (leftClickLastFrame && shiftClickLastFrame && navMeshShader->isAgent(probMeshID))
		OnSelectAgent(navMeshShader->SelectAgent(probeInstID));

	// Setting path start/end (CTRL)
	if (ctrlClickLastFrame && navMeshShader->isNavMesh(probMeshID))
	{
		if (!agent) // When no agent is selected
		{
			if (leftClickLastFrame)
			{
				if (!pathStart) pathStart = new float3(probedPos);
				else *pathStart = probedPos;
				navMeshShader->SetPathStart(pathStart);
				if (pathEnd) // if both start and end are set
					if (!navMeshNavigator->FindPath(*pathStart, *pathEnd, path, &distToEnd))
						navMeshShader->SetPath(pathStart, &path);
			}
			if (rightClickLastFrame)
			{
				if (!pathEnd) pathEnd = new float3(probedPos);
				else *pathEnd = probedPos;
				navMeshShader->SetPathEnd(pathEnd);
				if (pathStart) // if both start and end are set
					if (!navMeshNavigator->FindPath(*pathStart, *pathEnd, path, &distToEnd))
						navMeshShader->SetPath(pathStart, &path);
			}
		}
		else // When an agent is selected
		{
			if (rightClickLastFrame)
			{
				if (!pathEnd) pathEnd = new float3(probedPos);
				else *pathEnd = probedPos;
				navMeshShader->SetPathEnd(pathEnd);
				agent->SetTarget(*pathEnd);
				agent->UpdateNavigation(0);
				navMeshShader->SetPath(agent->GetPos(), agent->GetPath());
			}
		}
	}
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

		// Only calculate probe info when ctrl/shift-clicked
		if (ctrlClickLastFrame || shiftClickLastFrame)
		{
			// Identify probed instance
			probeInstID = coreStats.probedInstid;
			probeTriID = coreStats.probedTriid;
			probMeshID = renderer->GetInstanceMeshID(probeInstID);
			meshName = renderer->GetMesh(probMeshID)->name;

			// Get 3D probe position
			ViewPyramid p = camera->GetView();
			float3 unitRight = (p.p2 - p.p1) / scrwidth;
			float3 unitDown = (p.p3 - p.p1) / scrheight;
			float3 pixelLoc = p.p1 + probeCoords.x * unitRight + probeCoords.y * unitDown;
			probedPos = camera->position + normalize(pixelLoc - camera->position) * coreStats.probedDist;
		}

		if (guiMode == EDIT) HandleMouseInputEditMode();
		else if (guiMode == DEBUG) HandleMouseInputDebugMode();

		// Depth of field (SHIFT)
		if (shiftClickLastFrame)
			if (coreStats.probedDist < 1000) // prevents scene from going invisible
				camera->focalDistance = coreStats.probedDist;

		// Update camera and reset click delay booleans
		if (shiftClickLastFrame) changed = true;
		leftClickLastFrame = rightClickLastFrame = false;
		ctrlClickLastFrame = shiftClickLastFrame = altClickLastFrame = false;
	}

	// process button click
	if (leftClicked || rightClicked)
	{
		if (leftClicked) leftClickLastFrame = true;
		if (rightClicked) rightClickLastFrame = true;
		if (GetAsyncKeyState(VK_LSHIFT)) shiftClickLastFrame = true;
		if (GetAsyncKeyState(VK_RSHIFT)) shiftClickLastFrame = true;
		if (GetAsyncKeyState(VK_LCONTROL)) ctrlClickLastFrame = true;
		if (GetAsyncKeyState(VK_RCONTROL)) ctrlClickLastFrame = true;

		leftClicked = rightClicked = false;
		changed = true; // probing requires a camera refresh
	}

	// let the main loop know if the camera should update
	return changed;
}

} // namespace AI_UI

// EOF