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

#pragma once

#include "anttweakbar.h"

#include "platform.h" // GLFW
#include "rendersystem.h"
#include "navmesh_builder.h"
#include "navmesh_navigator.h"
#include "navmesh_shader.h"
#include "physics_placeholder.h"
#include "navmesh_agents.h"

#include "edit_ui.h"
#include "debug_ui.h"

namespace AI_UI {

// Shared objects
static RenderAPI* renderer = 0;
static NavMeshBuilder* navMeshBuilder = 0;
static PhysicsPlaceholder* rigidBodies = 0;
static NavMeshAgents* navMeshAgents = 0;

// Shared variables
static bool *camMoved, *hasFocus, *leftClicked, *rightClicked;
static int2 *probeCoords = 0;
static uint *scrwidth = 0, *scrheight = 0;

// UI objects
static NavMeshNavigator* navMeshNavigator = 0;
static NavMeshShader* navMeshShader = 0;
static OffMeshConnectionTool* s_omcTool = 0;
static AgentNavigationTool* s_agentTool = 0;
static PathDrawingTool* s_pathTool = 0;
static TwBar* settingsBar = 0, *buildBar = 0, *editBar = 0, *debugBar = 0;

// GUI state
enum GUIMODE { GUI_MODE_BUILD, GUI_MODE_EDIT, GUI_MODE_DEBUG };
static GUIMODE guiMode = GUI_MODE_BUILD;
static GUIMODE buildMode = GUI_MODE_BUILD, editMode = GUI_MODE_EDIT, debugMode = GUI_MODE_DEBUG; // AntTweakBar callback needs smth to point to
static int alphaActive = 220, alphaPassive = 80;
enum SELECTIONTYPE { NONE, POLY, EDGE, VERT, AGENT };
static SELECTIONTYPE selectionType = NONE;
static bool leftClickLastFrame = false, rightClickLastFrame = false;
static bool ctrlClickLastFrame = false, shiftClickLastFrame = false, altClickLastFrame = false;

// Settings bar
static CoreStats coreStats;
static float mraysincl = 0, mraysexcl = 0;
static std::string meshName;
static int probMeshID = -1, probeInstID = -1, probeTriID = -1;
static float3 probedPos;

// Build bar
static NavMeshConfig* config;
static bool builderErrorStatus = false;

// Edit bar
static int selectionID = -1, polygonArea = -1, polygonType = -1;
static float3* origin = new float3{ 0, 0, 0 };
static float3* selectionVerts[6] = { origin, origin, origin, origin, origin, origin };
static bool isOffMesh = false, isDetail = false;
static const dtPoly* selectedPoly;
static NavMeshShader::Vert* selectedVert;
static NavMeshShader::Edge* selectedEdge;

// Debug bar
static std::vector<NavMeshNavigator::PathNode> path;
static bool reachable;
static float3 pathStart, pathEnd;
static mat4 agentScale;

// Forward declarations
void InitFPSPrinter();
void PrintFPS(float deltaTime);
void OnSelectNavMesh(int InstID);
void InitAntTweakBars();


//  +-----------------------------------------------------------------------------+
//  |  InitGUI                                                                    |
//  |  Prepares a basic user interface.                                     LH2'19|
//  +-----------------------------------------------------------------------------+
static void InitGUI(RenderAPI *a_renderer, NavMeshBuilder *a_builder, PhysicsPlaceholder *a_physics, NavMeshAgents *a_agents,
	bool &a_camMoved, bool &a_hasFocus, bool &a_leftClicked, bool &a_rightClicked, int2 &a_probeCoords, uint &a_scrwidth, uint &a_scrheight)
{
	// init pointers to main
	renderer = a_renderer;
	navMeshBuilder = a_builder;
	rigidBodies = a_physics;
	navMeshAgents = a_agents;
	camMoved = &a_camMoved;
	hasFocus = &a_hasFocus;
	leftClicked = &a_leftClicked;
	rightClicked = &a_rightClicked;
	probeCoords = &a_probeCoords;
	scrwidth = &a_scrwidth;
	scrheight = &a_scrheight;
	config = a_builder->GetConfig();

	navMeshShader = new NavMeshShader(renderer, "data\\ai\\");
	s_omcTool = new OffMeshConnectionTool(navMeshBuilder, navMeshShader);
	s_agentTool = new AgentNavigationTool(navMeshShader);
	s_pathTool = new PathDrawingTool(navMeshShader, navMeshNavigator);

	// Init AntTweakBar
	InitAntTweakBars();

	InitFPSPrinter();
}

//  +-----------------------------------------------------------------------------+
//  |  PreRenderUpdate                                                            |
//  |  Updates the GUI after physics, and before rendering.                 LH2'19|
//  +-----------------------------------------------------------------------------+
static void PreRenderUpdate(float deltaTime)
{
	navMeshShader->UpdateAgentPositions();
}

//  +-----------------------------------------------------------------------------+
//  |  PostRenderUpdate                                                           |
//  |  Updates the GUI after rendering has taken place.                     LH2'19|
//  +-----------------------------------------------------------------------------+
static void PostRenderUpdate(float deltaTime)
{
	coreStats = renderer->GetCoreStats();
	mraysincl = coreStats.totalRays / (coreStats.renderTime * 1000);
	mraysexcl = coreStats.totalRays / (coreStats.traceTime0 * 1000);
}

//  +-----------------------------------------------------------------------------+
//  |  RemoveEditAssets                                                           |
//  |  Returns all settings related to DEBUG mode to their defaults.        LH2'19|
//  +-----------------------------------------------------------------------------+
static void RemoveDebugAssets()
{
	if (selectionType == AGENT)
	{
		selectionType = NONE;
		s_agentTool->Clear(); // deselects the shader
	}
	s_pathTool->Clear();

	navMeshShader->RemoveAllAgents();
	rigidBodies->Clean();
	navMeshAgents->Clean();
}

//  +-----------------------------------------------------------------------------+
//  |  RemoveEditAssets                                                           |
//  |  Returns all settings related to EDIT mode to their defaults.         LH2'19|
//  +-----------------------------------------------------------------------------+
static void RemoveEditAssets()
{
	s_omcTool->Clear();
	if (selectionType == VERT || selectionType == EDGE || selectionType == POLY)
	{
		navMeshShader->Deselect();
		selectionType = NONE;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  ClearNavMesh                                                               |
//  |  Removes all navmesh assets from the scene.                           LH2'19|
//  +-----------------------------------------------------------------------------+
static void ClearNavMesh()
{
	RemoveDebugAssets();
	RemoveEditAssets();

	navMeshShader->Clean();
	navMeshBuilder->Cleanup();

	if (navMeshNavigator) delete navMeshNavigator;
	navMeshNavigator = 0;
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshNavigator                                                           |
//  |  Refreshes the NavMeshNavigator and updates the agent size.           LH2'19|
//  +-----------------------------------------------------------------------------+
static void RefreshNavigator()
{
	// Get navigator
	if (navMeshNavigator) delete navMeshNavigator;
	navMeshNavigator = navMeshBuilder->GetNavigator();
	navMeshShader->UpdateMesh(navMeshNavigator);
	navMeshShader->AddNavMeshToScene();

	// Updating new config data
	float radius = config->m_walkableRadius * config->m_cs; // voxels to world units
	float height = config->m_walkableHeight * config->m_ch; // voxels to world units
	agentScale = mat4::Scale(make_float3(radius * 2, height, radius * 2));
}

//  +-----------------------------------------------------------------------------+
//  |  DrawGUI                                                                    |
//  |  Performs all GUI GL drawing.                                         LH2'19|
//  +-----------------------------------------------------------------------------+
static void DrawGUI(float deltaTime)
{
	navMeshShader->DrawGL();
	TwDraw();
	PrintFPS(deltaTime);
}

//  +-----------------------------------------------------------------------------+
//  |  OnSelectNavMesh                                                            |
//  |  Handles selecting/deselecting navmesh objects. nullptr to deselect.  LH2'19|
//  +-----------------------------------------------------------------------------+
static void OnSelectNavMesh(int instID)
{
	selectionType = NONE;
	navMeshShader->Deselect();
	selectionID = polygonArea = polygonType = -1;
	for (int i = 0; i < 6; i++) selectionVerts[i] = origin;
	isOffMesh = false; // TODO: can't get this info yet

	if (instID)
	{
		if (navMeshShader->isVert(probMeshID))
		{
			selectionType = VERT;
			selectedVert = navMeshShader->SelectVert(instID);
			selectionID = selectedVert->idx;
			selectionVerts[0] = selectedVert->pos;
			TwDefine(" editBar/v0 visible=true ");
		}
		else if (navMeshShader->isEdge(probMeshID))
		{
			selectionType = EDGE;
			selectedEdge = navMeshShader->SelectEdge(instID);
			selectionID = selectedEdge->idx;
			selectionVerts[0] = navMeshShader->GetVertPos(selectedEdge->v1);
			selectionVerts[1] = navMeshShader->GetVertPos(selectedEdge->v2);
		}
		else if (navMeshShader->isPoly(probMeshID))
		{
			selectionType = POLY;
			selectedPoly = navMeshShader->SelectPoly(probedPos, navMeshNavigator);
			selectionID = -1; // TODO
			for (size_t i = 0; i < selectedPoly->vertCount; i++)
				selectionVerts[i] = navMeshShader->GetVertPos(selectedPoly->verts[i]);
			polygonArea = selectedPoly->getArea();
			polygonType = selectedPoly->getType();
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  HandleMouseInputEditMode                                                   |
//  |  Process mouse input when in EDIT mode.                               LH2'19|
//  +-----------------------------------------------------------------------------+
static void HandleMouseInputEditMode()
{
	// Instance selecting (SHIFT) (L-CLICK)
	if (leftClickLastFrame && shiftClickLastFrame)
	{
		if (navMeshShader->isNavMesh(probMeshID)) OnSelectNavMesh(probeInstID);
		else OnSelectNavMesh(0);
	}

	// Adding off-mesh connections (CTRL)
	if (ctrlClickLastFrame)
	{
		if (leftClickLastFrame) s_omcTool->SetStart(probedPos);
		else if (rightClickLastFrame) s_omcTool->SetEnd(probedPos);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  HandleMouseInputDebugMode                                                  |
//  |  Process mouse input when in DEBUG mode.                              LH2'19|
//  +-----------------------------------------------------------------------------+
static void HandleMouseInputDebugMode()
{
	// Agent placement (SHIFT) (R-CLICK)
	if (rightClickLastFrame && shiftClickLastFrame)
	{
		if (navMeshShader->isPoly(probMeshID))
		{
			RigidBody* rb = rigidBodies->AddRB(agentScale, mat4::Identity(), mat4::Translate(probedPos));
			Agent* agent = navMeshAgents->AddAgent(navMeshNavigator, rb);
			navMeshShader->AddAgentToScene(agent);
		}
	}

	// Agent selecting (SHIFT) (L-CLICK)
	if (leftClickLastFrame && shiftClickLastFrame)
	{
		if (navMeshShader->isAgent(probMeshID)) // selecting
		{
			s_pathTool->Clear();
			Agent* agent = navMeshShader->SelectAgent(probeInstID);
			if (agent) selectionType = AGENT;
			else selectionType = NONE;
			s_agentTool->SelectAgent(agent);
		}
		else if (selectionType == AGENT) // deselecting
		{
			s_agentTool->Clear();
			selectionType = NONE;
		}
	}

	// Setting path start/end (CTRL)
	if (ctrlClickLastFrame && navMeshShader->isNavMesh(probMeshID))
	{
		if (selectionType == AGENT)
		{
			if (rightClickLastFrame) s_agentTool->SetTarget(probedPos);
		}
		else if (selectionType != AGENT)
		{
			if (leftClickLastFrame)
			{
				s_pathTool->SetStart(probedPos);
				pathStart = probedPos;
			}
			if (rightClickLastFrame)
			{
				s_pathTool->SetEnd(probedPos);
				pathEnd = probedPos;
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  HandleInput                                                                |
//  |  Process user input.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
static bool HandleInput(float frameTime)
{
	if (!*hasFocus) return false;

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
			float3 unitRight = (p.p2 - p.p1) / *scrwidth;
			float3 unitDown = (p.p3 - p.p1) / *scrheight;
			float3 pixelLoc = p.p1 + probeCoords->x * unitRight + probeCoords->y * unitDown;
			probedPos = camera->position + normalize(pixelLoc - camera->position) * coreStats.probedDist;
		}

		if (guiMode == GUI_MODE_EDIT) HandleMouseInputEditMode();
		else if (guiMode == GUI_MODE_DEBUG) HandleMouseInputDebugMode();

		// Depth of field (SHIFT)
		if (shiftClickLastFrame)
			if (coreStats.probedDist < 1000) // prevents scene from going invisible
				camera->focalDistance = coreStats.probedDist;

		// Update camera
		if (shiftClickLastFrame) changed = true;
	}

	// reset click delay booleans
	leftClickLastFrame = rightClickLastFrame = false;
	ctrlClickLastFrame = shiftClickLastFrame = altClickLastFrame = false;

	// process button click
	if (*leftClicked || *rightClicked)
	{
		if (*leftClicked) leftClickLastFrame = true;
		if (*rightClicked) rightClickLastFrame = true;
		if (GetAsyncKeyState(VK_LSHIFT) < 0) shiftClickLastFrame = true;
		if (GetAsyncKeyState(VK_RSHIFT) < 0) shiftClickLastFrame = true;
		if (GetAsyncKeyState(VK_LCONTROL) < 0) ctrlClickLastFrame = true;
		if (GetAsyncKeyState(VK_RCONTROL) < 0) ctrlClickLastFrame = true;

		*leftClicked = *rightClicked = false;
		changed = true; // probing requires a camera refresh
	}

	// let the main loop know if the camera should update
	return changed;
}























//  +-----------------------------------------------------------------------------+
//  |  TW_CALL BuildNavMesh                                                       |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL BuildNavMesh(void *data)
{
	if (guiMode != GUI_MODE_BUILD) return;

	builderErrorStatus = NMSUCCESS;
	ClearNavMesh();
	navMeshBuilder->Build(renderer->GetScene());
	navMeshBuilder->DumpLog();
	builderErrorStatus = (bool)navMeshBuilder->GetError();
	if (builderErrorStatus) return;
	RefreshNavigator();
	*camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL SaveNavMesh                                                        |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL SaveNavMesh(void *data)
{
	builderErrorStatus = NMSUCCESS;
	navMeshBuilder->Serialize();
	navMeshBuilder->DumpLog();
	builderErrorStatus = (bool)navMeshBuilder->GetError();
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL LoadNavMesh                                                        |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL LoadNavMesh(void *data)
{
	if (guiMode != GUI_MODE_BUILD) return;

	builderErrorStatus = NMSUCCESS;
	ClearNavMesh();
	navMeshBuilder->Deserialize();
	navMeshBuilder->DumpLog();
	builderErrorStatus = (bool)navMeshBuilder->GetError();
	if (builderErrorStatus) return;
	RefreshNavigator();
	*camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL CleanNavMesh                                                       |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL CleanNavMesh(void *data)
{
	if (guiMode != GUI_MODE_BUILD) return;
	ClearNavMesh();
	*camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL ApplyChanges                                                       |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL ApplyChanges(void *data)
{
	if (guiMode != GUI_MODE_EDIT) return;
	RefreshNavigator();
	*camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL DiscardChanges (TODO)                                              |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL DiscardChanges(void *data)
{
	if (guiMode != GUI_MODE_EDIT) return;
	printf("DiscardChanges hasn't been implemented yet.\n"); // DEBUG
	*camMoved = true;
}

//  +-----------------------------------------------------------------------------+
//  |  TW_CALL SwitchGUIMode                                                      |
//  |  Callback function for AntTweakBar button.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void TW_CALL SwitchGUIMode(void *data)
{
	builderErrorStatus = false;
	GUIMODE newMode = *((GUIMODE*)data);
	if (guiMode == newMode) return; // nothing changed

	// Cleaning up from the previous mode
	if (guiMode == GUI_MODE_BUILD)
	{

	}
	else if (guiMode == GUI_MODE_EDIT)
	{
		RemoveEditAssets();
		RefreshNavigator();
	}
	else if (guiMode == GUI_MODE_DEBUG)
	{
		RemoveDebugAssets();
	}

	// Set tab alpha to represent GUI mode
	if (newMode == GUI_MODE_BUILD)
	{
		TwSetParam(buildBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaActive);
		TwSetParam(editBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
		TwSetParam(debugBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
	}
	else if (newMode == GUI_MODE_EDIT)
	{
		if (!navMeshBuilder->HasIntermediateResults())
		{
			builderErrorStatus = true;
			printf("Edit mode requires internal build data. Build a new navmesh to edit it.\n");
			return;
		}
		TwSetParam(buildBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
		TwSetParam(editBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaActive);
		TwSetParam(debugBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
	}
	else if (newMode == GUI_MODE_DEBUG)
	{
		if (navMeshBuilder->IsClean())
		{
			builderErrorStatus = true;
			printf("No navmesh to debug. Build/load a navmesh to test it.\n");
			return;
		}
		TwSetParam(buildBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
		TwSetParam(editBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaPassive);
		TwSetParam(debugBar, NULL, "alpha", TW_PARAM_INT32, 1, &alphaActive);
	}

	guiMode = newMode;
	*camMoved = true;
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
	TwDefine(" Settings position='20 440' ");
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
//  |  Defines the layout of the 'Building' bar.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshBuildBar()
{
	TwDefine(" Building size='280 400' color='50 120 50' alpha=220");
	TwDefine(" Building help='NavMesh generation options' ");
	TwDefine(" Building resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" Building position='1300 20' ");
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

	TwAddButton(buildBar, "Activate BUILD mode", SwitchGUIMode, &buildMode, " label='Switch to BUILD mode' ");
	TwAddSeparator(buildBar, "buildactivateseparator", "");

	// create voxelgrid block
	TwAddVarRW(buildBar, "AABB min", float3Type, &config->m_bmin, " group='voxelgrid'");
	TwAddVarRW(buildBar, "AABB max", float3Type, &config->m_bmax, " group='voxelgrid'");
	TwAddVarRW(buildBar, "cell size", TW_TYPE_FLOAT, &config->m_cs, " group='voxelgrid' min=0");
	TwAddVarRW(buildBar, "cell height", TW_TYPE_FLOAT, &config->m_ch, " group='voxelgrid' min=0");
	TwSetParam(buildBar, "voxelgrid", "opened", TW_PARAM_INT32, 1, &closed);

	// create agent block
	TwAddVarRW(buildBar, "max slope", TW_TYPE_FLOAT, &config->m_walkableSlopeAngle, " group='agent' min=0 max=90");
	TwAddVarRW(buildBar, "min height", TW_TYPE_INT32, &config->m_walkableHeight, " group='agent' min=1");
	TwAddVarRW(buildBar, "max climb", TW_TYPE_INT32, &config->m_walkableClimb, " group='agent' min=0");
	TwAddVarRW(buildBar, "min radius", TW_TYPE_INT32, &config->m_walkableRadius, " group='agent' min=1");
	TwSetParam(buildBar, "agent", "opened", TW_PARAM_INT32, 1, &closed);

	// create filtering block
	TwAddVarRW(buildBar, "low hanging obstacles", TW_TYPE_BOOL8, &config->m_filterLowHangingObstacles, " group='filtering'");
	TwAddVarRW(buildBar, "ledge spans", TW_TYPE_BOOL8, &config->m_filterLedgeSpans, " group='filtering'");
	TwAddVarRW(buildBar, "low height spans", TW_TYPE_BOOL8, &config->m_filterWalkableLowHeightSpans, " group='filtering'");
	TwSetParam(buildBar, "filtering", "opened", TW_PARAM_INT32, 1, &closed);

	// create partitioning block
	TwAddVarRW(buildBar, "partition type", PartitionType, &config->m_partitionType, " group='partitioning'");
	TwAddVarRW(buildBar, "min region area", TW_TYPE_INT32, &config->m_minRegionArea, " group='partitioning' min=0");
	TwAddVarRW(buildBar, "min merged region", TW_TYPE_INT32, &config->m_mergeRegionArea, " group='partitioning' min=0");
	TwSetParam(buildBar, "partitioning", "opened", TW_PARAM_INT32, 1, &closed);

	// create rasterization block
	TwAddVarRW(buildBar, "max edge length", TW_TYPE_INT32, &config->m_maxEdgeLen, " group='poly mesh' min=0");
	TwAddVarRW(buildBar, "max simpl err", TW_TYPE_FLOAT, &config->m_maxSimplificationError, " group='poly mesh' min=0");
	TwAddVarRW(buildBar, "max verts per poly", TW_TYPE_INT32, &config->m_maxVertsPerPoly, " group='poly mesh' min=3 max=6");
	TwAddVarRW(buildBar, "detail sample dist", TW_TYPE_FLOAT, &config->m_detailSampleDist, " group='poly mesh'");
	TwAddVarRW(buildBar, "detail max err", TW_TYPE_FLOAT, &config->m_detailSampleMaxError, " group='poly mesh' min=0 help='een heleboelnie tinterresa nteinformatie'");
	TwSetParam(buildBar, "poly mesh", "opened", TW_PARAM_INT32, 1, &closed);

	// create output block
	TwAddVarRW(buildBar, "NavMesh ID", TW_TYPE_STDSTRING, &config->m_id, " group='output'");
	TwAddVarRW(buildBar, "Print build stats", TW_TYPE_BOOL8, &config->m_printBuildStats, " group='output'");
	TwAddVarRO(buildBar, "error code", TW_TYPE_BOOL8, &builderErrorStatus, " group='output' true='ERROR' false=''");
	TwAddSeparator(buildBar, "menuseparator0", "group='output'");
	TwAddButton(buildBar, "Build", BuildNavMesh, NULL, "group='output' label='Build' ");
	TwAddSeparator(buildBar, "menuseparator1", "group='output'");
	TwAddButton(buildBar, "Save", SaveNavMesh, NULL, "group='output' label='Save' ");
	TwAddSeparator(buildBar, "menuseparator2", "group='output'");
	TwAddButton(buildBar, "Load", LoadNavMesh, NULL, "group='output' label='Load' ");
	TwAddSeparator(buildBar, "menuseparator3", "group='output'");
	TwAddButton(buildBar, "Clean", CleanNavMesh, NULL, "group='output' label='Clean' ");
	TwAddSeparator(buildBar, "menuseparator4", "group='output'");
	TwSetParam(buildBar, "output", "opened", TW_PARAM_INT32, 1, &opened);
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshMenu                                                                |
//  |  Defines the layout of the 'Editing' bar.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshEditBar()
{
	TwDefine(" Editing size='280 400' color='50 120 50' alpha=80");
	TwDefine(" Editing help='Pathfinding options' ");
	TwDefine(" Editing resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" Editing position='1300 440' ");
	int opened = 1, closed = 0;
	TwStructMember float3Members[] = {
		{ "x", TW_TYPE_FLOAT, offsetof(float3, x), "" },
		{ "y", TW_TYPE_FLOAT, offsetof(float3, y), "" },
		{ "z", TW_TYPE_FLOAT, offsetof(float3, z), "" }
	};
	TwType float3Type = TwDefineStruct("vert", float3Members, 3, sizeof(float3), NULL, NULL);
	TwEnumVal objectSelectionType[] = {
		{ NONE, "-" },
		{ VERT, "Vert" },
		{ EDGE, "Edge" },
		{ POLY, "Poly" },
		{ AGENT, "Agent" }
	};
	TwType objectSelectionTypeType = TwDefineEnum("SelectionType", objectSelectionType, 5);

	TwAddButton(editBar, "Activate EDIT mode", SwitchGUIMode, &editMode, " label='Switch to EDIT mode' ");
	TwAddSeparator(editBar, "editactivateseparator", "");

	// create selection block
	TwAddVarRO(editBar, "Type", objectSelectionTypeType, &selectionType, "");
	TwAddVarRO(editBar, "ID", TW_TYPE_INT32, &selectionID, "");
	TwAddVarRO(editBar, "OffMesh", TW_TYPE_BOOL32, &isOffMesh, "");

	// create verts block
	TwAddVarRO(editBar, "v0", float3Type, selectionVerts[0], "group='Verts' visible=false ");
	TwAddVarRO(editBar, "v1", float3Type, selectionVerts[1], "group='Verts' visible=false ");
	TwAddVarRO(editBar, "v2", float3Type, selectionVerts[2], "group='Verts' visible=false ");
	TwAddVarRO(editBar, "v3", float3Type, selectionVerts[3], "group='Verts' visible=false ");
	TwAddVarRO(editBar, "v4", float3Type, selectionVerts[4], "group='Verts' visible=false ");
	TwAddVarRO(editBar, "v5", float3Type, selectionVerts[5], "group='Verts' visible=false ");
	TwSetParam(editBar, "Verts", "opened", TW_PARAM_INT32, 1, &opened);

	// saving
	TwAddSeparator(editBar, "editsaveseparator1", "");
	TwAddButton(editBar, "Apply", ApplyChanges, NULL, " label='Apply Changes' ");
	TwAddSeparator(editBar, "editsaveseparator2", "");
	TwAddButton(editBar, "Discard", DiscardChanges, NULL, " label='Discard Changes' ");
	TwAddSeparator(editBar, "editsaveseparator3", "");
	TwAddButton(editBar, "Save", SaveNavMesh, NULL, " label='Save NavMesh' ");
	TwAddSeparator(editBar, "editsaveseparator4", "");
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshMenu                                                                |
//  |  Defines the layout of the 'Debugging' bar.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshDebugBar()
{
	TwDefine(" Debugging size='200 400' color='50 120 50' alpha=80");
	TwDefine(" Debugging help='Pathfinding options' ");
	TwDefine(" Debugging resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" Debugging position='20 20' ");
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
	TwAddVarRO(debugBar, "Reachable", TW_TYPE_BOOL32, &reachable, "group='path'");
	TwSetParam(debugBar, "path", "opened", TW_PARAM_INT32, 1, &opened);

	// create agent block
	TwSetParam(debugBar, "agent", "opened", TW_PARAM_INT32, 1, &opened);
}

//  +-----------------------------------------------------------------------------+
//  |  InitGUI                                                                    |
//  |  Prepares a basic user interface.                                     LH2'19|
//  +-----------------------------------------------------------------------------+
void InitAntTweakBars()
{
	// Init AntTweakBar
	TwInit(TW_OPENGL_CORE, NULL);
	settingsBar = TwNewBar("Settings");
	buildBar = TwNewBar("Building");
	editBar = TwNewBar("Editing");
	debugBar = TwNewBar("Debugging");
	RefreshSettingsBar();
	RefreshBuildBar();
	RefreshEditBar();
	RefreshDebugBar();
}

} // namespace AI_UI

// EOF