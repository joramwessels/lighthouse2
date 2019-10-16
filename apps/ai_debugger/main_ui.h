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
#include "navmesh_assets.h"

namespace AI_UI {

// AntTweakBar data
float mraysincl = 0, mraysexcl = 0;
TwBar* settings = 0, *menu = 0;
HostMaterial currentMaterial; // will contain a copy of the material we're editing
bool currentMaterialConductor, currentMaterialDielectric;
int currentMaterialID = -1;
static CoreStats coreStats;

// Triangle probing
int probMeshID = -1;
float3 probedPos;
float3 pathStart, pathEnd;

// Navmesh generation
int navmeshMeshID = -1;
int navmeshInstID = -1;
static NavMeshConfig ui_nm_config;
static std::string ui_nm_id;
static bool ui_nm_errorcode = false;
static std::string ui_nm_meshFile = "";
static NavMeshAssets* navMeshAssets = 0;

//  +-----------------------------------------------------------------------------+
//  |  InitAntTweakBar                                                            |
//  |  Prepares a basic user interface.                                     LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshUI();
void InitAntTweakBar()
{
	TwInit( TW_OPENGL_CORE, NULL );
	settings = TwNewBar( "settings" );
	menu = TwNewBar( "NavMesh" );
	RefreshUI();

	navMeshAssets = new NavMeshAssets(renderer, navmesh->GetDir()); // TODO move somewhere else
}

// Button Callbacks
void TW_CALL BuildNavMesh(void *data)
{
	// Set configurations
	ui_nm_errorcode = NavMeshBuilder::NMSUCCESS;
	NavMeshConfig* config = navmesh->GetConfig();
	*config = ui_nm_config;
	config->m_id = ui_nm_id.c_str();

	// Build new mesh
	navmesh->Cleanup();
	navmesh->Build(renderer->GetScene());
	navmesh->DumpLog();
	ui_nm_errorcode = (bool)navmesh->GetError();
	if (ui_nm_errorcode) return;
	navMeshAssets->ReplaceMesh(navmesh);

	ui_nm_config = *config;
	ui_nm_id = config->m_id;
}
void TW_CALL SaveNavMesh(void *data)
{
	// Set configurations
	ui_nm_errorcode = NavMeshBuilder::NMSUCCESS;
	NavMeshConfig* config = navmesh->GetConfig();
	*config = ui_nm_config;
	config->m_id = ui_nm_id.c_str();

	navmesh->Serialize();
	navmesh->DumpLog();
	ui_nm_errorcode = (bool)navmesh->GetError();
}
void TW_CALL LoadNavMesh(void *data)
{
	// Set configurations
	ui_nm_errorcode = NavMeshBuilder::NMSUCCESS;
	NavMeshConfig* config = navmesh->GetConfig();
	*config = ui_nm_config;
	config->m_id = ui_nm_id.c_str();

	// Load mesh
	navmesh->Deserialize();
	navmesh->DumpLog();
	ui_nm_errorcode = (bool)navmesh->GetError();
	if (ui_nm_errorcode) return;
	navMeshAssets->ReplaceMesh(navmesh);

	ui_nm_config = *config;
	ui_nm_id = config->m_id;
}

// TW types
TwStructMember float3Members[] = {
	{ "x", TW_TYPE_FLOAT, offsetof(float3, x), "" },
	{ "y", TW_TYPE_FLOAT, offsetof(float3, y), "" },
	{ "z", TW_TYPE_FLOAT, offsetof(float3, z), "" }
};

//  +-----------------------------------------------------------------------------+
//  |  RefreshUI                                                                  |
//  |  AntTweakBar.                                                         LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshSettings()
{
	TwDefine(" settings size='200 400' color='50 120 50' alpha=220");
	TwDefine(" settings help='LightHouse2 data' ");
	TwDefine(" settings resizable=true movable=true iconifiable=true refresh=0.05 ");
	TwDefine(" settings position='20 20' ");
	int opened = 1, closed = 0;
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
	TwAddVarRO(settings, "Mesh ID", TW_TYPE_INT32, &probMeshID, "group='probing'");
	TwAddVarRO(settings, "Inst ID", TW_TYPE_INT32, &coreStats.probedInstid, "group='probing'");
	TwAddVarRO(settings, "Tri ID", TW_TYPE_INT32, &coreStats.probedTriid, "group='probing'");
	TwAddVarRO(settings, "Pos", float3Type, &probedPos, "group='probing'");
	TwAddVarRO(settings, "Start", float3Type, &pathStart, "group='probing'");
	TwAddVarRO(settings, "End", float3Type, &pathEnd, "group='probing'");
	TwSetParam(settings, "probing", "opened", TW_PARAM_INT32, 1, &closed);
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshUI                                                                  |
//  |  AntTweakBar.                                                         LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshMenu()
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
	TwType float3Type = TwDefineStruct("AABB", float3Members, 3, sizeof(float3), NULL, NULL);

	// create voxelgrid block
	TwAddVarRW(menu, "AABB min", float3Type, &ui_nm_config.m_bmin, " group='voxelgrid'");
	TwAddVarRW(menu, "AABB max", float3Type, &ui_nm_config.m_bmax, " group='voxelgrid'");
	TwAddVarRW(menu, "cell size", TW_TYPE_FLOAT, &ui_nm_config.m_cs, " group='voxelgrid' min=0");
	TwAddVarRW(menu, "cell height", TW_TYPE_FLOAT, &ui_nm_config.m_ch, " group='voxelgrid' min=0");
	TwSetParam(menu, "voxelgrid", "opened", TW_PARAM_INT32, 1, &closed);

	// create agent block
	TwAddVarRW(menu, "max slope", TW_TYPE_FLOAT, &ui_nm_config.m_walkableSlopeAngle, " group='agent' min=0 max=90");
	TwAddVarRW(menu, "min height", TW_TYPE_INT32, &ui_nm_config.m_walkableHeight, " group='agent' min=1");
	TwAddVarRW(menu, "max climb", TW_TYPE_INT32, &ui_nm_config.m_walkableClimb, " group='agent' min=0");
	TwAddVarRW(menu, "min radius", TW_TYPE_INT32, &ui_nm_config.m_walkableRadius, " group='agent' min=1");
	TwSetParam(menu, "agent", "opened", TW_PARAM_INT32, 1, &closed);

	// create filtering block
	TwAddVarRW(menu, "low hanging obstacles", TW_TYPE_BOOL8, &ui_nm_config.m_filterLowHangingObstacles, " group='filtering'");
	TwAddVarRW(menu, "ledge spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterLedgeSpans, " group='filtering'");
	TwAddVarRW(menu, "low height spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterWalkableLowHeightSpans, " group='filtering'");
	TwSetParam(menu, "filtering", "opened", TW_PARAM_INT32, 1, &closed);

	// create partitioning block
	TwAddVarRW(menu, "partition type", PartitionType, &ui_nm_config.m_partitionType, " group='partitioning'");
	TwAddVarRW(menu, "min region area", TW_TYPE_INT32, &ui_nm_config.m_minRegionArea, " group='partitioning' min=0");
	TwAddVarRW(menu, "min merged region", TW_TYPE_INT32, &ui_nm_config.m_mergeRegionArea, " group='partitioning' min=0");
	TwSetParam(menu, "partitioning", "opened", TW_PARAM_INT32, 1, &closed);

	// create rasterization block
	TwAddVarRW(menu, "max edge length", TW_TYPE_INT32, &ui_nm_config.m_maxEdgeLen, " group='poly mesh' min=0");
	TwAddVarRW(menu, "max simpl err", TW_TYPE_FLOAT, &ui_nm_config.m_maxSimplificationError, " group='poly mesh' min=0");
	TwAddVarRW(menu, "max verts per poly", TW_TYPE_INT32, &ui_nm_config.m_maxVertsPerPoly, " group='poly mesh' min=3 max=6");
	TwAddVarRW(menu, "detail sample dist", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleDist, " group='poly mesh'");
	TwAddVarRW(menu, "detail max err", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleMaxError, " group='poly mesh' min=0 help='een heleboelnie tinterresa nteinformatie'");
	TwSetParam(menu, "poly mesh", "opened", TW_PARAM_INT32, 1, &closed);

	// create output block
	TwAddVarRW(menu, "NavMesh ID", TW_TYPE_STDSTRING, &ui_nm_id, " group='output'");
	TwAddVarRW(menu, "Print build stats", TW_TYPE_BOOL8, &ui_nm_config.m_printBuildStats, " group='output'");
	TwAddVarRW(menu, "Print immediately", TW_TYPE_BOOL8, &ui_nm_config.m_printImmediately, " group='output'");
	TwAddSeparator(menu, "menuseparator0", "group='output'");
	TwAddVarRO(menu, "error code", TW_TYPE_BOOL8, &ui_nm_errorcode, " group='output' true='ERROR' false=''");
	TwAddSeparator(menu, "menuseparator1", "group='output'");
	TwAddButton(menu, "Build", BuildNavMesh, NULL, " label='Build' ");
	TwAddButton(menu, "Save", SaveNavMesh, NULL, " label='Save' ");
	TwAddSeparator(menu, "menuseparator2", "group='output'");
	TwAddButton(menu, "Load", LoadNavMesh, NULL, " label='Load' ");
	TwAddSeparator(menu, "menuseparator3", "group='output'");
	TwSetParam(menu, "output", "opened", TW_PARAM_INT32, 1, &opened);
}

//  +-----------------------------------------------------------------------------+
//  |  RefreshUI                                                                  |
//  |  AntTweakBar.                                                         LH2'19|
//  +-----------------------------------------------------------------------------+
void RefreshUI()
{
	RefreshSettings();
	RefreshMenu();
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

	// process button click
	if ((leftClicked || rightClicked) && GetAsyncKeyState(VK_LSHIFT))
	{
		if (coreStats.probedDist <= 0.0f)
		{
			printf("\tPROBED DIST == 0.0f"); // DEBUG
			return changed;
		}

		probMeshID = renderer->GetScene()->instances[coreStats.probedInstid];
		probedPos = camera->position + camera->direction * coreStats.probedDist;
		if (leftClicked) pathStart = probedPos;
		if (rightClicked) pathEnd = probedPos;

		camera->focalDistance = coreStats.probedDist;
		changed = true;
		leftClicked = rightClicked = false;
	}

	// let the main loop know if the camera should update
	return changed;
}





GLTexture* digit[10], *hud;
Shader* plainShader = 0, *shadowShader = 0;
float smoothed = 1.0f, smoothFactor = 0.1f;

void InitFPSPrinter()
{
	// load digits
	for (int i = 0; i < 10; i++)
	{
		char t[128] = "data//system//digit0.png";
		t[strlen(t) - 5] += i;
		digit[i] = new GLTexture(t, GL_LINEAR);
	}
	// load HUD
	hud = new GLTexture("data//system//hud.png", GL_LINEAR);
	// load shaders
	plainShader = new Shader("shaders/plain.vert", "shaders/plain.frag");
	shadowShader = new Shader("shaders/plain.vert", "shaders/plain_shadow.frag");
}

void DrawDigit(int d, float x, float y, float scale = 1.0f)
{
	plainShader->SetInputTexture(0, "color", digit[d]);
	mat4 T = mat4::Scale(make_float3(0.06f * scale, 0.1f * scale, 1));
	T.cell[12] = x, T.cell[13] = y;
	plainShader->SetInputMatrix("view", T);
	DrawQuad();
}

void DrawHUD(float x, float y)
{
	plainShader->SetInputTexture(0, "color", hud);
	float scale = 4.5f;
	mat4 T = mat4::Scale(scale * make_float3(0.06f, 0.1f, 1));
	T.cell[12] = x, T.cell[13] = y;
	plainShader->SetInputMatrix("view", T);
	DrawQuad();
}

void PrintFPS(float deltaTime)
{
	float fps = (int)(1.0f / deltaTime);
	smoothed = (1 - smoothFactor) * smoothed + smoothFactor * fps;
	if (smoothFactor > 0.05f) smoothFactor -= 0.05f;
	int ifps = smoothed * 10, d1 = (ifps / 1000) % 10, d2 = (ifps / 100) % 10, d3 = (ifps / 10) % 10, d4 = ifps % 10;
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	shadowShader->Bind();
	float xpos = -0.91f, ypos = -0.81f;
	DrawDigit(d1, xpos, ypos); xpos += 0.12f;
	DrawDigit(d2, xpos, ypos); xpos += 0.12f;
	DrawDigit(d3, xpos, ypos); xpos += 0.14f;
	DrawDigit(d4, xpos, ypos - 0.03f, 0.7f);
	shadowShader->Unbind();
	plainShader->Bind();
	xpos = -0.92f, ypos = -0.8f;
	DrawDigit(d1, xpos, ypos); xpos += 0.12f;
	DrawDigit(d2, xpos, ypos); xpos += 0.12f;
	DrawDigit(d3, xpos, ypos); xpos += 0.14f;
	DrawDigit(d4, xpos, ypos - 0.03f, 0.7f);
	plainShader->Unbind();
	glDisable(GL_BLEND);
}

} // namespace AI_UI

// EOF