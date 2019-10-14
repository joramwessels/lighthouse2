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
#include "PathFinding.h"

// AntTweakBar data
float mraysincl = 0, mraysexcl = 0;
TwBar* settings = 0, *menu = 0;
HostMaterial currentMaterial; // will contain a copy of the material we're editing
bool currentMaterialConductor, currentMaterialDielectric;
int currentMaterialID = -1;
static CoreStats coreStats;

// Navmesh generation
static NavMeshConfig ui_nm_config;
static bool ui_nm_save = false;

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
	// create collapsed statistics block
	TwAddVarRO(settings, "rays", TW_TYPE_UINT32, &coreStats.totalRays, " group='statistics'");
	TwAddVarRO(settings, "build time", TW_TYPE_FLOAT, &coreStats.bvhBuildTime, " group='statistics'");
	TwAddVarRO(settings, "render time", TW_TYPE_FLOAT, &coreStats.renderTime, " group='statistics'");
	TwAddVarRO(settings, "shade time", TW_TYPE_FLOAT, &coreStats.shadeTime, " group='statistics'");
	TwAddVarRO(settings, "mrays inc", TW_TYPE_FLOAT, &mraysincl, " group='statistics'");
	TwAddVarRO(settings, "mrays ex", TW_TYPE_FLOAT, &mraysexcl, " group='statistics'");
	TwAddSeparator(settings, "separator0", "group='statistics'");
	TwAddVarRO(settings, "probed tri", TW_TYPE_INT32, &coreStats.probedTriid, " group='statistics'");
	TwSetParam(settings, "statistics", "opened", TW_PARAM_INT32, 1, &closed);
	// create collapsed material block
	TwAddVarRO(settings, "name", TW_TYPE_STDSTRING, &currentMaterial.name, "group='material'");
	TwAddVarRO(settings, "origin", TW_TYPE_STDSTRING, &currentMaterial.origin, "group='material'");
	TwAddVarRO(settings, "ID", TW_TYPE_INT32, &currentMaterial.ID, "group='material'");
	TwAddVarRO(settings, "flags", TW_TYPE_UINT32, &currentMaterial.flags, "group='material'");
	TwAddVarRW(settings, "color", TW_TYPE_COLOR3F, &currentMaterial.color, "group='material'");
	TwAddVarRW(settings, "transmiss", TW_TYPE_COLOR3F, &currentMaterial.absorption, "group='material'");
	TwAddVarRW(settings, "metallic", TW_TYPE_FLOAT, &currentMaterial.metallic, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "subsurface", TW_TYPE_FLOAT, &currentMaterial.subsurface, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "specular", TW_TYPE_FLOAT, &currentMaterial.specular, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "roughness", TW_TYPE_FLOAT, &currentMaterial.roughness, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "specularTint", TW_TYPE_FLOAT, &currentMaterial.specularTint, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "anisotropic", TW_TYPE_FLOAT, &currentMaterial.anisotropic, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "sheen", TW_TYPE_FLOAT, &currentMaterial.sheen, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "sheenTint", TW_TYPE_FLOAT, &currentMaterial.sheenTint, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "clearcoat", TW_TYPE_FLOAT, &currentMaterial.clearcoat, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "ccoatGloss", TW_TYPE_FLOAT, &currentMaterial.clearcoatGloss, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "transmission", TW_TYPE_FLOAT, &currentMaterial.transmission, "group='material' min=0 max=1 step=0.01");
	TwAddVarRW(settings, "eta", TW_TYPE_FLOAT, &currentMaterial.eta, "group='material' min=0 max=2 step=0.01");
	TwAddSeparator(settings, "separator2", "group='material'");
	TwStructMember float2Members[] = {
		{ "x", TW_TYPE_FLOAT, offsetof(float2, x), "" },
		{ "y", TW_TYPE_FLOAT, offsetof(float2, y), "" }
	};
	TwType float2Type = TwDefineStruct("float2", float2Members, 2, sizeof(float2), NULL, NULL);
	TwStructMember mapMembers[] = {
		{ "ID", TW_TYPE_INT32, offsetof(HostMaterial::MapProps, textureID), "" },
		{ "scale", float2Type, offsetof(HostMaterial::MapProps, uvscale), "" },
		{ "offset", float2Type, offsetof(HostMaterial::MapProps, uvoffset), "" }
	};
	TwType mapType = TwDefineStruct("Texture0", mapMembers, 3, sizeof(HostMaterial::MapProps), NULL, NULL);
	TwAddVarRW(settings, "difftex0", mapType, &currentMaterial.map[TEXTURE0], " group='material' ");
	TwAddVarRW(settings, "difftex1", mapType, &currentMaterial.map[TEXTURE1], " group='material' ");
	TwAddVarRW(settings, "difftex2", mapType, &currentMaterial.map[TEXTURE2], " group='material' ");
	TwAddVarRW(settings, "nrmlmap0", mapType, &currentMaterial.map[NORMALMAP0], " group='material' ");
	TwAddVarRW(settings, "nrmlmap1", mapType, &currentMaterial.map[NORMALMAP1], " group='material' ");
	TwAddVarRW(settings, "nrmlmap2", mapType, &currentMaterial.map[NORMALMAP2], " group='material' ");
	TwAddSeparator(settings, "separator3", "group='material'");
	TwSetParam(settings, "material", "opened", TW_PARAM_INT32, 1, &closed);
	// create collapsed camera block

	TwType float3Type = TwDefineStruct("float3", float3Members, 3, sizeof(float3), NULL, NULL);
	TwAddVarRO(settings, "position", float3Type, &renderer->GetCamera()->position, "group='camera'");
	TwAddVarRO(settings, "direction", float3Type, &renderer->GetCamera()->direction, "group='camera'");
	TwAddVarRW(settings, "FOV", TW_TYPE_FLOAT, &renderer->GetCamera()->FOV, "group='camera' min=10 max=99 step=1");
	TwAddVarRW(settings, "focaldist", TW_TYPE_FLOAT, &renderer->GetCamera()->focalDistance, "group='camera' min=0.1 max=100 step=0.01");
	TwAddVarRW(settings, "aperture", TW_TYPE_FLOAT, &renderer->GetCamera()->aperture, "group='camera' min=0 max=1 step=0.001");
	TwAddVarRW(settings, "brightness", TW_TYPE_FLOAT, &renderer->GetCamera()->brightness, "group='camera' min=-1 max=1 step=0.01");
	TwAddVarRW(settings, "contrast", TW_TYPE_FLOAT, &renderer->GetCamera()->contrast, "group='camera' min=-1 max=1 step=0.01");
	TwAddVarRW(settings, "clampValue", TW_TYPE_FLOAT, &renderer->GetCamera()->clampValue, "group='camera' min=1 max=100 step=1");
	TwSetParam(settings, "camera", "opened", TW_PARAM_INT32, 1, &closed);
	// create the renderer block
	TwAddVarRW(settings, "epsilon", TW_TYPE_FLOAT, &renderer->GetSettings()->geometryEpsilon, "group='renderer'");
	TwAddVarRW(settings, "maxDirect", TW_TYPE_FLOAT, &renderer->GetSettings()->filterDirectClamp, "group='renderer' min=1 max=50 step=0.5");
	TwAddVarRW(settings, "maxIndirect", TW_TYPE_FLOAT, &renderer->GetSettings()->filterIndirectClamp, "group='renderer' min=1 max=50 step=0.5");
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

	// create voxelgrid block
	TwType float3Type = TwDefineStruct("AABB", float3Members, 3, sizeof(float3), NULL, NULL);
	TwAddVarRO(menu, "AABB min", float3Type, &ui_nm_config.m_bmin, " group='voxelgrid'");
	TwAddVarRO(menu, "AABB max", float3Type, &ui_nm_config.m_bmax, " group='voxelgrid'");
	TwAddVarRO(menu, "cell size", TW_TYPE_FLOAT, &ui_nm_config.m_cs, " group='voxelgrid'");
	TwAddVarRO(menu, "cell height", TW_TYPE_FLOAT, &ui_nm_config.m_ch, " group='voxelgrid'");
	TwSetParam(menu, "voxelgrid", "opened", TW_PARAM_INT32, 1, &closed);

	// create agent block
	TwAddVarRO(menu, "max slope", TW_TYPE_FLOAT, &ui_nm_config.m_walkableSlopeAngle, " group='agent'");
	TwAddVarRO(menu, "min height", TW_TYPE_INT32, &ui_nm_config.m_walkableHeight, " group='agent'");
	TwAddVarRO(menu, "max climb", TW_TYPE_INT32, &ui_nm_config.m_walkableClimb, " group='agent'");
	TwAddVarRO(menu, "min radius", TW_TYPE_INT32, &ui_nm_config.m_walkableRadius, " group='agent'");
	TwSetParam(menu, "agent", "opened", TW_PARAM_INT32, 1, &closed);

	// create filtering block
	TwAddVarRO(menu, "low hanging obstacles", TW_TYPE_BOOL8, &ui_nm_config.m_filterLowHangingObstacles, " group='filtering'");
	TwAddVarRO(menu, "ledge spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterLedgeSpans, " group='filtering'");
	TwAddVarRO(menu, "low height spans", TW_TYPE_BOOL8, &ui_nm_config.m_filterWalkableLowHeightSpans, " group='filtering'");
	TwSetParam(menu, "filtering", "opened", TW_PARAM_INT32, 1, &closed);

	// create partitioning block
	TwEnumVal partitionEV[] = {
		{ SAMPLE_PARTITION_WATERSHED, "Watershed" },
		{ SAMPLE_PARTITION_MONOTONE, "Monotone" },
		{ SAMPLE_PARTITION_LAYERS, "Layers" }
	};
	TwType PartitionType = TwDefineEnum("PartitionType", partitionEV, 3);
	TwAddVarRO(menu, "partition type", PartitionType, &ui_nm_config.m_partitionType, " group='partitioning'");
	TwAddVarRO(menu, "min region area", TW_TYPE_INT32, &ui_nm_config.m_minRegionArea, " group='partitioning'");
	TwAddVarRO(menu, "min merged region", TW_TYPE_INT32, &ui_nm_config.m_mergeRegionArea, " group='partitioning'");
	TwSetParam(menu, "partitioning", "opened", TW_PARAM_INT32, 1, &closed);

	// create rasterization block
	TwAddVarRO(menu, "max edge length", TW_TYPE_INT32, &ui_nm_config.m_maxEdgeLen, " group='poly mesh'");
	TwAddVarRO(menu, "max simpl err", TW_TYPE_FLOAT, &ui_nm_config.m_maxSimplificationError, " group='poly mesh'");
	TwAddVarRO(menu, "max verts per poly", TW_TYPE_INT32, &ui_nm_config.m_maxVertsPerPoly, " group='poly mesh'");
	TwAddVarRO(menu, "detail sample dist", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleDist, " group='poly mesh'");
	TwAddVarRO(menu, "detail max err", TW_TYPE_FLOAT, &ui_nm_config.m_detailSampleMaxError, " group='poly mesh'");
	TwSetParam(menu, "poly mesh", "opened", TW_PARAM_INT32, 1, &closed);

	// create output block
	TwAddVarRO(menu, "NavMesh ID", TW_TYPE_STDSTRING, &ui_nm_config.m_id, " group='output'");
	TwAddVarRO(menu, "Print build stats", TW_TYPE_BOOL8, &ui_nm_config.m_printBuildStats, " group='output'");
	TwAddVarRO(menu, "Save navmesh", TW_TYPE_BOOL8, &ui_nm_save, " group='output'");
	TwSetParam(menu, "output", "opened", TW_PARAM_INT32, 1, &opened);

	TwAddButton(menu, "build", NULL, NULL, " label='Build' ");
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

GLTexture* digit[10], *hud;
Shader* plainShader = 0, *shadowShader = 0;
float smoothed = 1.0f, smoothFactor = 0.1f;

void InitFPSPrinter()
{
	// load digits
	for (int i = 0; i < 10; i++)
	{
		char t[128] = "data//system//digit0.png";
		t[strlen( t ) - 5] += i;
		digit[i] = new GLTexture( t, GL_LINEAR );
	}
	// load HUD
	hud = new GLTexture( "data//system//hud.png", GL_LINEAR );
	// load shaders
	plainShader = new Shader( "shaders/plain.vert", "shaders/plain.frag" );
	shadowShader = new Shader( "shaders/plain.vert", "shaders/plain_shadow.frag" );
}

void DrawDigit( int d, float x, float y, float scale = 1.0f )
{
	plainShader->SetInputTexture( 0, "color", digit[d] );
	mat4 T = mat4::Scale( make_float3( 0.06f * scale, 0.1f * scale, 1 ) );
	T.cell[12] = x, T.cell[13] = y;
	plainShader->SetInputMatrix( "view", T );
	DrawQuad();
}

void DrawHUD( float x, float y )
{
	plainShader->SetInputTexture( 0, "color", hud );
	float scale = 4.5f;
	mat4 T = mat4::Scale( scale * make_float3( 0.06f, 0.1f, 1 ) );
	T.cell[12] = x, T.cell[13] = y;
	plainShader->SetInputMatrix( "view", T );
	DrawQuad();
}

void PrintFPS( float deltaTime )
{
	float fps = (int)(1.0f / deltaTime);
	smoothed = (1 - smoothFactor) * smoothed + smoothFactor * fps;
	if (smoothFactor > 0.05f) smoothFactor -= 0.05f;
	int ifps = smoothed * 10, d1 = (ifps / 1000) % 10, d2 = (ifps / 100) % 10, d3 = (ifps / 10) % 10, d4 = ifps % 10;
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	shadowShader->Bind();
	float xpos = -0.91f, ypos = -0.81f;
	DrawDigit( d1, xpos, ypos ); xpos += 0.12f;
	DrawDigit( d2, xpos, ypos ); xpos += 0.12f;
	DrawDigit( d3, xpos, ypos ); xpos += 0.14f;
	DrawDigit( d4, xpos, ypos - 0.03f, 0.7f );
	shadowShader->Unbind();
	plainShader->Bind();
	xpos = -0.92f, ypos = -0.8f;
	DrawDigit( d1, xpos, ypos ); xpos += 0.12f;
	DrawDigit( d2, xpos, ypos ); xpos += 0.12f;
	DrawDigit( d3, xpos, ypos ); xpos += 0.14f;
	DrawDigit( d4, xpos, ypos - 0.03f, 0.7f );
	plainShader->Unbind();
	glDisable( GL_BLEND );
}

// EOF