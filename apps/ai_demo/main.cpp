/* main.cpp - Copyright 2019 Utrecht University

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

#include "platform.h"
#include "system.h"
#include "rendersystem.h"
#include "recastnavigation.h"
#include "gui.h"

static RenderAPI* renderer = 0;
static GLTexture* renderTarget = 0;
static Shader* shader = 0;
static uint scrwidth = 0, scrheight = 0;
static bool running = true, hasFocus = true;
static bool leftButtonDown = false, leftClicked = false;
static bool sceneChanges = false;
static string materialFile;
static NavMeshBuilder navmesh("data\\ai");
static AI_DEMO_GUI* ai_demo_gui = 0;

#include "main_ui.h"
#include "main_tools.h"
#include "DetourNode.h"

static char navmesh_t[128] = "data//system//navmesh.png";
static GLTexture* navmeshtexture = 0;

int navmeshMeshID;
int navmeshInstanceID;

//  +-----------------------------------------------------------------------------+
//  |  PrepareGUI                                                                 |
//  |  Initialize the GUI.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void PrepareGUI()
{
	//ai_demo_gui = new AI_DEMO_GUI(renderer, 0);
}

//mat4 GetCameraMatrix()
//{
//	ViewPyramid view = renderer->GetCamera()->GetView();
//	mat4 translation = mat4::Translate(-view.pos);
//
//}

void DrawNode(float4 pos, float scale)
{
	mat4 T = mat4::Scale(make_float3(scale, scale, 1));
	shader->SetInputTexture(0, "color", navmeshtexture);
	T.cell[12] = pos.x, T.cell[13] = pos.y;
	shader->SetInputMatrix("view", T);
	DrawQuad();
}

void DrawNavMesh()
{
	return; // DEBUG
	navmeshtexture = new GLTexture(navmesh_t, GL_LINEAR);
	float scale = .01f;
	float x = 0.0f, y = 0.0f;

	mat4 view, projection;
	float4 world_pos;
	glGetFloatv(GL_MODELVIEW_MATRIX, view.cell);
	glGetFloatv(GL_PROJECTION_MATRIX, projection.cell);

	Camera* camera = renderer->GetCamera();

	//y = make_float3(0, 1, 0);
	//z = direction; // assumed to be normalized at all times
	//x = normalize(cross(z, y));
	//y = cross(x, z);

	shader->Bind();

	//const dtNodePool* pool = navmesh.GetQuery()->getNodePool();
	//if (pool)
	//{
	//	const float off = 0.5f;
	//	for (int i = 0; i < pool->getHashSize(); ++i)
	//		for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
	//		{
	//			const dtNode* node = pool->getNodeAtIdx(j + 1);
	//			if (!node) continue;
	//			world_pos = make_float4(node->pos[0], node->pos[1] + off, node->pos[2], 1.0f);
	//			DrawNode(world_pos * (view * projection), scale);
	//		}
	//
	//	for (int i = 0; i < pool->getHashSize(); ++i)
	//		for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
	//		{
	//			const dtNode* node = pool->getNodeAtIdx(j + 1);
	//			if (!node) continue;
	//			if (!node->pidx) continue;
	//			const dtNode* parent = pool->getNodeAtIdx(node->pidx);
	//			if (!parent) continue;
	//			world_pos = make_float4(node->pos[0], node->pos[1] + off, node->pos[2], 1.0f);
	//			DrawNode(world_pos * (view * projection), scale);
	//			world_pos = make_float4(parent->pos[0], parent->pos[1] + off, parent->pos[2], 1.0f);
	//			DrawNode(world_pos * (view * projection), scale);
	//		}
	//}

	const dtNavMesh* mesh = navmesh.GetMesh();
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile->header) continue;
		for (int j = 0; j < tile->header->vertCount; ++j)
		{
			const float* v = &tile->verts[j * 3];
			world_pos = make_float4(v[0], v[1], v[2], 1.0f);
			DrawNode(world_pos * (view * projection), scale);
		}
	}

	shader->Unbind();
}

//  +-----------------------------------------------------------------------------+
//  |  PrepareNavmesh                                                             |
//  |  Initialize the navmesh.                                              LH2'19|
//  +-----------------------------------------------------------------------------+
void PrepareNavmesh()
{
	navmesh.GetConfig()->SetCellSize(.2f, .2f);
	navmesh.GetConfig()->SetPolySettings(100, 1.0f, 10.0f, 20.0f, 6);
	navmesh.GetConfig()->SetAgentInfo(40.0f, 100, 10, 1);
	navmesh.SetID("tritest");
	//navmesh.Deserialize();
	//navmesh.SetID("testload");
	navmesh.Build(renderer->GetHostScene());
	navmesh.Serialize();
	navmesh.SaveAsMesh();
	navmesh.DumpLog();

	//ai_demo_gui->AddNodesToScene(&navmesh);

	navmeshMeshID = renderer->AddMesh("tritest.obj", "data\\ai\\", 1.0f);
	navmeshInstanceID = renderer->AddInstance(navmeshMeshID, mat4::Identity());// mat4::Translate(0.0f, -10.0f, 0.0f));
}

//  +-----------------------------------------------------------------------------+
//  |  PrepareScene                                                               |
//  |  Initialize a scene.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void PrepareScene()
{
	int meshID = renderer->AddMesh("nav_test.obj", "data\\", 1.0f);
	int instID = renderer->AddInstance(meshID, mat4::Identity());

	// initialize scene
	//renderer->AddScene("scene.gltf", "data\\pica\\", mat4::Translate(0, -10.2f, 0));
	// renderer->AddScene( "CesiumMan.glb", "data\\", mat4::Translate( 0, -2, -9 ) );
	// renderer->AddScene( "InterpolationTest.glb", "data\\", mat4::Translate( 0, 2, -5 ) );
	// renderer->AddScene( "AnimatedMorphCube.glb", "data\\", mat4::Translate( 0, 2, 9 ) );
	int rootNode = renderer->FindNode("RootNode (gltf orientation matrix)");
	renderer->SetNodeTransform(rootNode, mat4::RotateX(-PI / 2));
	int lightMat = renderer->AddMaterial(make_float3(100, 100, 80));
	int lightQuad = renderer->AddQuad(make_float3(0, -1, 0), make_float3(0, 26.0f, 0), 6.9f, 6.9f, lightMat);
	//renderer->AddInstance(lightQuad);

	renderer->AddDirectionalLight(make_float3(.5, .5, .5), make_float3(255.0f));
	renderer->AddPointLight(make_float3(0, 26.0f, 0), make_float3(255.0f));
}

//  +-----------------------------------------------------------------------------+
//  |  HandleInput                                                                |
//  |  Process user input.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
bool HandleInput( float frameTime )
{
	if (!hasFocus) return false;
	// handle keyboard input
	float translateSpeed = (GetAsyncKeyState( VK_SHIFT ) ? 15.0f : 5.0f) * frameTime, rotateSpeed = 2.5f * frameTime;
	bool changed = false;
	Camera* camera = renderer->GetCamera();
	if (GetAsyncKeyState( 'A' )) { changed = true; camera->TranslateRelative( make_float3( -translateSpeed, 0, 0 ) ); }
	if (GetAsyncKeyState( 'D' )) { changed = true; camera->TranslateRelative( make_float3( translateSpeed, 0, 0 ) ); }
	if (GetAsyncKeyState( 'W' )) { changed = true; camera->TranslateRelative( make_float3( 0, 0, translateSpeed ) ); }
	if (GetAsyncKeyState( 'S' )) { changed = true; camera->TranslateRelative( make_float3( 0, 0, -translateSpeed ) ); }
	if (GetAsyncKeyState( 'R' )) { changed = true; camera->TranslateRelative( make_float3( 0, translateSpeed, 0 ) ); }
	if (GetAsyncKeyState( 'F' )) { changed = true; camera->TranslateRelative( make_float3( 0, -translateSpeed, 0 ) ); }
	if (GetAsyncKeyState( 'B' )) changed = true; // force restart
	if (GetAsyncKeyState( VK_UP )) { changed = true; camera->TranslateTarget( make_float3( 0, -rotateSpeed, 0 ) ); }
	if (GetAsyncKeyState( VK_DOWN )) { changed = true; camera->TranslateTarget( make_float3( 0, rotateSpeed, 0 ) ); }
	if (GetAsyncKeyState( VK_LEFT )) { changed = true; camera->TranslateTarget( make_float3( -rotateSpeed, 0, 0 ) ); }
	if (GetAsyncKeyState( VK_RIGHT )) { changed = true; camera->TranslateTarget( make_float3( rotateSpeed, 0, 0 ) ); }
	// process left button click
	if (leftClicked && GetAsyncKeyState( VK_LSHIFT ))
	{
		int selectedMaterialID = renderer->GetTriangleMaterialID( coreStats.probedInstid, coreStats.probedTriid );
		if (selectedMaterialID != -1)
		{
			currentMaterial = *renderer->GetMaterial( selectedMaterialID );
			currentMaterialID = selectedMaterialID;
			currentMaterial.Changed(); // update checksum so we can track changes
		}
		camera->focalDistance = coreStats.probedDist;
		changed = true;
		leftClicked = false;
	}
	// let the main loop know if the camera should update
	return changed;
}

//  +-----------------------------------------------------------------------------+
//  |  HandleMaterialChange                                                       |
//  |  Update a scene material based on AntTweakBar.                        LH2'19|
//  +-----------------------------------------------------------------------------+
void HandleMaterialChange()
{
	if (currentMaterialConductor) currentMaterial.flags |= HostMaterial::ISCONDUCTOR;
	else currentMaterial.flags &= ~HostMaterial::ISCONDUCTOR;
	if (currentMaterialDielectric) currentMaterial.flags |= HostMaterial::ISDIELECTRIC;
	else currentMaterial.flags &= ~HostMaterial::ISDIELECTRIC;
	if (currentMaterial.Changed() && currentMaterialID != -1)
	{
		// put it back
		*renderer->GetMaterial( currentMaterialID ) = currentMaterial;
		renderer->GetMaterial( currentMaterialID )->MarkAsDirty();
		sceneChanges = true;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  main                                                                       |
//  |  Application entry point.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
int main()
{
	// initialize OpenGL
	InitGLFW();

	// initialize renderer: pick one
	// renderer = RenderAPI::CreateRenderAPI( "rendercore_optix7.dll" );				// OPTIX7 core, best for RTX devices
	renderer = RenderAPI::CreateRenderAPI( "rendercore_optixprime_b.dll" );		// OPTIX PRIME, best for pre-RTX CUDA devices
	// renderer = RenderAPI::CreateRenderAPI( "rendercore_optixrtx_b.dll" );		// OPTIX6 core, for reference
	// renderer = RenderAPI::CreateRenderAPI( "rendercore_softrasterizer.dll" );	// RASTERIZER, your only option if not on NVidia

	renderer->DeserializeCamera( "camera.xml" );
	// initialize ui
	InitAntTweakBar();
	InitFPSPrinter();
	// Initialize GUI
	PrepareGUI();
	// initialize scene
	PrepareScene();
	// Create navmesh
	PrepareNavmesh();
	// set initial window size
	ReshapeWindowCallback( 0, SCRWIDTH, SCRHEIGHT );
	// enter main loop
	Timer timer;
	timer.reset();
	float deltaTime = 0;
	while (!glfwWindowShouldClose( window ))
	{
		renderer->SynchronizeSceneData();
		Convergence c = Converge;
		if (sceneChanges) c = Restart, sceneChanges = false;
		// handle material changes
		HandleMaterialChange();
		// detect camera changes
		if (renderer->GetCamera()->Changed()) sceneChanges = true;
		// poll events, may affect probepos so needs to happen between HandleInput and Render
		glfwPollEvents();
		// render
		deltaTime = timer.elapsed();
		timer.reset();
		renderer->Render( c );
		coreStats = renderer->GetCoreStats();
		mraysincl = coreStats.totalRays / (coreStats.renderTime * 1000);
		mraysexcl = coreStats.totalRays / (coreStats.traceTime0 * 1000);
		if (HandleInput( deltaTime )) sceneChanges = true;
		// finalize and present
		shader->Bind();
		shader->SetInputTexture( 0, "color", renderTarget );
		shader->SetInputMatrix( "view", mat4::Identity() );
		DrawQuad();
		shader->Unbind();
		// draw navmesh
		DrawNavMesh();
		// draw ui
		TwDraw();
		PrintFPS( deltaTime );
		// finalize
		glfwSwapBuffers( window );
		if (!running) break;
	}
	// save camera
	renderer->SerializeCamera( "camera.xml" );
	// save material changes
	renderer->SerializeMaterials( materialFile.c_str() );
	// clean up
	renderer->Shutdown();
	glfwDestroyWindow( window );
	glfwTerminate();
	return 0;
}

// EOF