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
#include "navmesh_builder.h"
#include "navmesh_navigator.h"

static RenderAPI* renderer = 0;
static GLTexture* renderTarget = 0;
static Shader* shader = 0;
static uint scrwidth = 0, scrheight = 0, scrspp = 1;
static bool camMoved = false, hasFocus = true, running = true;
static bool leftClicked = false, rightClicked = false;
static int2 probeCoords;
static CoreStats coreStats;

static NavMeshBuilder* navMeshBuilder = 0;
static NavMeshNavigator* navMeshNavigator = 0;

#include "main_ui.h"
#include "main_tools.h"

//  +-----------------------------------------------------------------------------+
//  |  PrepareScene                                                               |
//  |  Initialize a scene.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void PrepareScene()
{
	// initialize scene
	//renderer->AddScene( "scene.gltf", "data\\pica\\", mat4::Translate( 0, -10.2f, 0 ) );
	// renderer->AddScene( "CesiumMan.glb", "data\\", mat4::Translate( 0, -2, -9 ) );
	// renderer->AddScene( "InterpolationTest.glb", "data\\", mat4::Translate( 0, 2, -5 ) );
	// renderer->AddScene( "AnimatedMorphCube.glb", "data\\", mat4::Translate( 0, 2, 9 ) );
	int meshID = renderer->AddMesh("nav_test.obj", "data\\", 1.0f);
	int instID = renderer->AddInstance(meshID, mat4::Identity());
	int rootNode = renderer->FindNode( "RootNode (gltf orientation matrix)" );
	renderer->SetNodeTransform( rootNode, mat4::RotateX( -PI / 2 ) );
	int lightMat = renderer->AddMaterial( make_float3( 100, 100, 80 ) );
	int lightQuad = renderer->AddQuad( make_float3( 0, -1, 0 ), make_float3( 0, 26.0f, 0 ), 6.9f, 6.9f, lightMat );
	//renderer->AddInstance( lightQuad );
	renderer->AddDirectionalLight(make_float3(-1), make_float3(255));

	// Navmesh builder
	navMeshBuilder = new NavMeshBuilder("data\\ai\\");
	navMeshBuilder->GetConfig()->SetCellSize(.3f, .2f);
	navMeshBuilder->GetConfig()->SetAgentInfo(10.0f, 10, 2, 2);
	navMeshBuilder->GetConfig()->SetPolySettings(12, 1.3f, 8.0f, 20.0f, 6);
	navMeshBuilder->GetConfig()->SetDetailPolySettings(6.0f, 1.0f);
	navMeshBuilder->GetConfig()->m_printBuildStats = true;

	AI_UI::ui_nm_config = *navMeshBuilder->GetConfig();
	AI_UI::ui_nm_id = navMeshBuilder->GetConfig()->m_id;
}

//  +-----------------------------------------------------------------------------+
//  |  main                                                                       |
//  |  Application entry point.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
int main()
{
	// initialize OpenGL and ImGui
	InitGLFW();

	// initialize renderer: pick one
	// renderer = RenderAPI::CreateRenderAPI( "rendercore_optix7.dll" );			// OPTIX7 core, best for RTX devices
	renderer = RenderAPI::CreateRenderAPI( "rendercore_optixprime_b.dll" );			// OPTIX PRIME, best for pre-RTX CUDA devices
	// renderer = RenderAPI::CreateRenderAPI( "rendercore_primeref.dll" );			// REFERENCE, for image validation
	// renderer = RenderAPI::CreateRenderAPI( "rendercore_optixrtx_b.dll" );		// OPTIX6 core, for reference
	// renderer = RenderAPI::CreateRenderAPI( "rendercore_softrasterizer.dll" );	// RASTERIZER, your only option if not on NVidia

	renderer->DeserializeCamera( "camera.xml" );
	// initialize scene
	PrepareScene();
	// initialize ui
	AI_UI::InitGUI();
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
		if (camMoved) c = Restart, camMoved = false;
		// detect camera changes
		if (renderer->GetCamera()->Changed()) camMoved = true;
		// poll events, may affect probepos so needs to happen between HandleInput and Render
		glfwPollEvents();
		// update animations
		for( int i = 0; i < renderer->AnimationCount(); i++ )
		{
			renderer->UpdateAnimation( i, deltaTime );
			camMoved = true; // will remain false if scene has no animations
		}
		// render
		deltaTime = timer.elapsed();
		timer.reset();
		renderer->Render( c );
		coreStats = renderer->GetCoreStats();
		AI_UI::mraysincl = coreStats.totalRays / (coreStats.renderTime * 1000);
		AI_UI::mraysexcl = coreStats.totalRays / (coreStats.traceTime0 * 1000);
		if (AI_UI::HandleInput( deltaTime )) camMoved = true;
		// postprocess
		shader->Bind();
		shader->SetInputTexture( 0, "color", renderTarget );
		shader->SetInputMatrix( "view", mat4::Identity() );
		DrawQuad();
		shader->Unbind();
		// draw ui
		AI_UI::DrawNavMesh();
		TwDraw();
		AI_UI::PrintFPS(deltaTime);
		// finalize
		glfwSwapBuffers( window );
		// terminate
		if (!running) break;
	}
	// clean up
	renderer->SerializeCamera( "camera.xml" );
	renderer->Shutdown();
	glfwDestroyWindow( window );
	glfwTerminate();
	return 0;
}

// EOF