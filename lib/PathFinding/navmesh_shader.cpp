/* navmesh_shader.cpp - Copyright 2019 Utrecht University
   
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

#include "DetourNode.h"

#include "platform.h"       // DrawLineStrip, DrawLines
#include "navmesh_shader.h"

namespace lighthouse2 {

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::UpdateMesh                                                  |
//  |  Removes the old navmesh assets and adds the new one.                 LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::UpdateMesh(NavMeshNavigator* navmesh)
{
	Clean();
	m_meshFileName = ".tmp." + std::string(navmesh->GetID()) + ".obj";
	ExtractVertsAndEdges(navmesh->GetDetourMesh());
	SaveAsMesh(navmesh);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::UpdateAgentPositions                                        |
//  |  Informs the renderer of the agent's new positions.                   LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::UpdateAgentPositions()
{
	for (std::vector<ShaderAgent>::iterator it = m_agents.begin(); it != m_agents.end(); it++)
		m_renderer->SetNodeTransform(it->instID, it->agent->GetTransform());
	m_renderer->SynchronizeSceneData();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawGL                                                      |
//  |  Draws all applicable GL shapes on the window.                        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawGL() const
{
	if (m_shadeVerts) ShadeVertsGL();
	if (m_shadeEdges) ShadeEdgesGL();
	if (m_shadeTris) ShadePolysGL();

	if (m_polySelect) DrawPolyHighlightGL();
	if (m_vertSelect) DrawVertHighlightGL();
	if (m_edgeSelect) DrawEdgeHighlightGL();
	if (m_agentSelect) DrawAgentHighlightGL();
	if (m_agentSelect) DrawAgentImpulse();

	if (m_path && !m_path->empty()) PlotPath();
	if (m_pathStart || m_pathEnd) DrawPathMarkers();
}





//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddNavMeshToScene                                           |
//  |  Adds all navmesh assets to the scene.                                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddNavMeshToScene()
{
	printf("Adding NavMesh assets to scene...\n");
	Timer timer;
	AddPolysToScene();
	AddVertsToScene();
	AddEdgesToScene();
	AddOMCsToScene();
	printf("NavMesh assets added in %.3fms\n", timer.elapsed());
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveNavMeshFromScene                                      |
//  |  Removes all navmesh assets from the scene.                           LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveNavMeshFromScene()
{
	RemovePolysFromScene();
	RemoveVertsFromScene();
	RemoveEdgesFromScene();
	RemoveOMCsFromScene();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddNavMeshToScene                                           |
//  |  Adds the navmesh triangles to the scene as a single mesh.            LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddPolysToScene()
{
	m_polyMeshID = m_renderer->AddMesh(m_meshFileName.c_str(), m_dir, 1.0f);
	m_renderer->GetMesh(m_polyMeshID)->name = "NavMesh";
	m_polyInstID = m_renderer->AddInstance(m_polyMeshID, mat4::Identity());
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveNavMeshFromScene                                      |
//  |  Removes the navmesh instance and mesh from the scene.                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemovePolysFromScene()
{
	if (m_polyInstID >= 0) m_renderer->RemoveInstance(m_polyInstID);
	m_polyInstID = -1;
	// TODO: Remove the old navmesh mesh to prevent memory leaks
	//if (m_navmeshMeshID >= 0) m_renderer->RemoveMesh(m_navmeshMeshID);
	//m_navmeshMeshID = -1;
	m_renderer->SynchronizeSceneData();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddNodesToScene                                             |
//  |  Adds all precomputed navmesh vertices as spheres to the scene.       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddVertsToScene()
{
	for (std::vector<Vert>::iterator i = m_verts.begin(); i != m_verts.end(); i++)
		i->instID = m_renderer->AddInstance(m_vertMeshID, mat4::Translate(*i->pos) * mat4::Scale(m_vertWidth));
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveVertsFromScene                                        |
//  |  Removes all vertex instances from the scene.                         LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveVertsFromScene()
{
	for (std::vector<Vert>::iterator i = m_verts.begin(); i != m_verts.end(); i++)
	{
		if (i->instID >= 0) m_renderer->RemoveInstance(i->instID);
		i->instID = -1;
	}
	m_renderer->SynchronizeSceneData();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddEdgesToScene                                             |
//  |  Adds all precomputed navmesh edges as cylinders to the scene.        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddEdgesToScene()
{
	for (std::vector<Edge>::iterator i = m_edges.begin(); i != m_edges.end(); i++)
	{
		float3 v1 = *m_verts[i->v1].pos, v2 = *m_verts[i->v2].pos;
		float3 v1v2 = v2 - v1; float len = length(v1v2); v1v2 /= len;
		mat4 sca = mat4::Scale(make_float3(m_edgeWidth, len, m_edgeWidth));
		mat4 rot = mat4::Rotate(cross({ 0, 1, 0 }, v1v2), -acosf(v1v2.y));
		mat4 tra = mat4::Translate((v1 + v2) / 2);
		i->instID = m_renderer->AddInstance(m_edgeMeshID, tra * rot * sca);
	}
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveEdgesFromScene                                        |
//  |  Removes all edge instances from the scene.                           LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveEdgesFromScene()
{
	for (std::vector<Edge>::iterator i = m_edges.begin(); i != m_edges.end(); i++)
	{
		if (i->instID >= 0) m_renderer->RemoveInstance(i->instID);
		i->instID = -1;
	}
	m_renderer->SynchronizeSceneData();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddOMCsToScene                                              |
//  |  Adds all off-mesh connections as cylinders/arrows to the scene.      LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddOMCsToScene()
{
	for (std::vector<OMC>::iterator i = m_OMCs.begin(); i != m_OMCs.end(); i++)
	{
		// Add the edge
		float3 v1 = *(float3*)i->omc->pos, v2 = *(float3*)(i->omc->pos + 3);
		float3 v1v2 = v2 - v1; float len = length(v1v2); v1v2 /= len;
		mat4 sca = mat4::Scale(make_float3(m_edgeWidth, len, m_edgeWidth));
		mat4 rot = mat4::Rotate(cross({ 0, 1, 0 }, v1v2), -acosf(v1v2.y));
		mat4 tra = mat4::Translate((v1 + v2) / 2);
		int meshID = (true ? m_directedEdgeMeshID : m_edgeMeshID); // TODO: replace 'true' with a check for bidirectionality
		i->edgeInstID = m_renderer->AddInstance(meshID, tra * rot * sca);

		// Add the two verts
		i->v1InstID = m_renderer->AddInstance(m_vertMeshID, mat4::Translate(v1) * mat4::Scale(i->omc->rad));
		i->v2InstID = m_renderer->AddInstance(m_vertMeshID, mat4::Translate(v2) * mat4::Scale(i->omc->rad));
	}
	m_renderer->SynchronizeSceneData();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveOMCsFromScene                                         |
//  |  Removes all off-mesh connection instances from the scene.            LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveOMCsFromScene()
{
	for (std::vector<OMC>::iterator i = m_OMCs.begin(); i != m_OMCs.end(); i++)
	{
		if (i->v1InstID >= 0) m_renderer->RemoveInstance(i->v1InstID);
		if (i->v2InstID >= 0) m_renderer->RemoveInstance(i->v2InstID);
		if (i->edgeInstID >= 0) m_renderer->RemoveInstance(i->edgeInstID);
		i->v1InstID = -1;
		i->v2InstID = -1;
		i->edgeInstID = -1;
	}
	m_renderer->SynchronizeSceneData();
}




//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ShadeTrianglesGL (TODO)                                     |
//  |  Plots all navmesh triangles using GL.                                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ShadePolysGL() const
{
	
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ShadeVertsGL                                                |
//  |  Plots all precomputed navmesh vertices using GL.                     LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ShadeVertsGL() const
{
	int count = (int)m_verts.size();
	std::vector<float3> world(count);
	std::vector<float2> screen(count);
	std::vector<float4> colors(count, m_vertColor);
	for (int i = 0; i < count; i++) world[i] = *m_verts[i].pos;
	m_renderer->GetCamera()->WorldToScreenPos(world.data(), screen.data(), count);
	DrawShapeOnScreen(screen, colors, GL_POINTS, m_vertWidthGL);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ShadeEdgesGL                                                |
//  |  Plots all precomputed polygon edges using GL.                        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ShadeEdgesGL() const
{
	int count = (int)m_edges.size();
	std::vector<float3> world(count * 2);
	std::vector<float2> screen(count * 2);
	std::vector<float4> colors(count * 2, m_edgeColor);
	for (int i = 0; i < count; i++)
	{
		world[i * 2]     = *m_verts[m_edges[i].v1].pos;
		world[i * 2 + 1] = *m_verts[m_edges[i].v2].pos;
	}
	m_renderer->GetCamera()->WorldToScreenPos(world.data(), screen.data(), count * 2);
	DrawShapeOnScreen(screen, colors, GL_LINES, m_edgeWidthGL);
}




//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddAgentToScene                                             |
//  |  Places an agent at the given position.                               LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddAgentToScene(Agent* agent)
{
	int instID = m_renderer->AddInstance(m_agentMeshID, agent->GetTransform());
	m_agents.push_back({ instID, agent });
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveAgent                                                 |
//  |  Removes an individual agent from the scene.                          LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveAgentFromScene(Agent* agent)
{
	for (std::vector<ShaderAgent>::iterator i = m_agents.begin(); i != m_agents.end(); i++)
		if (i->agent == agent)
		{
			if (i->instID >= 0) m_renderer->RemoveInstance(i->instID);
			m_renderer->SynchronizeSceneData();
			i->instID = -1;
			m_agents.erase(i);
			return;
		}
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveAllAgents                                             |
//  |  Removes all agents from the scene.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveAllAgents()
{
	for (std::vector<ShaderAgent>::iterator i = m_agents.begin(); i != m_agents.end(); i++)
	{
		if (i->instID >= 0) m_renderer->RemoveInstance(i->instID);
		i->instID = -1;
	}
	m_agents.clear();
	m_renderer->SynchronizeSceneData();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawAgentImpulse                                            |
//  |  Draws a line indicating the agent's movement.                        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawAgentImpulse() const
{
	if (!m_agentSelect) return;
	const float4 col = { .1f, .9f, .1f, 1.0f };
	const RigidBody* rb = m_agentSelect->agent->GetRB();
	float3 world[2] = { rb->m_pos, rb->m_pos + rb->m_impulse * 10.0f };
	std::vector<float4> colors{ col, col };
	std::vector<float2> screen(2);
	m_renderer->GetCamera()->WorldToScreenPos(world, screen.data(), 2);
	DrawShapeOnScreen(screen, colors, GL_LINES, 5.0f);
}




//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SelectPoly                                                  |
//  |  Highlights a polygon given a world position and a navmesh navigator. LH2'19|
//  +-----------------------------------------------------------------------------+
const dtPoly* NavMeshShader::SelectPoly(float3 pos, NavMeshNavigator* navmesh)
{
	Deselect();
	if (!navmesh) { m_polySelect = 0; return 0; }
	dtPolyRef ref;
	float3 posOnPoly;
	navmesh->FindNearestPoly(pos, ref, posOnPoly);
	return m_polySelect = navmesh->GetPoly(ref);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawPolyHighlightGL                                         |
//  |  Draws a highlighted polygon on the screen.                           LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawPolyHighlightGL() const
{
	std::vector<float3> world;
	std::vector<float2> screen(m_polySelect->vertCount);
	std::vector<float4> colors(m_polySelect->vertCount, m_highLightColor);
	for (int i = 0; i < m_polySelect->vertCount; i++)
		world.push_back(*m_verts[m_polySelect->verts[i]].pos);
	m_renderer->GetCamera()->WorldToScreenPos(world.data(), screen.data(), (int)world.size());

	DrawShapeOnScreen(screen, colors, GL_TRIANGLE_FAN);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SelectVert                                                  |
//  |  Hightlights a vertex instance.                                       LH2'19|
//  +-----------------------------------------------------------------------------+
NavMeshShader::Vert* NavMeshShader::SelectVert(int instanceID)
{
	Deselect();
	if (instanceID < 0) return 0;
	for (size_t i = 0; i < m_verts.size(); i++)
		if (m_verts[i].instID == instanceID) return m_vertSelect = &m_verts[i];
	return 0;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawVertHighlightGL                                         |
//  |  Draws a highlighted vertex on the screen.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawVertHighlightGL() const
{
	std::vector<float2> vertices(1);
	m_renderer->GetCamera()->WorldToScreenPos(m_vertSelect->pos, vertices.data(), 1);
	std::vector<float4> colors(1, m_highLightColor);

	DrawShapeOnScreen(vertices, colors, GL_POINTS, m_vertWidthGL);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SelectEdge                                                  |
//  |  Hightlights an edge instance.                                        LH2'19|
//  +-----------------------------------------------------------------------------+
NavMeshShader::Edge* NavMeshShader::SelectEdge(int instanceID)
{
	Deselect();
	if (instanceID < 0) return 0;
	for (size_t i = 0; i < m_edges.size(); i++)
		if (m_edges[i].instID == instanceID) return m_edgeSelect = &m_edges[i];
	return 0;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawEdgeHighlightGL                                         |
//  |  Draws a highlighted edge on the screen.                              LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawEdgeHighlightGL() const
{
	std::vector<float2> screen(2);
	std::vector<float4> colors(2, m_highLightColor);
	const float3 world[2] = { *m_verts[m_edgeSelect->v1].pos , *m_verts[m_edgeSelect->v2].pos };
	m_renderer->GetCamera()->WorldToScreenPos(world, screen.data(), 2);
	DrawShapeOnScreen(screen, colors, GL_LINE_STRIP, m_edgeWidthGL);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SelectAgent                                                 |
//  |  Hightlights an agent instance.                                       LH2'19|
//  +-----------------------------------------------------------------------------+
Agent* NavMeshShader::SelectAgent(int instanceID)
{
	Deselect();
	if (instanceID < 0) return 0;
	for (size_t i = 0; i < m_agents.size(); i++)
		if (m_agents[i].instID == instanceID) return (m_agentSelect = &m_agents[i])->agent;
	return 0;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawAgentHighlightGL                                        |
//  |  Draws a highlighted agent on the screen.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawAgentHighlightGL() const
{
	mat4 transform = m_agentSelect->agent->GetTransform();
	std::vector<float3> world(11);
	std::vector<float4> colors(11, m_highLightColor);
	std::vector<float2> screen(11);
	world[0] = make_float3(transform * float4{ 0, 0, 0, 1.0f });
	world[1] = make_float3(transform * float4{  .5f,  .5f, 0, 1.0f });
	world[2] = make_float3(transform * float4{ -.5f,  .5f, 0, 1.0f });
	world[3] = make_float3(transform * float4{  .5f, -.5f, 0, 1.0f });
	world[4] = make_float3(transform * float4{ -.5f, -.5f, 0, 1.0f });
	world[5] = make_float3(transform * float4{ 0, 0, 0, 1.0f });
	world[6] = make_float3(transform * float4{ 0,  .5f,  .5f, 1.0f });
	world[7] = make_float3(transform * float4{ 0,  .5f, -.5f, 1.0f });
	world[8] = make_float3(transform * float4{ 0, -.5f,  .5f, 1.0f });
	world[9] = make_float3(transform * float4{ 0, -.5f, -.5f, 1.0f });
	world[10] = make_float3(transform * float4{ 0, 0, 0, 1.0f });

	m_renderer->GetCamera()->WorldToScreenPos(world.data(), screen.data(), 11);
	DrawShapeOnScreen(screen, colors, GL_LINE_LOOP, 5.0f);
}



//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::PlotPath                                                    |
//  |  Plots the path calculated before as a series of lines                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::PlotPath() const
{
	std::vector<float3> world(m_path->size());
	std::vector<float2> screen(m_path->size());
	std::vector<float4> colors(m_path->size(), m_pathColor);
	for (size_t i = 0; i < m_path->size(); i++)
		world[i] = m_path->at(i).pos;

	m_renderer->GetCamera()->WorldToScreenPos(world.data(), screen.data(), (int)m_path->size());
	DrawShapeOnScreen(screen, colors, GL_LINE_STRIP, m_pathWidth);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawPathMarkers                                             |
//  |  Draws the start and end of the path as beacons.                      LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawPathMarkers() const
{
	if (!m_pathStart && !m_pathEnd) return;
	std::vector<float3> world;
	std::vector<float4> color;
	if (m_pathStart)
	{
		world.push_back(*m_pathStart);
		world.push_back(*m_pathStart + m_beaconLen);
		color.push_back(m_beaconStColor);
		color.push_back(float4());
	}
	if (m_pathEnd)
	{
		world.push_back(*m_pathEnd);
		world.push_back(*m_pathEnd + m_beaconLen);
		color.push_back(m_beaconEnColor);
		color.push_back(float4());
	}
	std::vector<float2> screen(world.size());
	m_renderer->GetCamera()->WorldToScreenPos(world.data(), screen.data(), (int)world.size());
	DrawShapeOnScreen(screen, color, GL_LINES, m_beaconWidth);
}





//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SetTmpVert                                                  |
//  |  Adds a temporary vertex to the scene.                                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::SetTmpVert(float3 pos, float width)
{
	RemoveTmpVert();
	m_tmpVert.pos = new float3(pos);
	m_tmpVert.instID = m_renderer->AddInstance(m_vertMeshID, mat4::Translate(pos) * mat4::Scale(width));
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveTMPVert                                               |
//  |  Removes the temporary vertex from the scene.                         LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveTmpVert()
{
	if (m_tmpVert.instID >= 0)
	{
		m_renderer->RemoveInstance(m_tmpVert.instID);
		m_renderer->SynchronizeSceneData();
		m_tmpVert.instID = -1;
		delete m_tmpVert.pos; // locally owned
		m_tmpVert.pos = 0;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddTmpOMC                                                   |
//  |  Adds a temporary off-mesh connection during runtime.                       |
//  |  *omc* should be a permanent pointer from the dtNavMesh object.       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddTmpOMC(float3 v0, float3 v1, float width)
{
	m_OMCs.push_back({ 0 });
	OMC* newOMC = m_OMCs._Mylast();

	// Add the edge
	float3 v1v2 = v1 - v0; float len = length(v1v2); v1v2 /= len;
	mat4 sca = mat4::Scale(make_float3(m_edgeWidth, len, m_edgeWidth));
	mat4 rot = mat4::Rotate(cross({ 0, 1, 0 }, v1v2), -acosf(v1v2.y));
	mat4 tra = mat4::Translate((v0 + v1) / 2);
	int meshID = (true ? m_directedEdgeMeshID : m_edgeMeshID); // TODO: replace 'true' with a check for bidirectionality
	newOMC->edgeInstID = m_renderer->AddInstance(meshID, tra * rot * sca);

	// Add the two verts
	newOMC->v1InstID = m_renderer->AddInstance(m_vertMeshID, mat4::Translate(v0) * mat4::Scale(width));
	newOMC->v2InstID = m_renderer->AddInstance(m_vertMeshID, mat4::Translate(v1) * mat4::Scale(width));

	m_renderer->SynchronizeSceneData();
}




//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::Clean                                                       |
//  |  Resets the internal NavMesh representation and stops shading it.     LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::Clean()
{
	RemoveNavMeshFromScene();
	RemoveNavMeshFromGL();
	RemoveAllAgents();
	RemoveTmpVert();

	Deselect();
	m_verts.clear();
	m_edges.clear();
	m_OMCs.clear();

	m_path = 0;
	m_pathStart = m_pathEnd = 0;

	RemoveFile((m_dir + m_meshFileName).c_str());
	m_meshFileName = "";
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddEdgeToEdgesAndPreventDuplicates                          |
//  |  Helper function for NavMeshShader::ExtractVertsAndEdges.             LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddEdgeToEdgesAndPreventDuplicates(int v1, int v2, const dtPoly* poly)
{
	for (std::vector<Edge>::iterator i = m_edges.begin(); i != m_edges.end(); i++)
		if ((i->v1 == v1 && i->v2 == v2) || (i->v1 == v2 && i->v2 == v1))
		{
			i->poly2 = poly; // edge already exists, so add the second poly
			return;
		}
	m_edges.push_back({ v1, v2, (int)m_edges.size(), -1, poly }); // add a new edge
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ExtractVertsAndEdges                                        |
//  |  Makes a list of all vertices and edges to aid shading.               LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ExtractVertsAndEdges(const dtNavMesh* mesh)
{
	int totalVerts = 0;
	for (int a = 0; a < mesh->getMaxTiles(); a++) if (mesh->getTile(a)->header)
	{
		dtMeshHeader* header = mesh->getTile(a)->header;
		totalVerts += header->vertCount + header->detailVertCount + header->offMeshConCount * 2;
	}
	m_verts.reserve(totalVerts); // total amount of verts
	m_edges.reserve(totalVerts); // at least this many edges

	int tileBaseIdx = 0, nVerts, nDetail, nOMC;
	for (int a = 0; a < mesh->getMaxTiles(); a++) if (mesh->getTile(a)->header)
	{
		const dtMeshTile* tile = mesh->getTile(a);
		nVerts = tile->header->vertCount;
		nDetail = tile->header->detailVertCount;
		nOMC = tile->header->offMeshConCount;

		// Adding Verts and their positions
		for (int i = 0; i < nVerts; ++i)
		{
			const float* v = &tile->verts[i * 3];
			m_verts.push_back({ (float3*)v, tileBaseIdx + i });
		}
		for (int i = 0; i < nDetail; ++i)
		{
			const float* v = &tile->detailVerts[i * 3];
			m_verts.push_back({ (float3*)v, tileBaseIdx + nVerts + i });
		}


		// Adding Vert polygon associations and Edges
		for (int b = 0; b < tile->header->polyCount; b++)
		{
			const dtPoly* poly = &tile->polys[b];
			if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;

			for (int c = 0; c < poly->vertCount; c++) // for every poly vert
			{
				m_verts[tileBaseIdx + poly->verts[c]].polys.push_back(Poly{ poly });
				if (c < poly->vertCount-1) // adding the first n-1 edges
					AddEdgeToEdgesAndPreventDuplicates(tileBaseIdx + poly->verts[c], tileBaseIdx + poly->verts[c + 1], poly);
				else // adding the last edge connecting the first vertex
					AddEdgeToEdgesAndPreventDuplicates(tileBaseIdx + poly->verts[c], tileBaseIdx + poly->verts[0], poly);
			}
		}

		// Adding off-mesh connections
		Poly omcPoly;
		int v1, v2;
		for (int j = 0; j < nOMC; j++)
		{
			const dtOffMeshConnection* omc = &tile->offMeshCons[j];
			omcPoly.omc = omc;
			v1 = tileBaseIdx + nVerts + nDetail + (j * 2);
			v2 = tileBaseIdx + nVerts + nDetail + (j * 2) + 1;
			m_verts.push_back({ (float3*)omc->pos, v1 });
			m_verts.back().polys.push_back(omcPoly);
			m_verts.push_back({ (float3*)(omc->pos+3), v2 });
			m_verts.back().polys.push_back(omcPoly);
			m_edges.push_back({ v1, v2, (int)m_edges.size(), -1 });
		}

		m_vertOffsets.push_back(int3{ tileBaseIdx, tileBaseIdx + nVerts, tileBaseIdx + nVerts + nDetail });
		tileBaseIdx += nVerts + nDetail + nOMC * 2;
	}
}





//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::WriteTileToMesh                                             |
//  |  Writes the given navmesh tile to the given open file.                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::WriteTileToMesh(const dtMeshTile* tile, FILE* f)
{
	// Writing vertices
	int vertCount = 0;
	for (int i = 0; i < tile->header->vertCount; ++i)
	{
		const float* v = &tile->verts[i * 3];
		fprintf(f, "v %.5f %.5f %.5f\n", v[0], v[1], v[2]);
		vertCount++;
	}
	for (int i = 0; i < tile->header->detailVertCount; ++i)
	{
		const float* v = &tile->detailVerts[i * 3];
		fprintf(f, "v %.5f %.5f %.5f\n", v[0], v[1], v[2]);
		vertCount++;
	}
	fprintf(f, "# %i vertices\n\n", vertCount);

	// Writing texture coordinates
	fprintf(f, "vt 0 0\n");
	fprintf(f, "vt 0 1\n");
	fprintf(f, "vt 1 1\n");

	// Writing normals
	int normCount = 0;
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly poly = tile->polys[i];
		if (poly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
			continue;
		const dtPolyDetail pd = tile->detailMeshes[i];

		// For each triangle in the polygon
		for (int j = 0; j < pd.triCount; ++j)
		{
			const unsigned char* tri = &tile->detailTris[(pd.triBase + j) * 4];

			// Find the three vertex pointers
			const float* v[3];
			for (int k = 0; k < 3; ++k)
			{
				if (tri[k] < poly.vertCount)
					v[k] = &tile->verts[poly.verts[tri[k]] * 3];
				else
					v[k] = &tile->detailVerts[(pd.vertBase + tri[k] - poly.vertCount) * 3];
			}

			// Calculate the normal
			float3 v0 = make_float3(v[0][0], v[0][1], v[0][2]);
			float3 v1 = make_float3(v[1][0], v[1][1], v[1][2]);
			float3 v2 = make_float3(v[2][0], v[2][1], v[2][2]);
			float3 n = cross(v1 - v0, v2 - v0);
			normalize(n);
			if (n.y < 0) n = -n; // ensures all normals point up

								 // Write the normal to the file
			fprintf(f, "vn %.5f %.5f %.5f\n", n.x, n.y, n.z);
			normCount++;
		}
	}
	fprintf(f, "# %i normals\n\n", normCount);

	// Writing faces
	int faceCount = 0;
	fprintf(f, "usemtl navmesh\n");
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly poly = tile->polys[i];

		// If it's an off-mesh connection, continue
		if (poly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;

		// For each triangle in the polygon
		const dtPolyDetail pd = tile->detailMeshes[i];
		for (int j = 0; j < pd.triCount; ++j)
		{
			const unsigned char* tri = &tile->detailTris[(pd.triBase + j) * 4];

			// Find the three vertex indices
			int v[3];
			for (int k = 0; k < 3; ++k)
			{
				if (tri[k] < poly.vertCount) v[k] = poly.verts[tri[k]];
				else v[k] = pd.vertBase + tri[k];
			}

			// Write the face to the file
			fprintf(f, "f");
			for (int k = 0; k < 3; k++)
				fprintf(f, " %i/%i/%i", v[k] + 1, k + 1, faceCount + 1); // +1 because .obj indices start at 1
			fprintf(f, "\n");

			faceCount++;
		}
	}
	fprintf(f, "# %i faces\n\n", faceCount);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SaveAsMesh                                                  |
//  |  Saves the navmesh as an .obj file.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::SaveAsMesh(NavMeshNavigator* navmesh)
{
	// Opening file
	Timer timer;
	const char* ID = navmesh->GetID();
	std::string filename = m_dir + m_meshFileName;
	printf("Saving navmesh as wavefront in '%s'... ", filename.c_str());
	FILE* f;
	fopen_s(&f, filename.c_str(), "w");
	if (!f)
	{
		printf("ERROR: File '%s' could not be opened\n", filename.c_str());
		return;
	}
	const dtNavMesh* mesh = navmesh->GetDetourMesh();
	if (!mesh)
	{
		printf("ERROR: navmesh is null\n", ID);
		return;
	}

	// Writing header
	fprintf(f, "#\n# Wavefront OBJ file\n");
	fprintf(f, "# Navigation mesh\n# ID: '%s'\n", ID);
	fprintf(f, "# Automatically generated by 'recastnavigation.cpp'\n");
	fprintf(f, "#\nmtllib %s\n\n", m_matFileName.c_str());

	// Writing one group per tile
	for (int i = 0; i < mesh->getMaxTiles(); ++i) if (mesh->getTile(i)->header)
	{
		fprintf(f, "g Tile%3i\n", i);
		WriteTileToMesh(mesh->getTile(i), f);
	}

	fclose(f);
	printf("%.3fms\n", timer.elapsed());
}

} // namespace lighthouse2

// EOF