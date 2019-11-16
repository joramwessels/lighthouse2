/* navmesh_shader.h - Copyright 2019 Utrecht University

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

//#ifdef PATHFINDINGBUILD
	#include "DetourNavMesh.h"
//#endif

#include "rendersystem.h"		// RenderAPI
#include "navmesh_navigator.h"  // NavMeshNavigator
#include "navmesh_agents.h"		// Agent

namespace lighthouse2 {

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader                                                              |
//  |  NavMeshShader class handles the visualization of the navmesh.        LH2'19|
//  +-----------------------------------------------------------------------------+
class NavMeshShader
{
public:
	NavMeshShader(RenderAPI* renderer, const char* dir)
		: m_renderer(renderer), m_dir(dir)
	{
		// Initialize meshes
		m_vertMeshID = m_renderer->AddMesh("vertex.obj", m_dir, 1.0f);
		m_agentMeshID = m_renderer->AddMesh("agent.obj", m_dir, 1.0f);
		m_edgeMeshID = m_renderer->AddMesh("edge.obj", m_dir, 1.0f);
		m_directedEdgeMeshID = m_renderer->AddMesh("arrow.obj", m_dir, 1.0f);
		m_renderer->GetScene()->meshes[m_vertMeshID]->name = "Vertex";
		m_renderer->GetScene()->meshes[m_agentMeshID]->name = "Agent";
		m_renderer->GetScene()->meshes[m_edgeMeshID]->name = "Edge";
		m_renderer->GetScene()->meshes[m_directedEdgeMeshID]->name = "DirectedEdge";
	};
	~NavMeshShader() {};

	struct Vert { float3* pos; int idx = -1, instID = -1; std::vector<const dtPoly*> polys; };
	struct Edge { int v1 = -1, v2 = -1, idx = -1, instID = -1; const dtPoly *poly1 = 0, *poly2 = 0; };
	struct OMC { const dtOffMeshConnection* omc; int v1InstID = -1, v2InstID = -1, edgeInstID = -1; };

	void UpdateMesh(NavMeshNavigator* navmesh);
	void UpdateAgentPositions();
	void DrawGL() const;

	// NavMesh scene shading
	void AddNavMeshToScene();
	void AddPolysToScene();
	void AddVertsToScene();
	void AddEdgesToScene();
	void AddOMCsToScene();
	void RemoveNavMeshFromScene();
	void RemovePolysFromScene();
	void RemoveVertsFromScene();
	void RemoveEdgesFromScene();
	void RemoveOMCsFromScene();

	// NavMesh GL shading
	void AddNavMeshToGL() { m_shadeTris = m_shadeVerts = m_shadeEdges = true; };
	void AddPolysToGL() { m_shadeTris = true; };
	void AddVertsToGL() { m_shadeVerts = true; };
	void AddEdgesToGL() { m_shadeEdges = true; };
	void RemoveNavMeshFromGL() { m_shadeTris = m_shadeVerts = m_shadeEdges = false; };
	void RemovePolysFromGL() { m_shadeTris = false; };
	void RemoveVertsFromGL() { m_shadeVerts = false; };
	void RemoveEdgesFromGL() { m_shadeEdges = false; };

	// Agents
	void AddAgentToScene(Agent* agent);
	void RemoveAgent(int instanceID);
	void RemoveAllAgents();

	// Object selection
	void Deselect() { m_vertSelect = 0; m_edgeSelect = 0; m_polySelect = 0; m_agentSelect = 0; };
	const dtPoly* SelectPoly(float3 pos, NavMeshNavigator* navmesh);
	Vert* SelectVert(int instanceID);
	Edge* SelectEdge(int instanceID);
	Agent* SelectAgent(int instanceID);

	// Path drawing
	void SetPath(const std::vector<NavMeshNavigator::PathNode>* path) { m_path = path; };
	void SetPathStart(const float3* start) { m_pathStart = start; };
	void SetPathEnd(const float3* end) { m_pathEnd = end; };

	// Editing
	void SetTmpVert(float3 pos, float vertWidth=.5f);
	void RemoveTmpVert();
	void AddTmpOMC(float3 v0, float3 v1, float vertWidth);

	void Clean();

	bool isAgent(int meshID) const { return meshID == m_agentMeshID; };
	bool isPoly(int meshID) const { return meshID == m_polyMeshID; };
	bool isVert(int meshID) const { return meshID == m_vertMeshID; };
	bool isEdge(int meshID) const { return meshID == m_edgeMeshID; };
	bool isNavMesh(int meshID) const { return isVert(meshID) || isEdge(meshID) || isPoly(meshID); };

	inline float3* GetVertPos(int idx) { return m_verts[idx].pos; };

private:
	RenderAPI* m_renderer;
	const char* m_dir;
	
	// NavMesh representation
	std::vector<Vert> m_verts; // (verts, detailVerts, offMeshVerts) * nTiles
	std::vector<Edge> m_edges;
	std::vector<OMC> m_OMCs;
	void ExtractVertsAndEdges(const dtNavMesh* navmesh);
	void AddEdgeToEdgesAndPreventDuplicates(int v1, int v2, const dtPoly* poly);

	// Scene shading
	int m_polyMeshID = -1, m_polyInstID = -1;
	int m_vertMeshID = -1, m_edgeMeshID = -1;
	int m_directedEdgeMeshID = -1;
	const float m_vertWidth = .3f, m_edgeWidth = .1f; // in world coordinates

	// GL shading
	bool m_shadeTris = false, m_shadeVerts = false, m_shadeEdges = false;
	void ShadePolysGL() const;
	void ShadeVertsGL() const;
	void ShadeEdgesGL() const;
	const float m_edgeWidthGL = 5.0f, m_vertWidthGL = 20.0f; // in pixels
	const float4 m_polyColor = { 0, 1.0f, 1.0f, 0.2f };	// rgba   TODO: make this affect meshes?
	const float4 m_vertColor = { 1.0f, 0, 1.0f, 0.2f };	// rgba
	const float4 m_edgeColor = { 1.0f, 0, 1.0f, 0.2f };	// rgba

	// Agents
	int m_agentMeshID = -1;
	struct ShaderAgent { int instID = -1; Agent* agent; };
	std::vector<ShaderAgent> m_agents;
	void DrawAgentImpulse() const;

	// Object selecting / highlighting
	const dtPoly* m_polySelect = 0;
	Vert* m_vertSelect = 0;
	Edge* m_edgeSelect = 0;
	ShaderAgent* m_agentSelect = 0;
	void DrawPolyHighlightGL() const;
	void DrawVertHighlightGL() const;
	void DrawEdgeHighlightGL() const;
	void DrawAgentHighlightGL() const;
	const float4 m_highLightColor = { 1.0f, 1.0f, 0.0f, .5f }; // rgba

	// Path drawing
	const std::vector<NavMeshNavigator::PathNode>* m_path = 0;
	const float3 *m_pathStart = 0, *m_pathEnd = 0; // only used for beacon drawing
	void PlotPath() const;
	void DrawPathMarkers() const;
	const float m_pathWidth = 3.0f, m_beaconWidth = 10.0f;
	const float3 m_beaconLen = make_float3(0.0f, 4.0f, 0.0f);
	const float4 m_pathColor = { 1.0f, 0.0f, 0.0f, 0.5f }; // rgba
	const float4 m_beaconStColor = { 0, 1.0f, 0, 1.0f };   // rgba
	const float4 m_beaconEnColor = { 1.0f, 0, 0, 1.0f };   // rgba

	// Editing
	Vert m_tmpVert;

	// File writing
	std::string m_meshFileName;
	const std::string m_matFileName = "navmesh.mtl";
	void WriteTileToMesh(const dtMeshTile* tile, FILE* file);
	void SaveAsMesh(NavMeshNavigator* navmesh);
};

} // namespace lighthouse2

// EOF