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

#include "rendersystem.h"
#include "navmesh_builder.h"

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
		// Add meshes
		m_vertMeshID = m_renderer->AddMesh("vertex.obj", m_dir, 1.0f);
		m_agentMeshID = m_renderer->AddMesh("agent.obj", m_dir, 1.0f);
		m_edgeMeshID = m_renderer->AddMesh("edge.obj", m_dir, 1.0f);
		m_renderer->GetScene()->meshes[m_vertMeshID]->name = "Vertex";
		m_renderer->GetScene()->meshes[m_agentMeshID]->name = "Agent";
		m_renderer->GetScene()->meshes[m_edgeMeshID]->name = "Edge";
	};
	~NavMeshShader() {};

	void UpdateMesh(NavMeshBuilder* navmesh);
	void DrawGL() const;

	// NavMesh scene shading
	void AddPolysToScene(NavMeshBuilder* navmesh);
	void AddVertsToScene();
	void AddEdgesToScene();
	void RemovePolysFromScene();
	void RemoveVertsFromScene();
	void RemoveEdgesFromScene();

	// NavMesh GL shading
	void AddPolysToGL() { m_shadeTris = true; };
	void AddVertsToGL() { m_shadeVerts = true; };
	void AddEdgesToGL() { m_shadeEdges = true; };
	void RemovePolysFromGL() { m_shadeTris = false; };
	void RemoveVertsFromGL() { m_shadeVerts = false; };
	void RemoveEdgesFromGL() { m_shadeEdges = false; };

	// Agents
	void AddAgentToScene(float3 pos);
	void RemoveAgent(int instanceID);
	void RemoveAllAgents();

	// Object selection
	void SelectPoly(float3 pos, NavMeshNavigator* navmesh);
	void SelectVert(int instanceID);
	void SelectEdge(int instanceID);
	void SelectAgent(int instanceID);

	void SetPath(float3 start, std::vector<float3>* path, bool owner = false)
		{ m_path = path; m_pathStart = start; m_pathOwner = owner; };

	void Clean();

	bool isAgent(int meshID) const { return meshID == m_agentMeshID; };
	bool isPoly(int meshID) const { return meshID == m_polyMeshID; };
	bool isVert(int meshID) const { return meshID == m_vertMeshID; };
	bool isEdge(int meshID) const { return meshID == m_edgeMeshID; };
	bool isNavMesh(int meshID) const { return isVert(meshID) || isEdge(meshID) || isPoly(meshID); };

private:
	RenderAPI* m_renderer;
	const char* m_dir;
	
	// NavMesh representation
	int m_polyMeshID = -1, m_polyInstID = -1;
	int m_vertMeshID = -1, m_edgeMeshID = -1;
	struct Vert { float3 pos; int idx = -1, instID = -1; std::vector<const dtPoly*> polys; };
	struct Edge { int v1 = -1, v2 = -1, instID = -1; const dtPoly *poly1 = 0, *poly2 = 0; };
	std::vector<Vert> m_verts;
	std::vector<Edge> m_edges;
	void ExtractVertsAndEdges(const dtNavMesh* navmesh);
	void AddEdgeToEdgesAndPreventDuplicates(int v1, int v2, const dtPoly* poly);
	float4 m_polyColor = { 0, 1.0f, 1.0f, 0.2f };		// rgba   TODO: make this affect meshes?
	float4 m_vertColor = { 1.0f, 0, 1.0f, 0.2f };		// rgba
	float4 m_edgeColor = { 1.0f, 0, 1.0f, 0.2f };		// rgba
	float m_vertWidth = .3f, m_edgeWidth = .1f;			// in world coordinates
	float m_edgeWidthGL = 5.0f, m_vertWidthGL = 10.0f;	// in pixels

	// GL shading
	bool m_shadeTris = false, m_shadeVerts = false, m_shadeEdges = false;
	void ShadePolysGL() const;
	void ShadeVertsGL() const;
	void ShadeEdgesGL() const;

	// Agents
	float m_agentHeight, m_agentRadius;
	int m_agentMeshID = -1;
	struct Agent { int instID = -1; float3 pos, dir; };
	std::vector<Agent> m_agents;

	// Object selecting / highlighting
	const Vert* m_vertSelect = 0;
	const Edge* m_edgeSelect = 0;
	const dtPoly* m_polySelect = 0;
	const Agent* m_agentSelect = 0;
	void Deselect() { m_vertSelect = 0; m_edgeSelect = 0; m_polySelect = 0; m_agentSelect = 0; };
	void DrawPolyHighlightGL() const;
	void DrawVertHighlightGL() const;
	void DrawEdgeHighlightGL() const;
	void DrawAgentHighlightGL() const;
	float4 m_highLightColor = { 1.0f, 1.0f, 0.0f, .5f }; // rgba

	// Path drawing
	std::vector<float3>* m_path = 0;
	bool m_pathOwner = false;
	float3 m_pathStart;
	void PlotPath() const;
	float4 m_pathColor = { 1.0f, 0.0f, 0.0f, 0.5f }; // rgba
	float m_pathWidth = 3.0f;

	// File writing
	void WriteMaterialFile();
	void WriteTileToMesh(const dtMeshTile* tile, FILE* file);
	void SaveAsMesh(NavMeshBuilder* navmesh);
	std::string GetObjFileName(const char* ID) const
		{ return ".tmp." + std::string(ID) + ".obj"; };
	std::string GetMatFileName() const { return "navmesh.mtl"; };
};

} // namespace lighthouse2

// EOF