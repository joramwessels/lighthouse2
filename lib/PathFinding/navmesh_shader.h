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
	void PlaceAgent(float3 pos);

	void AddPolysToScene(NavMeshBuilder* navmesh);
	void AddVertsToScene();
	void AddEdgesToScene();
	void RemovePolysFromScene();
	void RemoveVertsFromScene();
	void RemoveEdgesFromScene();

	void AddPolysToGL() { m_shadeTris = true; };
	void AddVertsToGL() { m_shadeVerts = true; };
	void AddEdgesToGL() { m_shadeEdges = true; };
	void RemovePolysFromGL() { m_shadeTris = false; };
	void RemoveVertsFromGL() { m_shadeVerts = false; };
	void RemoveEdgesFromGL() { m_shadeEdges = false; };

	void SelectPoly(float3 pos, NavMeshNavigator* navmesh);
	void SelectVert(int instanceID);
	void SelectEdge(int instanceID);

	void SetPath(float3 start, std::vector<float3>* path) { m_path = path; m_pathStart = start; };

	void Clean();

	bool isAgent(int meshID) const { return meshID == m_agentMeshID; };
	bool isPoly(int meshID) const { return meshID == m_navmeshMeshID; };
	bool isVert(int meshID) const { return meshID == m_vertMeshID; };
	bool isEdge(int meshID) const { return meshID == m_edgeMeshID; };
	bool isNavMesh(int meshID) const { return isVert(meshID) || isEdge(meshID) || isPoly(meshID); };

private:
	RenderAPI* m_renderer;
	const char* m_dir;

	// Mesh- and instance IDs
	int m_navmeshInstID = -1, m_startInstID = -1, m_endInstID = -1;
	int m_vertMeshID = -1, m_edgeMeshID = -1;
	int m_navmeshMeshID = -1, m_agentMeshID = -1;

	// Agents
	int m_agentHeight, m_agentRadius;
	
	// Verts and Edges
	float m_vertWidth = .3f, m_edgeWidth = .1f;
	struct Vert { float3 pos; int idx = -1, instID = -1; std::vector<const dtPoly*> polys; };
	struct Edge { int v1 = -1, v2 = -1, instID = -1; const dtPoly *poly1 = 0, *poly2 = 0; };
	std::vector<Vert> verts;
	std::vector<Edge> edges;
	void ExtractVertsAndEdges(const dtNavMesh* navmesh);
	void AddEdgeToEdgesAndPreventDuplicates(int v1, int v2, const dtPoly* poly);

	// Highlighting
	float4 m_highLightColor = { 1.0f, 1.0f, 0.0f, .5f }; // rgba
	float m_edgeHighlightWidth = 5.0f, m_vertHighlightWidth = 10.0f; // in pixels
	const Vert* m_vertSelect = 0;
	const Edge* m_edgeSelect = 0;
	const dtPoly* m_polySelect = 0;
	void Deselect() { m_vertSelect = 0; m_edgeSelect = 0; m_polySelect = 0; };
	void DrawPolyHighlightGL() const;
	void DrawVertHighlightGL() const;
	void DrawEdgeHighlightGL() const;

	// Path
	float4 m_pathColor = { 1.0f, 0.0f, 0.0f, 0.5f };
	float m_pathWidth = 3.0f;
	std::vector<float3>* m_path = 0;
	float3 m_pathStart;
	void PlotPath() const;

	// Shading
	bool m_shadeTris = false, m_shadeVerts = false, m_shadeEdges = false;
	void ShadePolysGL() const;
	void ShadeVertsGL() const;
	void ShadeEdgesGL() const;

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