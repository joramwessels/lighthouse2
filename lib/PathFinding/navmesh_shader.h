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
		m_vertMeshID = m_renderer->AddMesh("vertex.obj", m_dir, .01f);
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

	void AddNavMeshToScene(NavMeshBuilder* navmesh);
	void RemoveNavMeshFromScene();
	void AddVertsToScene();
	void RemoveVertsFromScene();
	void AddEdgesToScene();
	void RemoveEdgesFromScene();

	void AddNavMeshToGL(bool useGL = true) { m_shadeTris = useGL; };
	void AddVertsToGL(bool useGL = true) { m_shadeVerts = useGL; };
	void AddEdgesToGL(bool useGL = true) { m_shadeEdges = useGL; };

	void HighlightPoly(int triangleID);
	void HighlightVert(int instanceID);
	void HighlightEdge(int instanceID);

	void SetPath(float3 start, std::vector<float3>* path) { m_path = path; m_pathStart = start; };

	void Clean();

	bool isNavMesh(int meshID) const { return meshID == m_navmeshMeshID; };
	bool isAgent(int meshID) const { return meshID == m_agentMeshID; };
	bool isVert(int meshID) const { return meshID == m_vertMeshID; };
	bool isEdge(int meshID) const { return meshID == m_edgeMeshID; };

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
	struct Vert { float3 pos; int idx = -1, instID = -1; std::vector<const dtPoly*> polys; };
	struct Edge { int v1 = -1, v2 = -1, instID = -1; const dtPoly *poly1 = 0, *poly2 = 0; };
	std::vector<Vert> verts;
	std::vector<Edge> edges;
	void ExtractVertsAndEdges(const dtNavMesh* navmesh);
	void AddEdgeToEdgesAndPreventDuplicates(int v1, int v2, const dtPoly* poly);
	mat4 m_edgeScale = mat4::Scale(make_float3(1.0f, 1.0f, 1.0f)); // TODO
	float m_edgeWidth = .1f;

	// Highlighting
	int m_vertHighlightMeshID = -1, m_edgeHighlightMeshID = -1;
	Vert m_vertHighlight;
	Edge m_edgeHighlight;
	//dtPolyRef m_polyHighlight;
	void DrawVertHighlightGL() const;
	void DrawEdgeHighlightGL() const;

	float3 m_selectedTriColor = { 1.0f, 1.0f, 0.0f };

	// Path
	std::vector<float3>* m_path = 0;
	float3 m_pathStart;
	float4 m_pathColor = { 1.0f, 0.0f, 0.0f, 0.5f };
	float m_pathWidth = 3.0f;
	void PlotPath() const;

	// Shading
	bool m_shadeTris = false, m_shadeVerts = false, m_shadeEdges = false;
	void ShadeTrianglesGL() const;
	void ShadeVertsGL() const;
	void ShadeEdgesGL() const;

	//const std::vector<int>* GetPolyTriangleIndices(dtPolyRef poly, int tileIdx = 0);

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