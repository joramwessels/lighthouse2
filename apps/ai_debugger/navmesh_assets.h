/* navmesh_assets.h - Copyright 2019 Utrecht University

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

#include "DetourNavMesh.h" // TODO: replace all Detour calls with NavMeshNavigator calls?

#include "rendersystem.h"
#include "navmesh_builder.h"

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets                                                              |
//  |  NavMeshAssets class handles the representation of the navmesh.       LH2'19|
//  +-----------------------------------------------------------------------------+
class NavMeshAssets
{
public:
	NavMeshAssets(RenderAPI* renderer, const char* dir)
		: m_renderer(renderer), m_dir(dir)
	{
		// Add meshes
		m_nodeMeshID = m_renderer->AddMesh("node.obj", m_dir, .01f);
		m_agentMeshID = m_renderer->AddMesh("agent.obj", m_dir, 1.0f);
		m_edgeMeshID = m_renderer->AddMesh("agent.obj", m_dir, .01f);
		m_renderer->GetScene()->meshes[m_nodeMeshID]->name = "Vertex";
		m_renderer->GetScene()->meshes[m_agentMeshID]->name = "Agent";
		m_renderer->GetScene()->meshes[m_edgeMeshID]->name = "Edge";
	};
	~NavMeshAssets() {};

	void ReplaceMesh(NavMeshBuilder* navmesh);
	void PlaceAgent(float3 pos);
	void UpdatePath(NavMeshNavigator* navmesh, float3 start, float3 end, int maxSize = 100);
	void PlotPath(float3 start);
	void Clean();

	bool isNavMesh(int meshID) const { return meshID == m_navmeshMeshID; };
	bool isAgent(int meshID) const { return meshID == m_agentMeshID; };
	bool isNode(int meshID) const { return meshID == m_nodeMeshID; };
	bool isEdge(int meshID) const { return meshID == m_edgeMeshID; };

	float3* pathStart = 0, *pathEnd = 0;

private:
	RenderAPI* m_renderer;
	NavMeshNavigator* m_navmesh;
	const char* m_dir;

	int m_navmeshMeshID = -1, m_navmeshInstID = -1;
	int m_nodeMeshID = -1, m_edgeMeshID = -1;
	int m_agentMeshID = -1, m_startInstID = -1, m_endInstID = -1;
	int m_agentHeight, m_agentRadius;
	
	struct Node { int instID; float3 pos; };
	struct Edge { int instID; int n1, n2; };
	std::vector<Node> nodes;
	std::vector<Edge> edges;
	std::vector<std::vector<int>*> m_polyTriIdx; // The poly's triangle indices within the mesh

	float3 m_selectedTriColor = { 1.0f, 1.0f, 0.0f };
	mat4 m_edgeScale = mat4::Scale(make_float3(0.1f, 0.1f, 0.1f)); // TODO

	std::vector<float3> m_path;  // The last calculated path as world coordinates
	float4 m_pathColor = { 1.0f, 0.0f, 0.0f, 0.5f };
	float m_distToEnd = -1, m_pathWidth = 3.0f;

	void AddNodesToScene(NavMeshBuilder* navmesh);
	void AddNode(float x, float y, float z);
	void AddEdgesToScene(NavMeshBuilder* navmesh);
	void AddEdge(float3 node1, float3 node2);

	//const std::vector<int>* GetPolyTriangleIndices(dtPolyRef poly, int tileIdx = 0);

	void WriteMaterialFile();
	void WriteTileToMesh(const dtMeshTile* tile, FILE* file);
	void SaveAsMesh(NavMeshBuilder* navmesh);
	std::string GetObjFileName(const char* ID) const
		{ return ".tmp." + std::string(ID) + ".obj"; };
	std::string GetMatFileName() const { return "navmesh.mtl"; };
};

// EOF