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

#include "rendersystem.h"
#include "navmesh_builder.h"

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets                                                              |
//  |  NavMeshAssets class handles the scene assets of the navmesh.         LH2'19|
//  +-----------------------------------------------------------------------------+
class NavMeshAssets
{
public:
	NavMeshAssets(RenderAPI* renderer, const char* dir)
		: m_renderer(renderer), m_dir(dir)
	{
		// Add meshes
		m_nodeMeshID = m_renderer->AddMesh("node.obj", m_dir, .01f);
		m_agentMeshID = m_renderer->AddMesh("agent.obj", m_dir, .01f);
		// TODO: add cylinder mesh
	};
	~NavMeshAssets() {};

	void ReplaceMesh(NavMeshBuilder* navmesh);
	void Clean();

private:
	RenderAPI* m_renderer;
	const char* m_dir;

	int m_nodeMeshID, m_navmeshMeshID, m_navmeshInstID;
	int m_agentMeshID, m_startInstID, m_endInstID;
	int nodeCount;					// count of used node instances in scene
	std::vector<int> nodeInstances; // instance IDs of navmesh nodes

	mat4 m_cleanupTranform = mat4::Translate(10000.0f, 10000.0f, 10000.0f);

	void AddNodesToScene(NavMeshBuilder* navmesh);
	void AddNode(float x, float y, float z);
	void WriteMaterialFile();
	void WriteTileToMesh(const dtMeshTile* tile, FILE* file);
	void SaveAsMesh(NavMeshBuilder* navmesh);
	std::string GetObjFileName(const char* ID) const
		{ return ".tmp." + std::string(ID) + ".obj"; };
	std::string GetMatFileName() const { return "navmesh.mtl"; };
};

// EOF