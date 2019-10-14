/* gui.h - Copyright 2019 Utrecht University

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
#include "pathfinding.h"

class AI_DEMO_GUI
{
public:
	AI_DEMO_GUI(RenderAPI* renderer, int debug) : renderer(renderer)
	{
		// Add meshes
		nodeMeshID = renderer->AddMesh("node.obj", "data\\ai\\", .01f);

	};
	~AI_DEMO_GUI() {};

	void AddNodesToScene(NavMeshBuilder* navmesh);
	void AddNode(float x, float y, float z);
	void AddEdgesToScene(NavMeshBuilder* navmesh);
	void AddSurfacesToScene(NavMeshBuilder* navmesh);
	void Clean();

	RenderAPI* renderer;
	int nodeMeshID, edgeMeshID;  // renderer mesh ID of node/edge
	int nodeCount, edgeCount;    // count of used node/edge instances in scene
	std::vector<int> nodeInstances; // instance IDs of navmesh nodes
	std::vector<int> edgeInstances; // instance IDs of navmesh edges
	mat4 cleanupTranform = mat4::Translate(10000.0f, 10000.0f, 10000.0f);
};

// EOF