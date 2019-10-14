/* gui.cpp - Copyright 2019 Utrecht University
   
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

#include "gui.h"
#include "DetourNode.h"

//  +-----------------------------------------------------------------------------+
//  |  GUI::AddNodesToScene                                                       |
//  |  Adds a all navmesh nodes as spheres to the scene.                    LH2'19|
//  +-----------------------------------------------------------------------------+
void AI_DEMO_GUI::AddNodesToScene(NavMeshBuilder* navmesh)
{
	//const dtNodePool* pool = navmesh->GetQuery()->getNodePool();
	//if (pool)
	//{
	//	const float off = 0.5f;
	//	for (int i = 0; i < pool->getHashSize(); ++i)
	//		for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
	//		{
	//			const dtNode* node = pool->getNodeAtIdx(j + 1);
	//			if (!node) continue;
	//			AddNode(node->pos[0], node->pos[1] + off, node->pos[2]);
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
	//			AddNode(node->pos[0], node->pos[1] + off, node->pos[2]);
	//			AddNode(parent->pos[0], parent->pos[1] + off, parent->pos[2]);
	//		}
	//}

	const dtNavMesh* mesh = navmesh->GetMesh();
	for (int a = 0; a < mesh->getMaxTiles(); a++)
	{
		const dtMeshTile* tile = mesh->getTile(a);
		for (int i = 0; i < tile->header->vertCount; ++i)
		{
			const float* v = &tile->verts[i * 3];
			AddNode(v[0], v[1], v[2]);
		}
		for (int i = 0; i < tile->header->detailVertCount; ++i)
		{
			const float* v = &tile->detailVerts[i * 3];
			AddNode(v[0], v[1], v[2]);
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  GUI::AddNode                                                               |
//  |  Adds a node to the scene. If there still are already initialized unused    |
//  |  nodes, it translates one of those before allocating a new one.       LH2'19|
//  +-----------------------------------------------------------------------------+
void AI_DEMO_GUI::AddNode(float x, float y, float z)
{
	mat4 translate = mat4::Translate(x, y, z);
	if (nodeInstances.size() > nodeCount)
	{
		// Reusing allocated instances
		int instanceID = edgeInstances[nodeCount];
		renderer->SetNodeTransform(instanceID, translate);
	}
	else
	{
		// Out of unused instances
		int instanceID = renderer->AddInstance(nodeMeshID, translate);
		nodeInstances.push_back(instanceID);
	}
	nodeCount++;
}

//  +-----------------------------------------------------------------------------+
//  |                                                                         |
//  |                                               LH2'19|
//  +-----------------------------------------------------------------------------+
void AI_DEMO_GUI::AddEdgesToScene(NavMeshBuilder* navmesh)
{

}

//  +-----------------------------------------------------------------------------+
//  |                                                                         |
//  |                                               LH2'19|
//  +-----------------------------------------------------------------------------+
void AI_DEMO_GUI::AddSurfacesToScene(NavMeshBuilder* navmesh)
{

}

//  +-----------------------------------------------------------------------------+
//  |  Clean                                                                      |
//  |  Moves all instances to an invisible location untill needed.          LH2'19|
//  +-----------------------------------------------------------------------------+
void AI_DEMO_GUI::Clean()
{
	for (std::vector<int>::iterator it = nodeInstances.begin(); it < nodeInstances.end(); it++)
		renderer->SetNodeTransform(*it, cleanupTranform);
	for (std::vector<int>::iterator it = edgeInstances.begin(); it < edgeInstances.end(); it++)
		renderer->SetNodeTransform(*it, cleanupTranform);
	edgeCount = nodeCount = 0;
}

// EOF