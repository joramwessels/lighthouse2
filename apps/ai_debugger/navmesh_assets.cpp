/* navmesh_assets.cpp - Copyright 2019 Utrecht University
   
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

#include "navmesh_assets.h"
#include "DetourNode.h"

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets::ReplaceMesh                                                 |
//  |  Removes the old navmesh assets and adds the new one.                 LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAssets::ReplaceMesh(NavMeshBuilder* navmesh)
{
	Clean();
	SaveAsMesh(navmesh);
	std::string filename = GetObjFileName(navmesh->GetConfig()->m_id);
	m_navmeshMeshID = m_renderer->AddMesh(filename.c_str(), m_dir, 1.0f);
	m_navmeshInstID = m_renderer->AddInstance(m_navmeshMeshID, mat4::Identity());
	AddNodesToScene(navmesh);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets::AddNodesToScene                                             |
//  |  Adds a all navmesh nodes as spheres to the scene.                    LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAssets::AddNodesToScene(NavMeshBuilder* navmesh)
{
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
//  |  NavMeshAssets::AddNode                                                     |
//  |  Adds a node to the scene. If there still are already initialized unused    |
//  |  nodes, it translates one of those before allocating a new one.       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAssets::AddNode(float x, float y, float z)
{
	mat4 translate = mat4::Translate(x, y, z);
	if (nodeInstances.size() > nodeCount)
	{
		// Reusing allocated instances
		int instanceID = nodeInstances[nodeCount];
		m_renderer->SetNodeTransform(instanceID, translate);
	}
	else
	{
		// Out of unused instances
		int instanceID = m_renderer->AddInstance(m_nodeMeshID, translate);
		nodeInstances.push_back(instanceID);
	}
	nodeCount++;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets::Clean                                                       |
//  |  Moves all instances to an invisible location untill needed.          LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAssets::Clean()
{
	// NOTE: Ideally instances and meshes can be deleted from the core
	//		 to prevent a significant memory leaks, but by lack of a
	//		 delete function, meshes are moved to a remote location
	m_renderer->SetNodeTransform(m_navmeshInstID, m_cleanupTranform);
	for (std::vector<int>::iterator it = nodeInstances.begin(); it < nodeInstances.end(); it++)
		m_renderer->SetNodeTransform(*it, m_cleanupTranform);
	nodeCount = 0;
	// TODO: Remove old mesh file
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets::WriteMaterialFile                                           |
//  |  Writes a default .mtl file for the mesh.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAssets::WriteMaterialFile()
{
	// Opening file
	const char* filename = (m_dir + GetMatFileName()).c_str();
	FILE* f;
	fopen_s(&f, filename, "w");
	if (!f) printf("File '%s' could not be opened", filename);

	// Writing contents
	fprintf(f, "newmtl %s\n", "navmesh");
	fprintf(f, "Ka    %.2f %.2f %.2f\n", 0.0f, 1.0f, 1.0f);
	fprintf(f, "Kd    %.2f %.2f %.2f\n", 0.0f, 1.0f, 1.0f);
	fprintf(f, "Ks    %.2f %.2f %.2f\n", 0.0f, 0.0f, 0.0f);
	fprintf(f, "d     %.2f\n", 0.2f);
	fprintf(f, "Tr    %.2f\n", 0.8f);
	fprintf(f, "illum 1");

	fprintf(f, "\n\n");
	fprintf(f, "newmtl %s\n", "node");
	fprintf(f, "Ka    %.2f %.2f %.2f\n", 1.0f, 0.0f, 1.0f);
	fprintf(f, "Kd    %.2f %.2f %.2f\n", 1.0f, 0.0f, 1.0f);
	fprintf(f, "Ks    %.2f %.2f %.2f\n", 0.0f, 0.0f, 0.0f);
	fprintf(f, "d     %.2f\n", 0.2f);
	fprintf(f, "Tr    %.2f\n", 0.8f);
	fprintf(f, "illum 1");

	fprintf(f, "\n\n");
	fprintf(f, "newmtl %s\n", "agent");
	fprintf(f, "Ka    %.2f %.2f %.2f\n", 1.0f, 1.0f, 0.0f);
	fprintf(f, "Kd    %.2f %.2f %.2f\n", 1.0f, 1.0f, 0.0f);
	fprintf(f, "Ks    %.2f %.2f %.2f\n", 0.0f, 0.0f, 0.0f);
	fprintf(f, "d     %.2f\n", 0.6f);
	fprintf(f, "Tr    %.2f\n", 0.4f);
	fprintf(f, "illum 1");

	fclose(f);
	printf("Navmesh material file saved as '%s'", filename);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets::WriteTileToMesh                                             |
//  |  Writes the given navmesh tile to the given open file.                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAssets::WriteTileToMesh(const dtMeshTile* tile, FILE* f)
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
		if (poly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
			continue;
		const dtPolyDetail pd = tile->detailMeshes[i];

		// For each triangle in the polygon
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
				fprintf(f, " %i//%i", v[k] + 1, faceCount + 1); // +1 because .obj indices start at 1
			fprintf(f, "\n");
			faceCount++;
		}
	}
	fprintf(f, "# %i faces\n\n", faceCount);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAssets::SaveAsMesh                                                  |
//  |  Saves the navmesh as an .obj file.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAssets::SaveAsMesh(NavMeshBuilder* navmesh)
{
	// Opening file
	const char* ID = navmesh->GetConfig()->m_id;
	std::string filename = m_dir + GetObjFileName(ID);
	FILE* f;
	fopen_s(&f, filename.c_str(), "w");
	if (!f) printf("File '%s' could not be opened", filename.c_str());
	const dtNavMesh* mesh = navmesh->GetMesh();
	if (!mesh) printf("Navmesh '%s' can't be saved as mesh, m_navMesh is null", ID);
	if (!FileExists((m_dir + GetMatFileName()).c_str())) WriteMaterialFile();

	// Writing header
	fprintf(f, "#\n# Wavefront OBJ file\n");
	fprintf(f, "# Navigation mesh\n# ID: '%s'\n", ID);
	fprintf(f, "# Automatically generated by 'recastnavigation.cpp'\n");
	fprintf(f, "#\nmtllib %s\n\n", GetMatFileName().c_str());

	// Writing one group per tile
	for (int i = 0; i < mesh->getMaxTiles(); ++i) if (mesh->getTile(i)->header)
	{
		fprintf(f, "g Tile%2i\n", i);
		WriteTileToMesh(mesh->getTile(i), f);
	}

	fclose(f);
	printf("NavMesh mesh saved as '%s'", filename.c_str());
}

// EOF