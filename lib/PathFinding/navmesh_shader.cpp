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
void NavMeshShader::UpdateMesh(NavMeshBuilder* navmesh)
{
	Clean();
	ExtractVertsAndEdges(navmesh->GetMesh());
	
	AddPolysToScene(navmesh);
	AddVertsToScene();
	AddEdgesToScene();

	NavMeshConfig* config = navmesh->GetConfig();
	m_agentHeight = config->m_walkableHeight * config->m_ch; // voxels to world units
	m_agentRadius = config->m_walkableRadius * config->m_cs; // voxels to world units
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

	if (m_path && ! m_path->empty()) PlotPath();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::PlaceAgent                                                  |
//  |  Places the test agent at the given position.                         LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::PlaceAgent(float3 pos)
{
	mat4 move = mat4::Translate(pos);
	mat4 scale = mat4::Scale(make_float3(m_agentRadius * 2, m_agentHeight, m_agentRadius * 2));
	mat4 transform = move * scale;
	if (m_startInstID < 0)
		m_startInstID = m_renderer->AddInstance(m_agentMeshID, transform);
	else
		m_renderer->SetNodeTransform(m_startInstID, transform);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddNavMeshToScene                                           |
//  |  Adds the navmesh triangles to the scene as a single mesh.            LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddPolysToScene(NavMeshBuilder* navmesh)
{
	SaveAsMesh(navmesh);
	std::string filename = GetObjFileName(navmesh->GetConfig()->m_id);
	m_navmeshMeshID = m_renderer->AddMesh(filename.c_str(), m_dir, 1.0f);
	m_renderer->GetMesh(m_navmeshMeshID)->name = "NavMesh";
	m_navmeshInstID = m_renderer->AddInstance(m_navmeshMeshID, mat4::Identity());
	// TODO: remove .obj file (requires new platform function)
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveNavMeshFromScene                                      |
//  |  Removes the navmesh instance and mesh from the scene.                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemovePolysFromScene()
{
	if (m_navmeshInstID >= 0) m_renderer->RemoveInstance(m_navmeshInstID);
	m_navmeshInstID = -1;
	// TODO: Remove the old navmesh mesh to prevent memory leaks
	//if (m_navmeshMeshID >= 0) m_renderer->RemoveMesh(m_navmeshMeshID);
	//m_navmeshMeshID = -1;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddNodesToScene                                             |
//  |  Adds all precomputed navmesh vertices as spheres to the scene.       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddVertsToScene()
{
	for (std::vector<Vert>::iterator i = verts.begin(); i < verts.end(); i++)
		i->instID = m_renderer->AddInstance(m_vertMeshID, mat4::Translate(i->pos) * mat4::Scale(m_vertWidth));
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::RemoveVertsFromScene                                        |
//  |  Removes all vertex instances from the scene.                         LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::RemoveVertsFromScene()
{
	for (std::vector<Vert>::iterator i = verts.begin(); i < verts.end(); i++)
	{
		m_renderer->RemoveInstance(i->instID);
		i->instID = -1;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddEdgesToScene                                             |
//  |  Adds all precomputed navmesh edges as cylinders to the scene.        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddEdgesToScene()
{
	for (std::vector<Edge>::iterator i = edges.begin(); i < edges.end(); i++)
	{
		float3 v1 = verts[i->v1].pos, v2 = verts[i->v2].pos;
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
	for (std::vector<Edge>::iterator i = edges.begin(); i < edges.end(); i++)
	{
		m_renderer->RemoveInstance(i->instID);
		i->instID = -1;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ShadeTrianglesGL (TODO)                                     |
//  |  Plots all navmesh triangles using GL.                                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ShadePolysGL() const
{

}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ShadeVertsGL (TODO)                                         |
//  |  Plots all precomputed navmesh vertices using GL.                     LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ShadeVertsGL() const
{

}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ShadeEdgesGL (TODO)                                         |
//  |  Plots all precomputed polygon edges using GL.                        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ShadeEdgesGL() const
{

}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SelectPoly                                                  |
//  |  Highlights a polygon given a world position and a navmesh navigator. LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::SelectPoly(float3 pos, NavMeshNavigator* navmesh)
{
	Deselect();
	if (!navmesh) { m_polySelect = 0; return; }
	dtPolyRef ref;
	float3 posOnPoly;
	navmesh->FindNearestPoly(pos, &ref, &posOnPoly);
	m_polySelect = navmesh->GetPoly(ref);
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
		world.push_back(verts[m_polySelect->verts[i]].pos);
	m_renderer->GetCamera()->WorldToScreenPos(world.data(), screen.data(), world.size());

	DrawShapeOnScreen(screen, colors, GL_TRIANGLE_FAN);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SelectVert                                                  |
//  |  Hightlights a vertex instance.                                       LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::SelectVert(int instanceID)
{
	Deselect();
	if (instanceID < 0) return;
	int meshID = m_renderer->GetInstanceMeshID(instanceID);
	if (!isVert(meshID)) return;
	for (std::vector<Vert>::iterator i = verts.begin(); i < verts.end(); i++)
		if (i->instID == instanceID) m_vertSelect = new Vert(*i);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawVertHighlightGL                                         |
//  |  Draws a highlighted vertex on the screen.                            LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawVertHighlightGL() const
{
	std::vector<float2> vertices(1);
	m_renderer->GetCamera()->WorldToScreenPos(&m_vertSelect->pos, vertices.data(), 1);
	std::vector<float4> colors(1, m_highLightColor);

	DrawShapeOnScreen(vertices, colors, GL_POINTS, m_vertHighlightWidth);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::SelectEdge                                                  |
//  |  Hightlights an edge instance.                                        LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::SelectEdge(int instanceID)
{
	Deselect();
	if (instanceID < 0) return;
	int meshID = m_renderer->GetInstanceMeshID(instanceID);
	if (!isEdge(meshID)) return;
	for (std::vector<Edge>::iterator i = edges.begin(); i < edges.end(); i++)
		if (i->instID == instanceID) m_edgeSelect = new Edge(*i);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::DrawEdgeHighlightGL                                         |
//  |  Draws a highlighted edge on the screen.                              LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::DrawEdgeHighlightGL() const
{
	std::vector<float2> screen(2);
	std::vector<float4> colors(2, m_highLightColor);
	const float3 world[2] = { verts[m_edgeSelect->v1].pos , verts[m_edgeSelect->v2].pos };
	m_renderer->GetCamera()->WorldToScreenPos(world, screen.data(), 2);
	DrawShapeOnScreen(screen, colors, GL_LINE_STRIP, m_edgeHighlightWidth);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::PlotPath                                                    |
//  |  Plots the path calculated before as a series of lines                      |
//  |  using the renderer's viewpyramid to convert 3D to 2D.                LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::PlotPath() const
{
	// Allocating screen position array
	std::vector<float2> vertices;
	std::vector<float4> colors;
	vertices.resize(m_path->size() + 1);
	colors.resize(m_path->size() + 1, m_pathColor);

	// Converting world positions to screen positions
	Camera* view = m_renderer->GetCamera();
	view->WorldToScreenPos(m_path->data(), vertices.data() + 1, (int)m_path->size());
	view->WorldToScreenPos(&m_pathStart, &vertices[0], 1); // prepend start position

	DrawShapeOnScreen(vertices, colors, GL_LINE_STRIP, m_pathWidth);
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::Clean                                                       |
//  |  Resets the internal NavMesh representation and stops shading it.     LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::Clean()
{
	if (m_startInstID < 0) printf("NO KNOWN AGENT INST ID\n"); // DEBUG


	if (m_startInstID >= 0) m_renderer->RemoveInstance(m_startInstID);
	m_startInstID = -1;
	if (m_endInstID >= 0) m_renderer->RemoveInstance(m_endInstID);
	m_endInstID = -1;

	RemovePolysFromScene();
	RemoveVertsFromScene();
	RemoveEdgesFromScene();

	Deselect();
	verts.clear();
	edges.clear();

	m_path = 0;
	m_shadeTris = m_shadeVerts = m_shadeEdges = false;

	m_renderer->SynchronizeSceneData();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::AddEdgeToEdgesAndPreventDuplicates                          |
//  |  Helper function for NavMeshShader::ExtractVertsAndEdges.             LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::AddEdgeToEdgesAndPreventDuplicates(int v1, int v2, const dtPoly* poly)
{
	for (std::vector<Edge>::iterator i = edges.begin(); i < edges.end(); i++)
		if ((i->v1 == v1 && i->v2 == v2) || (i->v1 == v2 && i->v2 == v1))
		{
			i->poly2 = poly; // edge already exists, so add the second poly
			return;
		}
	edges.push_back({ v1, v2, -1, poly }); // add a new edge
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::ExtractVertsAndEdges                                        |
//  |  Makes a list of all vertices and edges to aid shading.               LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::ExtractVertsAndEdges(const dtNavMesh* mesh)
{
	for (int a = 0; a < mesh->getMaxTiles(); a++) // TODO: breaks for multiple tiles
	{
		// Adding Verts and their positions
		const dtMeshTile* tile = mesh->getTile(a);
		verts.reserve(tile->header->vertCount + tile->header->detailVertCount);
		for (int i = 0; i < tile->header->vertCount; ++i)
		{
			const float* v = &tile->verts[i * 3];
			verts.push_back({ { v[0], v[1], v[2] }, i });
		}
		for (int i = 0; i < tile->header->detailVertCount; ++i)
		{
			const float* v = &tile->detailVerts[i * 3];
			verts.push_back({ { v[0], v[1], v[2] }, i + tile->header->vertCount });
		}

		// Adding Vert polygon associations and Edges
		for (int b = 0; b < tile->header->polyCount; b++)
		{
			const dtPoly* poly = &tile->polys[b];
			for (int c = 0; c < poly->vertCount; c++)
			{
				verts[poly->verts[c]].polys.push_back(poly);
				if (c < poly->vertCount-1) // adding the first n-1 edges
					AddEdgeToEdgesAndPreventDuplicates(poly->verts[c], poly->verts[c + 1], poly);
				else // adding the last edge connecting the first vertex
					AddEdgeToEdgesAndPreventDuplicates(poly->verts[c], poly->verts[0], poly);
			}
		}
	}
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshShader::WriteMaterialFile                                           |
//  |  Writes a default .mtl file for the mesh.                             LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshShader::WriteMaterialFile()
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
void NavMeshShader::SaveAsMesh(NavMeshBuilder* navmesh)
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

} // namespace lighthouse2

// EOF