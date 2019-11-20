/* edit_ui.h - Copyright 2019 Utrecht University
   
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

#include "AntTweakBar.h"

#include "navmesh_builder.h"
#include "navmesh_shader.h"

//  +-----------------------------------------------------------------------------+
//  |  OffMeshConnectionTool                                                      |
//  |  Handles adding off mesh connections.                                 LH2'19|
//  +-----------------------------------------------------------------------------+
class OffMeshConnectionTool
{
public:
	OffMeshConnectionTool(NavMeshBuilder* builder, NavMeshShader* shader)
		: m_builder(builder), m_shader(shader) {};
	~OffMeshConnectionTool() {};

	//  +-----------------------------------------------------------------------------+
	//  |  OffMeshConnectionTool::SetStart                                            |
	//  |  Sets the first vertex of a new off-mesh connection.                  LH2'19|
	//  +-----------------------------------------------------------------------------+
	void SetStart(float3 pos)
	{
		m_v0 = pos;
		m_vertSet |= V0SET;
		if (m_vertSet == V0SET) m_shader->SetTmpVert(pos, m_defaultVertWidth);
		if (m_vertSet == BOTHSET) AddToScene();
	}

	//  +-----------------------------------------------------------------------------+
	//  |  OffMeshConnectionTool::SetEnd                                              |
	//  |  Sets the second vertex of a new off-mesh connection.                 LH2'19|
	//  +-----------------------------------------------------------------------------+
	void SetEnd(float3 pos)
	{
		m_v1 = pos;
		m_vertSet |= V1SET;
		if (m_vertSet == V1SET) m_shader->SetTmpVert(pos, m_defaultVertWidth);
		if (m_vertSet == BOTHSET) AddToScene();
	}

	//  +-----------------------------------------------------------------------------+
	//  |  OffMeshConnectionTool::Clear                                               |
	//  |  Resets the internal state and removes any leftovers.                 LH2'19|
	//  +-----------------------------------------------------------------------------+
	void Clear() { m_vertSet = NONESET; m_shader->RemoveTmpVert(); }

protected:
	NavMeshBuilder* m_builder;
	NavMeshShader* m_shader;

	float3 m_v0, m_v1;
	enum { V0SET = 0x1, V1SET = 0x2, BOTHSET = 0x3, NONESET = 0x0 };
	unsigned char m_vertSet = NONESET;
	const float m_defaultVertWidth = .5f;
	const bool m_defaultDirectionality = true;

	//  +-----------------------------------------------------------------------------+
	//  |  OffMeshConnectionTool::AddToScene                                          |
	//  |  Adds a new off-mesh connection to the builder and shader.            LH2'19|
	//  +-----------------------------------------------------------------------------+
	void AddToScene()
	{
		m_builder->AddOffMeshConnection(m_v0, m_v1, m_defaultVertWidth, m_defaultDirectionality);
		Clear();
		m_shader->AddTmpOMC(m_v0, m_v1, m_defaultVertWidth);
	}
};

class NavMeshSelectionTool
{
public:
	NavMeshSelectionTool(NavMeshShader* shader, NavMeshNavigator*& navigator, SELECTIONTYPE* selectionType)
		: m_shader(shader), m_navmesh(navigator), m_selectionType(selectionType) {};
	~NavMeshSelectionTool() {};

	void Deselect()
	{
		if (*m_selectionType == SELECTION_VERT ||
			*m_selectionType == SELECTION_EDGE ||
			*m_selectionType == SELECTION_POLY)
		{
			if (*m_selectionType == SELECTION_POLY) ApplyPolyChanges();
			m_shader->Deselect();
			m_selectionID = -1;
			TwDefine(" Editing/OffMesh visible=false ");
			TwDefine(" Editing/Detail visible=false ");
			TwDefine(" Editing/'Poly area' visible=false ");
			TwDefine(" Editing/Verts visible=false ");
			TwDefine(" Editing/v1 visible=false ");
			TwDefine(" Editing/v2 visible=false ");
			TwDefine(" Editing/v3 visible=false ");
			TwDefine(" Editing/v4 visible=false ");
			TwDefine(" Editing/v5 visible=false ");
			TwDefine(" Editing/flags visible=false ");
			*m_selectionType = SELECTION_NONE;
		}
	}

	void SelectVert(int instID)
	{
		Deselect();
		m_selectedVert = m_shader->SelectVert(instID);
		if (!m_selectedVert) return;
		m_selectionID = m_selectedVert->idx;

		m_isOffMesh = m_shader->isOffMeshVert(m_selectionID);
		m_isDetail = m_shader->isDetailVert(m_selectionID);
		TwDefine(" Editing/OffMesh visible=true ");
		TwDefine(" Editing/Detail visible=true ");
		m_verts[0] = *m_selectedVert->pos;
		TwDefine(" Editing/Verts visible=true ");
		TwDefine(" Editing/v0 visible=true ");

		*m_selectionType = SELECTION_VERT;
	}

	void SelectEdge(int instID)
	{
		Deselect();
		m_selectedEdge = m_shader->SelectEdge(instID);
		if (!m_selectedEdge) return;
		m_selectionID = m_selectedEdge->idx;

		m_isOffMesh = m_shader->isOffMeshVert(m_selectedEdge->v1);
		TwDefine(" Editing/OffMesh visible=true ");
		m_verts[0] = *m_shader->GetVertPos(m_selectedEdge->v1);
		m_verts[1] = *m_shader->GetVertPos(m_selectedEdge->v2);
		TwDefine(" Editing/Verts visible=true ");
		TwDefine(" Editing/v0 visible=true ");
		TwDefine(" Editing/v1 visible=true ");

		*m_selectionType = SELECTION_EDGE;
	}

	void SelectPoly(int triangleID)
	{
		Deselect();
		if (!m_navmesh) return;
		m_selectedPoly = m_shader->SelectPoly(triangleID);
		if (!m_selectedPoly) return;
		m_selectionID = m_selectedPoly->ref;

		m_polygonArea = m_selectedPoly->poly->getArea();
		for (int i = 0; i < NavMeshFlagMapping::maxFlags; i++)
			m_flags[i] = (m_selectedPoly->poly->flags & (0x1 << i));
		TwDefine(" Editing/flags visible=true ");
		TwDefine(" Editing/'Poly area' visible=true ");
		for (size_t i = 0; i < m_selectedPoly->poly->vertCount; i++)
			m_verts[i] = *m_shader->GetVertPos(m_selectedPoly->poly->verts[i]);
		TwDefine(" Editing/Verts visible=true ");
		TwDefine(" Editing/v0 visible=true ");
		TwDefine(" Editing/v1 visible=true ");
		TwDefine(" Editing/v2 visible=true ");
		if (m_selectedPoly->poly->vertCount > 3) TwDefine(" Editing/v3 visible=true ");
		if (m_selectedPoly->poly->vertCount > 4) TwDefine(" Editing/v4 visible=true ");
		if (m_selectedPoly->poly->vertCount > 5) TwDefine(" Editing/v5 visible=true ");

		*m_selectionType = SELECTION_POLY;
	}

	void ApplyPolyChanges()
	{
		// Apply flags and area type
		unsigned short polygonFlags = 0;
		for (int i = 0; i < NavMeshFlagMapping::maxFlags; i++) if (m_flags[i])
			polygonFlags |= (0x1 << i);
		m_navmesh->SetPolyFlags(m_selectedPoly->ref, polygonFlags);
		m_navmesh->SetAreaType(m_selectedPoly->ref, m_polygonArea);
	}

	const int* GetSelectionID() const { return &m_selectionID; };
	const float3* GetVert(int idx) const { if (idx < 0 || idx > 5) { return 0; } return &m_verts[idx]; };
	const bool* GetIsOffMesh() const { return &m_isOffMesh; };
	const bool* GetIsDetail() const { return &m_isDetail; };
	uint* GetPolyArea() { return &m_polygonArea; };
	bool* GetPolyFlag(int idx) { return &m_flags[idx]; };

protected:
	NavMeshShader* m_shader;
	NavMeshNavigator*& m_navmesh;
	SELECTIONTYPE* m_selectionType;
	const float3 origin = float3{ 0, 0, 0 };

	int m_selectionID = -1;
	float3 m_verts[6] = { origin, origin, origin, origin, origin, origin };
	bool m_isOffMesh = false, m_isDetail = false;
	uint m_polygonArea = 0;
	bool m_flags[NavMeshFlagMapping::maxFlags];

	NavMeshShader::Vert* m_selectedVert;
	NavMeshShader::Edge* m_selectedEdge;
	NavMeshShader::Poly* m_selectedPoly;
};

// EOF