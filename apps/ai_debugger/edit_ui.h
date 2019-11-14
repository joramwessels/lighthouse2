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

#include "navmesh_builder.h"
#include "navmesh_shader.h"

class OffMeshConnectionTool
{
public:
	OffMeshConnectionTool(NavMeshBuilder* builder, NavMeshShader* shader)
		: m_builder(builder), m_shader(shader) {};
	~OffMeshConnectionTool() {};

	void SetStart(float3 pos)
	{
		m_v0 = pos;
		m_vertSet |= V0SET;
		if (m_vertSet == V0SET) m_shader->SetTmpVert(pos, m_defaultVertWidth);
		if (m_vertSet == BOTHSET) AddToScene();
	}
	void SetEnd(float3 pos)
	{
		m_v1 = pos;
		m_vertSet |= V1SET;
		if (m_vertSet == V1SET) m_shader->SetTmpVert(pos, m_defaultVertWidth);
		if (m_vertSet == BOTHSET) AddToScene();
	}
	void Clear() { m_vertSet = NONESET; m_shader->RemoveTMPVert(); }

protected:
	NavMeshBuilder* m_builder;
	NavMeshShader* m_shader;

	float3 m_v0, m_v1;
	enum { V0SET = 0x1, V1SET = 0x2, BOTHSET = 0x3, NONESET = 0x0 };
	unsigned char m_vertSet = NONESET;
	const float m_defaultVertWidth = .5f;
	const bool m_defaultDirectionality = true;

	void AddToScene()
	{
		m_builder->AddOffMeshConnection(m_v0, m_v1, m_defaultVertWidth, m_defaultDirectionality);
		Clear();
		m_shader->AddTmpOMC(m_v0, m_v1, m_defaultVertWidth);
	}
};

