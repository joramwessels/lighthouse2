/* debug_ui.h - Copyright 2019 Utrecht University
   
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

#include "navmesh_navigator.h"
#include "navmesh_shader.h"
#include "agent.h"

class AgentNavigationTool
{
public:
	AgentNavigationTool(NavMeshShader* shader)
		: m_shader(shader) {};
	~AgentNavigationTool() {};

	void SelectAgent(Agent* agent)
	{
		Clear();
		if (!agent) return;
		m_agent = agent;
		m_pathv0 = *agent->GetPos();

		if (agent->GetTarget()) // if it's already moving
		{
			m_pathv1 = *agent->GetTarget();
			m_pathSet = true;
			m_shader->SetPath(agent->GetPath());
			m_shader->SetPathStart(&m_pathv0);
			m_shader->SetPathEnd(&m_pathv1);
		}
		else // if it's standing still
		{
			m_pathSet = false;
			m_shader->SetPath(0);
			m_shader->SetPathStart(0);
			m_shader->SetPathEnd(0);
		}
	}
	void SetTarget(float3 pos)
	{
		if (!m_agent) return;
		m_pathv0 = *m_agent->GetPos();
		m_pathv1 = pos;
		m_pathSet = true;
		m_agent->SetTarget(pos);
		m_agent->UpdateNavigation(0);
		m_shader->SetPath(m_agent->GetPath());
		m_shader->SetPathStart(&m_pathv0);
		m_shader->SetPathEnd(&m_pathv1);
	}
	void Clear()
	{
		if (m_agent) // if there was an agent selected at all
		{
			m_shader->Deselect();
			m_shader->SetPath(0);
			m_shader->SetPathStart(0);
			m_shader->SetPathEnd(0);
		}
		m_agent = 0;
		m_pathSet = false;
	}

protected:
	NavMeshShader* m_shader;

	Agent* m_agent = 0;
	bool m_pathSet = false;
	float3 m_pathv0, m_pathv1;
};

class PathDrawingTool
{
public:
	PathDrawingTool(NavMeshShader* shader, NavMeshNavigator*& navmesh)
		: m_shader(shader), m_navmesh(navmesh) {};
	~PathDrawingTool() {};

	void SetStart(float3 pos)
	{
		m_v0 = pos;
		m_vertSet |= V0SET;
		m_shader->SetPathStart(&m_v0);
		if (m_vertSet == BOTHSET && m_navmesh)
			if (!m_navmesh->FindPath(m_v0, m_v1, m_path, m_reachable))
				m_shader->SetPath(&m_path);
	}
	void SetEnd(float3 pos)
	{
		m_v1 = pos;
		m_vertSet |= V1SET;
		m_shader->SetPathEnd(&m_v1);
		if (m_vertSet == BOTHSET && m_navmesh)
			if (!m_navmesh->FindPath(m_v0, m_v1, m_path, m_reachable))
				m_shader->SetPath(&m_path);
	}
	void Clear()
	{
		m_vertSet = NONESET;
		m_shader->SetPath(0);
		m_shader->SetPathStart(0);
		m_shader->SetPathEnd(0);
		m_path.clear();
	}

protected:
	NavMeshShader* m_shader;
	NavMeshNavigator*& m_navmesh; // navMeshNavigator in main_ui can reallocate

	float3 m_v0, m_v1;
	std::vector<NavMeshNavigator::PathNode> m_path;
	bool m_reachable;
	enum { V0SET = 0x1, V1SET = 0x2, BOTHSET = 0x3, NONESET = 0x0 };
	unsigned char m_vertSet = NONESET;
};