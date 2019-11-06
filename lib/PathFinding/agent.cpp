/* agent.cpp - Copyright 2019 Utrecht University
   
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
#include "agent.h"

namespace lighthouse2 {

static const float s_agentTargetReachedDistance = 0.1f;

//  +-----------------------------------------------------------------------------+
//  |  Agent::UpdateMovement                                                      |
//  |  Called after every physics update to update the direction.           LH2'19|
//  +-----------------------------------------------------------------------------+
bool Agent::UpdateMovement(float deltaTime)
{
	if (!m_pathEnd) return false;
	if (length(m_rb->m_pos - m_path[m_targetIdx]) < s_agentTargetReachedDistance) // target reached
		if (m_targetIdx < m_pathCount - 1) m_targetIdx++; // next path target
		else return m_pathEnd = 0; // final target reached
	m_moveDir = normalize(m_rb->m_pos - m_path[m_targetIdx]);
	m_rb->AddImpulse(m_moveDir * m_maxLinAcc);
	return true;
};

//  +-----------------------------------------------------------------------------+
//  |  Agent::UpdateNavigation                                                    |
//  |  Called every AI tick to recalculate the path.                        LH2'19|
//  +-----------------------------------------------------------------------------+
bool Agent::UpdateNavigation(float deltaTime)
{
	if (!m_pathEnd) return false;
	if (m_navmesh->FindPath(m_rb->m_pos, *m_pathEnd, m_path.data(), m_maxPathCount, &m_pathCount))
		m_targetIdx = 0;
	for (int i = m_pathCount; i < m_maxPathCount; i++) m_path[i] = *m_pathEnd;
	return true;
}





//  +-----------------------------------------------------------------------------+
//  |  NavMeshAgents::AddAgent                                                    |
//  |  Adds an agent.                                                       LH2'19|
//  +-----------------------------------------------------------------------------+
Agent* NavMeshAgents::AddAgent(NavMeshNavigator* navmesh, RigidBody* rb)
{
	int idx;
	if (m_removedIdx.empty()) // no array holes
	{
		if (m_agentCount < m_maxAgents) idx = m_agentCount++; // capacity at the end
		else
		{   // looking for unknown dead agents
			idx = -1;
			for (int i = 0; i < m_maxAgents; i++) if (!m_agents[i].isAlive())
			{
				idx = i;
				break;
			}
			if (idx == -1) return 0; // none found, capacity full
		}
	}
	else // filling known holes first
	{
		idx = m_removedIdx.back();
		m_removedIdx.pop_back();
	}
	m_agents[idx] = Agent(navmesh, rb, m_maxPathSize);
	return &m_agents[idx];
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAgents::AddAgent                                                    |
//  |  Kills the specified agent.                                           LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAgents::RemoveAgent(Agent* agent)
{
	agent->Clean();
	int idx = (int)(agent - m_agents.data()) / sizeof(Agent);
	m_removedIdx.push_back(idx);
	agent->Kill();
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAgents::UpdateAgentMovement                                         |
//  |  Called after every physics tick to add all agent movement impulses.  LH2'19|
//  +-----------------------------------------------------------------------------+
bool NavMeshAgents::UpdateAgentMovement(float deltaTime)
{
	for (std::vector<Agent>::iterator it = m_agents.begin(); it != m_agents.end(); it++)
		if (it->isAlive()) it->UpdateMovement(deltaTime);
	return true;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAgents::UpdateAgentBehavior                                         |
//  |  Called every tick to update all agent plans.                               |
//  |  Only actually updates at the interval given to the constructor.      LH2'19|
//  +-----------------------------------------------------------------------------+
bool NavMeshAgents::UpdateAgentBehavior(float deltaTime)
{
	m_timeCounter += deltaTime;
	if (m_timeCounter < m_updateTimeInterval) return false;
	for (std::vector<Agent>::iterator it = m_agents.begin(); it != m_agents.end(); it++)
		if (it->isAlive()) it->UpdateNavigation(m_timeCounter);
	m_timeCounter = 0;
	return true;
}

//  +-----------------------------------------------------------------------------+
//  |  NavMeshAgents::Clean                                                       |
//  |  Removes all agents.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void NavMeshAgents::Clean()
{
	for (int i = 0; i < m_agentCount; i++) if (m_agents[i].isAlive()) m_agents[i].Clean();
	m_agentCount = 0;
}

} // namespace lighthouse2

// EOF