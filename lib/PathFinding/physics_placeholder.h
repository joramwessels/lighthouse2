/* physics_placeholder.h - Copyright 2019 Utrecht University
   
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

//#include "common_types.h" // float3
#include "system.h" // float3

namespace lighthouse2 {

static const float s_drag = .2f;

//  +-----------------------------------------------------------------------------+
//  |  RigidBody                                                                  |
//  |  A simple rigid body class, meant to be replaced by a physics engine. LH2'19|
//  +-----------------------------------------------------------------------------+
class RigidBody
{
public:
	RigidBody() { m_alive = false; };
	RigidBody(float3 pos) : m_pos(pos), m_vel(make_float3(0)),
		m_linAcc(make_float3(0)), m_impulse(make_float3(0)) {}

	void AddImpulse(float3 impulse) { m_impulse += impulse; }
	void Update(float deltaTime)
	{
		return; // DEBUG
		m_impulse -= m_vel * s_drag;
		m_linAcc += m_impulse * deltaTime;
		m_vel += m_linAcc * deltaTime;
		m_pos += m_vel * deltaTime;
	}
	void Kill() { m_alive = false; };
	bool isAlive() const { return m_alive == true; };

	float3 m_pos, m_vel, m_linAcc, m_impulse;

private:
	bool m_alive = true;
};

//  +-----------------------------------------------------------------------------+
//  |  PhysicsPlaceholder                                                         |
//  |  A simple physics class, meant to be replaced by a physics engine.    LH2'19|
//  +-----------------------------------------------------------------------------+
class PhysicsPlaceholder
{
public:
	PhysicsPlaceholder(int maxBodies) : m_maxBodies(maxBodies)
	{
		m_bodies = new RigidBody[maxBodies];
		m_bodyCount = 0;
	}
	~PhysicsPlaceholder() { if (m_bodies) delete[] m_bodies; m_bodies = 0; };

	//  +-----------------------------------------------------------------------------+
	//  |  RigidBody::AddRB                                                           |
	//  |  Adds a new rigid body.                                               LH2'19|
	//  +-----------------------------------------------------------------------------+
	RigidBody* AddRB(float3 pos)
	{
		int idx;
		if (m_removedIdx.empty()) // no array holes
		{
			if (m_bodyCount < m_maxBodies) idx = m_bodyCount++; // capacity at the end
			else
			{   // looking for unknown dead bodies
				idx = -1;
				for (int i = 0; i < m_maxBodies; i++) if (!m_bodies[i].isAlive())
				{
					idx = i;
					break;
				}
				if (idx == -1) return 0; // none found, capacity full
			}
		}
		else // filling holes first
		{
			idx = m_removedIdx.back();
			m_removedIdx.pop_back();
		}
		m_bodies[idx] = RigidBody(pos);
		return &m_bodies[idx];
	}

	//  +-----------------------------------------------------------------------------+
	//  |  RigidBody::RemoveRB                                                        |
	//  |  Kills the specified rigid body.                                      LH2'19|
	//  +-----------------------------------------------------------------------------+
	void RemoveRB(RigidBody* rb)
	{
		int idx = (int)(rb - m_bodies) / sizeof(RigidBody);
		m_removedIdx.push_back(idx);
		rb->Kill();
	}


	//  +-----------------------------------------------------------------------------+
	//  |  RigidBody::Clean                                                           |
	//  |  Removes all rigid bodies.                                            LH2'19|
	//  +-----------------------------------------------------------------------------+
	void Clean()
	{
		m_bodyCount = 0;
	}


	//  +-----------------------------------------------------------------------------+
	//  |  RigidBody::Update                                                          |
	//  |  Called every physics tick. Updates all active rigid bodies.          LH2'19|
	//  +-----------------------------------------------------------------------------+
	void Update(float deltaTime)
	{
		for (int i = 0; i < m_bodyCount; i++)
			if (m_bodies[i].isAlive()) m_bodies[i].Update(deltaTime);
	}

protected:
	int m_maxBodies, m_bodyCount;
	RigidBody* m_bodies;
	std::vector<int> m_removedIdx;

private:
	// not meant to be copied
	PhysicsPlaceholder(const PhysicsPlaceholder&) = delete;
	PhysicsPlaceholder& operator=(const PhysicsPlaceholder&) = delete;
};

} // namespace lighthouse2

// EOF