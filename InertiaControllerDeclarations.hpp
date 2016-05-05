/*
 * InertiaController.hpp
 *
 *  Created on: Mar 31, 2016
 *      Author: adam
 */



#include<stdint.h>

#pragma once

class InertiaPhysics
{
private:
	float m_pos;
	float m_vel;
	uint32_t m_timestamp;
	float m_lastForce;
	uint32_t m_forceStop;
	float m_minBound;
	float m_maxBound;
	float m_crashVelocity;
	void (*m_crashCallback)(const InertiaPhysics&); //Funkcja wywoływana, gdy zderzymy się ze ścianką ze zbyt dużą prędkością
#ifdef TASK_SCHEDULER
	Task m_task;
	friend void INERTIA_PHYSICS_CALLBACK();
#endif
public:
	InertiaPhysics(void (*crashCallback)(const InertiaPhysics&), float crashVelocity, float minBound=0, float maxBound=1, float startPos=0, float startVel=0);
	void update(uint32_t timestamp);
	void applyForce(float force, float time);
	float getPosition() const {return m_pos;}
	float getVelocity() const {return m_vel;}

	float getMaxBound() const {
		return m_maxBound;
	}

	float getMinBound() const {
		return m_minBound;
	}

	void setPos(float pos) {
		m_pos = pos;
	}

	void setVel(float vel) {
		m_vel = vel;
	}
#ifdef TASK_SCHEDULER
	Task& getTask() {return m_task;}
#endif


};

class InertiaController
{
private:
	float m_target;
	float m_maxForce;
	float m_maxVelocity;
	InertiaPhysics& m_physics;
#ifdef TASK_SCHEDULER
	Task m_task;
	friend void UpdateControler();
#endif
	static constexpr float s_timeAccuracy = 0.01;

public:
	InertiaController(InertiaPhysics& physics, float maxForce=0.01, float maxVelocity=1);

	void setMaxForce(float maxForce) {
		m_maxForce = maxForce;
	}

	void setTarget(float target) {
		m_target = target;
#ifdef TASK_SCHEDULER
		m_task.enableDelayed(0);
#endif
	}

	float getTarget() {
		return(m_target);
	}

	float updatePhysics();

	void setMaxVelocity(float maxVelocity) {
		m_maxVelocity = maxVelocity;
	}

	InertiaPhysics& getPhysics()
	{
		return(m_physics);
	}

#ifdef TASK_SCHEDULER
	Task& getTask() {return(m_task);}
#endif
};
