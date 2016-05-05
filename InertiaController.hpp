/*
 * InertiaController.cpp
 *
 *  Created on: Mar 31, 2016
 *      Author: adam
 */

#include <Arduino.h>
#include <InertiaControllerDeclarations.hpp>

#ifdef TASK_SCHEDULER
extern Scheduler TASK_SCHEDULER;
#endif

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//template <typename T> T abs(T val) {
//    return (T(0) < val) ? val : -val;
//}

#ifdef TASK_SCHEDULER
void UpdateControler()
{
	Task& T = TASK_SCHEDULER.currentTask();
	InertiaController& me = *((InertiaController*) T.getLtsPointer());

	float time = me.updatePhysics()*1000;
	Serial.print(time);
	Serial.print(" = ");
	uint32_t czas;
	if (abs(time)<10)
		czas=10;
	else
		czas = time;
	czas=max(0,czas-10);
	if ((abs(me.m_physics.getPosition()-me.getTarget())<1 ) && (abs(me.m_physics.getVelocity())<0.1))
	{
		Serial.println("infinity");
		me.m_task.disable();
	}
	else
	{
		Serial.println(czas);
		me.m_task.enableDelayed(czas);
	}
}
#endif


InertiaController::InertiaController(InertiaPhysics& physics, float maxForce, float maxVelocity):
		m_target(0),
		m_maxForce(maxForce),
		m_maxVelocity(maxVelocity),
		m_physics(physics)
#ifdef TASK_SCHEDULER
		,m_task(TASK_NEVER, TASK_FOREVER, &UpdateControler, &TASK_SCHEDULER, true)
#endif
{
#ifdef TASK_SCHEDULER
	m_task.setLtsPointer(this);
#endif
}

float InertiaController::updatePhysics() //zwraca czas, na który dalsza aktualizacja nie będzie potrzebna
{
/*Najpierw musimy ocenić, gdzie się zatrzymamy przy zadanej max. sile hamowania. Jeśli zatrzymamy się przed targetem,
 * to możemy jeszcze przyspieszać. Jeśli osiągneliśmy max prędkość, ale jeszcze nie w targecie to kontynuujemy.
 * Jeśli za lub w targecie to zaczynamy zwalniać z odpowednią siłą (miejmy nadzieję, że nie większą niż maxForce
 */

	//Najpierw liczymy, gdzie się zatrzymamy, jeśli zaczniemy hamować już teraz

	float pos=m_physics.getPosition();
	float vel=m_physics.getVelocity();
	bool bPositiveDirection = pos<m_target;
	float initialAccelerationVec = bPositiveDirection ? m_maxForce : -m_maxForce;

	float x_a = pos-(vel*vel)/2/initialAccelerationVec;
	float t_a = -vel/initialAccelerationVec;
	float s_middle = (m_target - x_a)/2;
	float t_s_middle = sqrt(2*s_middle/initialAccelerationVec);
	float v_middle = t_s_middle *  initialAccelerationVec;
	bool v_middleGTmaxV = abs(v_middle)>m_maxVelocity;
	float v_max = max(min(v_middle,m_maxVelocity),-m_maxVelocity);
	float t_b = v_max/initialAccelerationVec;
	float x_b = initialAccelerationVec*(t_b*t_b)/2;
	float x_c = v_middleGTmaxV ? m_target - x_b : x_b;
	float t_c = t_b + abs(x_c-x_b)/m_maxVelocity;
	//float x_d = x_b + x_c;
	float t_d = t_c + t_b;

	t_b = t_b + t_a;
	t_c = t_c + t_a;
	t_d = t_d + t_a;

	if (t_b > 0)
	{
		m_physics.applyForce(initialAccelerationVec, t_b);
		return(t_b);
	}
	if (t_c > 0)
	{
		m_physics.applyForce(0, t_c);
		return(t_c);
	}
	if (t_d > 0)
	{
		m_physics.applyForce(-initialAccelerationVec, t_d);
		return(t_d);
	}
	return(0);
}


#define abs(x) ((x)>0?(x):-(x))



InertiaPhysics::InertiaPhysics(void (*crashCallback)(const InertiaPhysics&),
		float crashVelocity, float minBound, float maxBound, float startPos,
		float startVel):
		m_pos(startPos), m_vel(startVel), m_timestamp(0), m_lastForce(0), m_forceStop(0),
		m_minBound(minBound), m_maxBound(maxBound),m_crashVelocity(crashVelocity), m_crashCallback(crashCallback)
#ifdef TASK_SCHEDULER
		,m_task(TASK_NEVER, TASK_FOREVER, &INERTIA_PHYSICS_CALLBACK, &TASK_SCHEDULER, true)
#endif
{
#ifdef TASK_SCHEDULER
	m_task.setLtsPointer(this);
#endif
}


void InertiaPhysics::update(uint32_t timestamp)
{
	float dt = float(timestamp - m_timestamp)/1000.;
//	if (timestamp < m_forceStop)
//	{
//	}
	m_pos += m_vel * dt;
	m_vel += m_lastForce * dt;
	if (m_pos < m_minBound)
	{
		if (abs(m_vel)>m_crashVelocity)
			(*m_crashCallback)(*this);
		m_pos = m_minBound;
		m_vel = 0;
		m_lastForce = 0;
	} else if (m_pos > m_maxBound )
	{
		if (abs(m_vel)>m_crashVelocity)
			(*m_crashCallback)(*this);
		m_pos = m_maxBound;
		m_vel = 0;
		m_lastForce = 0;
	}
	m_timestamp = timestamp;
}

void InertiaPhysics::applyForce(float force, float time)
{
	m_forceStop = m_timestamp + uint32_t(time * 1000);
	m_lastForce = force;
}



