/*
 * InertiaController.cpp
 *
 *  Created on: Mar 31, 2016
 *      Author: adam
 */
#include "InertiaController.hpp"
#include <Arduino.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//template <typename T> T abs(T val) {
//    return (T(0) < val) ? val : -val;
//}


InertiaController::InertiaController(float maxForce, float maxVelocity): m_target(0), m_maxForce(maxForce), m_maxVelocity(maxVelocity)
{ }

inline float howFarWeWillTravel(float x0, float v0, float a)
{
	return x0 + v0*v0 /2 /a;
}

//inline float min(float a, float b)
//{
//	if (a>b)
//		return b;
//	else
//		return a;
//}
//
//inline float max(float a, float b)
//{
//	if (a>b)
//		return a;
//	else
//		return b;
//}

float InertiaController::updatePhysics(InertiaPhysics& fader) //zwraca czas, na który dalsza aktualizacja nie będzie potrzebna
{
/*Najpierw musimy ocenić, gdzie się zatrzymamy przy zadanej max. sile hamowania. Jeśli zatrzymamy się przed targetem,
 * to możemy jeszcze przyspieszać. Jeśli osiągneliśmy max prędkość, ale jeszcze nie w targecie to kontynuujemy.
 * Jeśli za lub w targecie to zaczynamy zwalniać z odpowednią siłą (miejmy nadzieję, że nie większą niż maxForce
 */

	//Najpierw liczymy, gdzie się zatrzymamy, jeśli zaczniemy hamować już teraz

	float pos=fader.getPosition();
	float vel=fader.getVelocity();
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
	float x_d = x_b + x_c;
	float t_d = t_c + t_b;

	t_b = t_b + t_a;
	t_c = t_c + t_a;
	t_d = t_d + t_a;

	if (t_b > 0)
	{
		fader.applyForce(initialAccelerationVec, t_b);
		return(t_b);
	}
	if (t_c > 0)
	{
		fader.applyForce(0, t_c);
		return(t_c);
	}
	if (t_d > 0)
	{
		fader.applyForce(-initialAccelerationVec, t_d);
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
{

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



