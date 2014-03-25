/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
//  float wheelRadius = 1;
	float screenWidth = 90; //This is used in main code. This is also half of what is used may be
	float groundHeight = 0.5; //this is half of the ground height because, function doubles it
	float pebbleHeight = groundHeight/2;//same as above. This is also half of it.
	int pebbleNumber = 15;
	float pebbleWidth = 2;
	float wheelRadius = 6;
	float gap = wheelRadius*3;//this is gap between centers of wheels
  dominos_t::dominos_t()
  {
	  //This is for creating ground with those brown steps

    b2Body* b1;  
    {
      
      b2PolygonShape groundShape; 
      groundShape.SetAsBox(screenWidth,groundHeight);
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd);
	  b2FixtureDef groundFixtureDef;
	  groundFixtureDef.shape = &groundShape;
	  groundFixtureDef.friction = 0.5;
      b1->CreateFixture(&groundFixtureDef);
	  b2BodyDef pebbleDef; //These brown rectangles at the bottom of picture are called pebbles
	  b2PolygonShape pebbleShape;
	  pebbleShape.SetAsBox(pebbleWidth,pebbleHeight);//
	  b2Body* pebbles[pebbleNumber];
	  pebbleDef.position.Set(-screenWidth,-groundHeight-pebbleHeight);
	  for(int i=0;i<pebbleNumber;i++){
		  pebbles[i] = (*m_world).CreateBody(&pebbleDef);
		  pebbleDef.position.Set(-screenWidth+(i+1)*(2*screenWidth/pebbleNumber), -groundHeight-pebbleHeight);
		 (*pebbles[i]).CreateFixture(&pebbleShape, 0.0f);
	  }
	  b2FixtureDef pebbleFixtureDef;
	  pebbleFixtureDef.shape = &pebbleShape;
    }
        //upto here  
	//Here starts remaking of wheel :P
	b2CircleShape wheelShape;
	wheelShape.m_p.Set(0,0);
	wheelShape.m_radius = wheelRadius;
	b2BodyDef wheelBodyDef;
	wheelBodyDef.type = b2_dynamicBody;		
	wheelBodyDef.position.Set(0,groundHeight+wheelRadius);
	b2Body* wheelBody1 = (*m_world).CreateBody(&wheelBodyDef);
	b2FixtureDef wheelFixtureDef;
//	wheelFixtureDef.restitution = 0;
	wheelFixtureDef.shape = &wheelShape;
	(*wheelBody1).CreateFixture(&wheelFixtureDef);
	//wheel is made!!
	//How about a second wheel
	wheelBodyDef.position.Set(gap, groundHeight+wheelRadius);//this is actually 0+gap
	b2Body* wheelBody2 = (*m_world).CreateBody(&wheelBodyDef);
	(*wheelBody2).CreateFixture(&wheelFixtureDef);
	//One more wheel (not for nothing)
	wheelBodyDef.position.Set(gap+gap, groundHeight+wheelRadius);
	b2Body* wheelBody3 = (*m_world).CreateBody(&wheelBodyDef);
	(*wheelBody3).CreateFixture(&wheelFixtureDef);

	//Second victory!!
	//Here starts the joint for a wheel
	b2PolygonShape concShape;
	concShape.SetAsBox(0.5,2);
	b2BodyDef concBodyDef;
	concBodyDef.type = b2_dynamicBody;
	concBodyDef.angle = -0.7;
	concBodyDef.position.Set(wheelRadius*2,wheelRadius);
	b2Body* concBody = (*m_world).CreateBody(&concBodyDef);
	b2FixtureDef concFixtureDef;
//	concFixtureDef.density = 1000;
	concFixtureDef.shape = &concShape;
	(*concBody).CreateFixture(&concFixtureDef);
	//Now, the joint between those two
	b2RevoluteJointDef concJointDef;
	concJointDef.bodyA = concBody;
	concJointDef.bodyB = wheelBody1;
	concJointDef.localAnchorB.Set(wheelRadius/2,wheelRadius/2);
	concJointDef.localAnchorA.Set(-1,-1);
	(b2RevoluteJoint*)m_world->CreateJoint(&concJointDef);
	/*Now, we need to make a rod between circle1 and circle3. Let point on circle1 be called a and point on circle 3 be called b. we connect a to c1 by using revolute joint, similarly for b and c3. This ensures that the system is setup. First work is to setup a and b. Qns are 
	1. How do you locate a, b(It must be responsive. So, no hardcoding)
	2. How can we make revolute joints?(We have to give anchor points relative to bodies)
	*/


  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
