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
#include<math.h>
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
	float firstWheelCenterx = -10;//this is x position of center of first wheel
	float rodWidth = 1;
	float density = 1;
	float pi = 3.14159;
	float length_bigrectangle = 4*wheelRadius + 3;
	float breadth_bigrectangle = 10;
	float modifier = 7;
	float x_bigrectangle = firstWheelCenterx + wheelRadius*3;
	float y_bigrectangle = groundHeight + wheelRadius*2 + breadth_bigrectangle-2;
	float length_backrectangle = 8;
	float breadth_backrectangle = breadth_bigrectangle + wheelRadius/2;
	float x_backrectangle = x_bigrectangle - length_bigrectangle - length_backrectangle;
	float y_backrectangle = y_bigrectangle-wheelRadius/2;
	float height_exhaust = wheelRadius;
	float frontWidth = length_backrectangle -0;
	float frontHeight = (breadth_backrectangle - breadth_bigrectangle)*2;
  dominos_t::dominos_t()
  {
/*	  void keyboard(unsigned char key){
		  switch(key){
			  case 'm':
			  default :
				  base_sim_t::keyboard(key);*/
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
	wheelBodyDef.position.Set(firstWheelCenterx,groundHeight+wheelRadius);
	b2Body* wheelBody1 = (*m_world).CreateBody(&wheelBodyDef);
	b2FixtureDef wheelFixtureDef;
	wheelFixtureDef.density = density;
	wheelFixtureDef.filter.groupIndex = -2;	
//	wheelFixtureDef.restitution = 0;
	wheelFixtureDef.shape = &wheelShape;
	(*wheelBody1).CreateFixture(&wheelFixtureDef);
	//wheel is made!!
	//How about a second wheel
	wheelBodyDef.position.Set(firstWheelCenterx+gap, groundHeight+wheelRadius);//this is actually 0+gap
	b2Body* wheelBody2 = (*m_world).CreateBody(&wheelBodyDef);
	(*wheelBody2).CreateFixture(&wheelFixtureDef);
	//One more wheel (not for nothing)
	wheelBodyDef.position.Set(firstWheelCenterx+gap+gap, groundHeight+wheelRadius);
	b2Body* wheelBody3 = (*m_world).CreateBody(&wheelBodyDef);
	(*wheelBody3).CreateFixture(&wheelFixtureDef);

	//Second victory!!
	//Here starts the joint for a wheel
	/*Now, we need to make a rod between circle1 and circle3. Let point on circle1 be called a and point on circle 3 be called b. we connect a to c1 by using revolute joint, similarly for b and c3. This ensures that the system is setup. First work is to setup a and b. Qns are 
	1. How do you locate a, b(It must be responsive. So, no hardcoding)
	2. How can we make revolute joints?(We have to give anchor points relative to bodies)
	*/
	
	  ///chalo, chalo, chalo
	b2PolygonShape otrodShape;
	otrodShape.SetAsBox(gap, rodWidth/2);
	b2BodyDef otrodBodyDef;
	otrodBodyDef.type = b2_dynamicBody;
	otrodBodyDef.position.Set(firstWheelCenterx+(wheelRadius/2)+gap, wheelRadius*1.5+groundHeight);
	b2Body* otrodBody = (*m_world).CreateBody(&otrodBodyDef);
	b2FixtureDef otrodFixtureDef;
	otrodFixtureDef.density = 5*density;
	otrodFixtureDef.filter.groupIndex = -2;
	otrodFixtureDef.shape = &otrodShape;
	(*otrodBody).CreateFixture(&otrodFixtureDef);
	//This completes the rod between one and three 'd wheels


	//Joint joining them
	b2RevoluteJointDef orJointDef;
	orJointDef.bodyA = wheelBody1;
	orJointDef.bodyB = otrodBody;
	orJointDef.localAnchorA.Set(wheelRadius/2, wheelRadius/2);
	orJointDef.localAnchorB.Set(-gap,0);
	(b2RevoluteJoint*) m_world->CreateJoint(&orJointDef);
	//joint with first wheel is done

	//Joint joining them
//	b2RevoluteJointDef orJointDef;
	orJointDef.bodyA = wheelBody3;
//	orJointDef.bodyB = otrodBody;
//	orJointDef.localAnchorA.Set(wheelRadius/2, wheelRadius/2);
	orJointDef.localAnchorB.Set(gap,0);
	(b2RevoluteJoint*) m_world->CreateJoint(&orJointDef);
	//joint with third wheel is done

	//Here comes the holy joint connecting otrod and wheelBody2
	//here otrod acts as this holy joint
	otrodShape.SetAsBox(rodWidth/2,(wheelRadius/4)*sqrt(2));
	otrodBodyDef.position.Set(firstWheelCenterx+gap+wheelRadius/4, groundHeight+wheelRadius*(1+(1.0/4)));
	otrodBodyDef.angle = -pi/4; //angle is clockwise by default
	b2Body* holyrodBody = (*m_world).CreateBody(&otrodBodyDef);
//	(*holyrodBody).SetTransform(b2Vec2(firstWheelCenterx + gap + wheelRadius/4, groundHeight + wheelRadius*(1+0.25)), -pi/4);
	otrodFixtureDef.shape = &otrodShape;
	otrodFixtureDef.filter.groupIndex = 3;
	(*holyrodBody).CreateFixture(&otrodFixtureDef);
	
	//holyrodBody with wheelBody2
/*	orJointDef.bodyA = wheelBody2;
	orJointDef.bodyB = holyrodBody;
	orJointDef.localAnchorA.Set(0,0);
	orJointDef.localAnchorB.Set(-wheelRadius/4,-wheelRadius/4);
	(b2RevoluteJoint*) m_world->CreateJoint(&orJointDef);*/
	orJointDef.Initialize(wheelBody2, holyrodBody, (*wheelBody2).GetPosition());
	(*m_world).CreateJoint(&orJointDef);

/*	orJointDef.localAnchorA.Set(wheelRadius/2,wheelRadius/2);
	orJointDef.localAnchorB.Set(wheelRadius/4,wheelRadius/4);
	(b2RevoluteJoint*) (*m_world).CreateJoint(&orJointDef);*/
	orJointDef.Initialize(wheelBody2, holyrodBody, b2Vec2(firstWheelCenterx + gap + wheelRadius/2, groundHeight + wheelRadius + wheelRadius/2));
	(*m_world).CreateJoint(&orJointDef);
	orJointDef.Initialize(otrodBody, holyrodBody, b2Vec2(firstWheelCenterx + gap + wheelRadius/2, groundHeight + wheelRadius + wheelRadius/2));
	(*m_world).CreateJoint(&orJointDef);
	//holyrod is done with wheelBody2;
/*	
	//holyrod with otrodBody
	orJointDef.bodyA = otrodBody;
	orJointDef.localAnchorA.Set(0,0);
	orJointDef.localAnchorB.Set(wheelRadius/2,wheelRadius/4);
	(b2RevoluteJoint*) (*m_world).CreateJoint(&orJointDef);
*/


	/*b2RevoluteJointDef onethreeDef;
	onethreeDef.bodyA = wheelBody1;
	onethreeDef.bodyB = wheelBody3;
	onethreeDef.localAnchorA.Set(-wheelRadius/2,
*/
// Mahindar code ===============================================================================================================
		/*b2Vec2 rec_vertices[6];
	rec_vertices[0].Set(x_bigrectangle + length_bigrectangle,y_bigrectangle + breadth_bigrectangle);
	rec_vertices[1].Set(x_bigrectangle - length_bigrectangle,y_bigrectangle + breadth_bigrectangle);
	//rec_vertices[2].Set(x_bigrectangle - length_bigrectangle,y_bigrectangle - breadth_bigrectangle - wheelRadius);
	//rec_vertices[3].Set(x_bigrectangle - length_bigrectangle + modifier,y_bigrectangle - breadth_bigrectangle - wheelRadius);
	rec_vertices[2].Set(x_bigrectangle - length_bigrectangle ,y_bigrectangle - breadth_bigrectangle - wheelRadius);
	rec_vertices[3].Set(x_bigrectangle + length_bigrectangle ,y_bigrectangle - breadth_bigrectangle - wheelRadius);
	rec_vertices[4].Set(x_bigrectangle + length_bigrectangle - modifier,y_bigrectangle - breadth_bigrectangle - wheelRadius);
	rec_vertices[5].Set(x_bigrectangle + length_bigrectangle, y_bigrectangle - breadth_bigrectangle - wheelRadius);*/
	// This is for big bogey in train
	b2PolygonShape bigrectangle_shape;
	bigrectangle_shape.SetAsBox(length_bigrectangle,breadth_bigrectangle);
	b2BodyDef bigrectangle_def;
	bigrectangle_def.type = b2_dynamicBody;
	bigrectangle_def.position.Set(x_bigrectangle,y_bigrectangle);
	b2Body* bigrectangle_body = (*m_world).CreateBody(&bigrectangle_def);
	b2FixtureDef bigrectangle_fixture;
	bigrectangle_fixture.shape = &bigrectangle_shape;
	bigrectangle_fixture.density = 1.0;
	bigrectangle_fixture.filter.groupIndex = -2;
	(*bigrectangle_body).CreateFixture(&bigrectangle_fixture);
	
	// big rectangle wheel body joint def


	b2RevoluteJointDef br_wh_def1;
	br_wh_def1.Initialize(bigrectangle_body,wheelBody1,wheelBody1->GetPosition());
	b2RevoluteJointDef br_wh_def2;
	br_wh_def2.Initialize(bigrectangle_body,wheelBody2,wheelBody2->GetPosition());
	b2RevoluteJointDef br_wh_def3;
	br_wh_def3.Initialize(bigrectangle_body,wheelBody3,wheelBody3->GetPosition());
	m_world->CreateJoint(&br_wh_def1);
	m_world->CreateJoint(&br_wh_def2);
	m_world->CreateJoint(&br_wh_def3);
	/**m_world->CreateJoint(&br_wh_joint1);
	m_world->CreateJoint(&br_wh_joint2);
	m_world->CreateJoint(&br_wh_joint3);*/


	//realm of back rectangle

	b2PolygonShape backrectangle_shape;
	backrectangle_shape.SetAsBox(length_backrectangle,breadth_backrectangle);
	b2BodyDef backrectangle_def;
	backrectangle_def.type = b2_dynamicBody;
	backrectangle_def.position.Set(x_backrectangle,y_backrectangle);
	b2Body* backrectangle_body = (*m_world).CreateBody(&backrectangle_def);
	b2FixtureDef backrectangle_fixture;
	backrectangle_fixture.shape = &backrectangle_shape;
	backrectangle_fixture.density = 1.0;
	backrectangle_fixture.filter.groupIndex = -2;
	(*backrectangle_body).CreateFixture(&backrectangle_fixture);


	//big rectangle back rectangle joints
	
	
	b2Vec2 big_back_point1;
	big_back_point1.Set(x_backrectangle+length_backrectangle,y_backrectangle+breadth_backrectangle);
	b2RevoluteJointDef big_back_def1;
	big_back_def1.Initialize(bigrectangle_body,backrectangle_body,big_back_point1);
	m_world->CreateJoint(&big_back_def1);
	
	b2Vec2 big_back_point2;
	big_back_point2.Set(x_backrectangle + length_backrectangle,y_backrectangle - breadth_backrectangle);
	b2RevoluteJointDef big_back_def2;
	big_back_def2.Initialize(bigrectangle_body,backrectangle_body,big_back_point2);
	m_world->CreateJoint(&big_back_def2);

	/*b2Vec2 welding_point1;
	welding_point1.Set(x_backrectangle+length_backrectangle,y_backrectangle);
	b2WeldJointDef bir_bar_def1;
	bir_bar_def1.Initialize(bigrectangle_body,backrectangle_body,welding_point);
	m_world->CreateJoint(&bir_bar_def1);*/

	//Circle under the back rectangle


	b2CircleShape smallcircle_shape;
	smallcircle_shape.m_p.Set(0,0);
	smallcircle_shape.m_radius = wheelRadius/2;


	b2BodyDef smallcircle_def1;
	smallcircle_def1.type = b2_dynamicBody;
	smallcircle_def1.position.Set(x_backrectangle,groundHeight+wheelRadius/2);
	b2Body* smallcircle_body1 = (*m_world).CreateBody(&smallcircle_def1);



	b2FixtureDef smallcircle_fixture;
	smallcircle_fixture.shape = &smallcircle_shape;
	smallcircle_fixture.density = 1.0;
	smallcircle_fixture.filter.groupIndex = -2;
	(*smallcircle_body1).CreateFixture(&smallcircle_fixture);
	

	// Joint to connect back circle
	b2RevoluteJointDef sc_back_def;
	sc_back_def.Initialize(backrectangle_body,smallcircle_body1,smallcircle_body1->GetPosition());
	m_world->CreateJoint(&sc_back_def);
	//Mahindar code graciously ends=============================================================================================================
	
	bigrectangle_def.position.Set(firstWheelCenterx + 2*gap, y_bigrectangle +  breadth_bigrectangle + height_exhaust/2);
    bigrectangle_shape.SetAsBox(height_exhaust/2, height_exhaust/2);
	bigrectangle_fixture.filter.groupIndex = 1;
	bigrectangle_fixture.shape = &bigrectangle_shape;
	b2Body* exhaustBody = (*m_world).CreateBody(&bigrectangle_def);
	(*exhaustBody).CreateFixture(&bigrectangle_fixture);
	
	//..................................A new Start ....................................................
	//This is for right side body ...................................................||||||||||||||||||||||||||||
	backrectangle_shape.SetAsBox(rodWidth/2, breadth_backrectangle);
	backrectangle_def.position.Set(x_bigrectangle + length_bigrectangle + rodWidth/2, y_backrectangle);
	b2FixtureDef newFixture ; //this fixture without that groupIndex thing
	newFixture.shape = &backrectangle_shape;
	newFixture.density = 1;
	b2Body* frontl = (*m_world).CreateBody(&backrectangle_def);
	(*frontl).CreateFixture(&newFixture);
	/*//this makes the front left side of the front side
	||					 ||
	||this				 ||this is frontr
	||is frontl	         ||
	||--- box box box ---||
	||^					 ||
	||second height		 ||
	||2fw/4	2fw/4	2fw/4||
	||___	 _		____ ||
	||___|	|_|		|____||
	||^					 ||
	|||					 ||
	|||					 ||
	|||					 ||
	||frontheight		 ||
	|||					 ||
	|||					 ||
	|||					 ||
	||-__________________||
	||___________________||
		this is frontb*/
	//this width is frontWidth (actually halfed)
	//Here something the frontb
	backrectangle_shape.SetAsBox(frontWidth, rodWidth/2); //even frontWidth is half the original value
	backrectangle_def.position.Set(x_bigrectangle + length_bigrectangle + frontWidth, y_backrectangle-breadth_backrectangle + rodWidth/2);
	newFixture.shape = &backrectangle_shape;
	newFixture.density = 1;
	b2Body* frontb = (*m_world).CreateBody(&backrectangle_def);
	(*frontb).CreateFixture(&newFixture);
	//Once legend frontb is gone

	// Here starts the frontr
	backrectangle_shape.SetAsBox(rodWidth/2, breadth_backrectangle);
	backrectangle_def.position.Set(x_bigrectangle + length_bigrectangle - rodWidth/2 + frontWidth*2, y_backrectangle);
	newFixture.shape = &backrectangle_shape;
	b2Body* frontr = (*m_world).CreateBody(&backrectangle_def);
	(*frontr).CreateFixture(&newFixture);
	// Here ends the frontr

	//Now the rule of weld joints starts
	b2WeldJointDef weld;
	weld.Initialize(frontl, frontb, b2Vec2(x_bigrectangle+length_bigrectangle+rodWidth/2, y_backrectangle-breadth_backrectangle+rodWidth/2));//, b2Vec2(x_bigrectangle+length_bigrectangle+rodWidth/2, y_backrectangle-breadth_backrectangle+rodWidth/2));
	(*m_world).CreateJoint(&weld);
	weld.Initialize(frontr, frontb, b2Vec2(x_bigrectangle + length_bigrectangle + frontWidth*2 - rodWidth/2, y_backrectangle - breadth_backrectangle + rodWidth/2));//, b2Vec2(x_bigrectangle + length_bigrectangle + frontWidth*2 - rodWidth/2, y_backrectangle - breadth_backrectangle + rodWidth/2));
	(*m_world).CreateJoint(&weld);
	//Weld Joint has solved the problem of this welding. Have to see how long it works
	//Now starts the level which is frontHeight above the previous layer
	//Now the left most one
	backrectangle_shape.SetAsBox(frontWidth/4, rodWidth/2);
	backrectangle_def.position.Set(x_bigrectangle + length_bigrectangle + frontWidth/4, y_backrectangle - breadth_backrectangle + rodWidth + frontHeight + rodWidth/2);
	newFixture.shape = &backrectangle_shape;
	b2Body* ofirst = (*m_world).CreateBody(&backrectangle_def);
	(*ofirst).CreateFixture(&newFixture);
	//left most one is over

	//middle one
	backrectangle_shape.SetAsBox(frontWidth/4, rodWidth/2);
	backrectangle_def.position.Set(x_bigrectangle + length_bigrectangle + frontWidth, y_backrectangle - breadth_backrectangle + rodWidth + frontHeight + rodWidth/2);
	newFixture.shape = &backrectangle_shape;
	b2Body* osecond = (*m_world).CreateBody(&backrectangle_def);
	(*osecond).CreateFixture(&newFixture);
	//middle one is over

	backrectangle_shape.SetAsBox(frontWidth/4, rodWidth/2);
	backrectangle_def.position.Set(x_bigrectangle + length_bigrectangle + frontWidth*(7.0/4), y_backrectangle - breadth_backrectangle + rodWidth + frontHeight + rodWidth/2);
	newFixture.shape = &backrectangle_shape;
	b2Body* othird = (*m_world).CreateBody(&backrectangle_def);
	(*othird).CreateFixture(&newFixture);

	//now, we weld things by using distance joints
	weld.Initialize(frontl, ofirst, b2Vec2(x_bigrectangle+length_bigrectangle+rodWidth/2, y_backrectangle-breadth_backrectangle+rodWidth*(3.0/2)+frontHeight));
	(*m_world).CreateJoint(&weld);

	weld.Initialize(frontr, othird, b2Vec2(x_bigrectangle+length_bigrectangle+2*frontWidth-rodWidth/2, y_backrectangle-breadth_backrectangle+rodWidth*(3.0/2)+frontHeight));
	(*m_world).CreateJoint(&weld);
	
	//now, for the middle one we use distance joint because, we cannot afford to move it to weld

	b2DistanceJointDef distance;
	distance.Initialize(ofirst, osecond, b2Vec2(x_bigrectangle + length_bigrectangle + frontWidth/4, y_backrectangle-breadth_backrectangle + rodWidth*(3.0/2) + frontHeight), b2Vec2(x_bigrectangle + length_bigrectangle + frontWidth*(7.0/8), y_backrectangle-breadth_backrectangle + rodWidth*(3.0/2) + frontHeight));
	(*m_world).CreateJoint(&distance);


	distance.Initialize(othird, osecond, b2Vec2(x_bigrectangle + length_bigrectangle + 2*frontWidth-frontWidth/4, y_backrectangle-breadth_backrectangle + rodWidth*(3.0/2) + frontHeight), b2Vec2(x_bigrectangle + length_bigrectangle + frontWidth*(9.0/8), y_backrectangle-breadth_backrectangle + rodWidth*(3.0/2) + frontHeight));
	(*m_world).CreateJoint(&distance);

//Distance joint is done.

//Now, we have to fix the fontWindow to big rectangle
	big_back_def1.Initialize(bigrectangle_body, frontl, b2Vec2(x_bigrectangle + length_bigrectangle + rodWidth/2, y_backrectangle + breadth_backrectangle));
	(*m_world).CreateJoint(&big_back_def1);
	
	big_back_def1.Initialize(bigrectangle_body, frontl, b2Vec2(x_bigrectangle + length_bigrectangle + rodWidth/2, y_backrectangle - breadth_backrectangle));
	(*m_world).CreateJoint(&big_back_def1);






 	}
 	 sim_t *sim = new sim_t("Dominos", dominos_t::create);
 }
