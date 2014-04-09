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

#include <math.h>
#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
#include<iostream>
//using namespace std;

#include "dominos.hpp"
#include <math.h>
namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
//  float wheelRadius = 1;
	
	float screenWidth = 90; //This is used in main code. This is also half of what is used may be
	float groundHeight = 0.5; //this is half of the ground height because, function doubles it
	

	float wheelRadius = 6;
	float gap = wheelRadius*3;//this is gap between centers of wheels
	float firstWheelCenterx = -20;//this is x position of center of first wheel


	float rodWidth = 1;

	float density = 1;
	float pi = 3.14159;

	float length_bigrectangle = 4*wheelRadius + 6;//+6
	float breadth_bigrectangle = 10;

	float modifier = 7;
	float x_bigrectangle = firstWheelCenterx + wheelRadius*3 + 4;//+3
	float y_bigrectangle = groundHeight + wheelRadius*2 + breadth_bigrectangle-2;
	float length_backrectangle = 8;
	float breadth_backrectangle = breadth_bigrectangle + wheelRadius/2;
	float x_backrectangle = x_bigrectangle - length_bigrectangle - length_backrectangle ;
	float y_backrectangle = y_bigrectangle-wheelRadius/2;


	//	float frontWidth = length_backrectangle + 0.25;//it was actually 0.25 have to see which one is better
	float frontWidth = wheelRadius*sqrt(2);	
	float frontHeight = (breadth_backrectangle - breadth_bigrectangle)*2 + 2;


	float stabbingRodLength = 14;  //#testing
	float stabbingWidth = 2;
	float intraStabberWidth = 2*stabbingWidth;
	float stabberLength = 4;
	// Now, we have to position things such that a + b + p + firstWheelCenterx + gap = exco + m + d + c. So, p must be some big expression.Since p depends on initial location of pumpingRod, we have to set it such that p gets the desired value
	float whyco = y_bigrectangle - breadth_bigrectangle - stabbingWidth - intraStabberWidth/2 - rodWidth - frontHeight/2;
	float exco = x_bigrectangle + length_bigrectangle; //this is x coordinate of rock left of front side
	float stabberLag = 2.6; //this is the difference of x co ordinate of left end of frontl and right end of frontr
	float stabberx = exco - stabberLength/2 - stabberLag;
	float stabbery = y_bigrectangle - breadth_bigrectangle - stabbingWidth - rodWidth/2 - (intraStabberWidth-rodWidth)/2;



	//we have to ensure that ml + dl + cl + dl + ml is equal to l i.e., 2*frontWidth
	float ml = (2*frontWidth)/12;
	float dl = (2*frontWidth)/6;
	float cl = (2*frontWidth)/2;

	float frontLag = (1.0/10)*frontWidth;  //Let the lag between pumpingRod and frontr and frontl is 1/10 of frontWidth presently
	float lag = 0.1; //this is the difference between bottom of osecond and top of pumpingRod
	float widthPR = rodWidth*2; //this is the width of pumping rod


	//sfirst
	float secondHeight = frontHeight/2;
	float holeWidth = 2*frontWidth/6.0; //this is the width of hole at the second level.
	float sLength = (2*frontWidth - holeWidth)/2; //this is length of a plate at second level


//Bodies used globally
	b2Body* b1;  

	b2Body* wheelBody1;
    b2Body* wheelBody2;
    b2Body* wheelBody3;

    b2Body* otrodBody;
	b2Body* holyrodBody ;

	b2Body* bigrectangle_body;//!< Body object of Big Rectangle
	b2Body* backrectangle_body;//!< Body object of Back Rectangle

	b2Body* frontl;
	b2Body* frontb;
	b2Body* frontr;


	b2Body* ofirst;
	b2Body* osecond;

	b2Body* pumpingRod;

	b2Body* firstStab;
	b2Body* secondStab;

	b2Body* stabber;

	b2Body* pulley;

	b2Body* holder;
	b2Body* lcircle;
	b2Body* lrod;	
	b2Body* lnextcircle;

	b2Body* hcircle;

	b2Body* rightrod;

	b2Body* connector;


	
	

//Body Defs used globally
	b2BodyDef otrodBodyDef;  //!< \b Type : \b b2BodyDef .It is a definition for a body which is used to create a body object i.e. determining the position,dynamics
	b2BodyDef bigrectangle_def; //!< \b Type : \b b2BodyDef .It is a definition for a body which is used to create a body object i.e. determining the position,dynamics
	b2BodyDef backrectangle_def;//!< \b Type : \b b2BodyDef .It is a definition for a body which is used to create a body object i.e. determining the position,dynamics


//Body Shapes used globally
	b2PolygonShape otrodShape;//!< \b Type : \b b2PolygonShape .It is used to define a shape to polygon only convex
	b2PolygonShape bigrectangle_shape;//!< \b Type : \b b2PolygonShape .It is used to define a shape to polygon only convex
	b2PolygonShape backrectangle_shape;//!< \b Type : \b b2PolygonShape .It is used to define a shape to polygon only convex

//Body Fixtures used globally
	b2FixtureDef otrodFixtureDef;//!< \b Type : \b b2BodyFixtureDef .It acts like a cover for a body and gives bodies properties like shape, density, friction
	b2FixtureDef bigrectangle_fixture;//!< \b Type : \b b2BodyFixtureDef .It acts like a cover for a body and gives bodies properties like shape, density, friction
	b2FixtureDef backrectangle_fixture;//!< \b Type : \b b2BodyFixtureDef .It acts like a cover for a body and gives bodies properties like shape, density, friction
	b2FixtureDef newFixture ; //!< \b Type : \b b2BodyFixtureDef .It acts like a cover for a body and gives bodies properties like shape, density, friction

//Revolute Joints used globally
	b2RevoluteJointDef orJointDef; //!< \b Type : \b b2RevoluteJointDef . It is used to create a joint that restrictsmotion of a bodyB w.r.t bodyA allowing only rotation
	b2RevoluteJointDef big_back_def1;//!< \b Type : \b b2RevoluteJointDef . It is used to create a joint that restrictsmotion of a bodyB w.r.t bodyA allowing only rotation
	b2RevoluteJointDef big_back_def2;//!< \b Type : \b b2RevoluteJointDef . It is used to create a joint that restrictsmotion of a bodyB w.r.t bodyA allowing only rotation


//Weld Joints used globally;
	b2WeldJointDef weld;

//Prismatic Joints used globally;
	b2PrismaticJointDef psjointtest;
	cs296::base_sim_t* a = new base_sim_t();
  dominos_t::dominos_t()
  {
//	(*a).pumpingRod = pumpingRod;
//	m_world->b2World::SetContactListener(a);

	/*	  void keyboard(unsigned char key){
		  switch(key){
			  case 'm':
			  default :
				  base_sim_t::keyboard(key);*/
	  //This is for creating ground with those brown steps

    
    {
      
      b2PolygonShape groundShape; 
      groundShape.SetAsBox(4000,groundHeight);
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd);
	  b2FixtureDef groundFixtureDef;
	  groundFixtureDef.shape = &groundShape;
	  groundFixtureDef.friction = 0.5;
      b1->CreateFixture(&groundFixtureDef);
	  
	  
    }



    

  	// The three wheels are created here
	{
		b2CircleShape wheelShape;
		wheelShape.m_p.Set(0,0);
		wheelShape.m_radius = wheelRadius;
		b2BodyDef wheelBodyDef;
		wheelBodyDef.type = b2_dynamicBody;		
		wheelBodyDef.position.Set(firstWheelCenterx,groundHeight+wheelRadius);
		 wheelBody1 = (*m_world).CreateBody(&wheelBodyDef);
		b2FixtureDef wheelFixtureDef;
		wheelFixtureDef.density = density;
		wheelFixtureDef.filter.groupIndex = -2;	
		wheelFixtureDef.shape = &wheelShape;
		(*wheelBody1).CreateFixture(&wheelFixtureDef);
		wheelBodyDef.position.Set(firstWheelCenterx+gap, groundHeight+wheelRadius);//this is actually 0+gap
		 wheelBody2 = (*m_world).CreateBody(&wheelBodyDef);
		(*wheelBody2).CreateFixture(&wheelFixtureDef);
		wheelBodyDef.position.Set(firstWheelCenterx+gap+gap, groundHeight+wheelRadius);
		 wheelBody3 = (*m_world).CreateBody(&wheelBodyDef);
		(*wheelBody3).CreateFixture(&wheelFixtureDef);
	}

	



	
	
	

	//Rod between wheel 1 and wheel 3
	{	
		otrodShape.SetAsBox(gap, rodWidth/2);	
		otrodBodyDef.type = b2_dynamicBody;
		otrodBodyDef.angle = 0;
		otrodBodyDef.position.Set(firstWheelCenterx+(wheelRadius/2)+gap, wheelRadius*1.5+groundHeight);
		otrodBody = (*m_world).CreateBody(&otrodBodyDef);
		otrodFixtureDef.density = 1;  //<--------------------------------------------------------------------------------------------------->
		otrodFixtureDef.filter.groupIndex = -2;
		otrodFixtureDef.shape = &otrodShape;
		(*otrodBody).CreateFixture(&otrodFixtureDef);
		otrodFixtureDef.density = 1;



		orJointDef.bodyA = wheelBody1;
		orJointDef.bodyB = otrodBody;
		orJointDef.localAnchorA.Set(wheelRadius/2, wheelRadius/2);
		orJointDef.localAnchorB.Set(-gap,0);
		(b2RevoluteJoint*) m_world->CreateJoint(&orJointDef);
		//joint with first wheel is done

		orJointDef.bodyA = wheelBody3;
		orJointDef.localAnchorB.Set(gap,0);
		(b2RevoluteJoint*) m_world->CreateJoint(&orJointDef);
	}
	


	//Here comes the holy joint connecting otrod and wheelBody2
	//here otrod acts as this holy joint
	{
		otrodShape.SetAsBox(rodWidth/2,(wheelRadius/4)*sqrt(2));
		otrodBodyDef.position.Set(firstWheelCenterx+gap+wheelRadius/4, groundHeight+wheelRadius*(1+(1.0/4)));
		otrodBodyDef.angle = -pi/4; //angle is clockwise by default
		holyrodBody = (*m_world).CreateBody(&otrodBodyDef);
		otrodFixtureDef.shape = &otrodShape;
		otrodFixtureDef.filter.groupIndex = -2;
		(*holyrodBody).CreateFixture(&otrodFixtureDef);

		orJointDef.Initialize(wheelBody2, holyrodBody, (*wheelBody2).GetPosition());
		orJointDef.motorSpeed = 10.0f;
		orJointDef.maxMotorTorque = 400.0f;
		orJointDef.enableMotor = true;
		(*m_world).CreateJoint(&orJointDef);
		orJointDef.enableMotor = false;	

		orJointDef.Initialize(wheelBody2, holyrodBody, b2Vec2(firstWheelCenterx + gap + wheelRadius/2, groundHeight + wheelRadius + wheelRadius/2));
		(*m_world).CreateJoint(&orJointDef);
		orJointDef.Initialize(otrodBody, holyrodBody, b2Vec2(firstWheelCenterx + gap + wheelRadius/2, groundHeight + wheelRadius + wheelRadius/2));
		(*m_world).CreateJoint(&orJointDef);
	}


	/*! \section Big Rectangle
	* \brief It is the middle big box of the train
	*/
	{
		bigrectangle_shape.SetAsBox(length_bigrectangle,breadth_bigrectangle);
		bigrectangle_def.type = b2_dynamicBody;
		bigrectangle_def.position.Set(x_bigrectangle , y_bigrectangle);
		bigrectangle_body = (*m_world).CreateBody(&bigrectangle_def);

		
		bigrectangle_fixture.shape = &bigrectangle_shape;
		bigrectangle_fixture.density = 1; //this is a changed thing #added
		bigrectangle_fixture.filter.groupIndex = -2;
		(*bigrectangle_body).CreateFixture(&bigrectangle_fixture);	
		b2RevoluteJointDef br_wh_def1; //!< \b Type : \b b2RevoluteJointDef . It is used to create a joint that restricts motion of a bodyB w.r.t bodyA allowing only rotation		
		br_wh_def1.Initialize(bigrectangle_body,wheelBody1,wheelBody1->GetPosition());
		b2RevoluteJointDef br_wh_def2;//!< \b Type : \b b2RevoluteJointDef . It is used to create a joint that restricts motion of a bodyB w.r.t bodyA allowing only rotation
		br_wh_def2.Initialize(bigrectangle_body,wheelBody2,wheelBody2->GetPosition());
		b2RevoluteJointDef br_wh_def3;//!< \b Type : \b b2RevoluteJointDef . It is used to create a joint that restrictsmotion of a bodyB w.r.t bodyA allowing only rotation
		br_wh_def3.Initialize(bigrectangle_body,wheelBody3,wheelBody3->GetPosition());
		m_world->CreateJoint(&br_wh_def1);
		m_world->CreateJoint(&br_wh_def2);
		m_world->CreateJoint(&br_wh_def3);
	}

	
	
	
	
	
	/*! \section Back Rectangle
	* \brief It is the back box of the train
	*/
	{

		
		backrectangle_shape.SetAsBox(length_backrectangle,breadth_backrectangle);		
		backrectangle_def.type = b2_dynamicBody;
		backrectangle_def.position.Set(x_backrectangle,y_backrectangle);
		backrectangle_body = (*m_world).CreateBody(&backrectangle_def);		
		backrectangle_fixture.shape = &backrectangle_shape;
		backrectangle_fixture.density = 1.0;
		backrectangle_fixture.filter.groupIndex = -2;
		(*backrectangle_body).CreateFixture(&backrectangle_fixture);		
		
		b2Vec2 big_back_point1;//!< \b Type : \b b2vec2 .It store x and y co-ordinates
		big_back_point1.Set(x_backrectangle+length_backrectangle,y_backrectangle+breadth_backrectangle);		
		big_back_def1.Initialize(bigrectangle_body,backrectangle_body,big_back_point1);
		m_world->CreateJoint(&big_back_def1);		
		b2Vec2 big_back_point2;//!< \b Type : \b b2vec2 .It store x and y co-ordinates
		big_back_point2.Set(x_backrectangle + length_backrectangle,y_backrectangle - breadth_backrectangle);		
		big_back_def2.Initialize(bigrectangle_body,backrectangle_body,big_back_point2);
		m_world->CreateJoint(&big_back_def2);

	}



	//Circle under the back rectangle
	/*! \section Small Circle 
	* \brief Small Circle acts as a wheel which is below the Back rectangle and is connected to it using revolute joint
	*/
	{
		b2CircleShape smallcircle_shape;//!< \b Type : \b b2CircleShape .It creates a circle shape whose radius is given by us
		smallcircle_shape.m_p.Set(0,0);
		smallcircle_shape.m_radius = wheelRadius/2;

		b2BodyDef smallcircle_def1;//!< \b Type : \b b2BodyDef .It is a definition for a body which is used to create a body object i.e. determining the position,dynamics
		smallcircle_def1.type = b2_dynamicBody;
		smallcircle_def1.position.Set(x_backrectangle,groundHeight+wheelRadius/2);
		b2Body* smallcircle_body1 = (*m_world).CreateBody(&smallcircle_def1);//!< Body object of Small circle below the back rectangle

		b2FixtureDef smallcircle_fixture;//!< \b Type : \b b2BodyFixtureDef .It acts like a cover for a body and gives bodies properties like shape, density, friction
		smallcircle_fixture.shape = &smallcircle_shape;
		smallcircle_fixture.density = 1.0;
		smallcircle_fixture.filter.groupIndex = -2;
		(*smallcircle_body1).CreateFixture(&smallcircle_fixture);
		
		b2RevoluteJointDef sc_back_def;//!< \b Type : \b b2RevoluteJointDef . It is used to create a joint that restrictsmotion of a bodyB w.r.t bodyA allowing only rotation
		sc_back_def.Initialize(backrectangle_body,smallcircle_body1,smallcircle_body1->GetPosition());
		m_world->CreateJoint(&sc_back_def);
	}



	/*//this makes the front left side of the front side
	||					 ||
	||this				 ||this is frontr
	||is frontl	         ||
	||--- box box box ---||
	||^					 ||
	||second height		 ||
	||2fw/4	2fw/3	2fw/4||
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
	
	
		
		
	//frontl
	{
		backrectangle_shape.SetAsBox(rodWidth/2, (y_bigrectangle + breadth_bigrectangle - whyco)/2);
		backrectangle_def.position.Set(exco + rodWidth/2, (whyco + y_bigrectangle + breadth_bigrectangle)/2);		
		newFixture.filter.groupIndex = -2;
		newFixture.shape = &backrectangle_shape;
		newFixture.density = 1;
		newFixture.restitution = 1;          //#testing
		newFixture.filter.categoryBits = 0x0004;
		newFixture.filter.maskBits = 0x0002;
		frontl = (*m_world).CreateBody(&backrectangle_def);
		(*frontl).CreateFixture(&newFixture);
		newFixture.restitution = 0;        //#testing	

	//frontb
	
		backrectangle_shape.SetAsBox(frontWidth, rodWidth/2); //even frontWidth is half the original value
		backrectangle_def.position.Set(exco + frontWidth, whyco + rodWidth/2);
		newFixture.filter.groupIndex = 0;
		newFixture.shape = &backrectangle_shape;
		newFixture.density = 1;
		newFixture.friction = 0; //#testing
		frontb = (*m_world).CreateBody(&backrectangle_def);
		(*frontb).CreateFixture(&newFixture);
		newFixture.friction = 0;  //#testing
	

	//frontr
	
		backrectangle_shape.SetAsBox(rodWidth/2, (y_bigrectangle + breadth_bigrectangle - whyco)/2);
		backrectangle_def.position.Set(exco - rodWidth/2 + frontWidth*2, (whyco + y_bigrectangle + breadth_bigrectangle)/2);
		newFixture.shape = &backrectangle_shape;
		newFixture.restitution = 1;
		frontr = (*m_world).CreateBody(&backrectangle_def);
		(*frontr).CreateFixture(&newFixture);
		newFixture.restitution = 0;	

	//NWeld joints of frontr frontl frontb	
	
		weld.Initialize(frontl, frontb, b2Vec2(exco+rodWidth/2, whyco+rodWidth/2));//, b2Vec2(exco+rodWidth/2, whyco+rodWidth/2));
		(*m_world).CreateJoint(&weld);
		weld.Initialize(frontr, frontb, b2Vec2(exco + frontWidth*2 - rodWidth/2, whyco + rodWidth/2));//, b2Vec2(exco + frontWidth*2 - rodWidth/2, whyco + rodWidth/2));
		(*m_world).CreateJoint(&weld);

	//Connecting to Big rectangle
		big_back_def1.Initialize(bigrectangle_body, frontl, b2Vec2(exco + rodWidth/2, y_backrectangle + breadth_backrectangle));
		(*m_world).CreateJoint(&big_back_def1);
		
		big_back_def1.Initialize(bigrectangle_body, frontl, b2Vec2(exco + rodWidth/2, whyco));
		(*m_world).CreateJoint(&big_back_def1);
	}


	
	
	

	//ofirst the leftmost middle thing in frontbox
	{
		backrectangle_shape.SetAsBox(ml/2, rodWidth/2); //reducing frontWidth/4 to frontWidth/6 both on ofirst and othird
		backrectangle_def.position.Set(exco + ml/2, whyco + rodWidth + frontHeight + rodWidth/2);
		newFixture.shape = &backrectangle_shape;
		ofirst = (*m_world).CreateBody(&backrectangle_def);
		(*ofirst).CreateFixture(&newFixture);

		//middle one	
		backrectangle_shape.SetAsBox(cl/2, rodWidth/2);
		backrectangle_def.position.Set(exco + ml + dl + cl/2, whyco + rodWidth + frontHeight + rodWidth/2);
		newFixture.shape = &backrectangle_shape;
		osecond = (*m_world).CreateBody(&backrectangle_def);
		(*osecond).CreateFixture(&newFixture);

		
		psjointtest.bodyA = frontb;
		psjointtest.bodyB = osecond;
		psjointtest.localAxisA = b2Vec2(1,0);
		psjointtest.localAnchorB = b2Vec2(0,0);
		psjointtest.localAnchorA = b2Vec2(0, frontHeight + rodWidth);
		(*m_world).CreateJoint(&psjointtest);
	

		backrectangle_shape.SetAsBox(ml/2, rodWidth/2);
		backrectangle_def.position.Set(exco + ml + dl + cl + dl + ml/2, whyco + rodWidth + frontHeight + rodWidth/2);
		newFixture.shape = &backrectangle_shape;
		newFixture.density = 0;
		b2Body* othird = (*m_world).CreateBody(&backrectangle_def);
		(*othird).CreateFixture(&newFixture);

		weld.Initialize(frontl, ofirst, b2Vec2(exco+rodWidth/2, whyco+rodWidth*(3.0/2)+frontHeight));
		(*m_world).CreateJoint(&weld);
		weld.Initialize(frontr, othird, b2Vec2(exco+2*frontWidth-rodWidth/2, whyco+rodWidth*(3.0/2)+frontHeight));
		(*m_world).CreateJoint(&weld);
		

		b2DistanceJointDef distance;
		distance.Initialize(ofirst, osecond, b2Vec2(exco + frontWidth/4, whyco + rodWidth*(3.0/2) + frontHeight), b2Vec2(exco + frontWidth*(7.0/8), whyco + rodWidth*(3.0/2) + frontHeight));
		(*m_world).CreateJoint(&distance);
		distance.Initialize(othird, osecond, b2Vec2(exco + 2*frontWidth-frontWidth/4, whyco + rodWidth*(3.0/2) + frontHeight), b2Vec2(exco + frontWidth*(9.0/8), whyco + rodWidth*(3.0/2) + frontHeight));
		(*m_world).CreateJoint(&distance);
	}


		
	
	//This is the pumpingRod

	
	{
		backrectangle_shape.SetAsBox(widthPR/2, (frontHeight- lag)/2); //take care of this width of this rod is double the original rodWidth
		float a = (wheelRadius/2)*sqrt(2);
		float b = sqrt(pow((firstWheelCenterx + gap + wheelRadius/2 - stabberx), 2) + pow(((*wheelBody2).GetWorldCenter().y + wheelRadius/2 - stabbery),2));
		//cout << b << endl;
		//cout << exco + ml + dl + cl - a - b - firstWheelCenterx - gap << endl;
		float p = exco + ml + dl + cl - firstWheelCenterx - gap - a - b;//this is going to be the length of pusher
		backrectangle_def.position.Set(stabberx + p, whyco + rodWidth + (frontHeight-lag)/2);
//		backrectangle_def.linearVelocity.Set(5,0); //this is just for testing purposes #testing
		newFixture.shape = &backrectangle_shape;
		newFixture.restitution = 1;    //#testing for this to work we have to give the restitution to frontl also
		newFixture.density = density;
		pumpingRod = (*m_world).CreateBody(&backrectangle_def);
		(*pumpingRod).CreateFixture(&newFixture);
		newFixture.restitution = 0;     //#testing
		backrectangle_def.linearVelocity.Set(0,0);
		newFixture.density = 0;
		//Prismatic joint between pumpingRod an frontb
		psjointtest.bodyA = frontb;
		psjointtest.bodyB = pumpingRod;
		psjointtest.localAxisA = b2Vec2(1,0);
		psjointtest.localAnchorB = b2Vec2(0,0);
		psjointtest.localAnchorA = b2Vec2(0,frontHeight/2 + rodWidth/2);
		(*m_world).CreateJoint(&psjointtest);
	}
	
	
	//Spring Action on pumping rod
	
/*	{	
		b2Vec2 co_middlepoint;
		float x_middlepoint = bigrectangle_body->GetPosition().x + length_bigrectangle + frontWidth;
		float y_middlepoint = pumpingRod->GetPosition().y;
		co_middlepoint.Set(x_middlepoint,y_middlepoint);

		b2DistanceJointDef jointDef1;
		//float x_distance_joint = length_bigrectangle + bigrectangle_body->GetPosition().x + frontWidth;
		float x_distance_joint = frontr->GetPosition().x;
		float y_distance_joint = pumpingRod->GetPosition().y;
		b2Vec2 co_distance_joint;
		co_distance_joint.Set(x_distance_joint,y_distance_joint);
		jointDef1.Initialize(pumpingRod, frontr, co_middlepoint,co_distance_joint);
		//jointDef1.Initialize(pumpingRod, frontr, pumpingRod->GetPosition(),co_distance_joint);
		jointDef1.frequencyHz = 0.1f;
		jointDef1.dampingRatio = 0.0f;
		jointDef1.collideConnected = true;
		(*m_world).CreateJoint(&jointDef1);


		b2DistanceJointDef jointDef2;
		float x_distance_joint2 = frontl->GetPosition().x;
		float y_distance_joint2 = pumpingRod->GetPosition().y;
		b2Vec2 co_distance_joint2;
		co_distance_joint2.Set(x_distance_joint2,y_distance_joint2);
		jointDef2.Initialize(pumpingRod, frontl,co_middlepoint,co_distance_joint2);
		//jointDef2.Initialize(pumpingRod, frontl,pumpingRod->GetPosition(),co_distance_joint2);
		jointDef2.frequencyHz = 0.1f;
		jointDef2.dampingRatio = 0.0f;
		jointDef2.collideConnected = true;
		(*m_world).CreateJoint(&jointDef2);
	}*/


	//Creation of stabber

	{
		backrectangle_shape.SetAsBox(stabbingRodLength/2, rodWidth/2);
		backrectangle_def.position.Set(exco - stabbingRodLength/2 + rodWidth, y_bigrectangle - breadth_bigrectangle - stabbingWidth);
		firstStab = (*m_world).CreateBody(&backrectangle_def);
		newFixture.filter.groupIndex = -2;
		newFixture.shape = &backrectangle_shape;
		(*firstStab).CreateFixture(&newFixture);
	//This is for second stab

		backrectangle_shape.SetAsBox(stabbingRodLength/2, rodWidth/2);
		backrectangle_def.position.Set(exco - stabbingRodLength/2 + rodWidth, y_bigrectangle - breadth_bigrectangle - stabbingWidth - intraStabberWidth); //stabbingWidth is distance between bottom of big rectangle and first stab
	//	backrectangle_def.type = b2_staticBody; intraStabberWidth = 1.5*stabbingWidth;
		secondStab = (*m_world).CreateBody(&backrectangle_def);
		newFixture.filter.groupIndex = -2;
		newFixture.shape = &backrectangle_shape;
		(*secondStab).CreateFixture(&newFixture);

	//Now, the problem is to make them fixed to frontr
	
		big_back_def1.Initialize(firstStab, frontl, b2Vec2(exco + rodWidth/2, y_bigrectangle - breadth_bigrectangle - stabbingWidth));
		(*m_world).CreateJoint(&big_back_def1);

	//Now, fixing the second stab
		big_back_def1.Initialize(secondStab, frontl, b2Vec2(exco + rodWidth/2, y_bigrectangle - breadth_bigrectangle - stabbingWidth - intraStabberWidth));
		(*m_world).CreateJoint(&big_back_def1);
	}
	/*
_______________________________________________________________________________________________________this is bottom of big rectangle
                                                                                      |
 __________________________________________________________________                   |  this distance is stabberWidth
|__________________________________________________________________|---------------------------------
                                                                                      |
 __________________________________________________________________                   |  this is intraStabberWidth
|__________________________________________________________________|----------------------------------

......................................................................................................................................

                       | |
__________             | |
| stabber |____________| |
|         |_pusher_____| |this is pumping rod
|_________|            | |
                       | |

..................................................................................................................................
 
 */

	//stabber part
	
	{
		backrectangle_shape.SetAsBox(stabberLength/2, (intraStabberWidth - rodWidth)/2);
		backrectangle_def.position.Set(stabberx/* - rodWidth/2 - stabbingRodLength/2 + rodWidth*/, stabbery);
		newFixture.density = density;
		stabber = (*m_world).CreateBody(&backrectangle_def);
	
		newFixture.shape = &backrectangle_shape;
		newFixture.restitution = 1;  //this had the default of 0
		(*stabber).CreateFixture(&newFixture);
		newFixture.density = 0;
		newFixture.restitution = 0;
		//This is construction of prismatic joint
		b2PrismaticJointDef psjoint;
		psjoint.bodyA = firstStab;
		psjoint.bodyB = stabber;
		psjoint.localAxisA = b2Vec2(1,0);
		psjoint.localAnchorB = b2Vec2(0,0);
		psjoint.localAnchorA = b2Vec2(0,- intraStabberWidth/2);
		(*m_world).CreateJoint(&psjoint);
	}



	//Now, a rod between stabber and the pumping rod
	{
		float pushLength = (*pumpingRod).GetWorldCenter().x - (*stabber).GetWorldCenter().x; //By this, pusher has both its end points at centers of pumpingRod and stabber. 
		backrectangle_shape.SetAsBox(pushLength/2, rodWidth/2);
		//backrectangle_def.position.Set(exco + frontWidth + rodWidth/2 - pushLength/2, whyco + rodWidth + frontHeight/2);//this is original line
		backrectangle_def.position.Set(((*pumpingRod).GetWorldCenter().x + (*stabber).GetWorldCenter().x)/2, whyco + rodWidth + frontHeight/2);
		b2Body* pusher = (*m_world).CreateBody(&backrectangle_def);
		b2FixtureDef z;
		z.density = density;
		z.filter.groupIndex = -2;
		z.filter.categoryBits = 0x0002;
		z.filter.maskBits = 0x0004;
		z.shape = &backrectangle_shape;
//		newFixture.filter.groupIndex = -2;
//		newFixture.shape = &backrectangle_shape;
//		newFixture.density = 1;
		(*pusher).CreateFixture(&z);
	

		weld.Initialize(pusher, pumpingRod, b2Vec2(exco + frontWidth, whyco + rodWidth + frontHeight/2));
		(*m_world).CreateJoint(&weld);
		weld.Initialize(pusher, stabber, (*stabber).GetWorldCenter());
		(*m_world).CreateJoint(&weld);
		weld.Initialize(pusher, stabber, b2Vec2(exco - stabberLength/2, y_bigrectangle - breadth_bigrectangle - stabbingWidth - rodWidth/2 - (intraStabberWidth - rodWidth)/2));
		(*m_world).CreateJoint(&weld);
	}






/*//Let's start second level in front part
	|  2fw/             |
	|________   ________|
	|________   ________|
	|	  |		  |		|
	|	  |		  |		|   this height is secondHeight
	|	  |		  |		|
	|____ |_______| ____|
	|____  _______  ____|
	|
	|	
*/


	//sfirst,ssecond
	/*{
		
		backrectangle_shape.SetAsBox(sLength/2, rodWidth/2);
		backrectangle_def.position.Set(exco + sLength/2, whyco + rodWidth + frontHeight + rodWidth + secondHeight + rodWidth/2);
		b2Body* sfirst = (*m_world).CreateBody(&backrectangle_def);
		//cout << newFixture.filter.groupIndex << endl;
		newFixture.density = 0;
		newFixture.filter.groupIndex = 0;
		newFixture.shape = &backrectangle_shape;
		(*sfirst).CreateFixture(&newFixture);
		//So, there comes an end to the creation of sfirst

		//Now, the creation of ssecond
		backrectangle_def.position.Set(exco + 2*frontWidth - sLength/2, whyco + rodWidth + frontHeight + rodWidth + secondHeight + rodWidth/2);
		b2Body* ssecond = (*m_world).CreateBody(&backrectangle_def);
		newFixture.shape = &backrectangle_shape;
		(*ssecond).CreateFixture(&newFixture);
		//Ok, ssecond is also done. Now, joining them to frontl and frontr is remaining
		//creation of joint between sfirst and frontl
		orJointDef.Initialize(frontl, sfirst, b2Vec2(exco + rodWidth/2, whyco + rodWidth + frontHeight + rodWidth + secondHeight + rodWidth/2));
		(*m_world).CreateJoint(&orJointDef);
		//Now, the creation of joint between ssecond and frontr
		orJointDef.Initialize(frontr, ssecond, b2Vec2(exco + frontWidth*2 - rodWidth/2, whyco + rodWidth + frontHeight + rodWidth + secondHeight + rodWidth/2));
		(*m_world).CreateJoint(&orJointDef);
	}*/
		
	//pulley creation
	{
		backrectangle_shape.SetAsBox((cl+dl)/2-0.5, secondHeight/2);
		backrectangle_def.position.Set(frontWidth*(5.0/6) + exco , whyco + rodWidth + frontHeight + rodWidth + secondHeight/2);
		pulley = (*m_world).CreateBody(&backrectangle_def);
		newFixture.shape = &backrectangle_shape;
		newFixture.density = density;
		(*pulley).CreateFixture(&newFixture);
		newFixture.density = 0;		

		psjointtest.bodyB = pulley;
		psjointtest.localAnchorA = b2Vec2((*pulley).GetWorldCenter().x - (*frontb).GetWorldCenter().x, (*pulley).GetWorldCenter().y - (*frontb).GetWorldCenter().y);
		(*m_world).CreateJoint(&psjointtest);
	}





	//Now, the holder for the stabber



	b2CircleShape c;


	float holderHeight = (intraStabberWidth - rodWidth)*1.5; //this is same as width of stabber
	float error = holderHeight/6;//lowest circle
	float smallRadius = 0.2;//used near lowest circle
	{
		
		backrectangle_shape.SetAsBox(rodWidth, holderHeight/2);
		backrectangle_def.position.Set(exco - stabberLength/2 - stabberLag, y_bigrectangle - breadth_bigrectangle - stabbingWidth - rodWidth/2 - (intraStabberWidth-rodWidth)/2  - holderHeight/2);
		holder = (*m_world).CreateBody(&backrectangle_def);
		newFixture.filter.groupIndex = -2;
		newFixture.shape = &backrectangle_shape;
		(*holder).CreateFixture(&newFixture);
		newFixture.filter.groupIndex = 0;
		//Now, fixing that holder to the stabber
		orJointDef.Initialize(holder, stabber, (*stabber).GetWorldCenter());
		(*m_world).CreateJoint(&orJointDef);
	}


	//Now creating the lowest circle on holder
 	{
		
		c.m_p.Set(0,0);
		c.m_radius = smallRadius;
		backrectangle_def.position.Set((*holder).GetWorldCenter().x, (*holder).GetWorldCenter().y - holderHeight/2 + error);
		lcircle = (*m_world).CreateBody(&backrectangle_def);
		newFixture.shape = &c;
		newFixture.density = density;
		(*lcircle).CreateFixture(&newFixture);
		newFixture.density = 0;
		//fixing the lcircle to holder
		orJointDef.Initialize(holder, lcircle, (*lcircle).GetWorldCenter());
		(*m_world).CreateJoint(&orJointDef);
	}





	//lrod
	float ycoofpoint = y_bigrectangle - breadth_bigrectangle - stabbingWidth - intraStabberWidth - rodWidth/2; //this is y coordinate of second lowest point initially
	float pointHeight = ycoofpoint - (*lcircle).GetWorldCenter().y; //this is height of second point wrt lcircle
	//cout << pointHeight << endl;
	float theta = 30;
	float xcom = (*lcircle).GetWorldCenter().x + (pointHeight/tan(theta*pi/180))/2; //this is x coordinate of com of rod before rotating
	float ycom = (*lcircle).GetWorldCenter().y + pointHeight/2;      //this is y coordinate of com of rod before rotating
	float lcom = pointHeight/sin(theta*pi/180);
	float smallWidth = rodWidth/3;


	{
		backrectangle_def.position.Set(xcom,ycom);
		backrectangle_shape.SetAsBox(lcom/2, smallWidth/2);
		backrectangle_def.angle = theta*pi/180;
		lrod = (*m_world).CreateBody(&backrectangle_def);
		newFixture.filter.groupIndex = -2;
		newFixture.density = density;
		newFixture.shape = &backrectangle_shape;
		(*lrod).CreateFixture(&newFixture);
		newFixture.filter.groupIndex = 0;
		newFixture.density = 0;
	}



	//Now, the small circle

	{
		c.m_p.Set(0,0);
		backrectangle_def.position.Set((*lcircle).GetWorldCenter().x + pointHeight/tan(theta*pi/180), (*lcircle).GetWorldCenter().y + pointHeight);
		lnextcircle = (*m_world).CreateBody(&backrectangle_def);
		newFixture.shape = &c;
		newFixture.filter.groupIndex = -2;
		newFixture.density = density;
		(*lnextcircle).CreateFixture(&newFixture);
		newFixture.filter.groupIndex = 0;
		newFixture.density = 0;
		
		//Now, the revolute joint between lnextcircle and lrod
		orJointDef.Initialize(lnextcircle, lrod, (*lnextcircle).GetWorldCenter());
		(*m_world).CreateJoint(&orJointDef);
		
		//Now, revolute joint between lcircle and lrod

		orJointDef.Initialize(lcircle, lrod, (*lcircle).GetWorldCenter());
		(*m_world).CreateJoint(&orJointDef);
	}






	//NOw, the case of two higher points. Let highest one be called hcircle and other one by hnextcircle

	float alpha = 90 - theta;
	float extenstion = 0.9;

	
	{
		ycom = (*pulley).GetWorldCenter().y + extenstion;
		xcom = (*lnextcircle).GetWorldCenter().x - (ycom - (*lnextcircle).GetWorldCenter().y)/tan(alpha*pi/180);
		//Now, let's create a new circle
		backrectangle_def.position.Set(xcom, ycom);
		hcircle = (*m_world).CreateBody(&backrectangle_def);
		newFixture.filter.groupIndex = -2;
		newFixture.density = density;
		(*hcircle).CreateFixture(&newFixture);
		newFixture.filter.groupIndex = 0;
		newFixture.density = 0;
	}







	//Yes, hcircle is done, now, we have to connect a rod between hcircle and lnextcircle


	{
		xcom = ((*lnextcircle).GetWorldCenter().x + xcom)/2;
		ycom = ((*lnextcircle).GetWorldCenter().y + ycom)/2; //these two are the center of rod without rotating
		backrectangle_def.position.Set(xcom, ycom);
		backrectangle_shape.SetAsBox(smallWidth/2, ((*hcircle).GetWorldCenter().y - ((*lnextcircle).GetWorldCenter().y)/sin(alpha*pi/180))/2);
		float rightrod_angle = (90-alpha)*pi/180;
		backrectangle_def.angle = rightrod_angle;
		rightrod = (*m_world).CreateBody(&backrectangle_def);
		backrectangle_def.angle = 0;
		newFixture.shape = &backrectangle_shape;
		newFixture.density = density;
		newFixture.filter.groupIndex = -2;
		(*rightrod).CreateFixture(&newFixture);
		newFixture.density = 0;
		newFixture.filter.groupIndex = 0;
		
		orJointDef.Initialize(rightrod, hcircle, (*hcircle).GetWorldCenter());
		(*m_world).CreateJoint(&orJointDef);
		//Now, we have to connect lnextcircle and rightrod
		orJointDef.Initialize(rightrod, lnextcircle, (*lnextcircle).GetWorldCenter());
		(*m_world).CreateJoint(&orJointDef);
	}
		


	

	//Now, we have to connect pulley to rightrod at the y height of center of pulley
	{
		xcom = (*lnextcircle).GetWorldCenter().x - ((*pulley).GetWorldCenter().y - (*lnextcircle).GetWorldCenter().y)/tan(alpha*pi/180);
		backrectangle_shape.SetAsBox(((*pulley).GetWorldCenter().x - xcom)/2, smallWidth/2);
		backrectangle_def.position.Set((xcom + (*pulley).GetWorldCenter().x)/2, (*pulley).GetWorldCenter().y);
		b2FixtureDef ul_fixturedef;
		ul_fixturedef.density = density;
		ul_fixturedef.filter.groupIndex = -2;
		ul_fixturedef.filter.categoryBits = 0x0002;
		ul_fixturedef.filter.maskBits = 0x0004;
		connector= (*m_world).CreateBody(&backrectangle_def);
		ul_fixturedef.shape = &backrectangle_shape;
		(*connector).CreateFixture(&ul_fixturedef);
		newFixture.density = 0;
		newFixture.filter.groupIndex = 0;
		//successfully, the connector rod is done. Now, we have to weld it to rightrod and pulley
		weld.Initialize(connector, pulley, (*pulley).GetWorldCenter());
		(*m_world).CreateJoint(&weld);

		orJointDef.Initialize(connector, rightrod, b2Vec2(xcom, (*pulley).GetWorldCenter().y));
		(*m_world).CreateJoint(&orJointDef);
	}


	//creating a third level containing gas particles
	b2Body* enclosure;
	{
		float thirdHeight = 2*secondHeight;	
		backrectangle_shape.SetAsBox(frontWidth, rodWidth/2);
		backrectangle_def.position.Set(exco + frontWidth, whyco + rodWidth + frontHeight + rodWidth + secondHeight + rodWidth + thirdHeight + rodWidth/2);
		newFixture.density = density;
		enclosure = (*m_world).CreateBody(&backrectangle_def);
		(*enclosure).CreateFixture(&newFixture);

		//Now, welding it on left side
		weld.Initialize(enclosure, frontl, b2Vec2((*frontl).GetWorldCenter().x, (*enclosure).GetWorldCenter().y));
		(*m_world).CreateJoint(&weld);
		//Now, welding on right side
		weld.Initialize(enclosure, frontr, b2Vec2((*frontr).GetWorldCenter().x, (*enclosure).GetWorldCenter().y));
		(*m_world).CreateJoint(&weld);
	}


    

	

	//stabber_initialjoint rod :The rod between stabber and holyrod
	/*! \section Stabber_Intialjoint rod
	* \brief This rod joins the stabber and the holy rod 
	*/
	{
		b2Vec2 stabber_centre;//!< \b Type : \b b2Vec2 . It stores x and y co-ordinates of stab centre
		stabber_centre = stabber->GetPosition();
		b2Vec2 holyrod_end ;//!< \b Type : \b b2Vec2 . It stores x and y co-ordinates fo holy rod's end
		holyrod_end.Set(holyrodBody->GetPosition().x + wheelRadius/4,holyrodBody->GetPosition().y + wheelRadius/4);

		float length_stabber_initialjoint = sqrt(pow(stabber_centre.x - holyrod_end.x,2)+pow(stabber_centre.y - holyrod_end.y,2)); //!< \b Type : \b float.  It is the length of rod joining the stabber and the holy rod .
		float angle_stabber_initialjoint = atan(stabber_centre.y - holyrod_end.y/stabber_centre.x - holyrod_end.x)*pi/180;//!< \b Type : \b float. It has the angle of the rod joining the stabber and the holy rod w.r.t vertical.
		b2PolygonShape stabber_initialjoint_shape;//!< \b Type : \b b2PolygonShape. It helps us in creating Polygon shapes (convex)
		stabber_initialjoint_shape.SetAsBox(length_stabber_initialjoint/2,rodWidth/2);
		b2BodyDef stabber_initialjoint_def;//!< \b Type : \b b2BodyDef .It is a definition for a body which is used to create a body object i.e. determining the position,dynamics
		stabber_initialjoint_def.position.Set((stabber_centre.x + holyrod_end.x)/2,(stabber_centre.y + holyrod_end.y)/2);
		stabber_initialjoint_def.angle = - angle_stabber_initialjoint - pi/30;
		stabber_initialjoint_def.type = b2_dynamicBody;
		b2Body* stabber_initialjoint_body = (*m_world).CreateBody(&stabber_initialjoint_def);
		b2FixtureDef stabber_initialjoint_fixture;//!< \b Type : \b b2BodyFixtureDef .It acts like a cover for a body and gives bodies properties like shape, density, friction
		stabber_initialjoint_fixture.density = 1.0;
		stabber_initialjoint_fixture.filter.groupIndex = -2;
		stabber_initialjoint_fixture.shape = &stabber_initialjoint_shape;
		(*stabber_initialjoint_body).CreateFixture(&stabber_initialjoint_fixture);
		big_back_def1.Initialize(stabber,stabber_initialjoint_body,stabber->GetPosition()); 
		(*m_world).CreateJoint(&big_back_def1);
		big_back_def1.Initialize(holyrodBody,stabber_initialjoint_body,holyrod_end);
		(*m_world).CreateJoint(&big_back_def1);
	}




	// holyrod2 : The smaller horizontal rod connected to the end of holyrod 
	/*! \section The Holy Rod2
	* \brief The smaller horizontal rod connected to the end of holyrod
	*/

	b2Body* holyrodBody2;
	float var1 = 3.0;
	float x_holyrod2 = firstWheelCenterx+gap+wheelRadius/2+wheelRadius+var1;

	{
		
		otrodShape.SetAsBox(wheelRadius+var1,rodWidth/2);
		otrodBodyDef.position.Set(x_holyrod2,groundHeight+wheelRadius*(1+(1.0/2)));
		otrodBodyDef.angle = 0;
		holyrodBody2 = (*m_world).CreateBody(&otrodBodyDef);
		otrodFixtureDef.shape = &otrodShape;
		otrodFixtureDef.filter.groupIndex = -2;
		(*holyrodBody2).CreateFixture(&otrodFixtureDef);
		b2Vec2 point_holy2;//!< \b Type : \b b2Vec2 . It stores x and y co-ordinates of holyrod end
		point_holy2.Set(firstWheelCenterx+gap+wheelRadius/2,groundHeight+wheelRadius*(1+(1.0/2)));
		big_back_def1.Initialize(holyrodBody,holyrodBody2,point_holy2);
		(*m_world).CreateJoint(&big_back_def1);
	}




	//holyrod3 : The rod which is connected to center of big rectangle and is connected to the smaller horizontal rod
	/*! \section The Holy Rod3
	* \brief It is the rod with some angle with horizontal to holyrod2
	*/
	float length_holyrod3 = 5;
	b2Body* holyrodBody3;
	{
		float angle = -pi/3;
		float x_holyrod3 = x_holyrod2 + wheelRadius+var1+ length_holyrod3*sin(angle);
		otrodShape.SetAsBox(rodWidth/2,length_holyrod3);
		otrodBodyDef.position.Set(x_holyrod3,groundHeight+wheelRadius*(1+(1.0/2))+length_holyrod3*cos(-angle));
		otrodBodyDef.angle = -angle; //angle is clockwise by default
		holyrodBody3 = (*m_world).CreateBody(&otrodBodyDef);
	//	(*holyrodBody).SetTransform(b2Vec2(firstWheelCenterx + gap + wheelRadius/4, groundHeight + wheelRadius*(1+0.25)), -pi/4);
		otrodFixtureDef.shape = &otrodShape;
		otrodFixtureDef.filter.groupIndex = -2;
		(*holyrodBody3).CreateFixture(&otrodFixtureDef);
		orJointDef.Initialize(holyrodBody3,bigrectangle_body,(*holyrodBody3).GetWorldCenter());
		(*m_world).CreateJoint(&orJointDef);

		b2Vec2 point_holy3;//!< \b Type : \b b2Vec2 . It stores x and y co-ordinates of holyrod2's end
		point_holy3.Set(firstWheelCenterx+gap+wheelRadius/2+wheelRadius+var1+wheelRadius+var1,groundHeight+wheelRadius*(1+(1.0/2)));
		big_back_def1.Initialize(holyrodBody2,holyrodBody3,point_holy3);
		(*m_world).CreateJoint(&big_back_def1);
		b2Vec2 point_holy4;//!< \b Type : \b b2Vec2 . It stores x and y co-ordinates of top end of holyrod4's end
		point_holy4.Set(x_holyrod3 + length_holyrod3*sin(angle),groundHeight+wheelRadius*(1+(1.0/2))+length_holyrod3*2*cos(-angle));
		//big_back_def1.Initialize(bigrectangle_body,holyrodBody3,point_holy4);
		//(*m_world).CreateJoint(&big_back_def1);
	}




	//The rod which connects the holyrod3 and the right rod
	/*! \section The Last Rod
	* \brief It connects the right rod and the holyrod3
	*/
	{
		float length_LastRod_1 = 5.5; //! \b Type : \b float .It has the length of the Last Rod
		b2PolygonShape LastRod_1_shape;//!< \b Type : \b b2PolygonShape. It helps us in creating Polygon shapes (convex)
		LastRod_1_shape.SetAsBox(length_LastRod_1,smallWidth/2);
		b2BodyDef LastRod_1_def;//!< \b Type : \b b2BodyDef .It is a definition for a body which is used to create a body object i.e. determining the position,dynamics
		LastRod_1_def.type = b2_dynamicBody;
		LastRod_1_def.position.Set(connector->GetPosition().x - ((*pulley).GetWorldCenter().x -xcom)/2 - length_LastRod_1,connector->GetPosition().y);
		//LastRod_1_def.shape = &LastRod_1_shape;
		b2Body* LastRod_1_body = (*m_world).CreateBody(&LastRod_1_def);//!< Body of Last Rod
		b2FixtureDef LastRod_1_fixture; //!< \b Type : \b b2BodyFixtureDef .It acts like a cover for a body and gives bodies properties like shape, density, friction
		LastRod_1_fixture.shape = &LastRod_1_shape;
		LastRod_1_fixture.density = density;
		LastRod_1_fixture.filter.groupIndex = -2;
		(*LastRod_1_body).CreateFixture(&LastRod_1_fixture);

		b2Vec2 point_hrb3_lr_1;//!< \b Type : \b b2Vec2 . It stores x and y co-ordinates of Last Rod 's end
		point_hrb3_lr_1.Set(LastRod_1_body->GetPosition().x - length_LastRod_1,LastRod_1_body->GetPosition().y);
		orJointDef.Initialize(holyrodBody3,LastRod_1_body,point_hrb3_lr_1);
		(*m_world).CreateJoint(&orJointDef);

		
		point_hrb3_lr_1.Set(connector->GetPosition().x - ((*pulley).GetWorldCenter().x -xcom)/2,connector->GetPosition().y);
		orJointDef.Initialize(rightrod,LastRod_1_body,point_hrb3_lr_1);
		(*m_world).CreateJoint(&orJointDef);
	}
	{
		float parRadius = smallRadius/5; 
		int parPerLine = (2*frontWidth - 2*rodWidth)/(2*parRadius);
		float layerWidth = 0.5; 
		int noLayers = 6;
		b2CircleShape parShape; 
		parShape.m_p.Set(0,0); 
		parShape.m_radius = parRadius;
		b2Body* particles[parPerLine][noLayers];
		b2FixtureDef parFixture; 
		parFixture.restitution = 1;
		//parFixture.filter.groupIndex = -2;
		parFixture.density = 80; 
		parFixture.shape = &parShape;
		parFixture.filter.maskBits = 0x0004;
		parFixture.filter.categoryBits = 0x0002;
		b2BodyDef parBodyDef; 
		parBodyDef.type = b2_dynamicBody; 
		parBodyDef.bullet = true; 
		int velocityLimit = 10; 
		for(int j = 0; j < noLayers; j++){
		 for(int i = 0; i < parPerLine; i++){
		  parBodyDef.position.Set(exco + rodWidth - parRadius + (i+1)*2*parRadius, (*enclosure).GetWorldCenter().y - rodWidth/2 - parRadius - layerWidth*j); 
		  parBodyDef.linearVelocity.Set(rand()%velocityLimit, -rand()%velocityLimit); 
		  particles[i][j] = (*m_world).CreateBody(&parBodyDef); 
		  (*particles[i][j]).SetGravityScale(0.1); 
		  (*particles[i][j]).CreateFixture(&parFixture); 
			} 
		}
	}
	
	(*a).pumpingRod = pumpingRod;
	(*a).frontl = frontl;
/*	std::cout << exco << std::endl;
  (*a).excoa = exco;
	std::cout << (*a).excoa << std::endl;
	std::cout << whyco << std::endl;
  (*a).whycoa = whyco;
	std::cout << (*a).whycoa << std::endl;
	std::cout << ml << std::endl;
  (*a).mla = ml;
	std::cout << dl << std::endl;
  (*a).dla = dl;
	std::cout << cl << std::endl;
  (*a).cla = cl;
	std::cout << rodWidth << std::endl;
  (*a).rodWidtha = rodWidth;
	std::cout << frontHeight << std::endl;
  (*a).frontHeighta = frontHeight;
	std::cout << secondHeight << std::endl;
	(*a).secondHeighta = secondHeight;*/
	m_world->b2World::SetContactListener(a);


	

///////////////////////end of monday code .........................................................................
 	}
 	 sim_t *sim = new sim_t("Dominos", dominos_t::create);
 }
