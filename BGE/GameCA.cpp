#include "GameCA.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "CapsuleShape.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

bool keypressed = false;

using namespace BGE;

GameCA::GameCA(void)
{
	physicsFactory = NULL;
	dynamicsWorld = NULL;
	broadphase = NULL;
	dispatcher = NULL;
	solver = NULL;
	fullscreen = false;
}

GameCA::~GameCA(void)
{
}

bool GameCA::Initialise() 
{
	riftEnabled = false;
	// Set up the collision configuration and dispatcher
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
 
    // The world.
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	broadphase = new btAxisSweep3(worldMin,worldMax);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,0,0));
	physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	physicsFactory->CreateCapsuleShape(1.0, 0.8, glm::vec3(0,10,-30), glm::quat());
	physicsFactory->CreateCapsuleRagdoll(glm::vec3(0,10,-20));

	if (!Game::Initialise()) {
		return false;
	}

	camera->GetController()->position = glm::vec3(0,10, 20);
	
	return true;
}

void BGE::GameCA::Update(float timeDelta)
{
	dynamicsWorld->stepSimulation(timeDelta,100);
	Game::Update(timeDelta);

	if(keypressed){
		//btConeTwistConstraint
		if (keyState[21])//R
		{
			physicsFactory->spine_pelvis->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));
			physicsFactory->spine_pelvis->enableAngularMotor(true, -0.5, 1400);
		}
		if (keyState[23])//T
		{
			physicsFactory->spine_pelvis->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));
			physicsFactory->spine_pelvis->enableAngularMotor(true, +0.5, 1400);
		}

		if (keyState[28])//Y
		{
			physicsFactory->left_upper_leg_left_lower_leg->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->left_upper_leg_left_lower_leg->enableAngularMotor(true, -0.5, 1400);
		}
		if (keyState[24])//U
		{
			physicsFactory->left_upper_leg_left_lower_leg->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->left_upper_leg_left_lower_leg->enableAngularMotor(true, +0.5, 1400);
		}

		if (keyState[9])//F
		{
			physicsFactory->right_upper_leg_right_lower_leg->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->right_upper_leg_right_lower_leg->enableAngularMotor(true, -0.5, 1400);
		}
		if (keyState[10])//G
		{
			physicsFactory->right_upper_leg_right_lower_leg->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->right_upper_leg_right_lower_leg->enableAngularMotor(true, +0.5, 1400);
		}

		if (keyState[11])//H
		{
			physicsFactory->left_upper_arm_left_lower_arm->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->left_upper_arm_left_lower_arm->enableAngularMotor(true, -0.5, 1400);
		}
		if (keyState[13])//J
		{
			physicsFactory->left_upper_arm_left_lower_arm->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->left_upper_arm_left_lower_arm->enableAngularMotor(true, +0.5, 1400);
		}

		if (keyState[25])//V
		{
			physicsFactory->right_upper_arm_right_lower_arm->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->right_upper_arm_right_lower_arm->enableAngularMotor(true, -0.5, 1400);
		}
		if (keyState[5])//B
		{
			physicsFactory->right_upper_arm_right_lower_arm->setLimit(btScalar(-M_PI_2), btScalar(0));
			physicsFactory->right_upper_arm_right_lower_arm->enableAngularMotor(true, +0.5, 1400);
		}
		if (keyState[SDL_SCANCODE_UP])
		{
			//
		}
		if (keyState[SDL_SCANCODE_DOWN])
		{
			physicsFactory->CreateCapsuleShape(1.0, 0.8, glm::vec3(0,10,-30), glm::quat());
		}
		if (keyState[SDL_SCANCODE_LEFT])
		{
			//physicsFactory->CreateRagdoll(glm::vec3(0,10,-50));
		}
		if (keyState[SDL_SCANCODE_RIGHT])
		{
			physicsFactory->CreateCapsuleRagdoll(glm::vec3(0,10,-20));
		}
	
}

void BGE::GameCA::Cleanup()
{
	Game::Cleanup();
}