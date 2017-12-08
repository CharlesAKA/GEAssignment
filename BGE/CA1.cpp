#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
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
#include "CA1.h"

using namespace BGE;

CA1::CA1(void)
{
}

CA1::~CA1(void)
{
}

shared_ptr<PhysicsController> cyl;
std::shared_ptr<GameComponent> station;
float x = 5.0f;
float z = 0.0f;
float MaxMotor = 30.0f;
float MaxMotor2 = 5.0f;
bool enableMotor = true;
float LowLimit = -1.5f;
float UpLimit = 1.5f;
bool CA1::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	btVector3	worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	dynamicsWorld->setGravity(btVector3(0,-20,0));

	
	//adding dog head neck and body
	shared_ptr<PhysicsController> body = physicsFactory->CreateCapsule(4, 5, glm::vec3(x, 15, z), glm::angleAxis(90.0f, glm::vec3(1, 0, 0)));

	shared_ptr<PhysicsController> head = physicsFactory->CreateCapsule(2.5f, 2.5f, glm::vec3(x, 15, z), glm::angleAxis(90.0f, glm::vec3(1, 0, 0)));
	shared_ptr<PhysicsController> neck = physicsFactory->CreateCylinder(2, 4, glm::vec3(x, 15, z), glm::angleAxis(90.0f, glm::vec3(1, 0, 0)));

	btTransform LocalA, LocalB, LocalC, LocalD; //information to map the location
	LocalA.setIdentity();
	LocalB.setIdentity();
	LocalA.setOrigin(btVector3(1, 4, 0));
	LocalB.setRotation(GLToBtQuat(glm::angleAxis(-100.0f, glm::vec3(2, 0, 0))));//set what position it lands on
	LocalB.setOrigin(btVector3(1, 4, 0));
	btFixedConstraint * fixed = new btFixedConstraint(*head->rigidBody, *neck->rigidBody, LocalA, LocalB);
	dynamicsWorld->addConstraint(fixed);
	LocalC.setIdentity();
	LocalD.setIdentity();
	LocalC.setOrigin(btVector3(1, -6, 0));
	LocalD.setRotation(GLToBtQuat(glm::angleAxis(-10.0f, glm::vec3(5, 2, 0)))); // Not sure but this needs to be a minus angle
	LocalD.setOrigin(btVector3(1, 6, 0));
	btFixedConstraint * fixed2 = new btFixedConstraint(*neck->rigidBody, *body->rigidBody, LocalC, LocalD);
	dynamicsWorld->addConstraint(fixed2);

	//front legs and shulder joints
	shared_ptr<PhysicsController> joint = physicsFactory->CreateCylinder(1, 1, glm::vec3(x, 20, z), glm::quat());
	btHingeConstraint *shoulderjoint = new btHingeConstraint(*joint->rigidBody, *body->rigidBody, btVector3(0, 1, 0), btVector3(6, 7, 0), btVector3(0, 1, 0), btVector3(7, 0, 0), false);
	shoulderjoint->enableMotor(enableMotor);
	shoulderjoint->setMaxMotorImpulse(-MaxMotor);// adding the motor API allows you to set a target rotation
	shoulderjoint->setLimit(LowLimit, UpLimit);
	dynamicsWorld->addConstraint(shoulderjoint);
	shared_ptr<PhysicsController> leg1 = physicsFactory->CreateCapsule(1.5f, 4, glm::vec3(x+5, 15, z), glm::quat());
	btHingeConstraint *frontLeg = new btHingeConstraint(*leg1->rigidBody, *joint->rigidBody, btVector3(0, 7, 0), btVector3(1, 1, 0), btVector3(0, 3, 0), btVector3(1, 0, 0), false);
	dynamicsWorld->addConstraint(frontLeg);

	shared_ptr<PhysicsController> joinLocalB = physicsFactory->CreateCylinder(1, 1, glm::vec3(x, 20, z), glm::quat());
	btHingeConstraint *shoulderjoint2 = new btHingeConstraint(*joinLocalB->rigidBody, *body->rigidBody, btVector3(0, -1, 0), btVector3(-6, 7, 0), btVector3(0, 1, 0), btVector3(7, 0, 0), false);
	shoulderjoint2->enableMotor(enableMotor);
	shoulderjoint2->setMaxMotorImpulse(-MaxMotor);// adding the motor API allows you to set a target rotation
	shoulderjoint2->setLimit(LowLimit, UpLimit);//hinge rotation limit
	dynamicsWorld->addConstraint(shoulderjoint2);
	shared_ptr<PhysicsController> leg2 = physicsFactory->CreateCapsule(1.5f, 4, glm::vec3(x+5, 15, z), glm::quat());
	btHingeConstraint *frontLeg2 = new btHingeConstraint(*leg2->rigidBody, *joinLocalB->rigidBody, btVector3(0, 7, 0), btVector3(1, -1, 0), btVector3(0, 3, 0), btVector3(1, 0, 0), false);
	dynamicsWorld->addConstraint(frontLeg2);

	//back legs and shoulder joints
	shared_ptr<PhysicsController> joinLocalC = physicsFactory->CreateCylinder(1, 1, glm::vec3(x, 20, z), glm::quat());
	btHingeConstraint *shoulderjoint3 = new btHingeConstraint(*joinLocalC->rigidBody, *body->rigidBody, btVector3(0, -1, 0), btVector3(-6, -7, 0), btVector3(0, 1, 0), btVector3(7, 0, 0), false);
	shoulderjoint3->enableMotor(enableMotor);
	shoulderjoint3->setMaxMotorImpulse(-MaxMotor);// adding the motor API allows you to set a target rotation
	shoulderjoint3->setLimit(LowLimit, UpLimit);//hinge rotation limit
	dynamicsWorld->addConstraint(shoulderjoint3);
	shared_ptr<PhysicsController> leg3 = physicsFactory->CreateCapsule(1.5f, 4, glm::vec3(x+5, 15, z), glm::quat());
	btHingeConstraint *backLeg = new btHingeConstraint(*leg3->rigidBody, *joinLocalC->rigidBody, btVector3(0, 7, 0), btVector3(1, -1, 0), btVector3(0, 3, 0), btVector3(1, 0, 0), false);
	dynamicsWorld->addConstraint(backLeg);

	shared_ptr<PhysicsController> joinLocalD = physicsFactory->CreateCylinder(1, 1, glm::vec3(x, 20, z), glm::quat());
	btHingeConstraint *shoulder4 = new btHingeConstraint(*joinLocalD->rigidBody, *body->rigidBody, btVector3(0, 1, 0), btVector3(6, -7, 0), btVector3(0, 1, 0), btVector3(7, 0, 0), false);
	shoulder4->enableMotor(enableMotor);
	shoulder4->setMaxMotorImpulse(-MaxMotor);// adding the motor API allows you to set a target rotation
	shoulder4->setLimit(LowLimit, UpLimit);//hinge rotation limit
	dynamicsWorld->addConstraint(shoulder4);
	shared_ptr<PhysicsController> leg4 = physicsFactory->CreateCapsule(1.5f, 4, glm::vec3(x+5, 15, z), glm::quat());
	btHingeConstraint *backLeg2 = new btHingeConstraint(*leg4->rigidBody, *joinLocalD->rigidBody, btVector3(0, 7, 0), btVector3(1, 1, 0), btVector3(0, 3, 0), btVector3(1, 0, 0), false);
	dynamicsWorld->addConstraint(backLeg2);

	//tail
	shared_ptr<PhysicsController> tail = physicsFactory->CreateCapsule(1, 2, glm::vec3(x+5, 8, z-5), glm::quat());
	btHingeConstraint * hinget = new btHingeConstraint(*tail->rigidBody, *body->rigidBody, btVector3(0, 3, 0), btVector3(0, -12, 0), btVector3(1, 1, 1), btVector3(0, 1, 0), false);
	dynamicsWorld->addConstraint(hinget);
	if (!Game::Initialise()) {
		return false;
	}

	camera->transform->position = glm::vec3(0, 20, 40);

	return true;
}

void BGE::CA1::Update(float timeDelta)
{

	Game::Update(timeDelta);
}

void BGE::CA1::Cleanup()
{
	Game::Cleanup();
}