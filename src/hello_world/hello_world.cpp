#include <iostream>
#include "btBulletDynamicsCommon.h"

std::ostream &operator <<(std::ostream &, btVector3);

btTransform getTransform(btCollisionObject *);

btDynamicsWorld *createDynamicsWorld();
void destroyDynamicsWorld(btDynamicsWorld *);

btRigidBody *createRigidbody(btCollisionShape *shape, btScalar mass, btTransform transform);
void destroyRigidbody(btRigidBody *rigidbody);


int main() {
	btDynamicsWorld *dynamicsWorld = createDynamicsWorld();

	btRigidBody *ground = NULL;
	{
		btScalar mass(0);
		btTransform transform(btQuaternion::getIdentity(), btVector3(0, -56, 0));
		ground = createRigidbody(new btBoxShape(btVector3(50, 50, 50)), mass, transform);
	}
	dynamicsWorld->addRigidBody(ground);

	btRigidBody *ball = NULL;
	{
		btScalar mass(1);
		btTransform transform(btQuaternion::getIdentity(), btVector3(2, 10, 0));
		ball = createRigidbody(new btSphereShape(btScalar(1)), mass, transform);
	}
	dynamicsWorld->addRigidBody(ball);

	for (int i = 0; i < 150; ++i) {
		btScalar timeStep(1.0f / 60);
		int maxSubSteps = 10;
		dynamicsWorld->stepSimulation(timeStep, maxSubSteps);

		btCollisionObjectArray &collisionObjects = dynamicsWorld->getCollisionObjectArray();
		for (int j = 0; j < dynamicsWorld->getNumCollisionObjects(); ++j) {
			btTransform transform = getTransform(collisionObjects[j]);
			std::cout << "Object " << j << ": "<< transform.getOrigin() << "\n";
		}
	}
	destroyDynamicsWorld(dynamicsWorld);
	return 0;
}


std::ostream &operator <<(std::ostream &stream, btVector3 vec) {
	stream << "(" << vec.getX() << ", " << vec.getY() << ", " << vec.getZ() << ")";
	return stream;
}

btTransform getTransform(btCollisionObject *object) {
	btRigidBody *rigidbody = btRigidBody::upcast(object);
	btTransform transform;
	if (rigidbody && rigidbody->getMotionState()) {
		rigidbody->getMotionState()->getWorldTransform(transform);
	} else {
		transform = object->getWorldTransform();
	}
	return transform;
}

btDynamicsWorld *createDynamicsWorld() {
	btCollisionConfiguration *collisionConfig = new btDefaultCollisionConfiguration();
	btDispatcher *dispatcher = new btCollisionDispatcher(collisionConfig);
	btBroadphaseInterface *broadphase = new btDbvtBroadphase();
	btConstraintSolver *constraintSolver = new btSequentialImpulseConstraintSolver();
	btDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(
		dispatcher, broadphase, constraintSolver, collisionConfig
	);
	dynamicsWorld->setGravity(btVector3(0, -9.8f, 0));
	return dynamicsWorld;
}

void destroyDynamicsWorld(btDynamicsWorld *world) {
	btCollisionObjectArray &collisionObjects = world->getCollisionObjectArray();
	for (int i = 0; i < world->getNumCollisionObjects(); ++i) {
		btCollisionObject* obj = collisionObjects[i];
		world->removeCollisionObject(obj);
		destroyRigidbody(btRigidBody::upcast(obj));
	}
	btCollisionDispatcher *dispatcher = dynamic_cast<btCollisionDispatcher *>(world->getDispatcher());
	btCollisionConfiguration *collisionConfig = dispatcher->getCollisionConfiguration();
	btBroadphaseInterface *broadphase = world->getBroadphase();
	btConstraintSolver *constraintSolver = world->getConstraintSolver();
	// delete in the reverse order of creation
	delete world;
	delete constraintSolver;
	delete broadphase;
	delete dispatcher;
	delete collisionConfig;
}

btRigidBody *createRigidbody(btCollisionShape *shape, btScalar mass, btTransform transform) {
	btVector3 inertia(0, 0, 0);
	bool isDynamic = mass != 0.0f;
	if (isDynamic) {
		shape->calculateLocalInertia(mass, inertia);
	}
	btMotionState *motionState = new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo info(mass, motionState, shape, inertia);
	return new btRigidBody(info);
}

void destroyRigidbody(btRigidBody *rigidbody) {
	btMotionState *motionState = rigidbody->getMotionState();
	if (motionState) {
		delete motionState;
	}
	delete rigidbody->getCollisionShape();
	delete rigidbody;
}