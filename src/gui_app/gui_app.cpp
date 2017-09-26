#include <iostream>
#include "btBulletDynamicsCommon.h"
#include "OpenGLGuiHelper.h"
#include "SimpleOpenGL3App.h"
#include "b3Clock.h"

std::ostream &operator <<(std::ostream &, btVector3);

btTransform getTransform(btCollisionObject *);

btDiscreteDynamicsWorld *createDynamicsWorld();
void destroyDynamicsWorld(btDynamicsWorld *);

btRigidBody *createRigidbody(btCollisionShape *shape, btScalar mass, btTransform transform);
void destroyRigidbody(btRigidBody *rigidbody);

void initGUIHelper(GUIHelperInterface &, btDiscreteDynamicsWorld *);


int main() {
	btDiscreteDynamicsWorld *dynamicsWorld = createDynamicsWorld();
	btRigidBody *ground = NULL;
	{
		btScalar mass(0);
		btTransform transform(btQuaternion::getIdentity(), btVector3(0, -10.01, 0));
		ground = createRigidbody(new btBoxShape(btVector3(10, 10, 10)), mass, transform);
		ground->setRestitution(0.6);
	}
	dynamicsWorld->addRigidBody(ground);

	btRigidBody *ball = NULL;
	{
		btScalar mass(1);
		btTransform transform(btQuaternion::getIdentity(), btVector3(0, 5, 0));
		ball = createRigidbody(new btSphereShape(btScalar(0.5f)), mass, transform);
		ball->setRestitution(0.9);
	}
	dynamicsWorld->addRigidBody(ball);

	bool allowRetina = true;
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet GUI app", 1024, 768, allowRetina);

	bool useOpenGL2 = false;
	OpenGLGuiHelper gui(app, useOpenGL2);
	initGUIHelper(gui, dynamicsWorld);

	b3Clock clock;
	DrawGridData grid;
	grid.gridColor[0] = 0;  grid.gridColor[1] = 0;  grid.gridColor[2] = 0;
	grid.upAxis = app->getUpAxis();

	do {
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar timeStep = btScalar(clock.getTimeInSeconds());
		clock.reset();
		if (timeStep < 0.1) {
			timeStep = 0.1;
		}
		dynamicsWorld->stepSimulation(timeStep);

		gui.syncPhysicsToGraphics(dynamicsWorld);
		gui.render(dynamicsWorld);

		app->drawGrid(grid);

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	delete app;
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

btDiscreteDynamicsWorld *createDynamicsWorld() {
	btCollisionConfiguration *collisionConfig = new btDefaultCollisionConfiguration();
	btDispatcher *dispatcher = new btCollisionDispatcher(collisionConfig);
	btBroadphaseInterface *broadphase = new btDbvtBroadphase();
	btConstraintSolver *constraintSolver = new btSequentialImpulseConstraintSolver();
	btDiscreteDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(
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
	btRigidBody *rigidbody = new btRigidBody(info);
	// rigidbody->setUserIndex(-1);
	return rigidbody;
}

void destroyRigidbody(btRigidBody *rigidbody) {
	btMotionState *motionState = rigidbody->getMotionState();
	if (motionState) {
		delete motionState;
	}
	delete rigidbody->getCollisionShape();
	delete rigidbody;
}

void initGUIHelper(GUIHelperInterface &gui, btDiscreteDynamicsWorld *dynamicsWorld) {
	gui.setUpAxis(1);
	gui.createPhysicsDebugDrawer(dynamicsWorld);
	if (dynamicsWorld->getDebugDrawer()) {
		int debugMode = btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints;
		dynamicsWorld->getDebugDrawer()->setDebugMode(debugMode);
	}
	gui.autogenerateGraphicsObjects(dynamicsWorld);

	float distance = 8, pitch = 52, yaw = 35;
	float targetPos[] = {0, 0, 0};
	gui.resetCamera(distance, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
}