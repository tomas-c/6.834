#include <bullet/btBulletDynamicsCommon.h>
#include "tiny_obj_loader.h"
#include <vector>
#include <iostream>
#include <algorithm>

class CollisionChecker {
	private:
		btCollisionObject* colObject1;
		btCollisionShape* colShape1;
		btTriangleIndexVertexArray *meshArray;
		int* indexBase;
		float* vertexBase;

		btCollisionObject* colObject2;
		btCollisionShape* colShape2;

		btDefaultCollisionConfiguration* collisionConfiguration;
		btCollisionDispatcher* dispatcher;
		btBroadphaseInterface* overlappingPairCache;
		btCollisionWorld* collisionWorld;

		struct cContactCallback : public btCollisionWorld::ContactResultCallback
		{
			bool contacting;
			cContactCallback() {
				reset();
			}
			bool isOn() {
				return contacting;
			}
			bool reset() {
				contacting = false;
			}
			virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
			{
				contacting = true;
			}
		} contactIndicator;

		void init_world(const char* mesh_file) {
			collisionConfiguration = new btDefaultCollisionConfiguration();
			dispatcher = new	btCollisionDispatcher(collisionConfiguration);
			overlappingPairCache = new btDbvtBroadphase();
			collisionWorld = new btCollisionWorld(dispatcher,overlappingPairCache,collisionConfiguration);

			{
				colObject1 = new btCollisionObject();
				// Position
				btMatrix3x3 matrix;
				matrix.setIdentity();
				colObject1->getWorldTransform().setOrigin(btVector3(0., 0., 0.));
				colObject1->getWorldTransform().setBasis(matrix);
				// Collision
				load_mesh(mesh_file);
				colShape1 = new btBvhTriangleMeshShape(meshArray, false);
				colShape1->setMargin(0.);
				colObject1->setCollisionShape(colShape1);
			}

			
			{
				colObject2 = new btCollisionObject();
				// Position
				btMatrix3x3 matrix;
				matrix.setIdentity();
				colObject2->getWorldTransform().setOrigin(btVector3(0., 0., 1.));
				colObject2->getWorldTransform().setBasis(matrix);
				// Collision
				colShape2 = new btBoxShape(btVector3(0.2, 0.2, 0.4));
				colShape2->setMargin(0.);
				colObject2->setCollisionShape(colShape2);
				//colObject2->getWorldTransform().setRotation(btQuaternion(btVector3(1.,0.,0.), M_PI/2));
			}
			
		}

		void load_mesh(const char* filename) {
			std::vector<tinyobj::shape_t> shapes;
			std::vector<tinyobj::material_t> materials;
			
			std::string err = tinyobj::LoadObj(shapes, materials, filename);
			if (!err.empty()) {
				std::cerr << err << std::endl;
				return ;
			}

			tinyobj::mesh_t &mesh = shapes[0].mesh;

			indexBase = new int[mesh.indices.size()];
			std::copy(mesh.indices.begin(), mesh.indices.end(), indexBase);

			vertexBase = new float[mesh.positions.size()];
			std::copy(mesh.positions.begin(), mesh.positions.end(), vertexBase);

			meshArray = new btTriangleIndexVertexArray(mesh.indices.size()/3, indexBase, 3*sizeof(int), mesh.positions.size(), vertexBase, 3*sizeof(float));
		}

	public:
		CollisionChecker(const std::string mesh_file) {
			init_world(mesh_file.c_str());
		}

		~CollisionChecker() {
			delete[] indexBase; delete[] vertexBase;

			delete meshArray;

			delete colObject1; delete colObject2;
			delete colShape1; delete colShape2;

			delete collisionWorld;
			delete overlappingPairCache;
			delete dispatcher;
			delete collisionConfiguration;			
		}

		void setBox(float x, float y, float z) {
			delete colShape2;
			colShape2 = new btBoxShape(btVector3(x, y, z));
			colObject2->setCollisionShape(colShape2);

		}

		void setPosition(float x, float y, float z) {
			colObject2->getWorldTransform().setOrigin(btVector3(x,y,z));
		}

		void setRotation(float w, float x, float y, float z) {
			colObject2->getWorldTransform().setRotation(btQuaternion(x,y,z,w));
		}

		bool collides() {
			contactIndicator.reset();
			collisionWorld->contactPairTest(colObject1, colObject2, contactIndicator);
			return contactIndicator.isOn();
		}
};
