#include "CollisionChecker.h"

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>
#include <bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <bullet/BulletCollision/Gimpact/btCompoundFromGimpact.h>

#include "tiny_obj_loader.h"
#include <vector>
#include <iostream>
#include <algorithm>


class CollisionChecker {
	class Object {
	public:
		btCollisionObject* colObject;
		btCollisionShape* colShape;
		btTriangleIndexVertexArray* meshArray;
		btGImpactMeshShape *trimesh;
		int* indexBase; float* vertexBase;

		Object() {
			colObject = new btCollisionObject();
			colShape = NULL;
			meshArray = NULL;
			trimesh = NULL;
			indexBase = NULL;
			vertexBase = NULL;

			// Position
			/*btMatrix3x3 matrix;
			matrix.setIdentity();
			colObject->getWorldTransform().setOrigin(btVector3(0., 0., 0.));
			colObject->getWorldTransform().setBasis(matrix);*/
		}

		~Object() {
			if(colObject != NULL) delete colObject;
			if(colShape != NULL) delete colShape;
			if(meshArray != NULL) delete meshArray;
			if(trimesh != NULL) delete trimesh;
			if(indexBase != NULL) delete indexBase;
			if(vertexBase != NULL) delete vertexBase;
		}

		void load_collision_mesh(const char* filename) {
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

			btTriangleIndexVertexArray *meshArray = new btTriangleIndexVertexArray(mesh.indices.size()/3, indexBase, 3*sizeof(int), mesh.positions.size(), vertexBase, 3*sizeof(float));
			{
				btGImpactMeshShape *trimesh = new btGImpactMeshShape(meshArray);
				trimesh->updateBound();
				//#define USE_COMPOUND
				#ifdef USE_COMPOUND
					colShape = btCreateCompoundFromGimpactShape(trimesh,1);
					delete trimesh;
					trimesh=0;
				#else
					colShape = trimesh;
				#endif
			}

			colShape->setMargin(0.);
			colObject->setCollisionShape(colShape);
		}
	};

	private:
		Object object1, object2;

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

		void init_world(const char* mesh_file1, const char* mesh_file2) {
			collisionConfiguration = new btDefaultCollisionConfiguration();
			dispatcher = new btCollisionDispatcher(collisionConfiguration);
			overlappingPairCache = new btDbvtBroadphase();
			collisionWorld = new btCollisionWorld(dispatcher,overlappingPairCache,collisionConfiguration);

			btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

			object1.load_collision_mesh(mesh_file1);
			object2.load_collision_mesh(mesh_file2);
		}

	public:
		CollisionChecker(const std::string mesh_file1, const std::string mesh_file2) {
			init_world(mesh_file1.c_str(), mesh_file2.c_str());
		}

		~CollisionChecker() {
			delete collisionWorld;
			delete overlappingPairCache;
			delete dispatcher;
			delete collisionConfiguration;			
		}

		void setPosition1(float x, float y, float z) {
			object1.colObject->getWorldTransform().setOrigin(btVector3(x,y,z));
		}

		void setRotation1(float w, float x, float y, float z) {
			object1.colObject->getWorldTransform().setRotation(btQuaternion(x,y,z,w));
		}

		void setPosition2(float x, float y, float z) {
			object2.colObject->getWorldTransform().setOrigin(btVector3(x,y,z));
		}

		void setRotation2(float w, float x, float y, float z) {
			object2.colObject->getWorldTransform().setRotation(btQuaternion(x,y,z,w));
		}

		bool collides() {
			contactIndicator.reset();
			collisionWorld->contactPairTest(object1.colObject, object2.colObject, contactIndicator);
			return contactIndicator.isOn();
		}
};
