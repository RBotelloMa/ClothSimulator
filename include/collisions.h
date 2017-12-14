#pragma once

#include "auxTypes.h"
#include "vertex.h"
#include "matriz.h"
#include "spaceHash.h"
#include "world.h"
#include "numericCalculations.h"


namespace mySim {
	class CollisionEngine {
	public:

		CollisionEngine(SpaceHash * hash, float tolAngle, float tolSide, float tolMove, float collisionStifness) {
			hash_ = hash;
			tolAngle_ = tolAngle;
			tolSide_ = tolSide;
			tolMove_ = tolMove;
			collisionStifness_ = collisionStifness;
		}
		virtual ~CollisionEngine();

		CollisionType CheckColision(CollisionVT * vertexArray);
		CollisionType CheckColision2(CollisionVT * vertexArray, float deltaT);
		CollisionType CheckColision3(CollisionVT * vertexArray);
		
		void updateHashVertices(MallaTriangular * malla , float time);
		void updateHashTriangulos(MallaTriangular * malla, float time);
		void updateHash(World * world);
		
		void generateConstraints(World * world , float deltaT);
		void selfCollsionStep(World * world, float deltaT);


		void addCollisionConstraint(MyConstraint * constraint) { collisionConstraints_.push_back(constraint); }
		void clearCollsionConstraints();
		
		std::vector<MyConstraint *> & refCollisionConstraints() { return collisionConstraints_; };

		void applyCollisionConstraints(unsigned int solverIterations) {
			for (auto it = collisionConstraints_.begin(); it != collisionConstraints_.end(); it++) {
				(*it)->applyConstraintFull(solverIterations);
			}
		}

	private:
		SpaceHash * hash_;
		float tolAngle_; 
		float tolSide_; 
		float tolMove_;
		float collisionStifness_;

		//to change this
		std::vector<MyConstraint *> collisionConstraints_;

	};

}