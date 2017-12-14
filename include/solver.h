#pragma once

#include "world.h"
#include "collisions.h"

namespace mySim {
	class Solver
	{
	public:
		Solver();
		Solver(World * world);

		virtual ~Solver();

		const World * getWorld() const; //Solo lectura
		void setWorld(World * world); //Para añadir world
		World * refWorld(); //Para modificar world. Usar con precaución;
		
		void solverMainLoop(float deltaT);
		
		void applyExternalForces(float deltaT);
		void updatePreConstraints(float deltaT);
		void applyConstraints(float deltaT);
		void updateAfterConstraints(float deltaT);
		void dampVelocities(float deltaT, float kDamping);

		void updateTime(float deltaT) {
			world_->increaseTime(deltaT);
		}

		void setCollisionEngine(CollisionEngine * collisionEngine) { collisionEngine_ = collisionEngine; }
		void applyEstatico();

		void clearCollisionFlag();
		void velocityUpdate(float restitution, float friction);

	private:
		World * world_;
		CollisionEngine * collisionEngine_;
		unsigned int iterations_ = 14;
	};
}


