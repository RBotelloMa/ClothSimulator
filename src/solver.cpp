#include "include/solver.h"

using namespace mySim;

Solver::Solver() {};
Solver::Solver(World * world) {
	world_ = world;
};
Solver::~Solver() {
	delete collisionEngine_;
	delete world_;
}


const World * Solver::getWorld() const { return world_; } //Solo lectura
void Solver::setWorld(World * world) { world_ = world; } //Para añadir world
World * Solver::refWorld() { return world_; } //Para modificar world. Usar con precaución;

void Solver::solverMainLoop(float deltaT) {
	//update time
	updateTime(deltaT);
	
	///aplicamos fuerzas externas
	applyExternalForces(deltaT);
	
	///DampVelocities
	float kDamping = 0.00;
	dampVelocities(deltaT, kDamping);
	
	///update new positions pre constrains projections
	updatePreConstraints(deltaT);
	applyEstatico();
	clearCollisionFlag();


	//generate collisions
	collisionEngine_->selfCollsionStep(world_, deltaT);
	for (auto malla = world_->getArrayMallas().begin(); malla != world_->getArrayMallas().end(); malla++) {
		for (auto vect = (*malla)->getVertexArray()->v_.begin(); vect != (*malla)->getVertexArray()->v_.end(); vect++) {
			(*vect)->setPosition3();
		}
	}

	///poject constraints loop
	for (unsigned int i = 0; i < iterations_; ++i)	{
		applyConstraints(deltaT);
		collisionEngine_->applyCollisionConstraints(iterations_);
		applyEstatico();
	}
	
	///update constrained data
	updateAfterConstraints(deltaT);

	///VelocityUpdate
	float restitution = 1.0;
	float friction = 0.0f;

	//velocityUpdate(restitution, friction);
};

void Solver::applyExternalForces(float deltaT) {
	for (auto it = world_->getArrayMallas().begin(); it != world_->getArrayMallas().end(); it++) {
		for (auto V = (*it)->refVertexArray()->v_.begin(); V != (*it)->refVertexArray()->v_.end(); V++) {
			vec3f sumFuerzas = vec3f(0, 0, 0);
			for (auto it2 = world_->getExternalForces().begin(); it2 != world_->getExternalForces().end(); it2++) {
				ExternalForces * E = (*it2);
				vec3f aux1 = E->eval(vec3f(0, 0, 0));
				vec3f aux2 = E->eval((*V)->getPosition());
				vec3f fuerza = E->eval((*V)->getPosition());
				sumFuerzas = sumFuerzas + fuerza;
			}
			sumFuerzas;
			(*V)->setVelocity((*V)->getVelocity() + deltaT * (*V)->getWeight() * sumFuerzas);
		}
	}
}

void Solver::updatePreConstraints(float deltaT) {
	for (auto it = world_->getArrayMallas().begin(); it != world_->getArrayMallas().end(); it++) {
		for (auto V = (*it)->refVertexArray()->v_.begin(); V != (*it)->refVertexArray()->v_.end(); V++) {
			vec3f pos2 = (*V)->getPosition() + deltaT*(*V)->getVelocity();
			(*V)->setPosition2(pos2);
		}
	}
}


void Solver::applyConstraints(float deltaT) {
	for (auto it = world_->refArrayMallas().begin(); it != world_->refArrayMallas().end(); it++) {
		(*it)->applyAllConstraints(iterations_);
	}
}




void Solver::updateAfterConstraints(float deltaT) {
	for (auto it = world_->getArrayMallas().begin(); it != world_->getArrayMallas().end(); it++) {
		for (auto W = (*it)->refVertexArray()->v_.begin(); W != (*it)->refVertexArray()->v_.end(); W++) {
			Vertex * test = (*W);
			test->setVelocity2(test->getVelocity());
			test->setVelocity(((*W)->getPosition2() - (*W)->getPosition()) / deltaT);
			test->setPosition((*W)->getPosition2());
		}
	}
}


void Solver::dampVelocities(float deltaT, float kDamping) {
	for (auto it = world_->getArrayMallas().begin(); it != world_->getArrayMallas().end(); it++) {
		world_->getDampFunction()->damp((*it)->getVertexArray(), kDamping);
	}

}

void Solver::applyEstatico() {
	for (auto it = world_->getArrayMallas().begin(); it != world_->getArrayMallas().end(); it++) {
		for (auto it2 = (*it)->getVertexArray()->v_.begin(); it2 != (*it)->getVertexArray()->v_.end(); it2++) {
			if ((*it2)->getEstatico()) (*it2)->setPosition2((*it2)->getPosition());
		}
	}
}

void Solver::clearCollisionFlag() {
	for (auto malla = world_->getArrayMallas().begin(); malla != world_->getArrayMallas().end(); malla++) {
		for (auto vect = (*malla)->getVertexArray()->v_.begin(); vect != (*malla)->getVertexArray()->v_.end(); vect++) {
			(*vect)->setCollision(false);
		}
	}
}


void Solver::velocityUpdate(float restitution, float friction) {
	for (auto malla = world_->getArrayMallas().begin(); malla != world_->getArrayMallas().end(); malla++) {
		for (auto vect = (*malla)->getVertexArray()->v_.begin(); vect != (*malla)->getVertexArray()->v_.end(); vect++) {
			if ((*vect)->getCollision() == true) {

				vec3f normal = (*vect)->getPosition2() - (*vect)->getPosition3();
				normal.normalize();

				float f1 = dot(normal, ((*vect)->getVelocity()));
				vec3f norm = f1*normal;
				vec3f tan = ((*vect)->getVelocity()) - norm;
				tan = (1 - friction)*tan;
				if (f1 < 0) norm = -1 * norm;
				norm = restitution*norm;
				vec3f newSpeed = norm + tan;
				((*vect)->setVelocity(newSpeed));

			}
		}
	}
}