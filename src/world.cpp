#include "include/world.h"


using namespace mySim;

	
World::World() {};

void World::addMalla(Malla * malla) {
	arrayMallas_.push_back(malla);
	arrayNVertices_.push_back(malla->getNVertices());
}

const std::vector<Malla *> & World::getArrayMallas() const { return arrayMallas_; }//Solo lectura
std::vector<Malla *> & World::refArrayMallas() { return arrayMallas_; } // permite modificar. Usar con precaución.

void World::addExternalForce(ExternalForces * f) { arrayExternalForces_.push_back(f); }
std::vector<ExternalForces *> & World::getExternalForces() { return arrayExternalForces_; };//solo lectura

DampFunction * World::getDampFunction() { return damp_; }

void World::setDampFunction(DampFunction * damp) {
	damp_ = damp;
}