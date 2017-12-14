#include "include/constraint.h"
#include "include/malla.h"

using namespace mySim;

MyConstraint::MyConstraint(int cardinality, ConstraintFunction * funcion, VertexArray * vertices,
			float stifness, ConstraintType constraintType) {
			cardinality_ = cardinality;
			funcion_ = funcion;
			vertices_ = vertices;
			stifness_ = stifness;
			constraintType_ = constraintType;
		}

MyConstraint::~MyConstraint() {
	delete funcion_;
}


float MyConstraint::evaluateF(std::vector<vec3f> argument) {
	return funcion_->eval(argument);
}

float MyConstraint::evaluateF() {
	return funcion_->eval(vertices_);
}

bool MyConstraint::applyBool(std::vector<vec3f> argument) {
	if (constraintType_ == ConstraintType::EQUALITY)
		return true;
	if (constraintType_ == ConstraintType::INEQUALITY)
		return (evaluateF(argument) < 0);
}

bool MyConstraint::applyBool() {
	if (constraintType_ == ConstraintType::EQUALITY)
		return true;
	if (constraintType_ == ConstraintType::INEQUALITY)
		return (evaluateF() < 0);
}

void MyConstraint::applyConstraintFull(unsigned int solverIterations) {
	if (applyBool()) {
		funcion_->applyCorrectionFull(vertices_, stifness_, solverIterations);
	};
}

//Constraint especial para aristas//

//
AristaConstraint::AristaConstraint(Arista * arista, float stifness) : 
	MyConstraint(2, new DistanceConstraintFunction() , arista, stifness, ConstraintType::EQUALITY) {}

AristaConstraint::AristaConstraint(Arista * arista, float stifness, float distance) :
	MyConstraint(2, new DistanceConstraintFunction(distance), arista, stifness, ConstraintType::EQUALITY) {}

AristaConstraint::~AristaConstraint() {}

//Constraints para bending

BendingConstraint::BendingConstraint(DobleTriangulo * dobleTriangulo, float stifness) :
	MyConstraint(4, new BendingConstraintFunction(), dobleTriangulo, stifness, ConstraintType::EQUALITY) {}

BendingConstraint::BendingConstraint(DobleTriangulo * dobleTriangulo, float stifness, float angle) :
	MyConstraint(4, new BendingConstraintFunction(angle), dobleTriangulo, stifness, ConstraintType::EQUALITY) {}
BendingConstraint::~BendingConstraint() {}

//Constraints para colsionesVT

CollisionVTConstraint::CollisionVTConstraint(CollisionVT * collisionVT, float stifness) :
	MyConstraint(4, new CollisionVTConstraintFunction(), collisionVT, stifness, ConstraintType::INEQUALITY) {
}
CollisionVTConstraint::CollisionVTConstraint(CollisionVT * collisionVT, float stifness, float distance, CollisionType collisionType) :
	MyConstraint(4, new CollisionVTConstraintFunction(distance,collisionType), collisionVT, stifness, ConstraintType::INEQUALITY) {
}
CollisionVTConstraint::~CollisionVTConstraint() {
	DELETEV();
}


void MyConstraint::DELETEV() {
	delete vertices_;
}


//Constraints para balloons

BalloonConstraint::BalloonConstraint(VertexArray * verticesBalloon, std::vector<Triangulo*> & triangulos,float volumen, float presion):
	MyConstraint(verticesBalloon->v_.size(), new BalloonConstraintFunction(triangulos, volumen, presion), verticesBalloon, 1, ConstraintType::EQUALITY) {
}