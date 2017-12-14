#include "include/malla.h"


using namespace mySim;
Malla::Malla() {
	vertexArray_ = new VertexArray();
}

const VertexArray * Malla::getVertexArray() const { 
	return vertexArray_; 
}

VertexArray * Malla::getVertexArray2() {
	return vertexArray_;
}

const VertexArray * Malla::refVertexArray() const { 
	return vertexArray_; 
}
void Malla::setVertexArray(VertexArray * vertexArray){
	vertexArray_ = vertexArray;
}

const std::vector<MyConstraint *> & Malla::getConstraints() const {
	return constraintArray_;
}


void Malla::addVertex(Vertex * vertex) {
	vertexArray_->v_.push_back(vertex);

}
void Malla::addConstraint(MyConstraint * constraint) {
	constraintArray_.push_back(constraint);
}

unsigned int Malla::getNVertices() { 
	return (unsigned int)vertexArray_->v_.size(); 
}

void Malla::applyAllConstraints(unsigned int solverIterations) {
	for (auto it = constraintArray_.begin(); it != constraintArray_.end(); it++) {
		(*it)->applyConstraintFull(solverIterations);
	}
}

//MALLA TRIANGULAR

void MallaTriangular::addArista(Arista * arista, float stifness, float distance) {
	aristas_.push_back(arista);
	AristaConstraint * aristaC = new AristaConstraint(arista, stifness, distance);
	addConstraint(aristaC);
}

void MallaTriangular::addTriangulo(Triangulo * triangulo) {
	for (auto it = triangulos_.begin(); it != triangulos_.end(); it++) {
		if (DobleTriangulo::DobleTCheck(triangulo, (*it))) {
			DobleTriangulo * DT = new DobleTriangulo(triangulo, (*it));
			dobleTriangulos_.push_back(DT);
			addConstraint(new BendingConstraint(DT, bendingStifness_, bendingAngle_));
		}
	}
	triangulos_.push_back(triangulo);
	


	//Faltan contraints
}


const std::vector<Arista *> & MallaTriangular::getAristas() const {
	return aristas_;
};

const std::vector<Triangulo *> & MallaTriangular::getTriangulos() const {
	return triangulos_;
}

