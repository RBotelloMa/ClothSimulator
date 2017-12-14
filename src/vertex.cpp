#include "include/vertex.h"

using namespace mySim;

//VERTEX//
	
Vertex::Vertex(float mass, vec3f position, vec3f velocity) {
	id_ = IdNext();
	mass_ = mass;
	if (mass != 0) weight_ = 1.0f / mass;
	else weight_ = std::numeric_limits<float>::infinity();
	position_ = position;
	position2_ = position;
	velocity_ = velocity;
}

Vertex::Vertex(float mass, vec3f position, vec3f velocity, bool estatico) : Vertex(mass, position, velocity){
	estatico_ = estatico;
}



Vertex::~Vertex() {
}

float Vertex::getMass() { 
	return mass_; 
};
void Vertex::setMass(float mass) { 
	mass_ = mass; 
	weight_ = 1.0f / mass; 
};

float Vertex::getWeight() { 
	return weight_; 
}

const mySim::vec3f Vertex::getPosition() const { 
	return position_; 
};

void Vertex::setPosition(vec3f position) { 
	position_ = position; 
}

const mySim::vec3f Vertex::getPosition2() const { 
	return position2_; 
};
void Vertex::setPosition2(vec3f position2) { 
	position2_ = position2; 
}

const mySim::vec3f Vertex::getVelocity() const { 
	return velocity_; 
};
void Vertex::setVelocity(vec3f velocity) { 
	velocity_ = velocity; 
}

VERTEX_ID Vertex::IdNext() {
	static VERTEX_ID idCount = 0;
	return ++idCount;
}





//VERTEX ARRAY//

VertexArray::VertexArray() {};
VertexArray::VertexArray(std::vector<Vertex *> & v) {
	v_ = v;
}

//acceso y escritura transversal a posicion
const std::vector<vec3f> VertexArray::getPositions() const {
	std::vector<vec3f> positionRet;
	for (auto it = v_.begin(); it != v_.end(); it++) {
		positionRet.push_back((*it)->getPosition());
	}
	return positionRet;
}

const void VertexArray::setPositions(std::vector<vec3f> position) const {
	for (unsigned int i = 0; i < position.size(); i++) {
		v_[i]->setPosition(position[i]);
	}
}

//acceso y escritura transversal a posicion2
const std::vector<vec3f> VertexArray::getPositions2() const {
	std::vector<vec3f> position2Ret;
	for (auto it = v_.begin(); it != v_.end(); it++) {
		position2Ret.push_back((*it)->getPosition2());
	}
	return position2Ret;
}
const void VertexArray::setPositions2(std::vector<vec3f> position2) const {
	for (unsigned int i = 0; i < position2.size(); i++) {
		v_[i]->setPosition2(position2[i]);
	}
}

//acceso y escritura transversal a speed
const std::vector<vec3f> VertexArray::getSpeeds() const {
	std::vector<vec3f> speedRet;
	for (auto it = v_.begin(); it != v_.end(); it++) {
		speedRet.push_back((*it)->getVelocity());
	}
	return speedRet;
}

const void VertexArray::setSpeeds(std::vector<vec3f> speed) const {
	for (unsigned int i = 0; i < speed.size(); i++) {
		v_[i]->setVelocity(speed[i]);
	}
}

boundingBox  VertexArray::getBoundingBox() {
	bool ok = false;
	float minX, minY, minZ, maxX, maxY, maxZ;
	auto it = v_.begin();
	if (it != v_.end()) {
		ok = true;
		minX = (*it)->getPosition2().x;
		minY = (*it)->getPosition2().y;
		minZ = (*it)->getPosition2().z;
		maxX = (*it)->getPosition2().x;
		maxY = (*it)->getPosition2().y;
		maxZ = (*it)->getPosition2().z;
		it++;
	}
	for (; it != v_.end(); it++) {
		if (minX > (*it)->getPosition2().x) minX = (*it)->getPosition2().x;
		if (minY > (*it)->getPosition2().y) minY = (*it)->getPosition2().y;
		if (minZ > (*it)->getPosition2().z) minZ = (*it)->getPosition2().z;
		if (maxX < (*it)->getPosition2().x) maxX = (*it)->getPosition2().x;
		if (maxY < (*it)->getPosition2().y) maxY = (*it)->getPosition2().y;
		if (maxZ < (*it)->getPosition2().z) maxZ = (*it)->getPosition2().z;
	}
	if (ok) return boundingBox(minX, minY, minZ, maxX, maxY, maxZ);
	else return boundingBox();
}


bool VertexArray::vertexIsHere(Vertex * vertex) {
	for (auto it = v_.begin(); it != v_.end(); it++) {
		if ((*it)->getId() == vertex->getId())
			return true;
	}
	return false;
}
