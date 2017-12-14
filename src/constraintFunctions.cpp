#include "include/constraintFunctions.h"
#include "include/matriz.h"

using namespace mySim;

//CLASE PADRE//

ConstraintFunction::ConstraintFunction() {}

float ConstraintFunction::eval(std::vector<vec3f> & positions) { return 0; };
float ConstraintFunction::eval(VertexArray * vertexArray) {
	return 0;
}

std::vector<vec3f> ConstraintFunction::obtainCorrection(std::vector<vec3f> & positions, std::vector<float> & weigth) {
	return vectorOfZerosVec3f(positions.size());
}

std::vector<vec3f> ConstraintFunction::obtainCorrection(VertexArray * vertexArray) {
	return vectorOfZerosVec3f(vertexArray->v_.size());
}

void ConstraintFunction::applyCorrectionFull(VertexArray * vertexArray, float stifness , unsigned int solverIterations) {

}

///CLASES DERIVADAS///

//Función de constraint de distancia

DistanceConstraintFunction::DistanceConstraintFunction() {}
DistanceConstraintFunction::DistanceConstraintFunction(float distance) {
	distance_ = distance;
}
DistanceConstraintFunction::~DistanceConstraintFunction() {}
float DistanceConstraintFunction::eval(std::vector<vec3f> & positions) {
	if (positions.size() != 2) return ConstraintFunction::eval(positions);
	else {
		return ((positions[0] - positions[1]).norm() - distance_);
	}
};

float DistanceConstraintFunction::eval(VertexArray * vertexArray) {
	if (static_cast<Arista *> (vertexArray) == nullptr) return ConstraintFunction::eval(vertexArray);
	else {
		Vertex * V0 = vertexArray->v_[0];
		Vertex * V1 = vertexArray->v_[1];

		return ((V0->getPosition2() - V1->getPosition2()).norm() - distance_);
	}
}

std::vector<vec3f> DistanceConstraintFunction::obtainCorrection(std::vector<vec3f> & positions, std::vector<float> & weight) {
	if (positions.size() != 2) return ConstraintFunction::obtainCorrection(positions, weight);
	else {
		vec3f dp1 = (-weight[0] / (weight[0] + weight[1]))*
			((positions[0] - positions[1]).norm() - distance_)*
			(1.0f / (positions[0] - positions[1]).norm())*
			(positions[0] - positions[1]);
		vec3f dp2 = (weight[1] / (weight[0] + weight[1]))*
			((positions[0] - positions[1]).norm() - distance_)*
			(1.0f / (positions[0] - positions[1]).norm())*
			(positions[0] - positions[1]);
		std::vector<vec3f> ret;
		ret.push_back(dp1);
		ret.push_back(dp2);
		return ret;
	}
}
std::vector<vec3f> DistanceConstraintFunction::obtainCorrection(VertexArray * vertexArray) {
	//if (static_cast<Arista *> (vertexArray) == nullptr) return ConstraintFunction::obtainCorrection(vertexArray);
	//else {
		Vertex * V0 = vertexArray->v_[0];
		Vertex * V1 = vertexArray->v_[1];

		vec3f dp1 = -1 * (V0->getWeight() / (V0->getWeight() + V1->getWeight()))*
			((V0->getPosition2() - V1->getPosition2()).norm() - distance_)*
			(1.0f / (V0->getPosition2() - V1->getPosition2()).norm())*
			(V0->getPosition2() - V1->getPosition2());

		vec3f dp2 = (V1->getWeight() / (V0->getWeight() + V1->getWeight()))*
			((V0->getPosition2() - V1->getPosition2()).norm() - distance_)*
			(1.0f / (V0->getPosition2() - V1->getPosition2()).norm())*
			(V0->getPosition2() - V1->getPosition2());

		std::vector<vec3f> ret;
		ret.push_back(dp1);
		ret.push_back(dp2);
		return ret;
	//}
}

void DistanceConstraintFunction::applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations) {
	//if (static_cast<Arista *> (vertexArray) == nullptr) return ConstraintFunction::applyCorrectionFull(vertexArray, stifness, solverIterations);
	//else {
	float tol = 10e-6;

	Vertex * V0 = vertexArray->v_[0];
	Vertex * V1 = vertexArray->v_[1];

	const vec3f & p1 = V0->getPosition2();
	const vec3f & p2 = V1->getPosition2();

	float w1 = V0->getWeight();
	float w2 = V1->getWeight();

	
	bool estatico0 = vertexArray->v_[0]->getEstatico();
	bool estatico1 = vertexArray->v_[1]->getEstatico();


	if (estatico0) w1 = 0;
	if (estatico1) w2 = 0;




	vec3f dp1;
	if (w1 <=tol) dp1 = vec3f(0.0f,0.0f,0.0f);
	else dp1= -1 * (w1 / (w1 + w2))*
		((p1 - p2).norm() - distance_)*
		(1.0f / (p1 - p2).norm())*
		(p1 -p2);

	vec3f dp2;
	if (w2 <=tol) dp2 = vec3f(0.0f, 0.0f, 0.0f);
	else dp2 = (w2 / (w1 + w2))*
		((p1 - p2).norm() - distance_)*
		(1.0f / (p1 - p2).norm())*
		(p1 - p2);

	//Esto es el retorno por referencia de la función
	V0->setPosition2(p1 + (1-powf(1 - stifness, 1.0/solverIterations))*dp1);
	V1->setPosition2(p2 + (1-powf(1 - stifness, 1.0/solverIterations))*dp2);
	//}
}

void DistanceConstraintFunction::setDistance(float distance) { distance_ = distance; }


//Función constraint de bending


BendingConstraintFunction::BendingConstraintFunction() {}
BendingConstraintFunction::BendingConstraintFunction(float angle) { angle_ = angle; }
BendingConstraintFunction::~BendingConstraintFunction() {}

float BendingConstraintFunction::eval(VertexArray * vertexArray) { return 0; }

std::vector<vec3f> BendingConstraintFunction::obtainCorrection(VertexArray * vertexArray) { 
	const vec3f & p1 = vertexArray->v_[0]->getPosition2();
	const vec3f & p2 = vertexArray->v_[1]->getPosition2();
	const vec3f & p3 = vertexArray->v_[2]->getPosition2();
	const vec3f & p4 = vertexArray->v_[3]->getPosition2();

	float w1 = vertexArray->v_[0]->getWeight();
	float w2 = vertexArray->v_[1]->getWeight();
	float w3 = vertexArray->v_[2]->getWeight();
	float w4 = vertexArray->v_[3]->getWeight();

	bool estatico0 = vertexArray->v_[0]->getEstatico();
	bool estatico1 = vertexArray->v_[1]->getEstatico();
	bool estatico2 = vertexArray->v_[2]->getEstatico();
	bool estatico3 = vertexArray->v_[3]->getEstatico();


	if (estatico0) w1 = 0;
	if (estatico1) w2 = 0;
	if (estatico2) w3 = 0;
	if (estatico3) w4 = 0;



	vec3f n1 = cross(p2,p3);
	n1.normalize();

	vec3f n2 = cross(p2,p4);
	n2.normalize();

	float d = dot(n1, n2);

	vec3f q3 = (cross(p2, n2) + d*(cross(n1, p2))) / cross(p2, p3).norm();
	vec3f q4 = (cross(p2, n1) + d*(cross(n2, p2))) / cross(p2, p4).norm();
	vec3f q2 = (cross(p3, n2) + d*(cross(n1, p3))) / -(cross(p2, p3).norm()) - (cross(p4, n1) + d*(cross(n2, p4))) / cross(p2, p4).norm();
	vec3f q1 = (-1)*(q2+q3+q4);

	float div = w1*q1.norm2() + w2*q2.norm2() + w3*q3.norm2() + w4*q4.norm2();

	vec3f dp1, dp2, dp3, dp4;

	if (w1 == 0) dp1 = vec3f(0, 0, 0);
	else dp1 = -(w1*sqrtf(1 - d*d)*(acosf(d) - angle_) / div)*q1;

	if (w2 == 0) dp2 = vec3f(0, 0, 0);
	else dp2 = -(w2*sqrtf(1 - d*d)*(acosf(d) - angle_) / div)*q2;
	
	if (w3 == 0) dp3 = vec3f(0, 0, 0);
	else dp3 = -(w3*sqrtf(1 - d*d)*(acosf(d) - angle_) / div)*q3;
	
	if (w4 == 0) dp4 = vec3f(0, 0, 0);
	else dp4 = -(w4*sqrtf(1 - d*d)*(acosf(d) - angle_) / div)*q4;

	std::vector<vec3f> ret;
	ret.push_back(dp1);
	ret.push_back(dp2);
	ret.push_back(dp3);
	ret.push_back(dp4);

	return ret;
};

void BendingConstraintFunction::applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations) {
	
	float tol = 10e-7f;

	vec3f p1 = vertexArray->v_[0]->getPosition2();
	vec3f p2 = vertexArray->v_[1]->getPosition2();
	vec3f p3 = vertexArray->v_[2]->getPosition2();
	vec3f p4 = vertexArray->v_[3]->getPosition2();

	p2 = p2 - p1;
	p3 = p3 - p1;
	p4 = p4 - p1;
	p1 = p1 - p1;

	float  w1 = vertexArray->v_[0]->getWeight();
	float  w2 = vertexArray->v_[1]->getWeight();
	float  w3 = vertexArray->v_[2]->getWeight();
	float  w4 = vertexArray->v_[3]->getWeight();
	
	bool estatico0 = vertexArray->v_[0]->getEstatico();
	bool estatico1 = vertexArray->v_[1]->getEstatico();
	bool estatico2 = vertexArray->v_[2]->getEstatico();
	bool estatico3 = vertexArray->v_[3]->getEstatico();

	if (estatico0) w1 = 0;
	if (estatico1) w2 = 0;
	if (estatico2) w3 = 0;
	if (estatico3) w4 = 0;


	vec3f n1 = cross(p2, p3);
	n1.normalize();

	vec3f n2 = cross(p2, p4);
	n2.normalize();

	float d = dot(n1, n2);

	if (d < -1) d = -1;
	if (d > 1) d = 1;

	vec3f q3 = (cross(p2, n2) + d*(cross(n1, p2))) / cross(p2, p3).norm();
	vec3f q4 = (cross(p2, n1) + d*(cross(n2, p2))) / cross(p2, p4).norm();
	vec3f q2 = (cross(p3, n2) + d*(cross(n1, p3))) / -(cross(p2, p3).norm())
		- (cross(p4, n1) + d*(cross(n2, p4))) / cross(p2, p4).norm();
	vec3f q1 = (-1)*(q2 + q3 + q4);

	float div = w1*q1.norm2() + w2*q2.norm2() + w3*q3.norm2() + w4*q4.norm2();
	if (div < tol) div = tol;

	vec3f dp1, dp2, dp3, dp4;

	float t = acosf(d);





	if (w1 == 0) dp1 = vec3f(0, 0, 0);
	else dp1 = -(w1*sqrtf(1 - d*d)*(t - angle_) / div)*q1;

	if (w2 == 0) dp2 = vec3f(0, 0, 0);
	else dp2 = -(w2*sqrtf(1 - d*d)*(t - angle_) / div)*q2;

	if (w3 == 0) dp3 = vec3f(0, 0, 0);
	else dp3 = -(w3*sqrtf(1 - d*d)*(t - angle_) / div)*q3;

	if (w4 == 0) dp4 = vec3f(0, 0, 0);
	else dp4 = -(w4*sqrtf(1 - d*d)*(t - angle_) / div)*q4;

	vertexArray->v_[0]->setPosition2(vertexArray->v_[0]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp1);
	vertexArray->v_[1]->setPosition2(vertexArray->v_[1]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp2);
	vertexArray->v_[2]->setPosition2(vertexArray->v_[2]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp3);
	vertexArray->v_[3]->setPosition2(vertexArray->v_[3]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp4);
}

void BendingConstraintFunction::setAngle(float angle) { angle_ = angle; }


//Función constraint de collision

CollisionVTConstraintFunction::CollisionVTConstraintFunction() {}
CollisionVTConstraintFunction::CollisionVTConstraintFunction(float distance, CollisionType type) {
	distance_ = distance; 
	type_ = type;

}
CollisionVTConstraintFunction::~CollisionVTConstraintFunction() { }

float CollisionVTConstraintFunction::eval(VertexArray * vertexArray) {
	//Vertex array es CollisionVT
	vec3f q = vertexArray->v_[0]->getPosition2();
	vec3f p1 = vertexArray->v_[1]->getPosition2();
	vec3f p2 = vertexArray->v_[2]->getPosition2();
	vec3f p3 = vertexArray->v_[3]->getPosition2();

	vec3f n = cross(p2 - p1, p3 - p1);
	n.normalize();
	if (type_ == CollisionType::NEGATIVE) n = -1 * n;
	return(dot(q - p1, n) - distance_);
}


void CollisionVTConstraintFunction::applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations) {

	float tol = 10e-6;

	vec3f q = vertexArray->v_[0]->getPosition2();
	vec3f p1 = vertexArray->v_[1]->getPosition2();
	vec3f p2 = vertexArray->v_[2]->getPosition2();
	vec3f p3 = vertexArray->v_[3]->getPosition2();

	

	q = q - p1;
	p2 = p2 - p1;
	p3 = p3 - p1;
	p1 = p1 - p1;

	vec3f n = cross(p2, p3);
	n.normalize();
	if (type_ == CollisionType::NEGATIVE) n = -1 * n;

	float fact = 1.0f/cross(p2, p3).norm();

	vec3f Gq = n;
	vec3f Gp2 = fact*(cross(p3, q) + dot(n, q)*cross(n, p3));
	vec3f Gp3 = -fact*(cross(p2, q) + dot(n, q)*cross(n, p2));
	vec3f Gp1 = -1*(Gq + Gp2 + Gp3);

	

	float C = dot(q, n) - distance_;

	float w0 = vertexArray->v_[0]->getWeight();
	float w1 = vertexArray->v_[1]->getWeight();
	float w2 = vertexArray->v_[2]->getWeight();
	float w3 = vertexArray->v_[3]->getWeight();

	bool estatico0 = vertexArray->v_[0]->getEstatico();
	bool estatico1 = vertexArray->v_[1]->getEstatico();
	bool estatico2 = vertexArray->v_[2]->getEstatico();
	bool estatico3 = vertexArray->v_[3]->getEstatico();


	if (estatico0) w0 = 0;
	if (estatico1) w1 = 0;
	if (estatico2) w2 = 0;
	if (estatico3) w3 = 0;


	float fact2 = w0*Gq.norm2() + w1*Gp1.norm2() + w2*Gp2.norm2() + w3*Gp3.norm2();

	float s;
	if (fact2 <= tol) s = 1;
	else s = C / fact2;

	vec3f dp0, dp1, dp2, dp3;



	if (w0 <= tol) dp0 = vec3f(0, 0, 0);
	else dp0 = -s*w0*Gq;
	if (w1 <= tol) dp1 = vec3f(0, 0, 0);
	else dp1 = -s*w1*Gp1;
	if (w2 <= tol) dp2 = vec3f(0, 0, 0);
	else dp2 = -s*w2*Gp2;
	if (w3 <= tol) dp3 = vec3f(0, 0, 0);
	else dp3 = -s*w3*Gp3;


	vertexArray->v_[0]->setPosition2(vertexArray->v_[0]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp0);
	vertexArray->v_[1]->setPosition2(vertexArray->v_[1]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp1);
	vertexArray->v_[2]->setPosition2(vertexArray->v_[2]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp2);
	vertexArray->v_[3]->setPosition2(vertexArray->v_[3]->getPosition2() + (1 - powf(1 - stifness, 1.0 / solverIterations))*dp3);

}


BalloonConstraintFunction::BalloonConstraintFunction(std::vector<Triangulo *> & triangulos, float volumen, float presion) {
	triangulos_ = triangulos;
	volumen_ = volumen;
	presion_ = presion;
}

float BalloonConstraintFunction::eval(VertexArray * vertexArray) {
	float ret = 0;
	for (auto it = triangulos_.begin(); it != triangulos_.end(); it++) {
		vec3f p1 = (*it)->v_[0]->getPosition2();
		vec3f p2 = (*it)->v_[1]->getPosition2();
		vec3f p3 = (*it)->v_[2]->getPosition2();
		ret = ret + dot(cross(p1, p2), p3);
	}
	ret = ret - presion_*volumen_;
	return ret;
}

void BalloonConstraintFunction::applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations) {

	std::vector<VERTEX_ID> IDs;
	std::vector<vec3f> grads;
	

	for (auto it = vertexArray->v_.begin(); it != vertexArray->v_.end(); it++) {
		IDs.push_back((*it)->getId());
		grads.push_back(vec3f(0, 0, 0));
	}
	int size = IDs.size();

	for (auto it = triangulos_.begin(); it != triangulos_.end(); it++) {
		VERTEX_ID ID0 = (*it)->v_[0]->getId();
		VERTEX_ID ID1 = (*it)->v_[1]->getId();
		VERTEX_ID ID2 = (*it)->v_[2]->getId();

		vec3f pos0 = (*it)->v_[0]->getPosition2();
		vec3f pos1 = (*it)->v_[1]->getPosition2();
		vec3f pos2 = (*it)->v_[2]->getPosition2();
		
		vec3f vec0 = cross(pos1, pos2);
		vec3f vec1 = cross(pos2, pos0);
		vec3f vec2 = cross(pos0, pos1);

		for (int i = 0; i < size; i++) {
			if (ID0 == IDs[i]) grads[i] = grads[i] + vec0;
			if (ID1 == IDs[i]) grads[i] = grads[i] + vec1;
			if (ID2 == IDs[i]) grads[i] = grads[i] + vec2;
		}
	}
	
	float sum = 0;
	for (int i = 0; i < size; i++) {
		float w = vertexArray->v_[i]->getWeight();
		float norm2 = grads[i].norm2();
		sum = sum + (w*norm2);
	}

	float s = eval(vertexArray) / (sum);

	for (int i = 0; i < size; i++) {
		vec3f corr = grads[i];
		vec3f newpos = vertexArray->v_[i]->getPosition2() - s*vertexArray->v_[i]->getWeight()*corr;
		vertexArray->v_[i]->setPosition2(newpos);
	}
}

