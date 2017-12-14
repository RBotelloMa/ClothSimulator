#include "include/collisions.h"



using namespace mySim;

CollisionEngine::~CollisionEngine() {
	delete hash_;
	for (auto it = collisionConstraints_.begin(); it != collisionConstraints_.end(); ++it) {
		delete(*it);
	};
}


void CollisionEngine::clearCollsionConstraints() {
	for (auto it = collisionConstraints_.begin(); it != collisionConstraints_.end(); ++it) {
		delete(*it);
	};
	collisionConstraints_.clear();
}



CollisionType CollisionEngine::CheckColision(CollisionVT * vertexArray) {
	
	float tol = 10e-10;
	
	CollisionType ret = CollisionType::POSITIVE;
	
	vec3f q0 = vertexArray->v_[0]->getPosition();
	vec3f p0 = vertexArray->v_[0]->getPosition2();

	vec3f p1 = vertexArray->v_[1]->getPosition2();
	vec3f p2 = vertexArray->v_[2]->getPosition2();
	vec3f p3 = vertexArray->v_[3]->getPosition2();

	q0 = q0 - p1;
	p0 = p0 - p1;
	p2 = p2 - p1;
	p3 = p3 - p1;
	p1 = p1 - p1;

	vec3f v = p0 - q0;


	vec3f n = cross(p2, p3);
	n.normalize();

	if (dot(n, q0) < 0) {
		n = -1 * n;
		ret = CollisionType::NEGATIVE;
	}

	bool atraviesa = false;
	if (dot(n, p0) <= 0) {
		atraviesa = true;
	}

	//atravesadores rápido (rayos rápidos que atraviesan el plano)

	matriz3f M(p2, p3, n);
	matriz3f IM = M.inversa();

	vec3f Q = IM*q0;
	vec3f V = (IM*v).normalize();
	if(V.z>=tol || V.z<=-tol) V = V / abs(V.z);
	vec3f P = IM*p0;


	/*if (P.z<vertexArray->getClothThickness() / 2 && P.z>-vertexArray->getClothThickness() / 1000) {
		return CollisionType::EXCLUDE_COLLISION;
	}*/

	/*if (P.x + P.y <= 1 + tolSide_ && P.x >= -tolSide_ && P.y >= -tolSide_ && P.z <= vertexArray->getClothThickness()/20) {
		return ret;
	}*/

	float h = Q.z;
	vec3f C = Q + h*V;
	
	if (atraviesa) {
		if (C.x + C.y <= 1 + tolSide_ && C.x >= -tolSide_ && C.y >= -tolSide_) {
			return ret;
		}
	}
	return CollisionType::NOT_COLLISION;
}

CollisionType CollisionEngine::CheckColision2(CollisionVT * vertexArray, float deltaT) {

	float tol = 10e-6;
	float tol2 = 10e-6;

	CollisionType ret = CollisionType::POSITIVE;

	vec3f q0 = vertexArray->v_[0]->getPosition();
	vec3f q1 = vertexArray->v_[1]->getPosition();
	vec3f q2 = vertexArray->v_[2]->getPosition();
	vec3f q3 = vertexArray->v_[3]->getPosition();

	vec3f pp0 = vertexArray->v_[0]->getPosition2();
	vec3f pp1 = vertexArray->v_[1]->getPosition2();
	vec3f pp2 = vertexArray->v_[2]->getPosition2();
	vec3f pp3 = vertexArray->v_[3]->getPosition2();

	vec3f v0 = pp0 - q0;
	vec3f v1 = pp1 - q1;
	vec3f v2 = pp2 - q2;
	vec3f v3 = pp3 - q3;

	//C0 = q0+t*v0; C2 = q2 + t*v2; C3 = q3 + t*v3
	float t;
	bool col = numericCalc::newtonTest(q0, v0, q1, v1, q2, v2, q3, v3, 0, 1, 1, 100, tol, t);

	if (!col) return CollisionType::NOT_COLLISION;
	if (t<-tol|| t> 1+tol){
		return CollisionType::NOT_COLLISION;
	}

	vec3f p0 = q0 + t*v0;
	vec3f p1 = q1 + t*v1;
	vec3f p2 = q2 + t*v2;
	vec3f p3 = q3 + t*v3;

	p0 = p0 - p1;
	p2 = p2 - p1;
	p3 = p3 - p1;
	vec3f q0r = q0 - p1;


	vec3f n = cross(p2, p3);
	n.normalize();

	if (dot(n, q0r) < 0) {
		n = -1 * n;
		ret = CollisionType::NEGATIVE;
	}

	matriz3f M(p2, p3, n);
	matriz3f IM = M.inversa();

	vec3f P = IM*p0;

	if (P.x + P.y <= 1 && P.x >= 0 && P.y >= 0 && P.z <= tol2 && P.z >= -tol2) {
		return ret;
	}

	return CollisionType::NOT_COLLISION;
}

CollisionType CollisionEngine::CheckColision3(CollisionVT * vertexArray) {

	float tol = 10e-6;
	

	CollisionType ret = CollisionType::POSITIVE;

	vec3f q0 = vertexArray->v_[0]->getPosition();
	vec3f q1 = vertexArray->v_[1]->getPosition();
	vec3f q2 = vertexArray->v_[2]->getPosition();
	vec3f q3 = vertexArray->v_[3]->getPosition();

	vec3f pp0 = vertexArray->v_[0]->getPosition2();
	vec3f pp1 = vertexArray->v_[1]->getPosition2();
	vec3f pp2 = vertexArray->v_[2]->getPosition2();
	vec3f pp3 = vertexArray->v_[3]->getPosition2();

	vec3f v0 = pp0 - q0;
	vec3f v1 = pp1 - q1;
	vec3f v2 = pp2 - q2;
	vec3f v3 = pp3 - q3;

	vec3f k1 = v0 - v1;
	vec3f k2 = cross((v2 - v1), (v3 - v2));
	vec3f k3 = q0 - q1;
	vec3f k4 = cross(q2 - q1, v3 - v2) + cross(v2 - v1, q3 - q2);
	vec3f k5 = cross(q2 - q1, q3 - q2);

	float f3 = dot(k1, k2);
	float f2 = dot(k3, k2) + dot(k1, k4);
	float f1 = dot(k3, k4) + dot(k1, k5);
	float f0 = dot(k3, k5);

	float t;
	bool b1 = numericCalc::degreeThreeTest(f3, f2, f1, f0, 0.0f, 1.0f, 20, tol, t);

	if (!b1) return CollisionType::NOT_COLLISION;
	else {
		vec3f p0 = q0 + t*v0;
		vec3f p1 = q1 + t*v1;
		vec3f p2 = q2 + t*v2;
		vec3f p3 = q3 + t*v3;
		

		p0 = p0 - p1;
		p2 = p2 - p1;
		p3 = p3 - p1;
		vec3f q0r = q0-p1;
		
		vec3f n = cross(p2, p3);
		n.normalize();

		/*if (dot(n, q0r) > -tol && dot(n, q0r) <tol) {
			ret = CollisionType::NOT_COLLISION;//desplazamiento plano
		}*/

		if (dot(n, q0r) < -tol) {
			//n = -1 * n;
			ret = CollisionType::NEGATIVE;
		}

		matriz3f M(p2, p3, n);
		matriz3f IM = M.inversa();

		vec3f P = IM*p0;

		if (P.x + P.y <= 1 && P.x >= 0 && P.y >= 0) {
			return ret;
		}

		return CollisionType::NOT_COLLISION;
	}
}



//has update 
void CollisionEngine::updateHashVertices(MallaTriangular * malla, float time) {
	const VertexArray * VArray = malla->getVertexArray();
	for (auto it = VArray->v_.begin(); it != VArray->v_.end(); it++) {
		hash_->addMovingVertex(*it, time);
	}
}
void CollisionEngine::updateHashTriangulos(MallaTriangular * malla, float time) {
	const std::vector<Triangulo*> & TArray = malla->getTriangulos();
	for (auto it = TArray.begin(); it != TArray.end(); it++) {
		hash_->addTriangle(*it, time);
	}
}
void CollisionEngine::updateHash(World * world) {
	for (auto it = world->getArrayMallas().begin(); it != world->getArrayMallas().end(); it++) {
		updateHashVertices(static_cast<MallaTriangular*>(*it), world->getTime());
	}
	for (auto it = world->getArrayMallas().begin(); it != world->getArrayMallas().end(); it++) {
		updateHashTriangulos(static_cast<MallaTriangular*>(*it), world->getTime());
	}
}

//collisions calculations

void CollisionEngine::generateConstraints(World * world, float deltaT) {
	
	clearCollsionConstraints();

	int dimx, dimy, dimz;
	float time = world->getTime();
	for (int i = 0; i < hash_->getHashSize(); i++) {
		//celda es vec3i(ix,iy,iz)
		if (hash_->goToTimestamp(i) == time) {
			for (auto V = hash_->goToPoints(i)->v_.begin(); V != hash_->goToPoints(i)->v_.end(); V++) {
				for (auto T = hash_->goToTriangles(i).begin(); T != hash_->goToTriangles(i).end(); T++) {
					if (!((*T)->vertexIsHere(*V))) {
						bool test = ((*T)->checkCollID(*V));
						if (!test) {
							CollisionVT * cvt = new CollisionVT(*V, *T);
							CollisionType type = CollisionType::NOT_COLLISION;
							type = CheckColision3(cvt);
							if (type == CollisionType::NOT_COLLISION) type = CheckColision(cvt);
							if (type != CollisionType::NOT_COLLISION) {
								//crear constraint
								CollisionVTConstraint * coll = new CollisionVTConstraint(cvt, collisionStifness_, cvt->getClothThickness(), type);
								//for (auto it = cvt->v_.begin(); it != cvt->v_.end(); it++) {
									//(*it)->setCollision(true);
								//}
								cvt->v_[0]->setCollision(true);
								//añadir constraint a world->collisionConstraints_ (metodo necesario);
								addCollisionConstraint(coll);
							}
							else delete cvt;
						}
					}
				}
			}
		}
	}
}

void CollisionEngine::selfCollsionStep(World * world, float deltaT) {
	updateHash(world);
	generateConstraints(world, deltaT);
};


