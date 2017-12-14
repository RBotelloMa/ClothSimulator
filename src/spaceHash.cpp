#include "include/spaceHash.h"


using namespace mySim;

SpaceHash::~SpaceHash() {
	for (auto it = hashTablePoints_.begin(); it != hashTablePoints_.end(); it++) {
		delete(*it);
	}
};


SpaceHash::SpaceHash(int hashSize, vec3f boxLenght) {
		boxLenght_ = boxLenght;
		hashSize_ = hashSize;
		for (int i=0; i< hashSize_; i++){
			VertexArray * va = new VertexArray();
			std::vector<Triangulo*> ta;
			float f = 0.0f;
			hashTablePoints_.push_back(va);
			hashTableTriangles_.push_back(ta);
			hashTableTimestamp_.push_back(f);
		}
	}

void SpaceHash::addVertex(Vertex * vertex, float time, vec3i quadrant) {
	int hashValue = ijk_To_Array(quadrant);
	if (hashTableTimestamp_[hashValue] < time) {
		hashTablePoints_[hashValue]->v_.clear();
		hashTableTriangles_[hashValue].clear();
		hashTableTimestamp_[hashValue] = time;
	}
	hashTablePoints_[hashValue]->v_.push_back(vertex);
}

void SpaceHash::addMovingVertex2(Vertex * vertex, float time) {
	float tol = 10e-10f;
	
	vec3f x0 = vertex->getPosition();
	vec3f p0 = vertex->getPosition2();
	vec3f v = p0 - x0;

	v.normalize();

	vec3i Z = obtainQuadrant(x0);
	vec3i ZEnd = obtainQuadrant(p0);
	vec3f Zf = vec3f(Z.x*boxLenght_.x, Z.y*boxLenght_.y, Z.z*boxLenght_.z);
	vec3f x = x0 - Zf;
	
	//do
	addVertex(vertex, time, Z);
	while(!(Z.x == ZEnd.x && Z.y == ZEnd.y && Z.z == ZEnd.z)){

		int C1, C2, C3;
		float F1, F2, F3;
		float G1, G2, G3;
		
		if (v.x > tol)      { C1 = 1;   F1 = boxLenght_.x;  G1 = 0;     }
		else if(v.x <-tol)   { C1 = -1;  F1 = 0;      G1 = boxLenght_.x; }
		else              { C1 = 0;                            }


		if (v.y > tol)      { C2 = 1;   F2 = boxLenght_.y;  G2 = 0;     }
		else if (v.y <-tol)  { C2 = -1;  F2 = 0;      G2 = boxLenght_.y; }
		else              { C2 = 0;                            }

		if (v.z > tol)      { C3 = 1;   F3 = boxLenght_.z;  G3 = 0;     }
		else if (v.z <-tol)  { C3 = -1;  F3 = 0;      G3 = boxLenght_.z; }
		else              { C3 = 0;                            }

		float l1, l2, l3;

		if (C1 != 0) { l1 = (F1-x.x) / v.x; }
		if (C2 != 0) { l2 = (F2-x.y) / v.y; }
		if (C3 != 0) { l3 = (F3-x.z) / v.z; }

		bool T1, T2, T3;

		T1 = ((C1 != 0) && (C2 == 0 || l1 <= l2) && (C3 == 0 || l1 <= l3));
		T2 = ((C2 != 0) && (C1 == 0 || l2 <= l1) && (C3 == 0 || l2 <= l3));
		T3 = ((C3 != 0) && (C1 == 0 || l3 <= l1) && (C2 == 0 || l3 <= l2));

		float L;

		if (T1) L = l1;
		else if (T2) L = l2;
		else if (T3) L = l3;
		else{//no debería pasar esto nunca
			printf("HOLLY SHITTTTTTTT");
		}

		x = x + L*v;//nueva posición

		int Sum = (int) T1 + (int) T2 + (int) T3;

		if (Sum == 1) {
			if (T1) {
				Z = Z + vec3i(C1, 0, 0);
				x.x = G1;
				addVertex(vertex, time, Z);
			}
			else if (T2) {
				Z = Z + vec3i(0, C2, 0);
				x.y = G2;
				addVertex(vertex, time, Z);
			}
			else {
				Z = Z + vec3i(0, 0, C3);
				x.z = G3;
				addVertex(vertex, time, Z);
			}
		}
		else if (Sum == 2) {
			if (T1) {
				addVertex(vertex, time, Z + vec3i(C1, 0, 0));
				if (T2) {
					addVertex(vertex, time, Z + vec3i(0, C2, 0));
					Z = Z + vec3i(C1, C2, 0);
					x.x = G1;
					x.y = G2;
					addVertex(vertex, time, Z);
				}
				else {
					addVertex(vertex, time, Z + vec3i(0, 0, C3));
					Z = Z + vec3i(C1, 0, C3);
					x.x = G1;
					x.z = G3;
					addVertex(vertex, time, Z);
				}
			}
			else {
				addVertex(vertex, time, Z + vec3i(0, C2, 0));
				addVertex(vertex, time, Z + vec3i(0, 0, C3));
				Z = Z + vec3i(0, C2, C3);
				x.y = G2;
				x.z = G3;
				addVertex(vertex, time, Z);
			}


		}
		else if (Sum == 3) {
			addVertex(vertex, time, Z + vec3i(C1, 0, 0));
			addVertex(vertex, time, Z + vec3i(0, C2, 0));
			addVertex(vertex, time, Z + vec3i(0, 0, C3));
			addVertex(vertex, time, Z + vec3i(C1, C2, 0));
			addVertex(vertex, time, Z + vec3i(C1, 0, C3));
			addVertex(vertex, time, Z + vec3i(0, C2, C3));
			Z = Z + vec3i(C1, C2, C3);
			x.x = G1;
			x.y = G2;
			x.z = G3;
			addVertex(vertex, time, Z + vec3i(C1, C2, C3));
		}
	}
}

void SpaceHash::addMovingVertex(Vertex * vertex, float time) {
	float tol = 10e-10f;

	vec3f x0 = vertex->getPosition();
	vec3f p0 = vertex->getPosition2();
	vec3i Hash1 = obtainQuadrant(x0);
	vec3i Hash2 = obtainQuadrant(p0);

	int minx, miny, minz, maxx, maxy, maxz;
	
	if (Hash1.x <= Hash2.x) { minx = Hash1.x; maxx = Hash2.x; }
	else { minx = Hash2.x; maxx = Hash1.x; }

	if (Hash1.y <= Hash2.y) { miny = Hash1.y; maxy = Hash2.y; }
	else { miny = Hash2.y; maxy = Hash1.y; }

	if (Hash1.z <= Hash2.z) { minz = Hash1.z; maxz = Hash2.z; }
	else { minz = Hash2.z; maxz = Hash1.z; }

	vec3f minHash(minx, miny, minz);
	vec3f maxHash(maxx, maxy, maxz);


	for (int ix = minHash.x; ix <= maxHash.x; ix++) {
		for (int iy = minHash.y; iy <= maxHash.y; iy++) {
			for (int iz = minHash.z; iz <= maxHash.z; iz++) {
				addVertex(vertex, time, vec3i(ix, iy, iz));
			}
		}
	}
}



void SpaceHash::addTriangle(Triangulo * triangulo, float time) {
	boundingBox bb = triangulo->getBoundingBox();
	triangulo->clearCollId();
	vec3i minHash = obtainQuadrant(bb.minPoint);
	vec3i maxHash = obtainQuadrant(bb.maxPoint);

	for (int ix = minHash.x; ix <= maxHash.x; ix++) {
		for (int iy = minHash.y; iy <= maxHash.y; iy++) {
			for (int iz = minHash.z; iz <= maxHash.z; iz++) {
				int hashValue = ijk_To_Array(vec3i(ix, iy, iz));
				if (hashTableTimestamp_[hashValue] < time) {
					//no se hace nada, pues no hay puntos en el cuadrante
				}
				else {
					hashTableTriangles_[hashValue].push_back(triangulo);
				}
			}
		}
	}
}

VertexArray * SpaceHash::goToPoints(int posicion) {
	return hashTablePoints_[posicion];
}

std::vector<Triangulo*> & SpaceHash::goToTriangles(int posicion) {
	return hashTableTriangles_[posicion];
}

float SpaceHash::goToTimestamp(int posicion) {
	return hashTableTimestamp_[posicion];
}

vec3i SpaceHash::obtainQuadrant(vec3f v) {
	int iX = floor(((v.x) / boxLenght_.x));
	int iY = floor(((v.y) / boxLenght_.y));
	int iZ = floor(((v.z) / boxLenght_.z));

	return vec3i(iX, iY, iZ);
};


int SpaceHash::ijk_To_Array(vec3i v) {	
	int p1 = 73856093;
	int p2 = 19349663;
	int p3 = 83492791;
	
	int n = hashSize_;

	return ((((v.x*p1)^(v.y*p2)^(v.z*p3)) % n)+n)%n;


}