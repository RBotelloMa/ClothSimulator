#pragma once

#include "auxTypes.h"

namespace mySim {
	//Clase para los vétices de las mallas.
class Vertex {
	public:
		unsigned int getId() { return id_; };
		void setId(unsigned int id) { id_ = id; }

		float getMass();
		void setMass(float mass);

		float getWeight();

		const vec3f getPosition() const;
		void setPosition(vec3f position);

		const vec3f getPosition2() const;
		void setPosition2(vec3f position2);

		const vec3f getVelocity() const;
		void setVelocity(vec3f velocity);

		const vec3f getVelocity2() const { return velocity2_; }
		void setVelocity2(vec3f velocity) { velocity2_ = velocity; }


		Vertex(float mass, vec3f position, vec3f velocity);
		Vertex(float mass, vec3f position, vec3f velocity, bool statico);
		virtual ~Vertex();

		bool getEstatico() { return estatico_; }
		void setEstatico(bool estatico) { estatico_ = estatico; }

		void setCollision(bool collision) { colliding_ = collision; }
		bool getCollision() { return colliding_; }

		vec3f getPosition3() { return position3_; }
		void setPosition3() {
			position3_ = position2_;
		}

	protected:
		VERTEX_ID IdNext();

	private:
		VERTEX_ID id_;
		float mass_;
		vec3f position_;
		vec3f velocity_;
		vec3f velocity2_;
		float weight_;
		vec3f position2_;
		bool estatico_=false;
		bool colliding_ = false;
		vec3f position3_;//posición estimada antes de aplicar ninguna constraint
	};


	//Clase de vectores de vértices de este tipo con funcionalidad extra
class VertexArray {
	public:
		VertexArray();
		VertexArray(std::vector<Vertex *> & v);
		virtual ~VertexArray() {};

		void TrueDelete() {
			for (auto it = v_.begin(); it != v_.end(); it++) {
				delete (*it);
			}
		}

		//acceso y escritura transversal a posicion
		const std::vector<vec3f> getPositions() const;
		const void setPositions(std::vector<vec3f> positions) const;

		//acceso y escritura transversal a posicion2
		const std::vector<vec3f> getPositions2() const;
		const void setPositions2(std::vector<vec3f> positions2) const;

		//acceso y escritura transversal a speed
		const std::vector<vec3f> getSpeeds() const;
		const void setSpeeds(std::vector<vec3f> speeds) const;

		virtual boundingBox getBoundingBox();

		bool vertexIsHere(Vertex * vertex);

		//De momento dejaremos esta variable miembro como pública para conservar los métodos de std::vector; Las alternativas no aportan demasiado
		std::vector<Vertex *> v_;
	};

	//Arista
class Arista : public VertexArray {
	public:
		Arista() {}
		Arista(Vertex * v1, Vertex * v2) {
			v_.push_back(v1);
			v_.push_back(v2);
		}
	};

	//Triángulo
class Triangulo : public VertexArray {
	public:
		Triangulo() {};
		Triangulo(Vertex * v1, Vertex * v2, Vertex * v3, float clothThickness) {
			v_.push_back(v1);
			v_.push_back(v2);
			v_.push_back(v3);
			clothThickness_ = clothThickness;
		}
		virtual ~Triangulo() {}
		virtual boundingBox getBoundingBox() {
			vec3f v02 = v_[0]->getPosition2();
			vec3f v12 = v_[1]->getPosition2();
			vec3f v22 = v_[2]->getPosition2();

			vec3f n2 = cross(v12 - v02, v22 - v02).normalize();
			n2 = clothThickness_*n2;

			vec3f v01 = v_[0]->getPosition();
			vec3f v11 = v_[1]->getPosition();
			vec3f v21 = v_[2]->getPosition();

			vec3f n1 = cross(v11 - v01, v21 - v01).normalize();
			n1 = clothThickness_*n1;

			float minX, minY, minZ, maxX, maxY, maxZ;
			
			vec3f aux = v01 + n1;
			//comparaciones
			minX = maxX = aux.x;
			minY = maxY = aux.y;
			minZ = maxZ = aux.z;
			aux = v11 + n1;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v21 + n1;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v01 - n1;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v11 - n1;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v21 - n1;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;


			aux = v02 + n2;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v12 + n2;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v22 + n2;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v02 - n2;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v12 - n2;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			aux = v22 - n2;
			if (aux.x < minX) minX = aux.x;  if (aux.y < minY) minY = aux.y;  if (aux.z < minZ) minZ = aux.z;
			if (aux.x > maxX) maxX = aux.x;  if (aux.y > maxY) maxY = aux.y;  if (aux.z > maxZ) maxZ = aux.z;
			
		
			return boundingBox(minX, minY, minZ, maxX, maxY, maxZ);
		}
		float getClothThickness() { return clothThickness_; }

		bool checkCollID(Vertex * vertex) {
			bool b=false;
			for (auto it = collisionID_.begin(); it != collisionID_.end(); it++) {
				if (vertex->getId() == (*it)) b = true;
			}
			if (b) return true;
			else {
				collisionID_.push_back(vertex->getId());
				return false;
			}
		}

		void clearCollId() {
			collisionID_.clear();
		}

	private:
		float clothThickness_;
		std::vector<VERTEX_ID> collisionID_;
	};

	//DobleTriangulo
class DobleTriangulo : public VertexArray {
	public:
		DobleTriangulo() {};
		DobleTriangulo(Triangulo *T1, Triangulo * T2) {
			//Suponemos que check ok
			int count = 0;
			int a1, a2;
			int b1, b2;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					if (T1->v_[i]->getId() == T2->v_[j]->getId()) {
						if (count == 0) {
							a1 = i;
							b1 = j;
						}
						if (count == 1) {
							a2 = i;
							b2 = j;
						}
						count++;
					}
				}
			}
			//math trick
			v_.push_back(T1->v_[a1]);
			v_.push_back(T1->v_[a2]);
			v_.push_back(T1->v_[3-a1-a2]);
			v_.push_back(T2->v_[3 - b1 - b2]);
		}
		static bool DobleTCheck(Triangulo *T1, Triangulo * T2) {
			int count = 0;
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					if (T1->v_[i]->getId() == T2->v_[j]->getId()) {
						count++;
					}
				}
			}
			if (count == 2) return true;
			else return false;
		}
	};

class CollisionVT : public VertexArray {
	public:
		CollisionVT() {};
		CollisionVT(Vertex* vertex, Triangulo* triangulo) {
			v_.push_back(vertex);
			v_.push_back(triangulo->v_[0]);
			v_.push_back(triangulo->v_[1]);
			v_.push_back(triangulo->v_[2]);
			clothThickness_ = triangulo->getClothThickness();
		}
		float getClothThickness() { return clothThickness_; }
	private:
		float clothThickness_;

	};

}



