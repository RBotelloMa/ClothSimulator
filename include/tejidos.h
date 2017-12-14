#pragma once

#include "malla.h"
#define XY_TO_ARRAY(x,y,M)  (((M)*(x))+(y)) 

namespace mySim {

	class ExampleMalla : public MallaTriangular {
	public:
		
		virtual ~ExampleMalla() {}

		ExampleMalla(int size, float side, float Slenght, float mass ,vec3f lbCorner, vec3f inertia, float stifness) {
			const VertexArray * array = this->getVertexArray();
			float diag = sqrt(2.0f * Slenght*Slenght);
			for (int i = 0; i <= size; i++) {
				for (int j = 0; j <= size; j++) {
					mySim::Vertex * v = new mySim::Vertex(mass, lbCorner + vec3f(side*(float)j, side*(float)i, 0), inertia);
					this->addVertex(v);
				}
			}

			//array->v_[XY_TO_ARRAY(0, 0, (size + 1))]->setMass(100000000000000000000.0f);
			//array->v_[XY_TO_ARRAY(size, size, (size + 1))]->setMass(100000000000000000000.0f);
			//array->v_[XY_TO_ARRAY(0, size/2-1, (size + 1))]->setMass(100000000000000000000.0f);
			//array->v_[XY_TO_ARRAY(size, size/2-1, (size + 1))]->setMass(100000000000000000000.0f);
			
			/*for (int k = 0; k <= size; k++) {
				array->v_[XY_TO_ARRAY(k, size / 2, (size + 1))]->setMass(100000000000000000000.0f);
			}*/

			for (int i = 0; i <= size; i++) {
				for (int j = 0; j < size; j++) {
					//filas
					Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
					Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
					this->addArista(new Arista(v1, v2), stifness, Slenght);
				}
			}

			for (int j = 0; j <= size; j++) {
				for (int i = 0; i < size; i++) {
					//columnas
					Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size+1)];
					Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j, size+1)];
					this->addArista(new Arista(v1, v2), stifness, Slenght);
				}
			}

			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size; j++) {
					if (i % 2 == 0) {
						//diagonales
						if (j % 2 == 0) {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
						else {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
					}
					else {
						if (j % 2 == 0) {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
						else {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
					}
				}
			}
		}
	};

	class ExampleMalla2 : public ExampleMalla {
	public:

		virtual ~ExampleMalla2() {}

		ExampleMalla2(int size, float side, float Slenght, float mass,vec3f lbCorner, vec3f inertia,float stifness, float bendingStifness, float bendingAngle, float clothThickness) : ExampleMalla(size, side, Slenght, mass, lbCorner, inertia, stifness) {
			const VertexArray * arrayV = this->getVertexArray();
			setBendingStifness(bendingStifness);
			setBendingAngle(bendingAngle);
			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size; j++) {
					if (i % 2 == 0) {
						//diagonales
						if (j % 2 == 0) {
							
							Vertex * v1 = arrayV->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = arrayV->v_[XY_TO_ARRAY(i+1, j+1, size + 1)];
							Vertex * v3 = arrayV->v_[XY_TO_ARRAY(i, j+1, size + 1)];
							Vertex * v4 = arrayV->v_[XY_TO_ARRAY(i+1, j, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
						else {
							
							Vertex * v1 = arrayV->v_[XY_TO_ARRAY(i+1, j, size + 1)];
							Vertex * v2 = arrayV->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							Vertex * v3 = arrayV->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v4 = arrayV->v_[XY_TO_ARRAY(i + 1, j+1, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
					}
					else {
						if (j % 2 == 0) {
						
							Vertex * v1 = arrayV->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = arrayV->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							Vertex * v3 = arrayV->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v4 = arrayV->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
						else {
							
							Vertex * v1 = arrayV->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = arrayV->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							Vertex * v3 = arrayV->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							Vertex * v4 = arrayV->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
					}
				}
			}
			

		}
	};


	class MallaCuatroEsquinas : public MallaTriangular {
	public:
		virtual ~MallaCuatroEsquinas() {}

		MallaCuatroEsquinas(int size, float side, float Slenght, float mass, vec3f lbCorner, vec3f inertia, float stifness, float bendingStifness, float bendingAngle, float clothThickness) {
			const VertexArray * array = this->getVertexArray();
			float diag = sqrt(2.0f * Slenght*Slenght);
			for (int i = 0; i <= size; i++) {
				for (int j = 0; j <= size; j++) {
					mySim::Vertex * v = new mySim::Vertex(mass, lbCorner + vec3f(side*(float)j, side*(float)i, 0), inertia);
					this->addVertex(v);
				}
			}

			array->v_[XY_TO_ARRAY(0, 0, (size + 1))]->setEstatico(true);
			array->v_[XY_TO_ARRAY(size, size, (size + 1))]->setEstatico(true);
			array->v_[XY_TO_ARRAY(0, size, (size + 1))]->setEstatico(true);
			array->v_[XY_TO_ARRAY(size, 0, (size + 1))]->setEstatico(true);

			/*for (int k = 0; k <= size; k++) {
			array->v_[XY_TO_ARRAY(k, size / 2, (size + 1))]->setMass(100000000000000000000.0f);
			}*/

			for (int i = 0; i <= size; i++) {
				for (int j = 0; j < size; j++) {
					//filas
					Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
					Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
					this->addArista(new Arista(v1, v2), stifness, Slenght);
				}
			}

			for (int j = 0; j <= size; j++) {
				for (int i = 0; i < size; i++) {
					//columnas
					Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
					Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
					this->addArista(new Arista(v1, v2), stifness, Slenght);
				}
			}

			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size; j++) {
					if (i % 2 == 0) {
						//diagonales
						if (j % 2 == 0) {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
						else {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
					}
					else {
						if (j % 2 == 0) {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
						else {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
					}
				}
			}
		

		setBendingStifness(bendingStifness);
		setBendingAngle(bendingAngle);
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				if (i % 2 == 0) {
					//diagonales
					if (j % 2 == 0) {

						Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
						Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
						Vertex * v3 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
						Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
						this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
						this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
					}
					else {

						Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
						Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
						Vertex * v3 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
						Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
						this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
						this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
					}
				}
				else {
					if (j % 2 == 0) {

						Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
						Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
						Vertex * v3 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
						Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
						this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
						this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
					}
					else {

						Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
						Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
						Vertex * v3 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
						Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
						this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
						this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
					}
				}
			}
		}


	}
	};





	class MallaMitad : public MallaTriangular {
	public:
		virtual ~MallaMitad() {}

		MallaMitad(int size, float side, float Slenght, float mass, vec3f lbCorner, vec3f inertia, float stifness, float bendingStifness, float bendingAngle, float clothThickness) {
			const VertexArray * array = this->getVertexArray();
			float diag = sqrt(2.0f * Slenght*Slenght);
			for (int i = 0; i <= size; i++) {
				for (int j = 0; j <= size; j++) {
					mySim::Vertex * v = new mySim::Vertex(mass, lbCorner + vec3f(side*(float)j, side*(float)i, 0), inertia);
					this->addVertex(v);
				}
			}



			for (int k = 0; k <= size; k++) {
			array->v_[XY_TO_ARRAY(k, size / 2, (size + 1))]->setEstatico(true);
			}

			for (int k = 0; k <= size; k++) {
				array->v_[XY_TO_ARRAY(k, size, (size + 1))]->setEstatico(true);
			}

			for (int i = 0; i <= size; i++) {
				for (int j = 0; j < size; j++) {
					//filas
					Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
					Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
					this->addArista(new Arista(v1, v2), stifness, Slenght);
				}
			}

			for (int j = 0; j <= size; j++) {
				for (int i = 0; i < size; i++) {
					//columnas
					Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
					Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
					this->addArista(new Arista(v1, v2), stifness, Slenght);
				}
			}

			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size; j++) {
					if (i % 2 == 0) {
						//diagonales
						if (j % 2 == 0) {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
						else {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
					}
					else {
						if (j % 2 == 0) {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
						else {
							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addArista(new Arista(v1, v2), stifness, diag);
						}
					}
				}
			}


			setBendingStifness(bendingStifness);
			setBendingAngle(bendingAngle);
			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size; j++) {
					if (i % 2 == 0) {
						//diagonales
						if (j % 2 == 0) {

							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							Vertex * v3 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
						else {

							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							Vertex * v3 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
					}
					else {
						if (j % 2 == 0) {

							Vertex * v1 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							Vertex * v3 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
						else {

							Vertex * v1 = array->v_[XY_TO_ARRAY(i, j, size + 1)];
							Vertex * v2 = array->v_[XY_TO_ARRAY(i + 1, j + 1, size + 1)];
							Vertex * v3 = array->v_[XY_TO_ARRAY(i, j + 1, size + 1)];
							Vertex * v4 = array->v_[XY_TO_ARRAY(i + 1, j, size + 1)];
							this->addTriangulo(new Triangulo(v1, v2, v3, clothThickness));
							this->addTriangulo(new Triangulo(v1, v2, v4, clothThickness));
						}
					}
				}
			}


		}




	};


	class Balloon : public MallaTriangular {
	public:
		Balloon(vec3f center, float stretch, float bending, float angle, float size) : MallaTriangular() { 
			center_ = center; 
			stifness_ = stretch; 
			bend_ = bending; 
			size_ = size;
			icosaedro();
			//addBalloonConstraint();
		}
		float phi() { return 1 + sqrt(5) / 2; }

		void icosaedro() {
			setBendingStifness(0);
			setBendingAngle(0);

			float lenght = size_*lenght_;

			float speed = 1.5f;

			Vertex * v0 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, 1, phi()), vec3f(0.0f, 0.0f, speed));
			Vertex * v1 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, -1, phi()), vec3f(0.0f, 0.0f, speed));
			Vertex * v2 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, 1, -phi()), vec3f(0.0f, 0.0f, speed));
			Vertex * v3 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, -1, -phi()), vec3f(0.0f, 0.0f, speed));
			Vertex * v4 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, phi(), 0), vec3f(0.0f, 0.0f, speed));
			Vertex * v5 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, phi(), 0), vec3f(0.0f, 0.0f, speed));
			Vertex * v6 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, -phi(), 0), vec3f(0.0f, 0.0f, speed));
			Vertex * v7 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, -phi(), 0), vec3f(0.0f, 0.0f, speed));
			Vertex * v8 = new mySim::Vertex(1.0f, center_ + size_*vec3f(phi(), 0, 1), vec3f(0.0f, 0.0f, speed));
			Vertex * v9 = new mySim::Vertex(1.0f, center_ + size_*vec3f(phi(), 0, -1), vec3f(0.0f, 0.0f, speed));
			Vertex * v10 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-phi(), 0, 1), vec3f(0.0f, 0.0f, speed));
			Vertex * v11 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-phi(), 0, -1), vec3f(0.0f, 0.0f, speed));

			this->addVertex(v0);
			this->addVertex(v1);
			this->addVertex(v2);
			this->addVertex(v3);
			this->addVertex(v4);
			this->addVertex(v5);
			this->addVertex(v6);
			this->addVertex(v7);
			this->addVertex(v8);
			this->addVertex(v9);
			this->addVertex(v10);
			this->addVertex(v11);

			this->addArista(new Arista(v0, v1), stifness_, lenght);
			this->addArista(new Arista(v2, v3), stifness_, lenght);
			this->addArista(new Arista(v4, v5), stifness_, lenght);
			this->addArista(new Arista(v6, v7), stifness_, lenght);
			this->addArista(new Arista(v8, v9), stifness_, lenght);
			this->addArista(new Arista(v10, v11), stifness_, lenght);

			this->addArista(new Arista(v0, v4), stifness_, lenght);
			this->addArista(new Arista(v0, v5), stifness_, lenght);
			this->addArista(new Arista(v0, v8), stifness_, lenght);
			this->addArista(new Arista(v0, v10), stifness_, lenght);


			this->addArista(new Arista(v1, v6), stifness_, lenght);
			this->addArista(new Arista(v1, v7), stifness_, lenght);
			this->addArista(new Arista(v1, v8), stifness_, lenght);
			this->addArista(new Arista(v1, v10), stifness_, lenght);

			this->addArista(new Arista(v2, v4), stifness_, lenght);
			this->addArista(new Arista(v2, v5), stifness_, lenght);
			this->addArista(new Arista(v2, v9), stifness_, lenght);
			this->addArista(new Arista(v2, v11), stifness_, lenght);

			this->addArista(new Arista(v3, v6), stifness_, lenght);
			this->addArista(new Arista(v3, v7), stifness_, lenght);
			this->addArista(new Arista(v3, v9), stifness_, lenght);
			this->addArista(new Arista(v3, v11), stifness_, lenght);

			this->addArista(new Arista(v4, v8), stifness_, lenght);
			this->addArista(new Arista(v4, v9), stifness_, lenght);
			this->addArista(new Arista(v5, v10), stifness_, lenght);
			this->addArista(new Arista(v5, v11), stifness_, lenght);
			this->addArista(new Arista(v6, v8), stifness_, lenght);
			this->addArista(new Arista(v6, v9), stifness_, lenght);
			this->addArista(new Arista(v7, v10), stifness_, lenght);
			this->addArista(new Arista(v7, v11), stifness_, lenght);



			this->addTriangulo(new Triangulo(v0, v1, v8, 0.1f));
			this->addTriangulo(new Triangulo(v0, v1, v10, 0.1f));
			this->addTriangulo(new Triangulo(v0, v4, v5, 0.1f));
			this->addTriangulo(new Triangulo(v0, v4, v8, 0.1f));
			this->addTriangulo(new Triangulo(v0, v5, v10, 0.1f));

			this->addTriangulo(new Triangulo(v1, v6, v7, 0.1f));
			this->addTriangulo(new Triangulo(v1, v6, v8, 0.1f));
			this->addTriangulo(new Triangulo(v1, v7, v10, 0.1f));

			this->addTriangulo(new Triangulo(v2, v3, v9, 0.1f));
			this->addTriangulo(new Triangulo(v2, v3, v11, 0.1f));
			this->addTriangulo(new Triangulo(v2, v4, v5, 0.1f));
			this->addTriangulo(new Triangulo(v2, v4, v9, 0.1f));
			this->addTriangulo(new Triangulo(v2, v5, v11, 0.1f));
			
			this->addTriangulo(new Triangulo(v3, v6, v7, 0.1f));
			this->addTriangulo(new Triangulo(v3, v6, v9, 0.1f));
			this->addTriangulo(new Triangulo(v3, v7, v11, 0.1f));
			
			this->addTriangulo(new Triangulo(v4, v8, v9, 0.1f));
			this->addTriangulo(new Triangulo(v5, v10, v11, 0.1f));		
			this->addTriangulo(new Triangulo(v6, v8, v9, 0.1f));
			this->addTriangulo(new Triangulo(v7, v10, v11, 0.1f));		
		
		}

		void addBalloonConstraint() {
			BalloonConstraint * balloonC = new BalloonConstraint(this->getVertexArray2(), this->getTriangulos2(), 0, 0);
			addConstraint(balloonC);
		}


		void fijar(bool b) {
			for (auto it = this->getVertexArray()->v_.begin(); it != this->getVertexArray()->v_.end(); it++)
			{
				(*it)->setEstatico(b);
			}


		}

	private:
		vec3f center_;
		float bend_;
		float stifness_;
		float lenght_ = 2;
		float size_;


	};
	class Balloon2 : public MallaTriangular {
	public:
		Balloon2(vec3f center, float stretch, float bending, float angle, float size) : MallaTriangular() {
			center_ = center;
			stifness_ = stretch;
			bend_ = bending;
			size_ = size;
			
			caja();
			addBalloonConstraint();
		}
		

		void caja() {
			setBendingStifness(0);
			setBendingAngle(0);

			float speed = -10.0f;
			float speedx = -1.5f;
			float speedy = 1.5f;
			float sqr2 = sqrt(2);

			Vertex * v0 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, -1, -1), vec3f(speedx, speedy,speed));
			Vertex * v1 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, 0, -1), vec3f(speedx, speedy, speed));
			Vertex * v2 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, 1, -1), vec3f(speedx, speedy, speed));
			Vertex * v3 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, -1, -1), vec3f(speedx, speedy, speed));
			Vertex * v4 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, 0, -1.2), vec3f(speedx, speedy, speed));
			Vertex * v5 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, 1, -1), vec3f(speedx, speedy, speed));
			Vertex * v6 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, -1, -1), vec3f(speedx, speedy, speed));
			Vertex * v7= new mySim::Vertex(1.0f, center_ + size_*vec3f(1, 0, -1), vec3f(speedx, speedy, speed));
			Vertex * v8 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, 1, -1), vec3f(speedx, speedy, speed));

			Vertex * v10 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, -1, 0), vec3f(speedx, speedy, speed));
			Vertex * v11 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1.2, 0, 0), vec3f(speedx, speedy, speed));
			Vertex * v12 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, 1, 0), vec3f(speedx, speedy, speed));
			Vertex * v13 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-0, -1.2, 0), vec3f(speedx, speedy, speed));
			Vertex * v15 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, 1.2, 0), vec3f(speedx, speedy, speed));
			Vertex * v16 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, -1, 0), vec3f(speedx, speedy, speed));
			Vertex * v17 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1.2, 0, 0), vec3f(speedx, speedy, speed));
			Vertex * v18 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, 1, 0), vec3f(speedx, speedy, speed));
			
			Vertex * v20 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, -1, 1), vec3f(speedx, speedy, speed));
			Vertex * v21 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, 0, 1), vec3f(speedx, speedy, speed));
			Vertex * v22 = new mySim::Vertex(1.0f, center_ + size_*vec3f(-1, 1, 1), vec3f(speedx, speedy, speed));
			Vertex * v23 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, -1, 1), vec3f(speedx, speedy, speed));
			Vertex * v24 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, 0, 1.2), vec3f(speedx, speedy, speed));
			Vertex * v25 = new mySim::Vertex(1.0f, center_ + size_*vec3f(0, 1, 1), vec3f(speedx, speedy, speed));
			Vertex * v26 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, -1, 1), vec3f(speedx, speedy, speed));
			Vertex * v27 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, 0, 1), vec3f(speedx, speedy, speed));
			Vertex * v28 = new mySim::Vertex(1.0f, center_ + size_*vec3f(1, 1, 1), vec3f(speedx, speedy, speed));

			this->addVertex(v0);
			this->addVertex(v1);
			this->addVertex(v2);
			this->addVertex(v3);
			this->addVertex(v4);
			this->addVertex(v5);
			this->addVertex(v6);
			this->addVertex(v7);
			this->addVertex(v8);
			
			this->addVertex(v10);
			this->addVertex(v11);
			this->addVertex(v12);
			this->addVertex(v13);
			this->addVertex(v15);
			this->addVertex(v16);
			this->addVertex(v17);
			this->addVertex(v18);

			this->addVertex(v20);
			this->addVertex(v21);
			this->addVertex(v22);
			this->addVertex(v23);
			this->addVertex(v24);
			this->addVertex(v25);
			this->addVertex(v26);
			this->addVertex(v27);
			this->addVertex(v28);



			this->addArista(new Arista(v0, v1), stifness_, 1*size_);
			this->addArista(new Arista(v0, v3), stifness_, 1*size_);
			this->addArista(new Arista(v1, v2), stifness_, 1*size_);
			this->addArista(new Arista(v1, v4), stifness_, 1*size_);
			this->addArista(new Arista(v2, v5), stifness_, 1*size_);
			this->addArista(new Arista(v3, v4), stifness_, 1*size_);
			this->addArista(new Arista(v3, v6), stifness_, 1*size_);
			this->addArista(new Arista(v4, v5), stifness_, 1*size_);
			this->addArista(new Arista(v4, v7), stifness_, 1*size_);
			this->addArista(new Arista(v5, v8), stifness_, 1*size_);
			this->addArista(new Arista(v6, v7), stifness_, 1*size_);
			this->addArista(new Arista(v7, v8), stifness_, 1*size_);
			
			this->addArista(new Arista(v0, v4), stifness_, sqr2*size_);
			this->addArista(new Arista(v2, v4), stifness_, sqr2*size_);
			this->addArista(new Arista(v4, v6), stifness_, sqr2*size_);
			this->addArista(new Arista(v4, v8), stifness_, sqr2*size_);

			
			this->addArista(new Arista(v10, v11), stifness_, 1*size_);
			this->addArista(new Arista(v10, v13), stifness_, 1*size_);
			this->addArista(new Arista(v11, v12), stifness_, 1*size_);
			this->addArista(new Arista(v12, v15), stifness_, 1*size_);
			this->addArista(new Arista(v13, v16), stifness_, 1*size_);
			this->addArista(new Arista(v15, v18), stifness_, 1*size_);
			this->addArista(new Arista(v16, v17), stifness_, 1*size_);
			this->addArista(new Arista(v17, v18), stifness_, 1*size_);
			
			this->addArista(new Arista(v20, v21), stifness_, 1*size_);
			this->addArista(new Arista(v20, v23), stifness_, 1*size_);
			this->addArista(new Arista(v21, v22), stifness_, 1*size_);
			this->addArista(new Arista(v21, v24), stifness_, 1*size_);
			this->addArista(new Arista(v22, v25), stifness_, 1*size_);
			this->addArista(new Arista(v23, v24), stifness_, 1*size_);
			this->addArista(new Arista(v23, v26), stifness_, 1*size_);
			this->addArista(new Arista(v24, v25), stifness_, 1*size_);
			this->addArista(new Arista(v24, v27), stifness_, 1*size_);
			this->addArista(new Arista(v25, v28), stifness_, 1*size_);
			this->addArista(new Arista(v26, v27), stifness_, 1*size_);
			this->addArista(new Arista(v27, v28), stifness_, 1*size_);

			this->addArista(new Arista(v20, v24), stifness_, sqr2*size_);
			this->addArista(new Arista(v22, v24), stifness_, sqr2*size_);
			this->addArista(new Arista(v24, v26), stifness_, sqr2*size_);
			this->addArista(new Arista(v24, v28), stifness_, sqr2*size_);

			this->addArista(new Arista(v0, v10), stifness_, 1*size_);
			this->addArista(new Arista(v1, v11), stifness_, 1*size_);
			this->addArista(new Arista(v2, v12), stifness_, 1*size_);
			this->addArista(new Arista(v3, v13), stifness_, 1*size_);
			this->addArista(new Arista(v5, v15), stifness_, 1*size_);
			this->addArista(new Arista(v6, v16), stifness_, 1*size_);
			this->addArista(new Arista(v7, v17), stifness_, 1*size_);
			this->addArista(new Arista(v8, v18), stifness_, 1*size_);

			this->addArista(new Arista(v10, v20), stifness_, 1*size_);
			this->addArista(new Arista(v11, v21), stifness_, 1*size_);
			this->addArista(new Arista(v12, v22), stifness_, 1*size_);
			this->addArista(new Arista(v13, v23), stifness_, 1*size_);
			this->addArista(new Arista(v15, v25), stifness_, 1*size_);
			this->addArista(new Arista(v16, v26), stifness_, 1*size_);
			this->addArista(new Arista(v17, v27), stifness_, 1*size_);
			this->addArista(new Arista(v18, v28), stifness_, 1*size_);

			this->addArista(new Arista(v0, v13), stifness_, sqr2*size_);
			this->addArista(new Arista(v6, v13), stifness_, sqr2*size_);
			this->addArista(new Arista(v20, v13), stifness_, sqr2*size_);
			this->addArista(new Arista(v26, v13), stifness_, sqr2*size_);

			this->addArista(new Arista(v6, v17), stifness_, sqr2*size_);
			this->addArista(new Arista(v8, v17), stifness_, sqr2*size_);
			this->addArista(new Arista(v26, v17), stifness_, sqr2*size_);
			this->addArista(new Arista(v28, v17), stifness_, sqr2*size_);

			this->addArista(new Arista(v2, v15), stifness_, sqr2*size_);
			this->addArista(new Arista(v8, v15), stifness_, sqr2*size_);
			this->addArista(new Arista(v22, v15), stifness_, sqr2*size_);
			this->addArista(new Arista(v28, v15), stifness_, sqr2*size_);

			this->addArista(new Arista(v0, v11), stifness_, sqr2*size_);
			this->addArista(new Arista(v2, v11), stifness_, sqr2*size_);
			this->addArista(new Arista(v20, v11), stifness_, sqr2*size_);
			this->addArista(new Arista(v22, v11), stifness_, sqr2*size_);

			
			this->addTriangulo(new Triangulo(v0, v1, v4, 0.01f));
			this->addTriangulo(new Triangulo(v1, v2, v4, 0.01f));
			this->addTriangulo(new Triangulo(v2, v5, v4, 0.01f));
			this->addTriangulo(new Triangulo(v5, v8, v4, 0.01f));
			this->addTriangulo(new Triangulo(v8, v7, v4, 0.01f));
			this->addTriangulo(new Triangulo(v7, v6, v4, 0.01f));
			this->addTriangulo(new Triangulo(v6, v3, v4, 0.01f));
			this->addTriangulo(new Triangulo(v3, v0, v4, 0.01f));
			

			this->addTriangulo(new Triangulo(v0, v10, v11, 0.01f));
			this->addTriangulo(new Triangulo(v10, v20, v11, 0.01f));
			this->addTriangulo(new Triangulo(v20, v21, v11, 0.01f));
			this->addTriangulo(new Triangulo(v21, v22, v11, 0.01f));
			this->addTriangulo(new Triangulo(v22, v12, v11, 0.01f));
			this->addTriangulo(new Triangulo(v12, v2, v11, 0.01f));
			this->addTriangulo(new Triangulo(v2, v1, v11, 0.01f));
			this->addTriangulo(new Triangulo(v1, v0, v11, 0.01f));
			
			
			
			this->addTriangulo(new Triangulo(v0, v3, v13, 0.01f));
			this->addTriangulo(new Triangulo(v3, v6, v13, 0.01f));
			this->addTriangulo(new Triangulo(v6, v16, v13, 0.01f));
			this->addTriangulo(new Triangulo(v16, v26, v13, 0.01f));
			this->addTriangulo(new Triangulo(v26, v23, v13, 0.01f));
			this->addTriangulo(new Triangulo(v23, v20, v13, 0.01f));
			this->addTriangulo(new Triangulo(v20, v10, v13, 0.01f));
			this->addTriangulo(new Triangulo(v10, v0, v13, 0.01f));
			
			this->addTriangulo(new Triangulo(v2, v12, v15, 0.01f));
			this->addTriangulo(new Triangulo(v12, v22, v15, 0.01f));
			this->addTriangulo(new Triangulo(v22, v25, v15, 0.01f));
			this->addTriangulo(new Triangulo(v25, v28, v15, 0.01f));
			this->addTriangulo(new Triangulo(v28, v18, v15, 0.01f));
			this->addTriangulo(new Triangulo(v18, v8, v15, 0.01f));
			this->addTriangulo(new Triangulo(v8, v5, v15, 0.01f));
			this->addTriangulo(new Triangulo(v5, v2, v15, 0.01f));
			
			this->addTriangulo(new Triangulo(v6, v7, v17, 0.01f));
			this->addTriangulo(new Triangulo(v7, v8, v17, 0.01f));
			this->addTriangulo(new Triangulo(v8, v18, v17, 0.01f));
			this->addTriangulo(new Triangulo(v18, v28, v17, 0.01f));
			this->addTriangulo(new Triangulo(v28, v27, v17, 0.01f));
			this->addTriangulo(new Triangulo(v27, v26, v17, 0.01f));
			this->addTriangulo(new Triangulo(v26, v16, v17, 0.01f));
			this->addTriangulo(new Triangulo(v16, v6, v17, 0.01f));

			this->addTriangulo(new Triangulo(v20, v23, v24, 0.01f));
			this->addTriangulo(new Triangulo(v23, v26, v24, 0.01f));
			this->addTriangulo(new Triangulo(v26, v27, v24, 0.01f));
			this->addTriangulo(new Triangulo(v27, v28, v24, 0.01f));
			this->addTriangulo(new Triangulo(v28, v25, v24, 0.01f));
			this->addTriangulo(new Triangulo(v25, v22, v24, 0.01f));
			this->addTriangulo(new Triangulo(v22, v21, v24, 0.01f));
			this->addTriangulo(new Triangulo(v21, v20, v24, 0.01f));


		}

		void addBalloonConstraint() {
			BalloonConstraint * balloonC = new BalloonConstraint(this->getVertexArray2(), this->getTriangulos2(), size_*size_*size_*8*6, 3.5);
			addConstraint(balloonC);
		}


		void fijar(bool b) {
			for (auto it = this->getVertexArray()->v_.begin(); it != this->getVertexArray()->v_.end(); it++)
			{
				(*it)->setEstatico(b);
			}


		}

	private:
		vec3f center_;
		float bend_;
		float stifness_;
		float lenght_ = 2;
		float size_;


	};

}
