#pragma once

#include "auxTypes.h"
#include "vertex.h"
#include "constraint.h"



namespace mySim {
	class Malla {
		//Estudiar como cargar mallas de manera sencilla. De momento, vértice a vértice;
	public:
		Malla();
		virtual ~Malla() {
			vertexArray_->TrueDelete();
			delete(vertexArray_);
			for (auto it = constraintArray_.begin(); it != constraintArray_.end(); it++) {
				delete (*it);
			}
		};

		unsigned int getNVertices();

		const VertexArray * getVertexArray() const;
		VertexArray *getVertexArray2();

		void setVertexArray(VertexArray * vertexArray);
		const VertexArray * refVertexArray() const;

		const std::vector<MyConstraint *> & getConstraints() const;

		void addVertex(Vertex * vertex);
		void addConstraint(MyConstraint * constraint);
		void applyAllConstraints(unsigned int solverIterations);

		void setEstatico() {
			for (auto it = vertexArray_->v_.begin(); it != vertexArray_->v_.end(); it++) {
				(*it)->setEstatico(true);
			}
		}

	private:
		VertexArray * vertexArray_;
		std::vector<MyConstraint *> constraintArray_;

	};

	//Clases derivadas
	class MallaTriangular : public Malla {
	public:
		MallaTriangular() : Malla() {}
		virtual ~MallaTriangular() {
			for (auto it = aristas_.begin(); it != aristas_.end(); it++){
				delete (*it);
			}
			for (auto it = triangulos_.begin(); it != triangulos_.end(); it++) {
				delete (*it);
			}
			for (auto it = dobleTriangulos_.begin(); it != dobleTriangulos_.end(); it++) {
				delete (*it);
			}
			
		}
		void addArista(Arista * arista, float stifness, float distance);
		void addTriangulo(Triangulo * triangulo);
		const std::vector<Arista *> & getAristas() const;
		const std::vector<Triangulo *> & getTriangulos() const;
		void setBendingStifness(float b) { bendingStifness_ = b; }
		void setBendingAngle(float b) { bendingAngle_ = b; }
		std::vector<Triangulo *>& getTriangulos2() { return triangulos_; }

	private:
		std::vector<Arista *> aristas_;
		std::vector<Triangulo *> triangulos_;
		std::vector<DobleTriangulo *> dobleTriangulos_;
		float bendingStifness_;
		float bendingAngle_ = 2.14f;
	};

	//Clases con constraint trian

	

}




