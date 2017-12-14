#pragma once

#include "auxTypes.h"
#include "vertex.h"

namespace mySim {

	class SpaceHash {
	public:
		SpaceHash(int hashSize, vec3f boxLenght);
		virtual ~SpaceHash();

		int getHashSize() { return hashSize_; }

		void addVertex(Vertex * vertex, float time, vec3i quadrant);
		void addMovingVertex(Vertex * vertex, float time);
		void addMovingVertex2(Vertex * vertex, float time);

		void addTriangle(Triangulo * triangulo, float time);

		VertexArray * goToPoints(int posicion);

		std::vector<Triangulo*> & goToTriangles(int posicion);

		float goToTimestamp(int posicion);

		vec3i obtainQuadrant(vec3f v);//TODO::cambiar nombre por obtener cuadrante


	private:
		int hashSize_;
		vec3f boxLenght_;
		std::vector<VertexArray *> hashTablePoints_;
		std::vector<std::vector<Triangulo*>> hashTableTriangles_;
		std::vector<float> hashTableTimestamp_;

		int ijk_To_Array(vec3i v);//función del hashPropiamente hablando

	};



}