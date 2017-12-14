#pragma once

#include "auxTypes.h"
#include "vertex.h"

//Clase padre de todas las funciones de constraint que el solver podrá utilizar.
//Se puede estudiar incluir una clase derivada que admita funciones lambda/punteros a funciones.

namespace mySim {

	class ConstraintFunction {//clase abstracta
	public:
		ConstraintFunction();
		virtual ~ConstraintFunction() {};
		
		virtual float eval(std::vector<vec3f> & positions); //dudoso
		virtual float eval(VertexArray * vertexArray);

		virtual std::vector<vec3f> obtainCorrection(std::vector<vec3f> & positions, std::vector<float> & weigth); //dudoso
		virtual std::vector<vec3f> obtainCorrection(VertexArray * vertexArray);

		virtual void applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations);
	};



	//clases derivadas que se utilizarán
	class DistanceConstraintFunction : public ConstraintFunction {
	public:
		DistanceConstraintFunction();
		DistanceConstraintFunction(float distance);
		~DistanceConstraintFunction();
		virtual float eval(std::vector<vec3f> & positions);
		virtual float eval(VertexArray * vertexArray);
		
		virtual std::vector<vec3f> obtainCorrection(std::vector<vec3f> & positions, std::vector<float> & weight);
		virtual std::vector<vec3f> obtainCorrection(VertexArray * vertexArray);

		virtual void applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations);

		void setDistance(float distance);

	private: 
		float distance_;
	};

	class BendingConstraintFunction : public ConstraintFunction {
	public:
		BendingConstraintFunction();
		BendingConstraintFunction(float angle);
		~BendingConstraintFunction();

		virtual float eval(VertexArray * vertexArray);

		virtual std::vector<vec3f> obtainCorrection(VertexArray * vertexArray);

		virtual void applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations);

		void setAngle(float angle);

	private:
		float angle_;
	};

	class CollisionVTConstraintFunction : public ConstraintFunction {
	public:
		CollisionVTConstraintFunction();
		CollisionVTConstraintFunction(float distance, CollisionType type);
		~CollisionVTConstraintFunction();

		virtual float eval(VertexArray * vertexArray);
		//virtual std::vector<vec3f> obtainCorrection(VertexArray * vertexArray);
		virtual void applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations);

		
	private:
		CollisionType type_;
		float distance_;
	};


	class BalloonConstraintFunction : public ConstraintFunction {
	public:
		
		BalloonConstraintFunction(std::vector<Triangulo *> & triangulos, float volumen, float presion);

		virtual float eval(VertexArray * vertexArray);
		virtual void applyCorrectionFull(VertexArray * vertexArray, float stifness, unsigned int solverIterations);


	private:
		std::vector<Triangulo *> triangulos_;
		float volumen_;
		float presion_;
	};




}
