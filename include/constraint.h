#pragma once

#include "auxTypes.h"
#include "constraintFunctions.h"


namespace mySim {

	class MyConstraint {
	public:
		
		MyConstraint(int cardinality, ConstraintFunction * funcion, VertexArray * vertices,
			float stifness, ConstraintType constraintType);
		virtual ~MyConstraint();

		float evaluateF(std::vector<vec3f> argument);//old
		float evaluateF();
		
		bool applyBool(std::vector<vec3f> argument);//old
		bool applyBool();

		void applyConstraintFull(unsigned int solverIterations);

		void DELETEV();



	private:
		unsigned int cardinality_;
		ConstraintFunction * funcion_;
		VertexArray * vertices_;
		float stifness_;
		ConstraintType constraintType_;
	};


	//Constraint especial para aristas. Se configura con solo indicarle los índices sobre los que debe actuar y la fuerza (opcionalmente, la distancia);
	class AristaConstraint : public MyConstraint {
	public:
		AristaConstraint(Arista * arista, float stifness);
		AristaConstraint(Arista * arista, float stifness, float distance);
		virtual ~AristaConstraint();
	};


	//Constraint para bending
	class BendingConstraint : public MyConstraint {
	public:
		BendingConstraint(DobleTriangulo * dobleTriangulo, float stifness);
		BendingConstraint(DobleTriangulo * dobleTriangulo, float stifness, float angle);
		virtual ~BendingConstraint();
	};


	class CollisionVTConstraint : public MyConstraint {
	public:
		CollisionVTConstraint(CollisionVT * collisionVT, float stifness);
		CollisionVTConstraint(CollisionVT * collisionVT, float stifness, float distance, CollisionType collisionType);
		virtual ~CollisionVTConstraint();

	};


	class BalloonConstraint : public MyConstraint {
	public:
		BalloonConstraint(VertexArray * verticesBalloon, std::vector<Triangulo*> & triangulos,float volumen, float presion);
	
		virtual ~BalloonConstraint() {}

	private:
		std::vector<Triangulo *> * triangulos;

	};

}

