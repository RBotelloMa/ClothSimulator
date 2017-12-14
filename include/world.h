#pragma once

#include "auxTypes.h"
#include "malla.h"
#include "externalForces.h"
#include "dampFunctions.h"

namespace mySim {

	class World
	{
	public:
		World();
		virtual ~World() {
			delete damp_;
			for (auto it = arrayExternalForces_.begin(); it != arrayExternalForces_.end(); it++) {
				delete(*it);
			}
			for (auto it = arrayMallas_.begin(); it != arrayMallas_.end(); it++) {
				delete(*it);
			}
		};

		void resetTime() { time_ = 0; }
		void increaseTime(float deltaT) { time_ += deltaT; }
		float getTime() { return time_; }

		void addMalla(Malla * malla);

		const std::vector<Malla *> & getArrayMallas() const; //Solo lectura
		std::vector<Malla *> & refArrayMallas(); // permite modificar. Usar con precaución.

		void addExternalForce(ExternalForces * f);
		std::vector<ExternalForces *> & getExternalForces();//solo lectura

		DampFunction * getDampFunction();
		void setDampFunction(DampFunction * damp);
		

	private:
		std::vector<unsigned int> arrayNVertices_;
		std::vector<Malla *> arrayMallas_;
		std::vector<ExternalForces *> arrayExternalForces_;
		
		DampFunction * damp_;
		float time_;
	};

}



