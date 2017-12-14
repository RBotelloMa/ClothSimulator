#pragma once

#include "auxTypes.h"

namespace mySim {

	//Clase padre de todas las fuerzas externas.
	class ExternalForces {
	public:
		ExternalForces() {  }
		virtual ~ExternalForces() {  }
		virtual vec3f eval(vec3f position) { return vec3f(0.0f, 0.0f, 0.0f); };
	};

	//Clases derivadas de external forces
	class GlobalForce : public ExternalForces {
	public:
		GlobalForce(vec3f F) { force_ = F; }
		~GlobalForce() {}
		virtual vec3f eval(vec3f position) { return force_; };

	private:
		vec3f force_;
	};
}



