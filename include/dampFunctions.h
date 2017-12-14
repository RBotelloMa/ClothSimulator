#pragma once

#include "auxTypes.h"
#include "matriz.h"

//Clase de funciones que se utilizarán para amortiguar velocidad.
//Estas funciones alteran el valor del input (son procedimientos, tecnicamente ).


namespace mySim {
	class DampFunction {//clase abstracta
	public:
		DampFunction() {}
		virtual ~DampFunction() {}
		virtual void damp(const VertexArray * vArray, float kDamping) {};
	};

//clases derivadas que se utilizarán

	class MomentumDamp : public DampFunction {
	public:
		MomentumDamp() {}
		~MomentumDamp() {}

		virtual void damp(const VertexArray * vArray, float kDamping) {
			vec3f posCM = vec3f(0, 0, 0);
			vec3f velCM = vec3f(0, 0, 0);
			float sumM = 0;
			vec3f L = vec3f(0, 0, 0);
			matriz3f I(vec3f(0, 0, 0), vec3f(0, 0, 0), vec3f(0, 0, 0));


			for (auto it = vArray->v_.begin(); it != vArray->v_.end(); it++) {
				posCM = posCM + (*it)->getMass()*(*it)->getPosition();
				velCM = velCM + (*it)->getMass()*(*it)->getVelocity();
				sumM = sumM + (*it)->getMass();
			}
			posCM = posCM / sumM;
			velCM = velCM / sumM;

			for (auto it = vArray->v_.begin(); it != vArray->v_.end(); it++) {
				vec3f ri = (*it)->getPosition() - posCM;
				matriz3f gri = matriz3f::crossProductMatrix(ri);
				L = L + cross(ri, (*it)->getMass()*(*it)->getVelocity());
				I = I + (*it)->getMass()*gri*gri.transpuesta();
			}
			vec3f omega = I.inversa()*L;

			for (auto it = vArray->v_.begin(); it != vArray->v_.end(); it++) {
				vec3f ri = (*it)->getPosition() - posCM;
				vec3f deltaV = velCM + cross(omega, ri) - (*it)->getVelocity();
				(*it)->setVelocity((*it)->getVelocity() + kDamping*deltaV);
			}
		};
	};
}

