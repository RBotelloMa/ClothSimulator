#pragma once
#include "auxTypes.h"

namespace mySim {
	struct matriz3f {
		matriz3f() {}
		matriz3f(vec3f col1, vec3f col2, vec3f col3) {
			c1 = col1;
			c2 = col2;
			c3 = col3;
		}
		float determinante() {
			return (
				(c1.x * c2.y * c3.z +
					c1.y * c2.z * c3.x +
					c1.z * c2.x * c3.y)
				- (c1.x * c2.z * c3.y +
					c1.y * c2.x * c3.z +
					c1.z * c2.y * c3.x)
				);
		}

		matriz3f adjunta() {
			vec3f col1(c2.y*c3.z - c2.z*c3.y, c2.z*c3.x - c2.x*c3.z, c2.x*c3.y - c2.y*c3.x);
			vec3f col2(c1.z*c3.y - c1.y*c3.z, c1.x*c3.z - c1.z*c3.x, c1.y*c3.x - c1.x*c3.y);
			vec3f col3(c1.y*c2.z - c1.z*c2.y, c1.z*c2.x - c1.x*c2.z, c1.x*c2.y - c1.y*c2.x);

			return matriz3f(col1, col2, col3);
		}

		void transponer() {
			float aux;
			aux = c1.y; c1.y = c2.x; c2.x = aux;
			aux = c1.z; c1.z = c3.x; c3.x = aux;
			aux = c2.z; c2.z = c3.y; c3.y = aux;
		}

		matriz3f transpuesta() {
			return matriz3f(vec3f(c1.x, c2.x, c3.x), vec3f(c1.y, c2.y, c3.y), vec3f(c1.z, c2.z, c3.z));
		}

		void multiplicarf(float f) {
			c1 = f*c1;
			c2 = f*c2;
			c3 = f*c3;
		}

		matriz3f inversa() {
			float d = determinante();
			if (d == 0) return (matriz3f(vec3f(0, 0, 0), vec3f(0, 0, 0), vec3f(0, 0, 0)));
			matriz3f ret = adjunta();
			ret.transponer();
			ret.multiplicarf(1.0f / d);
			return ret;
		}

		static matriz3f crossProductMatrix(vec3f a);


		vec3f c1;
		vec3f c2;
		vec3f c3;
	};

	vec3f operator*(matriz3f M, vec3f v);
	matriz3f operator*(float f, matriz3f M);
	matriz3f operator*(matriz3f A, matriz3f B);
	matriz3f operator+(matriz3f M1, matriz3f M2);

	
}


