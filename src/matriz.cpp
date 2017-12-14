#include "include/matriz.h"


using namespace mySim;

vec3f mySim::operator*(matriz3f M, vec3f v) {
	matriz3f Mt = M.transpuesta();
	return vec3f(dot(Mt.c1, v), dot(Mt.c2, v), dot(Mt.c3, v));
}

matriz3f mySim::operator*(float f, matriz3f M) {
	return matriz3f(f*M.c1, f*M.c2, f*M.c3);
}

matriz3f mySim::operator*(matriz3f A, matriz3f B) {
	return(matriz3f(A*B.c1, A*B.c2, A*B.c3));
}


matriz3f mySim::operator+(matriz3f M1, matriz3f M2) {
	return matriz3f(M1.c1 + M2.c1, M1.c2 + M2.c2, M1.c3 + M2.c3);
}

matriz3f  matriz3f::crossProductMatrix(vec3f a) {
	return matriz3f(vec3f(0, a.z, -a.y), vec3f(-a.z, 0, a.x), vec3f(a.y, -a.x, 0));
}