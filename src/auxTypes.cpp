#include "include/auxTypes.h"


using namespace mySim;


vec3f mySim::cross(vec3f u, vec3f v) {
	return vec3f(u.y*v.z - u.z*v.y, u.z*v.x - u.x*v.z, u.x*v.y - u.y*v.x);
}

float mySim::dot(vec3f u, vec3f v) {
	return (u.x*v.x + u.y*v.y + u.z*v.z);
};


vec3f mySim::operator+(vec3f u, vec3f v) {
	return vec3f(u.x + v.x, u.y + v.y, u.z + v.z);
}
vec3f mySim::operator-(vec3f u, vec3f v) {
	return vec3f(u.x - v.x, u.y - v.y, u.z - v.z);
}
vec3f mySim::operator*(float f, vec3f u) {
	return vec3f(f*u.x, f*u.y, f*u.z);
}
vec3f mySim::operator/(vec3f u, float f) {
	float s = 1.0f / f;
	return s*u;
}

std::vector<vec3f> mySim::vectorOfZerosVec3f(unsigned int N) {
	std::vector<vec3f> ret;
	for (int i=0; i<N; i++){
		ret.push_back(vec3f(0.0f, 0.0f, 0.0f));
	}
	return ret;
}



vec3i mySim::operator+(vec3i v1, vec3i v2) {
	return(vec3i(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z));
}