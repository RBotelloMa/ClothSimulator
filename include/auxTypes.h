#pragma once
#include <vector>;
#include <memory>

namespace mySim {

//struct vec3f
struct vec3f {
	vec3f(float i, float j, float k) {
		x = i;
		y = j;
		z = k;
	};
	vec3f() {};
	float norm() { return sqrt(x*x + y*y + z*z); }
	float norm2() { return x*x + y*y + z*z; }
	vec3f normalize() {
		float f = norm();
		float tol = 10e-16f;
		if (f >= tol) {
			x = x / f;
			y = y / f;
			z = z / f;
			return (*this);
		}
		else return (vec3f(0, 0, 0));
	}
	float x, y, z;
};

//operadores de vec3f
vec3f operator+(vec3f u, vec3f v);
vec3f operator-(vec3f u, vec3f v);
vec3f operator*(float f, vec3f u);
vec3f operator/(vec3f u, float f);

vec3f cross(vec3f u, vec3f v);
float dot(vec3f u, vec3f v);


//tipos de constraint
enum ConstraintType {
	EQUALITY,
	INEQUALITY
};

typedef unsigned int VERTEX_ID;


//función de generacoón auxiliar

std::vector<vec3f> vectorOfZerosVec3f(unsigned int N);

//vec3ui
struct vec3i {
	vec3i(int i, int j, int k) {
		x = i;
		y = j;
		z = k;
	};
	vec3i() {};
	
	vec3f tovec3f() {
		return vec3f(x, y, z);
	}

	int x, y, z;
};

vec3i operator+(vec3i v1, vec3i v2);




struct boundingBox {
	boundingBox() {};
	~boundingBox() {}
	boundingBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
		minPoint = vec3f(minX, minY, minZ);
		maxPoint = vec3f(maxX, maxY, maxZ);
	};
	
	vec3f minPoint;
	vec3f maxPoint;
};

enum CollisionType {
	//EXCLUDE_COLLISION=-1,
	NOT_COLLISION=0,
	POSITIVE=1,
	NEGATIVE=-1
};


}

