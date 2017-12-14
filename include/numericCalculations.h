#pragma once

#include "auxTypes.h"

namespace mySim {

	class numericCalc {
	public:
		static float polyEval3(float a3, float a2, float a1, float a0, float t) {
			return a3*t*t*t + a2*t*t + a1*t + a0;
		}

		static bool bisectionMethod3Grade(float a3, float a2, float a1, float a0, float t0, float t1, float tol, int nSteps, float & SOL);

		static bool bisectionVectorT(vec3f p, vec3f pv, vec3f o, vec3f ov, vec3f a, vec3f av, vec3f b, vec3f bv, float t0, float t1, int nSteps, float tol, float & SOL);

		static bool barridoTest(vec3f p, vec3f pv, vec3f o, vec3f ov, vec3f a, vec3f av, vec3f b, vec3f bv, float t0, float t1, int nSteps, float tol, float & SOL);

		static bool newtonTest(vec3f p, vec3f pv, vec3f o, vec3f ov, vec3f a, vec3f av, vec3f b, vec3f bv, float t0, float t1,float t, int nSteps, float tol, float & SOL);

		static bool  degreeThreeTest(float f3, float f2, float f1, float f0, float t0, float t1, int nMax, float tol, float & SOL);
		
	private:

		static bool  degreeThreeTestCont1(float f3, float f2, float f1, float f0, float t0, float t1, int nMax, float tol, float & SOL, bool s0, bool s1);
		static bool  degreeThreeTestCont2(float f3, float f2, float f1, float f0, float t0, float t1, int nMax, float tol, float & SOL, bool s0, bool s1);
	};

}