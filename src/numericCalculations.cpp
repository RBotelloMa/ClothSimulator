#include "include/numericCalculations.h"
#include <iostream>

using namespace mySim;

bool numericCalc::bisectionMethod3Grade(float a3, float a2, float a1, float a0, float t0, float t1, float tol, int nSteps, float & SOL) {
	if (nSteps == 0) return false;

	bool s0 = (polyEval3(a3, a2, a1, a0, t0) <= 0);
	bool s1 = (polyEval3(a3, a2, a1, a0, t1) <= 0);


	if (s0 == s1) {
		return false;
	}


	float tn = (t0 + t1) / 2.0f;

	float ptn = polyEval3(a3, a2, a1, a0, tn);

	if (ptn <= tol && -tol <= ptn) {
		SOL = tn;

		return true;
	}
	bool sn = (ptn <= 0);

	if (sn == s0) return bisectionMethod3Grade(a3, a2, a1, a0, tn, t1, tol, nSteps - 1, SOL);
	else return bisectionMethod3Grade(a3, a2, a1, a0, t0, tn, tol, nSteps - 1, SOL);
}





bool numericCalc::bisectionVectorT(vec3f p, vec3f pv, vec3f o, vec3f ov, vec3f a, vec3f av, vec3f b, vec3f bv, float t0, float t1, int nSteps, float tol, float & SOL) {
	//dot(a + t*av, b + t*bv);
	if (nSteps == 0) return false;
	vec3f n0 = cross(a + t0*av - o - t0*ov, b + t0*bv - o - t0*ov);
	vec3f n1 = cross(a + t1*av - o - t1*ov, b + t1*bv - o - t1*ov);

	float d0 = dot(n0, p + t0*pv - o - t0*ov);
	float d1 = dot(n1, p + t1*pv - o - t1*ov);

	if (-tol <= d0 && d0 <= tol) {
		SOL = t0;
		return true;
	}
	if (-tol <= d1 && d1 <= tol) {
		SOL = t1;
		return true;
	}

	bool s0 = (d0<=0);
	bool s1 = (d1<=0);

	if (s0 == s1) return false;

	float tn = (t0 + t1) / 2.0f;
	vec3f ntn = cross(a + tn*av, b + tn*bv - o - tn*ov);

	float dtn = dot(ntn, p + tn*pv - o - tn*ov);
	
	if (-tol <= dtn && dtn <= tol) {
		SOL = tn;
		return true;
	}

	bool sn = (dtn <= 0);

	if (sn == s0) {
		return bisectionVectorT(p,pv,o,ov,a, av, b, bv, tn, t1, nSteps, tol, SOL);
	}
	else {
		return bisectionVectorT(p,pv,o,ov,a, av, b, bv, t0, tn, nSteps, tol, SOL);
	}
}


bool numericCalc::barridoTest(vec3f p, vec3f pv, vec3f o, vec3f ov, vec3f a, vec3f av, vec3f b, vec3f bv, float t0, float t1, int nSteps, float tol, float & SOL) {
	
	float plusT = (t1 - t0) / nSteps;
	vec3f n0;
	float d0;
	float t;

	for (int i = 0; i < nSteps; i++) {
		t = t0 + i*plusT;
		n0 = cross(a + t*av - o - t*ov, b + t*bv - o - t*ov);
		d0 = dot(n0, p + t*pv - o - t*ov);

		if (d0 <= tol&&d0 >= -tol) {
			SOL = t0 + plusT*i;
			return true;
		}
	}

	return false;
}



bool numericCalc::newtonTest(vec3f p, vec3f pv, vec3f o, vec3f ov, vec3f a, vec3f av, vec3f b, vec3f bv, float t0, float t1, float t, int nSteps, float tol, float & SOL) {

	if (nSteps <= 0) return false;
	vec3f n0 = cross(a + t*av - o - t*ov, b + t*bv - o - t*ov);
	float f0 = dot(n0, p + t*pv - o - t*ov);

	if (f0 <= tol&&f0 >= -tol) {
		SOL = t;
		return true;
	}

	vec3f Dn0 = cross(av - ov, b + t*bv - o - t*ov) + cross(a + t*av - o - t*ov, b + t*bv - o - t*ov);
	float Df0 = dot(Dn0, p + t*pv - o - t*ov) + dot(n0, pv - ov);

	float nextT = t - f0 / Df0;

	return newtonTest(p, pv, o, ov, a, av, b, bv, t0, t1, nextT, nSteps-1, tol, SOL);
}


bool numericCalc::degreeThreeTest(float f3, float f2, float f1, float f0, float t0, float t1, int nMax, float tol,float & SOL) {
	
	float F0 = polyEval3(f3, f2, f1, f0, t0);
	float F1 = polyEval3(f3, f2, f1, f0, t1);


	if (F0<tol && F0>-tol) { 
		SOL = t0;
		return true; 
	}

	if (F1<tol && F1>-tol) {
		SOL = t1;
		return true;
	}

	bool s0 = (F0 < -tol);
	bool s1 = (F1 < -tol);
	
	float tn = 0.5f*(t0 + t1);
	float Fn = polyEval3(f3, f2, f1, f0, tn);
	bool sn = (Fn < -tol);

	if (Fn<tol && Fn>-tol) {
		SOL = tn;
		return true;
	}


	if (s0 != s1) {
		if (sn != s0) {	
			return degreeThreeTestCont1(f3, f2, f1, f0, t0, tn, nMax - 1, tol, SOL, s0, sn);
		}
		else {
			return degreeThreeTestCont1(f3, f2, f1, f0, tn, t1, nMax - 1, tol, SOL, sn, s1);
		}
	}
	else {
		float SOL1, SOL2;
		bool b1 = degreeThreeTestCont2(f3, f2, f1, f0, t0, tn, (nMax - 1) / 2, tol, SOL1, s0, sn);
		if (b1) {
			SOL = SOL1;
			return true;
		}
		bool b2 = degreeThreeTestCont2(f3, f2, f1, f0, tn, t1, (nMax - 1) / 2, tol, SOL2, sn, s1);
		if (b1) {
			SOL = SOL2;
			return true;
		}
		else return false;
	}
}


bool numericCalc::degreeThreeTestCont1(float f3, float f2, float f1, float f0, float t0, float t1, int nMax, float tol, float & SOL, bool s0, bool s1) {
	//if (nMax < 0) return false;//aqui quitar esto

	float tn = 0.5f*(t0 + t1);
	float Fn = polyEval3(f3, f2, f1, f0, tn);
	bool sn = (Fn < -tol);

	if (Fn<tol && Fn>-tol) {
		SOL = tn;
		return true;
	}
	if (s0 != s1) {
		if (sn != s0) {
			return degreeThreeTestCont1(f3, f2, f1, f0, t0, tn, nMax - 1, tol, SOL, s0, sn);
		}
		else {
			return degreeThreeTestCont1(f3, f2, f1, f0, tn, t1, nMax - 1, tol, SOL, sn, s1);
		}
	}
}

bool numericCalc::degreeThreeTestCont2(float f3, float f2, float f1, float f0, float t0, float t1, int nMax, float tol, float & SOL, bool s0, bool s1) {
	if (nMax <= 0) return false;
	
	float tn = 0.5f*(t0 + t1);
	float Fn = polyEval3(f3, f2, f1, f0, tn);
	bool sn = (Fn < -tol);

	if (Fn<tol && Fn>-tol) {
		SOL = tn;
		return true;
	}


	if (s0 != s1) {
		if (sn != s0) {
			return degreeThreeTestCont1(f3, f2, f1, f0, t0, tn, nMax - 1, tol, SOL, s0, sn);
		}
		else {
			return degreeThreeTestCont1(f3, f2, f1, f0, tn, t1, nMax - 1, tol, SOL, sn, s1);
		}
	}
	else {
		float SOL1, SOL2;
		bool b1 = degreeThreeTestCont2(f3, f2, f1, f0, t0, tn, (nMax - 1), tol, SOL1, s0, sn);
		if (b1) {
			SOL = SOL1;
			return true;
		}

		bool b2 = degreeThreeTestCont2(f3, f2, f1, f0, tn, t1, (nMax - 1), tol, SOL2, sn, s1);
		if (b1) {
			SOL = SOL2;
			return true;
		}
		else return false;
	}
}