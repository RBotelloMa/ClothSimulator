#pragma once
#include "animation.h"
#include <qapplication.h>
#include "solver.h"
#include <iostream>
#include "tejidos.h"

namespace mySim {
class Demos {
	public:
		Demos() {}
		virtual ~Demos();

		void setSolver(Solver * solver) {solver_ = solver;}
		Solver * getSolver() {return solver_;}

		void setQApplication(QApplication * application) {application_ = application;}
		QApplication * getQApplication() {return application_;}

		void setViewer(Viewer * viewer) {viewer_ = viewer;}
		Viewer * getViewer() {return viewer_;}

		virtual int run(){
			viewer_->setSolver(solver_);
			viewer_->setWindowTitle("animation");
			viewer_->show();
			return application_->exec();
		}

	private:
		Solver * solver_;
		QApplication * application_;
		Viewer * viewer_;
	};

class DemoChoqueTelas : public Demos {
	public:
		DemoChoqueTelas(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness);
};

class DemoBalloon : public Demos {
public:
	DemoBalloon::DemoBalloon(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness);
};


class DemoChoqueTelas2 : public Demos {
public:
	DemoChoqueTelas2::DemoChoqueTelas2(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness);
};


class DemoChoqueTelas3 : public Demos {
public:
	DemoChoqueTelas3::DemoChoqueTelas3(int argc, char **argv, 
		float stifnessStretching1, float bendingStifness1, 
		float stifnessStretching2, float bendingStifness2, 
		float stifnessStretching3, float bendingStifness3, float clothThickness);
};

class DemoChoqueTelas4 : public Demos {
public:
	DemoChoqueTelas4::DemoChoqueTelas4(int argc, char **argv,
		float stifnessStretching1, float bendingStifness1,
		float stifnessStretching2, float bendingStifness2,
		float stifnessStretching3, float bendingStifness3, float clothThickness);
};


class DemoColisionConstraint1 : public Demos {
public:
		DemoColisionConstraint1(int argc, char **argv, float tolSidef, float clothThickness); 
};

class DemoColisionConstraint2 : public Demos {
public:
		DemoColisionConstraint2(int argc, char **argv, float tolSidef, float clothThickness);
};

class DemoColisionConstraint3 : public Demos {
public:
		DemoColisionConstraint3(int argc, char **argv, float tolSidef, float clothThickness);
};

class DemoApilaTelas : public Demos {
public:
	DemoApilaTelas(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness);
};


}