#include "include/demos.h"

using namespace mySim;

Demos::~Demos() {
	delete viewer_;
	delete application_;
	delete solver_;
}



DemoChoqueTelas::DemoChoqueTelas(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness) : Demos() {
	int hashSize = 100000;
	mySim::vec3f boxDim(0.1f, 0.1f, 0.1f);
	float tolAngle = 0.00f;
	float tolSide = 0.0f;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, 0));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	float bendingAngle = 3.14f;
	mySim::ExampleMalla2 * malla3 = new mySim::ExampleMalla2(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-0.0f, 0.0f, 1.0f), mySim::vec3f(0.0f, 0.0f, -2.5f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla3);

	mySim::ExampleMalla2 * malla4 = new mySim::ExampleMalla2(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-0.65f, -0.0f, -1.0f), mySim::vec3f(0.0f, 0.0f, +2.5f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla4);

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}

DemoChoqueTelas2::DemoChoqueTelas2(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness) : Demos() {
	int hashSize = 100000;
	mySim::vec3f boxDim(0.1f, 0.1f, 0.1f);
	float tolAngle = 0.00f;
	float tolSide = 0.0f;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, -9.8f));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	float bendingAngle = 3.14f;
	mySim::ExampleMalla2 * malla3 = new mySim::ExampleMalla2(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-0.0f, -0.5f, 1.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla3);

	auto point = malla3->getVertexArray()->v_.begin();
	(*point)->setEstatico(true);
	point = malla3->getVertexArray()->v_.end();
	point--;
	(*point)->setEstatico(true);

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}


DemoChoqueTelas3::DemoChoqueTelas3::DemoChoqueTelas3(int argc, char **argv,
													float stifnessStretching1, float bendingStifness1,
													float stifnessStretching2, float bendingStifness2,
													float stifnessStretching3, float bendingStifness3, float clothThickness) : Demos() {
	int hashSize = 100000;
	mySim::vec3f boxDim(0.05f, 0.05f, 0.05f);
	float tolAngle = 0.00f;
	float tolSide = 0.0f;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, -9.8f));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	float bendingAngle = 3.14f;
	mySim::MallaCuatroEsquinas * malla1 = new mySim::MallaCuatroEsquinas(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(1.5f, -0.5f, 1.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching1, bendingStifness1, bendingAngle, thick);
	world->addMalla(malla1);

	mySim::MallaCuatroEsquinas * malla2 = new mySim::MallaCuatroEsquinas(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(0.0f, -0.5f, 1.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching2, bendingStifness2, bendingAngle, thick);
	world->addMalla(malla2);

	mySim::MallaCuatroEsquinas * malla3 = new mySim::MallaCuatroEsquinas(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-1.5f, -0.5f, 1.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching3, bendingStifness3, bendingAngle, thick);
	world->addMalla(malla3);

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}


DemoChoqueTelas4::DemoChoqueTelas4(int argc, char **argv,
	float stifnessStretching1, float bendingStifness1,
	float stifnessStretching2, float bendingStifness2,
	float stifnessStretching3, float bendingStifness3, float clothThickness) : Demos() {
	int hashSize = 100000;
	mySim::vec3f boxDim(0.05f, 0.05f, 0.05f);
	float tolAngle = 0.00f;
	float tolSide = 0.0f;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, -9.8f));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	float bendingAngle = 3.14f;
	mySim::MallaMitad * malla1 = new mySim::MallaMitad(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(1.5f, -0.5f, 1.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching1, bendingStifness1, bendingAngle, thick);
	world->addMalla(malla1);

	mySim::MallaMitad * malla2 = new mySim::MallaMitad(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(0.0f, -0.5f, 1.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching2, bendingStifness2, bendingAngle, thick);
	world->addMalla(malla2);

	mySim::MallaMitad * malla3 = new mySim::MallaMitad(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-1.5f, -0.5f, 1.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching3, bendingStifness3, bendingAngle, thick);
	world->addMalla(malla3);

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}






DemoColisionConstraint1::DemoColisionConstraint1(int argc, char **argv, float tolSidef, float clothThickness) : Demos() {
	int hashSize = 100000;
	mySim::vec3f boxDim(0.1f, 0.1f, 0.1f);
	float tolAngle = 0.00f;
	float tolSide = tolSidef;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, -9.8f));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	///////////////////////////////////////////////////////////////////////////////////////////////

	mySim::MallaTriangular * malla = new mySim::MallaTriangular();
	mySim::VertexArray * vertices = new mySim::VertexArray();
	float Infinity = std::numeric_limits<float>::infinity();

	float diagL = sqrtf(1.25f);

	mySim::Vertex * v0 = new mySim::Vertex(10000.0f, mySim::vec3f(-0.5f, -0.5f, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), true);
	mySim::Vertex * v1 = new mySim::Vertex(10000.0f, mySim::vec3f(-0.5f, 0.5f, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), true);
	mySim::Vertex * v2 = new mySim::Vertex(10000.0f, mySim::vec3f(0.5f, 0.0f, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), true);

	vertices->v_.push_back(v0);
	vertices->v_.push_back(v1);
	vertices->v_.push_back(v2);

	float ang = 3.14f;
	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 20; j++) {
			float l = (float)i;
			l = l * 0.10f;
			float k = (float)j;
			k = k * 0.10f;
			mySim::MallaTriangular * mallaT = new mySim::MallaTriangular();
			mySim::VertexArray * verticesT = new mySim::VertexArray();
			mySim::Vertex * test = new mySim::Vertex(1.0f, mySim::vec3f(-1.0f + l, -1.0f + k, 1.5f), mySim::vec3f(0.0f, 0.0f, -5.0f));
			verticesT->v_.push_back(test);
			mallaT->setVertexArray(verticesT);
			world->addMalla(mallaT);
		}
	}

	mySim::Arista * arista0 = new mySim::Arista(v0, v1);
	mySim::Arista * arista1 = new mySim::Arista(v0, v2);
	mySim::Arista * arista2 = new mySim::Arista(v1, v2);

	mySim::Triangulo * triangulo1 = new mySim::Triangulo(v1, v0, v2, thick);

	malla->setVertexArray(vertices);

	malla->addArista(arista0, 1, 1.0f);
	malla->addArista(arista1, 1, diagL);
	malla->addArista(arista2, 1, diagL);

	malla->addTriangulo(triangulo1);

	world->addMalla(malla);

	//////////////////////////////////////////////////////////////////////////////////

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}

DemoColisionConstraint2::DemoColisionConstraint2(int argc, char **argv, float tolSidef, float clothThickness) {
	int hashSize = 100000;
	mySim::vec3f boxDim(0.1f, 0.1f, 0.1f);
	float tolAngle = 0.00f;
	float tolSide = tolSidef;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, 9.8f));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	///////////////////////////////////////////////////////////////////////////////////////////////

	mySim::MallaTriangular * malla = new mySim::MallaTriangular();
	mySim::VertexArray * vertices = new mySim::VertexArray();
	float Infinity = std::numeric_limits<float>::infinity();

	float diagL = sqrtf(1.25f);

	mySim::Vertex * v0 = new mySim::Vertex(1.0f, mySim::vec3f(-0.5f, -0.5f, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f));
	mySim::Vertex * v1 = new mySim::Vertex(1.0f, mySim::vec3f(-0.5f, 0.5f, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f));
	mySim::Vertex * v2 = new mySim::Vertex(1.0f, mySim::vec3f(0.5f, 0.0f, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f));

	vertices->v_.push_back(v0);
	vertices->v_.push_back(v1);
	vertices->v_.push_back(v2);

	float ang = 3.14f;
	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 20; j++) {
			float l = (float)i;
			l = l * 0.10f;
			float k = (float)j;
			k = k * 0.10f;
			mySim::MallaTriangular * mallaT = new mySim::MallaTriangular();
			mySim::VertexArray * verticesT = new mySim::VertexArray();
			mySim::Vertex * test = new mySim::Vertex(1.0, mySim::vec3f(-1.0f + l, -1.0f + k, 0.5f), -1 * mySim::vec3f(0.0f, 0.0f, 0.0f), true);
			verticesT->v_.push_back(test);
			mallaT->setVertexArray(verticesT);
			world->addMalla(mallaT);
		}
	}

	mySim::Arista * arista0 = new mySim::Arista(v0, v1);
	mySim::Arista * arista1 = new mySim::Arista(v0, v2);
	mySim::Arista * arista2 = new mySim::Arista(v1, v2);

	mySim::Triangulo * triangulo1 = new mySim::Triangulo(v1, v0, v2, thick);

	malla->setVertexArray(vertices);

	malla->addArista(arista0, 1, 1.0f);
	malla->addArista(arista1, 1, diagL);
	malla->addArista(arista2, 1, diagL);

	malla->addTriangulo(triangulo1);

	world->addMalla(malla);

	//////////////////////////////////////////////////////////////////////////////////

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}

DemoColisionConstraint3::DemoColisionConstraint3(int argc, char **argv, float tolSidef, float clothThickness) {
	int hashSize = 10000;
	mySim::vec3f boxDim(0.1f, 0.1f, 0.1f);
	float tolAngle = 0.00f;
	float tolSide = tolSidef;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, 0));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	///////////////////////////////////////////////////////////////////////////////////////////////

	mySim::MallaTriangular * malla = new mySim::MallaTriangular();
	mySim::VertexArray * vertices = new mySim::VertexArray();
	float Infinity = std::numeric_limits<float>::infinity();

	float diagL = sqrtf(1.25f);

	mySim::Vertex * v0 = new mySim::Vertex(1000, mySim::vec3f(0.5f, -0.5f, -10.5f), mySim::vec3f(0.0f, 0.0f, 100.0f));
	mySim::Vertex * v1 = new mySim::Vertex(1000, mySim::vec3f(0.5f, 0.5f, -10.5f), mySim::vec3f(0.0f, 0.0f, 100.0f));
	mySim::Vertex * v2 = new mySim::Vertex(1000, mySim::vec3f(-0.5f, 0.0f, -10.5f), mySim::vec3f(0.0f, 0.0f, 100.0f));

	vertices->v_.push_back(v0);
	vertices->v_.push_back(v1);
	vertices->v_.push_back(v2);

	float ang = 3.14f;
	for (int I = 0; I < 10; I++) {
		for (int i = 0; i < 10; i++) {
			for (int j = 0; j < 10; j++) {
				float l = (float)i;
				l = l * 0.2f;
				float k = (float)j;
				k = k * 0.2f;
				float m = (float)I;
				m = m*0.2f;
				mySim::MallaTriangular * mallaT = new mySim::MallaTriangular();
				mySim::VertexArray * verticesT = new mySim::VertexArray();
				mySim::Vertex * test = new mySim::Vertex(1.0f, mySim::vec3f(-1.0f + l, -1.0f + k, 0.5f + m), mySim::vec3f(0, 0, 0));
				verticesT->v_.push_back(test);
				mallaT->setVertexArray(verticesT);
				world->addMalla(mallaT);
			}
		}
	}

	mySim::Arista * arista0 = new mySim::Arista(v0, v1);
	mySim::Arista * arista1 = new mySim::Arista(v0, v2);
	mySim::Arista * arista2 = new mySim::Arista(v1, v2);

	mySim::Triangulo * triangulo1 = new mySim::Triangulo(v0, v1, v2, thick);

	malla->setVertexArray(vertices);

	malla->addArista(arista0, 1, 1.0f);
	malla->addArista(arista1, 1, diagL);
	malla->addArista(arista2, 1, diagL);

	malla->addTriangulo(triangulo1);

	world->addMalla(malla);

	//////////////////////////////////////////////////////////////////////////////////

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}



DemoApilaTelas::DemoApilaTelas(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness) : Demos() {
	int hashSize = 10000;
	mySim::vec3f boxDim(0.1f, 0.1f, 0.1f);
	float tolAngle = 0.00f;
	float tolSide = 0.0f;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0, -9.8f));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	float bendingAngle = 3.14f;
	mySim::ExampleMalla2 * malla3 = new mySim::ExampleMalla2(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-0.25f, -0.25f, 3.0f), mySim::vec3f(0.0f, 0.0f, -1.3f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla3);

	mySim::ExampleMalla2 * malla4 = new mySim::ExampleMalla2(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-0.5f, -0.5f, 2.0f), mySim::vec3f(0.0f, 0.0f, +1.2f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla4);

	/*mySim::ExampleMalla2 * malla5 = new mySim::ExampleMalla2(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-0.5f, -0.5f, 0.2f), mySim::vec3f(0.0f, 0.0f, -0.1f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla5);*/

	mySim::ExampleMalla2 * malla6 = new mySim::ExampleMalla2(10, 0.1f, 0.1f, 1.0f, mySim::vec3f(-0.25f, -0.25f, 5.1f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla6);

	mySim::ExampleMalla2 * malla7 = new mySim::ExampleMalla2(10, 0.2f, 0.2f, 1.0f, mySim::vec3f(-1.00f, -1.00f, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching, bendingStifness, bendingAngle, thick);
	world->addMalla(malla7);

	malla7->setEstatico();

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}

DemoBalloon::DemoBalloon(int argc, char **argv, float stifnessStretching, float bendingStifness, float clothThickness) : Demos() {
	int hashSize = 100000;
	mySim::vec3f boxDim(0.1f, 0.1f, 0.1f);
	float tolAngle = 0.00f;
	float tolSide = 0.0f;
	float tolMove = 0.000001f;
	float stifness = 1.0f;
	float thick = clothThickness;

	mySim::World * world = new mySim::World();
	mySim::SpaceHash * hash = new mySim::SpaceHash(hashSize, boxDim);
	mySim::CollisionEngine * collEngine = new mySim::CollisionEngine(hash, tolAngle, tolSide, tolMove, stifness);
	mySim::GlobalForce * gravity = new mySim::GlobalForce(vec3f(0, 0,-9.8));
	mySim::MomentumDamp * damp = new mySim::MomentumDamp();

	world->addExternalForce(gravity);
	world->setDampFunction(damp);
	world->resetTime();

	setSolver(new Solver());
	getSolver()->setWorld(world);
	getSolver()->setCollisionEngine(collEngine);

	float bendingAngle = 3.14f;
	mySim::Balloon2 * mallab = new mySim::Balloon2(vec3f(-1+0.3,-1+0.3,2), 0.2,0,0,0.1);
	mallab->fijar(false);
	world->addMalla(mallab);

	mySim::MallaCuatroEsquinas * malla3 = new mySim::MallaCuatroEsquinas(1, 4.0f, 4.0f, 1.0f, mySim::vec3f(-1+-2.8f,-1+0, 0.0f), mySim::vec3f(0.0f, 0.0f, 0.0f), stifnessStretching, bendingStifness, bendingAngle, thick);
	malla3->setEstatico();
	world->addMalla(malla3);

	setQApplication(new QApplication(argc, argv));
	setViewer(new Viewer());
}