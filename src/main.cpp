/****************************************************************************

 Copyright (C) 2002-2014 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.7.0.

 http://www.libqglviewer.com - contact@libqglviewer.com

 This file may be used under the terms of the GNU General Public License 
 versions 2.0 or 3.0 as published by the Free Software Foundation and
 appearing in the LICENSE file included in the packaging of this file.
 In addition, as a special exception, Gilles Debunne gives you certain 
 additional rights, described in the file GPL_EXCEPTION in this package.

 libQGLViewer uses dual licensing. Commercial/proprietary software must
 purchase a libQGLViewer Commercial License.

 This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/

#ifdef _MSC_VER
#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")
#endif




#include "include/demos.h"

int main(int argc, char **argv) {
	
	
	float clothThickness0 = 0.01f;
	float stifnessStretching = 0.9f;
	float stifnessBending = 0.5f;
	mySim::DemoChoqueTelas * D1 = new mySim::DemoChoqueTelas(argc, argv, stifnessStretching, stifnessBending, clothThickness0);
	D1->run();
	delete D1;
	
	float clothThicknessDB = 0.1f;
	float stifnessStretchingDB = 1.0f;
	float stifnessBendingDB = 0.0f;
	mySim::DemoBalloon * DB = new mySim::DemoBalloon(argc, argv, stifnessStretchingDB, stifnessBendingDB, clothThicknessDB);
	DB->run();
	delete DB;

	


	float clothThickness01 = 0.05f;
	float stifnessStretching1 = 0.9f;
	float stifnessBending1 = 0.1f;
	mySim::DemoChoqueTelas2 * D11 = new mySim::DemoChoqueTelas2(argc, argv, stifnessStretching1, stifnessBending1, clothThickness01);
	D11->run();
	delete D11;
	
	float clothThickness03 = 0.05f;
	float stifnessStretching01 = 0.1f;
	float stifnessBending01 = 0.05f;
	float stifnessStretching02 = 0.3f;
	float stifnessBending02 = 0.05f;
	float stifnessStretching03 = 1.0f;
	float stifnessBending03 = 0.05f;
	mySim::DemoChoqueTelas3 * D13 = new mySim::DemoChoqueTelas3(argc, argv, 
																stifnessStretching01, stifnessBending01, 
																stifnessStretching02, stifnessBending02, 
																stifnessStretching03, stifnessBending03, 
																clothThickness03);
	D13->run();
	delete D13;


	float clothThickness13 = 0.05f;
	float stifnessStretching11 = 0.5f;
	float stifnessBending11 = 0.05f;
	float stifnessStretching12 = 0.5f;
	float stifnessBending12 = 0.3f;
	float stifnessStretching13 = 0.5f;
	float stifnessBending13 = 1.0f;
	mySim::DemoChoqueTelas4 * D23 = new mySim::DemoChoqueTelas4(argc, argv,
		stifnessStretching11, stifnessBending11,
		stifnessStretching12, stifnessBending12,
		stifnessStretching13, stifnessBending13,
		clothThickness13);
	D23->run();
	delete D23;

	

	
	{
		float tolSide = 0.0f;
		float  clothThickness = 0.01f;
		mySim::DemoColisionConstraint1 * D2 = new mySim::DemoColisionConstraint1(argc, argv, tolSide, clothThickness);
		D2->run();
		delete D2;

	}

	
	{
		float tolSide2 = 0.0f;
		float  clothThickness2 = 0.01f;
		mySim::DemoColisionConstraint2 * D3 = new mySim::DemoColisionConstraint2(argc, argv, tolSide2, clothThickness2);
		D3->run();
		delete D3;
	}
	


	{
		float tolSide3 = 0.0f;
		float  clothThickness3 = 0.05f;
		mySim::DemoColisionConstraint3 * D4 = new mySim::DemoColisionConstraint3(argc, argv, tolSide3, clothThickness3);
		D4->run();
		delete D4;

	}

 
	/*float clothThickness0t = 0.1f;
	float stifnessStretchingt = 1.0f;
	float stifnessBendingt = 0.05f;
	mySim::DemoApilaTelas * D5 = new mySim::DemoApilaTelas(argc, argv, stifnessStretchingt, stifnessBendingt, clothThickness0t);
	D5->run();
	delete D5;*/

}
