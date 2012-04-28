#ifndef __COMMAND_H__
#include "Command.h"
#endif //__COMMAND_H__

#ifndef __C3DFILEINFO_H__
#include "C3dFileInfo.h"
#endif	//__C3DFILEINFO_H__

#ifndef __ARTICULATEDBODY_H__
#include "ArticulatedBody.h"
#endif	//__ARTICULATEDBODY_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif

#ifndef __TRANSFORM_H__
#include "Transform.h"
#endif	//__TRANSFORM_H__

#include "Logger.h"
#include "Command.h"
#include "IKSolver.h"
#include <cfloat>



int readSkelFile( FILE* file, ArticulatedBody* skel );

extern RealTimeIKUI *UI;

void LoadModel(void *v)
{
  char *params = (char*)v;
  if(!params){
    params = (char*)fl_file_chooser("Open File?", "{*.skel}", "../src/skels" );
  }

  if(!params)
    return;

  FILE *file = fopen(params, "r");
    
  if(file == NULL){
    cout << "Skel file does not exist" << endl;
    return;
  }

  ArticulatedBody *mod = new ArticulatedBody();
  UI->mData->mModels.push_back(mod);
  UI->mData->mSelectedModel = mod;

  readSkelFile(file, mod);
  UI->CreateDofSliderWindow();

  mod->InitModel();
  UI->mGLWindow->mShowModel = true;
  UI->mShowModel_but->value(1);
  UI->mGLWindow->refresh();
  
  cout << "number of dofs in model: " << UI->mData->mModels[0]->GetDofCount() << endl;
}

void Solution(void *v)
{
    //cout << "TODO: Solve inverse kinematics problem" << endl;
    //bool test = UI->mData->mSelectedModel->mLimbs[0]->mTransforms[0]->IsDof();

	// setup some initial parameters for out solver
	const int NUM_ITERATIONS = 300;
	const int NUM_FRAMES = 50;
	const int PRINT_FREQUENCY = 30;//30;
	const int STEP_INCREASE_FREQUENCY = 20;
	const double STEP_INCREASE_FACTOR = 4.0;
	const double STEP_DECREASE_FACTOR = 2.0;
	const int EPSILON_INCREASE_FREQUENCY = 2;
	const double EPSILON_INCREASE_FACTOR = 2.0;
	double epsilon = 0.1;
	double stepSize = 0.01;

	// use different parameters if smaller number of handles
	if (UI->mData->mSelectedModel->GetHandleCount() < 10)
	{
		epsilon = 0.0001;
		stepSize = 0.5;
	}

	// setup solver
	IKSolver solver(epsilon, stepSize, NUM_ITERATIONS, NUM_FRAMES, PRINT_FREQUENCY, STEP_INCREASE_FREQUENCY,
					STEP_INCREASE_FACTOR, STEP_DECREASE_FACTOR, 
					EPSILON_INCREASE_FREQUENCY, EPSILON_INCREASE_FACTOR, UI->mData->mSelectedModel);
	solver.Initialize();

	// run solver
	solver.SolveLoop();
}

void Exit(void *v)
{
  exit(0);
}

void LoadC3d(void *v)
{
  if(!UI->mData->mSelectedModel){
    cout << "Load skeleton first";
    return;
  }
  char *params = (char*)v;
  if(!params){
    params = fl_file_chooser("Open File?", "{*.c3d}", "mocap/" );
  }

  if(!params)
    return;
  
  char *c3dFilename = new char[80];
  
  // load single c3d file
 
  C3dFileInfo *openFile = new C3dFileInfo(params);
  openFile->LoadFile();
  UI->mData->mSelectedModel->mOpenedC3dFile = openFile;
  cout << "number of frames in c3d: " << openFile->GetFrameCount() << endl;

  UI->InitControlPanel();
  UI->mGLWindow->mShowConstraints = true;
  UI->mShowConstr_but->value(1);
}

