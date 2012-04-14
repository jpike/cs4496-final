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

//----------------------------------------------------------------------
// Typedefs
//----------------------------------------------------------------------
typedef std::vector<Marker*> MarkerList;


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
    cout << "TODO: Solve inverse kinematics problem" << endl;
    bool test = UI->mData->mSelectedModel->mLimbs[0]->mTransforms[0]->IsDof();

	const double EPSILON = 0.01;

	int numFrames = UI->mData->mSelectedModel->mOpenedC3dFile->GetFrameCount();
	int frameCounter = 0;

	double objectiveFunction = IK_Solver::EvaluateObjectiveFunction(frameCounter);	

	/* NOTE: Probably don't want to uncomment this section below until
	it is fully implemented due to possibility of infinite while loop 
	
	while (objectiveFunction > EPSILON && frameCounter < numFrames)
	{
		Vec3d direction = IK_Solver::ComputeMoveDirection();
		IK_Solver::UpdateState(direction);
		objectiveFunction = IK_Solver::EvaluateObjectiveFunction();
	}

	*/
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

//----------------------------------------------------------------------
// TODO: Finish implementing evaluation of objective function
// NOTE: THIS FUNCTION HAS NOT BEEN TESTED
//----------------------------------------------------------------------
double IK_Solver::EvaluateObjectiveFunction(int frameNum)
{
	double value = 0.0;

	// loop over all handles on model, evaluating constraint
	MarkerList & modelHandles = UI->mData->mSelectedModel->mHandleList;
	for (int i = 0; i < modelHandles.size(); i++)
	{
		// get handle on model and contraint position
		Marker * handle = modelHandles[i];
		Vec3d & constraintPos = UI->mData->mSelectedModel->mOpenedC3dFile->GetMarkerPos(frameNum, i);

		// if constraint is 0, 0, 0, then we don't have a constraint to actually deal with
		if (constraintPos == vl_zero)
		{
			continue;
		}

		// evaluate constraint function
		Vec3d constraint = IK_Solver::EvaluateConstraint(handle, constraintPos);
		
		// convert raw constraint value to form for objective function sum
		double currentValue = sqrlen(constraint);

		// add value to current sum
		value += currentValue;
	}

	return value;
}

//----------------------------------------------------------------------
// TODO: Finish implementing evaluation of constraint
// NOTE: THIS FUNCTION HAS NOT BEEN TESTED
//----------------------------------------------------------------------
Vec3d IK_Solver::EvaluateConstraint(Marker * handle, Vec3d & constraintPos)
{
	// returns vector from desired position to current position
	// can be used to get distance between 2 positions
	return (handle->mGlobalPos - constraintPos);
}