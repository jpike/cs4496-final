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

#include <cfloat>

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
	const int NUM_ITERATIONS = 1;

	int numFrames = UI->mData->mSelectedModel->mOpenedC3dFile->GetFrameCount();
	int frameCounter = 0;

	Matd constraintVector;	// hold C[i] values for a frame
	
	double objectiveFunction = DBL_MAX;	

	// max iteration controlled version of loop
	for (frameCounter = 0; (frameCounter < NUM_ITERATIONS && objectiveFunction > EPSILON && frameCounter < numFrames); frameCounter++)
	{
		// compute matrix of contraint vectors (1 xyz vector per row)
		IK_Solver::CreateConstraintVector(frameCounter, constraintVector);
		// compute objective function value using constraints
		objectiveFunction = IK_Solver::EvaluateObjectiveFunction(frameCounter, constraintVector);
		
		// jacobian computation
		IK_Solver::CreateJacobian(frameCounter);
	}

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
// Evaluates the entire objective function for a given frame.
// This has been verified for some simple cases.
// 
// Uses a pre-computed constraint vector to avoid having to compute
// these constraint values multiple times (as this pre-computed
// constraint vector can be utilized in other cases).
//----------------------------------------------------------------------
double IK_Solver::EvaluateObjectiveFunction(int frameNum, Matd & constraintVector)
{
#ifdef _DEBUG
	Logger::OpenLogFile("objective.txt");

	Logger::Print("Frame Num: ");
	Logger::PrintLine(frameNum);
#endif

	double value = 0.0;

	// loop over all handles on model, evaluating constraint
	MarkerList & modelHandles = UI->mData->mSelectedModel->mHandleList;

#ifdef _DEBUG
	Logger::Print("Number of Handles: ");
	Logger::PrintLine(modelHandles.size());
#endif

	for (int i = 0; i < modelHandles.size(); i++)
	{

#ifdef _DEBUG
		Logger::Print("\nHandle Num: ");
		Logger::Print(i);
#endif

		// get constraint from vector
		Vecd constraint = constraintVector[i];
		
		// convert raw constraint value to form for objective function sum
		double currentValue = sqrlen(constraint);

#ifdef _DEBUG
		Logger::Print("\tConstraint Vec: ");
		Logger::Print(constraint);
		Logger::Print("\tConstraint Value: ");
		Logger::PrintLine(currentValue);
#endif

		// add value to current sum
		value += currentValue;
	}

#ifdef _DEBUG
	Logger::Print("\nFinal Constraint Value: ");
	Logger::PrintLine(value);

	Logger::CloseLogFile("objective.txt");
#endif

	return value;
}

//----------------------------------------------------------------------
// Evaluates constraint for a given handle and target position.
// Has been verified with some simple test cases.
//----------------------------------------------------------------------
Vec3d IK_Solver::EvaluateConstraint(Marker * handle, Vec3d & constraintPos)
{
	// returns vector from desired position to current position
	// can be used to get distance between 2 positions
	return (handle->mGlobalPos - constraintPos);
}

//----------------------------------------------------------------------
// Populates the vector parameter with current constraint values
// for the current frame.  If no constraint should be added
// for a given marker handle, then a zero vector is filled in
// at the appropriate index.
//----------------------------------------------------------------------
void IK_Solver::CreateConstraintVector(int frameNum, Matd & constraintVector)
{
#ifdef _DEBUG
	Logger::OpenLogFile("constraint.txt");

	Logger::Print("Frame Num: ");
	Logger::PrintLine(frameNum);
#endif

	// loop over all handles on model, evaluating constraint
	MarkerList & modelHandles = UI->mData->mSelectedModel->mHandleList;

	// set size of matrix to hold values
	// 1 row per handle
	// 3 columns for x, y, z components
	constraintVector.SetSize(modelHandles.size(), 3);

#ifdef _DEBUG
	Logger::Print("Number of Handles: ");
	Logger::PrintLine(modelHandles.size());
#endif

	for (int i = 0; i < modelHandles.size(); i++)
	{
		// get handle on model and contraint position
		Marker * handle = modelHandles[i];
		Vec3d & constraintPos = UI->mData->mSelectedModel->mOpenedC3dFile->GetMarkerPos(frameNum, i);

#ifdef _DEBUG
		Logger::Print("\nHandle Num: ");
		Logger::Print(i);
		Logger::Print("\tHandle Pos: ");
		Logger::Print(handle->mGlobalPos);
		Logger::Print("\tConstraint Pos: ");
		Logger::Print(constraintPos);
#endif

		// if constraint is 0, 0, 0, then we don't have a constraint to actually deal with
		if (constraintPos == vl_zero)
		{
			constraintVector[i] = vl_zero;	// put zero for constraint value (no movement for marker)
#ifdef _DEBUG
		Logger::Print("\tConstraint Vec: ");
		Logger::PrintLine(vl_zero);
#endif
			continue;
		}

		// evaluate constraint function
		Vec3d constraint = IK_Solver::EvaluateConstraint(handle, constraintPos);

#ifdef _DEBUG
		Logger::Print("\tConstraint Vec: ");
		Logger::PrintLine(constraint);
#endif

		// add to constraint vector
		constraintVector[i] = constraint;
	}

#ifdef _DEBUG
	Logger::CloseLogFile("constraint.txt");
#endif

}

//----------------------------------------------------------------------
// Populates the matrix parameter with appropriate Jacobian
// values.
// TODO: NOT COMPLETE.  STILL A MAJOR WORK IN PROGRESS.
// SOME PORTIONS PROBABLY NOT CORRECT
//----------------------------------------------------------------------
void IK_Solver::CreateJacobian(int frameNum)
{
#ifdef _DEBUG
	Logger::OpenLogFile("jacobian.txt");

	Logger::Print("Frame Num: ");
	Logger::PrintLine(frameNum);
#endif

	// we need to compute derivatives for each handle
	MarkerList & modelHandles = UI->mData->mSelectedModel->mHandleList;

#ifdef _DEBUG
	Logger::Print("Number of Handles: ");
	Logger::PrintLine(modelHandles.size());
#endif

	for (int i = 0; i < modelHandles.size(); i++)
	{
#ifdef _DEBUG
		Logger::Print("\nHandle Num: ");
		Logger::Print(i);
#endif

		// get important related values (handle, parent, etc.)
		Marker * handle = modelHandles[i];
		TransformNode * node = UI->mData->mSelectedModel->mLimbs[handle->mNodeIndex];
		Mat4d parentTransform = node->mParentTransform;

#ifdef _DEBUG
		Logger::Print("\tParent Transform: ");
		Logger::PrintLine(parentTransform);
#endif

		// loop over all transforms for current node
		// create base transform (identity) as we go through transforms in this node
		Mat4d base = vl_I;	
		for (int j = 0; j < node->mTransforms.size(); j++)
		{
			Transform * transform = node->mTransforms[j];
			// DOF case - need to compute derivatives
			if (transform->IsDof())
			{
				// loop over all degress of freedom
				for (int k = 0; k < transform->GetDofCount(); k++)
				{
					Mat4d deriv = transform->GetDeriv(k);
					base = base * deriv;

#ifdef _DEBUG
					Logger::Print("\tDofDeriv: ");
					Logger::Print(deriv);
					Logger::Print("\tBase: ");
					Logger::PrintLine(base);
#endif
				}
			}
			// non-DOF case - need to add regular transform
			else
			{
				Mat4d mat = transform->GetTransform();
				base = base * mat;
#ifdef _DEBUG
					Logger::Print("\tNonDofMat: ");
					Logger::Print(mat);
					Logger::Print("\tBase: ");
					Logger::PrintLine(base);
#endif
			}
		}

#ifdef _DEBUG
		Logger::Print("Final Base: ");
		Logger::PrintLine(base);
#endif
	}

#ifdef _DEBUG
	Logger::CloseLogFile("jacobian.txt");
#endif
}