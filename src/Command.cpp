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

//----------------------------------------------------------------------
// Typedefs
//----------------------------------------------------------------------
//typedef std::vector<Marker*> MarkerList;

//----------------------------------------------------------------------
// Static Variables
//----------------------------------------------------------------------
//DofIdToMarkerMap IK_Solver::dofIdToMarkerMap;

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

void NavigateModel(Model * selectedModel)
{
	std::ofstream modelFile("logs/model.txt");

	modelFile << "Dofs" << std::endl;
	std::vector<Dof *> dofList = selectedModel->mDofList.mDofs;
	for (int i = 0; i < dofList.size(); i++)
	{
		Dof * dof = dofList[i];
		modelFile << i << "\t" << dof << "\t" << dof->mId << "\t" << dof->GetName() << "\t" << dof->ReturnType() << "\t" 
					<< dof->GetTransformValue() << "\t" << dof->mVal << "\t" << dof->mLowerBound << "\t" << dof->mUpperBound << std::endl;
	}

	modelFile << "Handles" << std::endl;
	std::vector<Marker *> handleList = selectedModel->mHandleList;
	for (int i = 0; i < handleList.size(); i++)
	{
		Marker * marker = handleList[i];
		modelFile << i << "\t" << marker << "\t" << marker->mIndex << "\t" << marker->mMarkerOrder << "\t" << marker->mName << "\t" 
					<< marker->mNodeIndex << "\t" << marker->mOffset << "\t" << marker->mGlobalPos << std::endl;
	}

	modelFile << "Limbs" << std::endl;
	std::vector<TransformNode *> limbList = selectedModel->mLimbs;
	for (int i = 0; i < limbList.size(); i++)
	{
		TransformNode * node = limbList[i];

		modelFile << "Limb" << std::endl;
		modelFile << i << "\t" << node << "\t" << node->mIndex << "\t" << node->mName << std::endl;

		modelFile << "Node Handles" << std::endl;
		std::vector<Marker *> nodeHandles = node->mHandles;
		for (int j = 0; j < nodeHandles.size(); j++)
		{
			modelFile << j << "\t" << nodeHandles[i] << std::endl;
		}

		modelFile << "Node Transforms" << std::endl;
		std::vector<Transform *> transforms = node->mTransforms;
		for (int j = 0; j < transforms.size(); j++)
		{
			Transform * transform = transforms[j];
			modelFile << "Transform" << std::endl;
			modelFile << j << "\t" << transform << "\t" << transform->mIndex << "\t" << transform->GetIndex() << "\t" 
						<< transform->IsDof() << "\t" << transform->GetDofCount() << std::endl;

			if (!transform->IsDof())
				continue;

			modelFile << "Transform Dofs " << j << std::endl;
			for (int k = 0; k < transform->GetDofCount(); k++)
			{
				Dof * dof = transform->GetDof(k);
				modelFile << k << "\t" << dof << "\t" << dof->mId << "\t" << dof->GetName() << "\t" << dof->ReturnType() << "\t" 
					<< dof->GetTransformValue() << "\t" << dof->mVal << "\t" << dof->mLowerBound << "\t" << dof->mUpperBound << std::endl;
			}
		}
	}

	modelFile.close();
}

void Solution(void *v)
{
    cout << "TODO: Solve inverse kinematics problem" << endl;
    bool test = UI->mData->mSelectedModel->mLimbs[0]->mTransforms[0]->IsDof();

	const double EPSILON = 0.001;
	const double STEP_SIZE = 0.5;
	const int NUM_ITERATIONS = 3;

	IKSolver solver(EPSILON, STEP_SIZE, NUM_ITERATIONS, UI->mData->mSelectedModel);
	solver.Initialize();

	solver.SolveLoop();

	// NOTE: MOST STUFF BELOW IS MOSTLY OBSOLETE
	// Main stuff has been refactored to IKSolver class, which should
	// be more maintainable then the mess of code used in initial approach

	/*

	Model * selectedModel = UI->mData->mSelectedModel;
	NavigateModel(selectedModel);

	int numFrames = UI->mData->mSelectedModel->mOpenedC3dFile->GetFrameCount();
	int frameCounter = 0;

	Matd constraintVector;	// hold C[i] values for a frame
	
	double objectiveFunction = DBL_MAX;	

	std::vector<Mat4d> derivatives;

	// max iteration controlled version of loop
	for (frameCounter = 0; (frameCounter < NUM_ITERATIONS && objectiveFunction > EPSILON && frameCounter < numFrames); frameCounter++)
	{
		// Print DOFs
		IK_Solver::PrintDofs(frameCounter);

		// compute matrix of contraint vectors (1 xyz vector per row)
		IK_Solver::CreateConstraintVector(frameCounter, constraintVector);
		// compute objective function value using constraints
		objectiveFunction = IK_Solver::EvaluateObjectiveFunction(frameCounter, constraintVector);

		// compute derivatives for each transform matrix
		IK_Solver::CreateMatrixDerivatives(derivatives);
		
		// jacobian computation - commented out for now since doesn't work (not complete yet)
		//IK_Solver::CreateJacobian(frameCounter);
	}*/

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
	Logger::OpenLogFile("logs/objective.txt");

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

	Logger::CloseLogFile("logs/objective.txt");
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
	Logger::OpenLogFile("logs/constraint.txt");

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
	Logger::CloseLogFile("logs/constraint.txt");
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
	Logger::OpenLogFile("logs/jacobian.txt");

	Logger::Print("Frame Num: ");
	Logger::PrintLine(frameNum);
#endif

	std::map<int, Vec3d> jacobians;

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
		Vec4d localPos(handle->mOffset, 1.0);
		TransformNode * node = UI->mData->mSelectedModel->mLimbs[handle->mNodeIndex];
		Mat4d parentTransform = node->mParentTransform;

#ifdef _DEBUG
		Logger::PrintLine("\tParent Transform: ");
		Logger::PrintLine(parentTransform);
#endif

		// loop over all transforms for current node
		// create base transform (identity) as we go through transforms in this node
		for (int j = 0; j < node->mTransforms.size(); j++)
		{
			Transform * transform = node->mTransforms[j];
			// DOF case - need to compute derivatives
			if (transform->IsDof())
			{
				// loop over all degress of freedom
				for (int k = 0; k < transform->GetDofCount(); k++)
				{
					// calculate derivative
					Mat4d deriv = transform->GetDeriv(k);
					// get index into dof list
					int dofIndex = transform->GetDof(k)->mId;
					std::cout << dofIndex << "\t" << transform->GetDof(k)->GetTransformValue() << std::endl;
					// get matrix before this derivative
					Mat4d preMatrix = vl_I;
					for (int m = 0; m < j; m++)
					{
						preMatrix = preMatrix * node->mTransforms[m]->GetTransform();
					}
					// get matrix after this derivative
					Mat4d postMatrix = vl_I;
					for (int n = j+1; n < node->mTransforms.size(); n++)
					{
						postMatrix = postMatrix * node->mTransforms[n]->GetTransform();
					}
					// compose final value
					Mat4d finalMatrix = parentTransform * preMatrix * deriv * postMatrix;
					Vec4d jacobianValue = finalMatrix * localPos;
#ifdef _DEBUG
					Logger::Print("Dof: ");
					Logger::Print(dofIndex);
					Logger::Print("\tJacobianValue: ");
					Logger::PrintLine(jacobianValue);
#endif
				}
			}

		}


	}

#ifdef _DEBUG
	Logger::CloseLogFile("logs/jacobian.txt");
#endif
}

//----------------------------------------------------------------------
// Computes derivatives of each matrix
//----------------------------------------------------------------------
void IK_Solver::CreateMatrixDerivatives(std::vector<Mat4d> & derivatives)
{
#ifdef _DEBUG
	Logger::OpenLogFile("logs/derivatives.txt");
#endif

	// clear derivatives from past evaluation
	derivatives.clear();

	// we need to compute derivatives for each handle
	MarkerList & modelHandles = UI->mData->mSelectedModel->mHandleList;

#ifdef _DEBUG
	Logger::Print("Number of Handles: ");
	Logger::PrintLine(modelHandles.size());
#endif

	// loop over all handles, computing derivatives
	for (int i = 0; i < modelHandles.size(); i++)
	{
		// get handle and node
		Marker * handle = modelHandles[i];
		TransformNode * node = UI->mData->mSelectedModel->mLimbs[handle->mNodeIndex];

#ifdef _DEBUG
		Logger::Print("\nHandle Num: ");
		Logger::PrintLine(i);
#endif

		std::cout << "Handle Num: " << i << std::endl;

		// loop over all transforms
		for (int j = 0; j < node->mTransforms.size(); j++)
		{
			Transform * transform = node->mTransforms[j];

			std::cout << "Transform: " << transform->GetIndex() << " " << j << std::endl;

#ifdef _DEBUG
			Logger::PrintLine("\nOriginal Transform: ");
			Logger::PrintLine(transform->GetTransform());
#endif

			// dof case - compute derivative
			if (transform->IsDof())
			{
				// loop over all dofs
				for (int k = 0; k < transform->GetDofCount(); k++)
				{
					Dof * dof = transform->GetDof(k);
					//std::cout << "DofID: " << dof->mId << "DofValue: " << dof->mVal << "DofType: " << dof->ReturnType() << std::endl;

					Mat4d deriv = transform->GetDeriv(k);
					derivatives.push_back(deriv);

#ifdef _DEBUG
					Logger::Print("\nDofDeriv: ");
					Logger::PrintLine(k);
					Logger::PrintLine(deriv);
#endif
				}
			}
			// non-dof case - "derivative" should be all zeroes
			else
			{
				Mat4d deriv = vl_zero;
				derivatives.push_back(deriv);
#ifdef _DEBUG
				Logger::PrintLine("\nNonDofDeriv: ");
				Logger::PrintLine(deriv);
#endif
			}
		}
	}

#ifdef _DEBUG
	Logger::CloseLogFile("logs/derivatives.txt");
#endif
}

//----------------------------------------------------------------------
// Prints DOFs to file
//----------------------------------------------------------------------
void IK_Solver::PrintDofs(int frameNum)
{
#ifdef _DEBUG
	Logger::OpenLogFile("logs/dofs.txt");

	Logger::Print("Frame Num: ");
	Logger::PrintLine(frameNum);
#endif

	// get DOF list
	int dofCount = UI->mData->mSelectedModel->GetDofCount();
	DofList dofList = UI->mData->mSelectedModel->mDofList;

	// loop over and print
	for (int i = 0; i < dofCount; i++)
	{
		double dof = dofList.GetDof(i);
#ifdef _DEBUG
		Logger::Print("Dof ");
		Logger::Print(i);
		Logger::Print(": \t");
		Logger::PrintLine(dof);
#endif
	}

#ifdef _DEBUG
	Logger::CloseLogFile("logs/dofs.txt");
#endif
}