#ifndef __ROTATEEULER_H__
#include "RotateEuler.h"
#endif	//__ROTATEEULER_H__

#include <FL/gl.h>
#include <assert.h>
#include <string.h>

extern double sinTable[361];
extern double cosTable[361];

static double TORADIANS( double degrees )
{
  return (degrees * 3.14159) / 180.0;
}

RotateEuler::RotateEuler( int axis, double angle, double min, double max, char* name, DofList& dofs ) : 
  mAxis(0,0,0)
{
  mIndex = 1;
  mAxis[axis] = 1.0;
  mDof = new EulerDof( name, min, max, dofs.mDofs.size(), *this);
  mAngle = angle;

  dofs.AddDof( mDof );
}

RotateEuler::RotateEuler( istream& in, DofList* dofs ) : mAxis(0,0,0)
{
  mIndex = 1;
  if ( dofs )
    {
      char buf[1024];
      in >> buf;
      assert(in);
      char* name = new char[strlen(buf)+1];
      strcpy( name, buf );
      int axis;
      in >> axis;
      assert(in);
      mAxis[axis] = 1.0;
      double min, max;
      in >> mAngle >> min >> max;
      assert(in);
      mAngle = TORADIANS( mAngle );
      mDof = new EulerDof( name, TORADIANS(min), TORADIANS(max), dofs->mDofs.size(), *this );
      dofs->AddDof( mDof );
    }
  else
    {
      int axis;
      in >> axis;
      assert(in);
      mAxis[axis] = 1.0;
      in >> mAngle;
      assert(in);
      mAngle = TORADIANS( mAngle );
      mDof = 0;
    }
}

void RotateEuler::Apply()
{
  glRotated(180.0 * mAngle / 3.14159, mAxis[0], mAxis[1], mAxis[2] ); 
}

Mat4d RotateEuler::GetTransform()
{
  Mat4d m;
  int axis;
  double cosAngle;
  double sinAngle;
  cosAngle = cos(mAngle);
  sinAngle = sin(mAngle);
  if(mAxis[0])
    axis = 0;
  else if(mAxis[1])
    axis = 1;
  else if(mAxis[2])
    axis  = 2;

  // calculate coefficients in !row! major order
  switch(axis)
    {	case 0:
	  m[0][0] = 1;
	  m[0][1] = 0;
	  m[0][2] = 0;
 
	  m[1][0] = 0;
	  m[1][1] = cosAngle;
	  m[1][2] = -sinAngle;
			

	  m[2][0] = 0;
	  m[2][1] = sinAngle;
	  m[2][2] = cosAngle;
	  break;

    case 1:
      m[0][0] = cosAngle;
      m[0][1] = 0;
      m[0][2] = sinAngle;
 
      m[1][0] = 0;
      m[1][1] = 1;
      m[1][2] = 0;
			

      m[2][0] = -sinAngle; 
      m[2][1] = 0;
      m[2][2] = cosAngle;
      break;
		
    case 2:
      m[0][0] = cosAngle;
      m[0][1] = -sinAngle;
      m[0][2] = 0;
 
      m[1][0] = sinAngle;
      m[1][1] = cosAngle;
      m[1][2] = 0;
			

      m[2][0] = 0;
      m[2][1] = 0;
      m[2][2] = 1;
      break;
    }
	
  m[0][3] = 0.0;
  m[1][3] = 0.0;
  m[2][3] = 0.0;
  m[3][0] = 0;			
  m[3][1] = 0;
  m[3][2] = 0;			
  m[3][3] = 1;
	
  return m;
}

//----------------------------------------------------------------------
// Derivative implementation.
// TODO: NEEDS TO BE VERIFIED
//
// cos -sin 0 0
// sin cos 0 0
// 0 0 1 0
// 0 0 0 1
//
// Derivative should be:
// 
// -sin	-cos 0 0
// cos	-sin 0 0
// 0	0	 0 0
// 0	0	 0 0
//----------------------------------------------------------------------
Mat4d RotateEuler::GetDeriv(int dof)
{
	Mat4d matrix = vl_zero;

	double sine = sin(mAngle);
	double cosine = cos(mAngle);

	if (mAxis[0])
	{
		matrix[1][1] = -sine;
		matrix[1][2] = -cosine;
		matrix[2][1] = cosine;
		matrix[2][2] = -sine;
	}
	else if (mAxis[1])
	{
		matrix[0][0] = -sine;
		matrix[0][2] = cosine;
		matrix[2][0] = -cosine;
		matrix[2][2] = -sine;
	}
	else if (mAxis[2])
	{
		matrix[0][0] = -sine;
		matrix[0][1] = -cosine;
		matrix[1][0] = cosine;
		matrix[1][1] = -sine;
	}

	return matrix;
}