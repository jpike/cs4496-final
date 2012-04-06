// Only commands that can be scripted will be processed as command lines

#ifndef __UICALLBACK_H__
#include "UICallback.h"
#endif	//__UICALLBACK_H__

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__

#ifndef __MODEL_H__
#include "Model.h"
#endif	//__HMODEL_H__

#ifndef	__MARKER_H__
#include "Marker.h"
#endif	//__MARKER_H__

#ifndef __COMMAND_H__
#include "Command.h"
#endif //__COMMAND_H__

#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif //RealTimeIKui_h

#ifndef	__PHOWARDDATA_H__
#include "PhowardData.h"
#endif

extern GLenum DRAW_STYLE;
extern RealTimeIKUI *UI;

void LoadModel_cb(Fl_Widget *o, void *v)
{
  LoadModel(NULL);
}

void Motion_cb(Fl_Widget *o, void *v)
{
}

void LoadMocap_cb(Fl_Widget *o, void *v)
{
  LoadC3d(NULL);
}

void SaveMotion_cb(Fl_Widget *o, void *v)
{
}


void Exit_cb(Fl_Widget *o, void *v)
{
  Exit(NULL);
}

void PlayStop_cb(Fl_Widget *o, void *v)
{
  if(UI->mPlay_but->value()){
    UI->mPlay_but->label("STOP");
  }else{
    UI->mPlay_but->label("@>");
  }
}

void Frame_scr_cb(Fl_Widget *o, void *v)
{
  Fl_Value_Slider* slider = (Fl_Value_Slider*)o;
  int currentFrame = int(slider->value());
  UI->mFrameCounter_cou->value(currentFrame);
}

void Speed_cb(Fl_Widget *o, void *v)
{
  UI->mData->mHowFast = UI->mSpeed_rol->value();

}

void FrameCounter_cb(Fl_Widget *o, void *v)
{
  double val = UI->mFrameCounter_cou->value();
  int beginFrame = int(UI->mBegin_sli->value());
  int endFrame = int(UI->mEnd_sli->value());
  bool looping = UI->mLoop_but->value();

  if(val < beginFrame){
    if(looping)
      val = beginFrame;
  }else if( val > endFrame){
    if(looping)
      val = beginFrame;
  }

  // val is unwarped time
  int tw = val;
    
  UI->mFrame_sli->value(val);
  UI->mFrameCounter_cou->value(val);
}

void RecordMotion_cb(Fl_Widget *o, void *v)
{
}

void StillShot_cb(Fl_Widget *o, void *v)
{
}

void ShowModel_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    UI->mShowModel_but->value(0);
    return;
  }

  int status = ((Fl_Button*)o)->value();
  UI->mGLWindow->mShowModel = status;

  UI->mGLWindow->refresh();

}

void ShowConstr_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    UI->mShowConstr_but->value(0);
    return;
  }

  int status = ((Fl_Button*)o)->value();
  UI->mGLWindow->mShowConstraints = status;

  UI->mGLWindow->refresh();

}

void ShowViz_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() == 0){
    UI->mShowViz_but->value(0);
    return;
  }
  int status = ((Fl_Button*)o)->value();
  UI->mGLWindow->mShowViz = status;
  if(status){
    DRAW_STYLE = GLU_SILHOUETTE;
  }else{
    DRAW_STYLE = GLU_FILL;
  }
  UI->mGLWindow->refresh();
}

void ShowModel_ite_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    return;
  }
  int status = !UI->mGLWindow->mShowModel;
  UI->mGLWindow->mShowModel = status;

  UI->mShowModel_but->value(status);
  UI->mGLWindow->refresh();

}

void ShowConstr_ite_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    return;
  }
  int status = !UI->mGLWindow->mShowConstraints;
  UI->mGLWindow->mShowConstraints = status;

  UI->mShowConstr_but->value(status);
  UI->mGLWindow->refresh();
}

void ShowViz_ite_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
    return;
  }
  int status = !UI->mGLWindow->mShowViz;
  UI->mGLWindow->mShowViz = status;
  UI->mShowViz_but->value(status);
  if(status){
    DRAW_STYLE = GLU_SILHOUETTE;
  }else{
    DRAW_STYLE = GLU_FILL;
  }
  UI->mGLWindow->refresh();
}

void Terminate_cb(Fl_Widget *o, void *v)
{
  UI->mData->mTermination = true;
}

void About_cb(Fl_Widget *o, void *v)
{
}

void ShowSliders_cb(Fl_Widget *o, void *v)
{
  if(UI->mData->mModels.size() <= 0){
  }

  if(!UI->mDofSliderWindow->shown()){	
    int nDof = UI->mData->mSelectedModel->GetDofCount();
    for(int i = 0; i < nDof; i++){
	Dof *currDof = UI->mData->mSelectedModel->mDofList.mDofs[i];
	UI->mDofs_sli[i]->maximum(currDof->mUpperBound);
	UI->mDofs_sli[i]->minimum(currDof->mLowerBound);
	UI->mDofs_sli[i]->value(currDof->mVal);
    }
    UI->mDofSliderWindow->show();
  }else{
    UI->mDofSliderWindow->hide();
  }

}


void DofSliders_cb(Fl_Widget *o, void *v)
{
  int index = (long int)v;
  Fl_Value_Slider* slider = (Fl_Value_Slider*)o;

  UI->mData->mSelectedModel->mDofList.SetDof(index, slider->value());
}

void Solve_cb(Fl_Widget *o, void *v)
{
    Solution(NULL);
}
