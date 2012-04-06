#ifndef RealTimeIKui_h
#include "RealTimeIKui.h"
#endif	//RealTimeIKui_h

#ifndef __PHYLTERFLGLWINDOW_H__
#include "PhylterGLWindow.h"
#endif	//__PHYLTERFLGLWINDOW_H__
#include <time.h>

static double TICK = 0.02;

RealTimeIKUI *UI;

void onTimer(void *)
{

  if(UI->mPlay_but->value())
    UI->Increment();
      
  UI->mGLWindow->refresh();
  
  Fl::add_timeout(TICK, onTimer);

}

int main(int argc, char **argv) 
{
  UI = new RealTimeIKUI();
  
  Fl::visual(FL_DOUBLE|FL_INDEX);
  UI->Show();
  Fl::add_timeout(TICK, onTimer);
  return Fl::run();
}
