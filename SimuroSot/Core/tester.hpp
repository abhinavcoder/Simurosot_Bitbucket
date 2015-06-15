# pragma once
#include "../mainheaders.hpp"
#include "../common/include/thread.h"
#include "../common/include/cs.hpp"
#include "../Tactics/tactic.h"
#include "../winDebugger/Client.h"

#include "comdef.h"
#include "../Utils/pathPlanners.h"


#include "../Skills/skillSet.h"
#include "../Core/beliefState.h"
#include "../common/include/geometry.hpp"



using namespace std;
using namespace MyStrategy;

class Tester : public Util::Thread
{
public:
  bool &running;
  bool isFirstRun;
  BeliefState &state;
 
  Tester(bool &running_, BeliefState &state_):
	  running(running_),state(state_), isFirstRun(true) {}
  
  void run()
  {
     static MyStrategy::SParam sp , spline;
	 static MyStrategy::SParam sp1 , sp2;
     
	 static bool firstRun = true ;
	 sp.GoToPointP.x =  0;//+ BOT_RADIUS);
	 sp.GoToPointP.y = 0 ;//+ BOT_RADIUS;
     sp.GoToPointP.align = false;
     sp.GoToPointP.finalslope = PI/2;

	 sp1.GoToPointDWP.x = 0 ;
	 sp1.GoToPointDWP.y = 0 ;
	 sp1.GoToPointDWP.finalslope = PI/2 ;

	 spline.SplineGoToPointP.x = 0;
	 spline.SplineGoToPointP.y = 0;
	 spline.SplineGoToPointP.finalSlope = PI/2 ;
	 spline.SplineGoToPointP.finalVelocity = 0;
   static SkillSet spline0(&state,0);
    static SkillSet p1(&state,0);

 static Tactic::Param pAttack;
    pAttack.AttackP.rotateOnError = true;
    /*FILE *f = fopen("/home/robo/botplot/compare_dataset/botlog.txt", "w");
    fclose(f);
    f = fopen("/home/robo/botplot/compare_dataset/response.txt", "w");
    fclose(f);*/
   isFirstRun= false;
    //    Util::Timer timer;
	  
    //      timer.start();
     char debug[250];
      sprintf(debug,"Ball Pos = %d, %d \n", state.pr_ballOurSide,state.pr_ballOppSide);

	 	spline0.executeSkill(SkillSet::SplineGoToPoint,spline);
    return;

  }
};