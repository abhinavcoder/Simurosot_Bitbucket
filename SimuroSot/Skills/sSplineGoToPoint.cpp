#include "stdafx.h"
#include "skillSet.h"
#include "../Core/beliefState.h"
#include "../common/include/config.h"
#include "pose.h"
#include "../HAL/comm.h"
#include "trajectory-generators.cpp"
#include <iostream>
#include "../winDebugger/Client.h"

#define PREDICTION_PACKET_DELAY 0
using namespace Util;

namespace MyStrategy
{
		
	void SkillSet::_splineGoToPointTrack(int botid, Pose start, Pose end, float finalvel){

		if(!algoController){
			if(traj)
				algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
		}
		int vl,vr;
		if(!direction)start.setTheta(start.theta() - PI);
		algoController->genControls(start, end, vl, vr, finalvel);
		assert(vl <= 125 && vl >= -125);
		assert(vr <= 125 && vr >= -125);
		if(direction)comm->sendCommand(botid, vl, vr); //maybe add mutex
		else comm->sendCommand(botid, -vr, -vl);
	}

	void SkillSet::_splineGoToPointInitTraj(int botid, Pose start, Pose end, float finalvel){

		direction = isFrontDirected(start, end);
		if(!direction)start.setTheta(start.theta() - PI);

		if(traj)delete traj;
		traj = TrajectoryGenerators::cubic(start, end ,0,0,0,0); //may need to modify vle,vls,vre,vrs
		
		if(algoController)delete algoController;
		algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
		
		_splineGoToPointTrack(botid,start,end,finalvel);
	}
	
 
  void SkillSet::splineGoToPoint(const SParam& param)
  {
	  char Debug[250] ;
	 static bool iniTraj = true ;
	Vector2D<int> dpoint;
    float finalvel;
    finalvel  = param.GoToPointP.finalVelocity;
	
		static int start_x=0, start_y=0,start_ang=0;

	static int counter = 0 ; 
	if(counter < 10 )
	{
		start_x += state->homePos[botID].x;
		start_y += state->homePos[botID].y;
		start_ang += state->homeAngle[botID];

	 counter++ ;
	 comm->sendCommand(botID,0,0);
	 return ;
	}
	else
	{
	  if(counter==11 ) 
		  iniTraj = false ;
	  else 
		  counter++ ;
	}

	//sprintf(Debug,"iniTraj = %d",iniTraj);
	//Client::debugClient->SendMessages(Debug);

	if(iniTraj/*param.SplineGoToPointP.initTraj*/){
		Pose start(start_x/10, start_y/10, start_ang/10);
		Pose end(param.SplineGoToPointP.x, param.SplineGoToPointP.y, param.SplineGoToPointP.finalSlope);
		start_x = start_y = start_ang = 0;
		sprintf(Debug, "poition  : %f   %f\n", start.x() , start.y());
		Client::debugClient->SendMessages(Debug);

		Vector2D<int> dest ; dest.x = end.x() ; dest.y = end.y() ;
		int dist = Vector2D<int>::dist(state->homePos[botID],dest) ;
		if(dist<0.5*BOT_BALL_THRESH)
		{
			iniTraj = true ; counter = 0 ;
			comm->sendCommand(botID,0,0);
			return ;
		}

		_splineGoToPointInitTraj(botID, start, end, finalvel);
	}
	else{
		Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
		Pose end(param.SplineGoToPointP.x, param.SplineGoToPointP.y, param.SplineGoToPointP.finalSlope);
		sprintf(Debug, "poition  : %f   %f\n", start.x() , start.y());
		Client::debugClient->SendMessages(Debug);

		Vector2D<int> dest ; dest.x = end.x() ; dest.y = end.y() ;
		int dist = Vector2D<int>::dist(state->homePos[botID],dest) ;
		if(dist<0.5*BOT_BALL_THRESH)
		{
			iniTraj = true ; counter = 0 ;
			comm->sendCommand(botID,0,0);
			return ;
		}
		_splineGoToPointTrack(botID, start, end, finalvel);
	}
	
   }
}