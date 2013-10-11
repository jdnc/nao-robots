/*

#include "KickSideMiddleModule.h"

#include <memory/FrameInfoBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>



KickSideMiddleModule::KickSideMiddleModule():kick_vel(1.0),init_sideKickMotion(false)
{
    
}

void KickSideMiddleModule::initSideKick(float kick_vel,bool l_r) { 
    if(!init_sideKickMotion)
    {   
        initSpecialMotion();  
        motionID= "kickDiagonalNaoL";
        if(!l_r)  motionID= "kickDiagonalNaoR";
        if(kick_vel!=1.0)
        {   
             SpecialMotionModule::VEL=kick_vel;       
        }
        initialize_MotionSequence();
  	    std::cout<<"finishInitialized"<<std::endl;
        init_sideKickMotion=true;
    }
   
}

bool KickSideMiddleModule::isKickingSide()
{
    return isDoingSpecialMotion();
}

void KickSideMiddleModule::processSideKick() {

  processed_joint_commands_->send_stiffness_ = false;


 // std::cout<<"Start"<<getup_state_<<std::endl;
  // state 0: turn off stiffness
  if (specialmotion_state_== 0) {
    specialmotion_pose_count_ = 0;
    // stiffness should already be off from lua command
    //processed_joint_commands_->send_stiffness_ = true;
    //processed_joint_commands_->stiffness_time_ = 10;
    //for (int i = 0; i < NUM_JOINTS; i++)
    //  processed_joint_commands_->stiffness_[i] = 0.0;
    specialmotion_state_=1;
    specialmotion_state_time_ = frame_info_->seconds_since_start;
  }


  // state 1: wait for us to finish the fall (a fraction of second)
  else if (specialmotion_state_== 1) {
    double diff = frame_info_->seconds_since_start - specialmotion_state_time_;
    // turn stiffness back on
    if (diff<=0.75)
    {
        std::cout<<"Wait for the fall end"<<std::endl;
        if (walk_request_->tilt_fallen_counter_ != 0)//fall on the back do some adjustment before fall
        {
            prepareKick();
        }
    }
    if (diff > 0.75){
      specialmotion_pose_count_ = 0;
      processed_joint_commands_->send_stiffness_ = true;
      processed_joint_commands_->stiffness_time_ = 10;
      for (int i = 0; i < NUM_JOINTS; i++)
        processed_joint_commands_->stiffness_[i] = 1.0;
      specialmotion_state_ = 3;  //added just ignore the cross part
    }
    
  }

  // state 3: start side kicking
  else if (specialmotion_state_ == 3) {
     
        executeMotionSequence();

   // std::cout << "continue getup " << std::endl;// << std::flush;
        double diff = frame_info_->seconds_since_start - specialmotion_state_time_;
        if (diff > waitFinishedTime){
            std::cout << "KickSide complete" << std::endl;// << std::flush;
            specialmotion_state_ = 4;
        }
    
  }

  // state 4: wait for getup
  else if (specialmotion_state_ == 4){
    specialmotion_state_ = 5;
    specialmotion_state_time_ = frame_info_->seconds_since_start;
  }

  // state 5: wait for it to be stable
  else if (specialmotion_state_ == 5){
    double diff = frame_info_->seconds_since_start - specialmotion_state_time_;
    if (diff > 0.5)
      specialmotion_state_ = 6;
  }
  else 
  {
     std::cout<<"Wrong state number: "<< specialmotion_state_<<std::endl<<std::flush;
  }
  
  
}

void KickSideMiddleModule::prepareKick()
{
   
}

*/
