#include <yarp/os/all.h>

#include "locoman_service_1_thread.h"
#include "locoman_service_1_constants.h"

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <GYM/yarp_command_interface.hpp>
#include <fstream>
#include <unistd.h>
#include <limits>


//TODO 
#include <iCub/iDynTree/yarp_kdl.h>
#include <locoman/utils/screws.hpp>
#include <locoman/utils/kinematics.hpp>
#include <locoman/utils/kinetostatics.hpp>
#include <locoman/utils/locoman_utils.hpp>
#include <locoman/utils/algebra.hpp>


using namespace yarp::math;

locoman_service_1_thread::locoman_service_1_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph ),
    CoM_w_cmd(3, 0.0) ,
    CoM_w_up(3, 0.0) ,
    CoM_w_dw(3, 0.0) ,
    FC_size(24)  ,
    FC_HANDS_size(24) ,
    WINDOW_size(15) , //30 //50  // 15
    FC_DES(FC_size, 0.0) , 
    FC_DES_LEFT_sensor(6, 0.0) ,
    FC_DES_RIGHT_sensor(6,0.0),
    FC_FILTERED(FC_size),
    FC_WINDOW(FC_size, WINDOW_size ) ,
    //
    FC_HANDS_DES(FC_HANDS_size, 0.0) ,
    FC_DES_LEFT_HAND_sensor(6, 0.0) ,
    FC_DES_RIGHT_HAND_sensor(6, 0.0) ,
    FC_HANDS_FILTERED(FC_HANDS_size) ,
    FC_HANDS_WINDOW(FC_HANDS_size, WINDOW_size ) ,
    //---------------------------------
    // number of fc components on the feet
    zero_3(3, 0.0) ,
    Zeros_6_6(6,6) ,
    Eye_6(6,6) ,
    Eye_3(3,3) ,
    Eye_4(4,4) ,
    B(6,3) ,
    Kc( 24 , 24) ,
    Kc_f_rh( 36 ,36 ) ,
    ft_l_ankle(6,0.0) ,
    ft_r_ankle(6,0.0) ,
    ft_l_wrist(6,0.0) ,
    ft_r_wrist(6,0.0) ,
    fc_offset_left(12, 0.0) ,
    fc_offset_right(12, 0.0) ,
    fc_offset_left_hand(12, 0.0) ,
    fc_offset_right_hand(12, 0.0),
    
    //
    map_l_fcToSens(6,12) ,
    map_r_fcToSens(6,12) ,  
    map_r_hand_fcToSens(6,12) ,
    map_l_hand_fcToSens(6,12) ,
    map_l_fcToSens_PINV(12,6) ,
    map_r_fcToSens_PINV(12,6) ,
    map_l_hand_fcToSens_PINV(12,6) ,
    map_r_hand_fcToSens_PINV(12,6) ,
    //
  fc_l_c1_filt(3, 0.0) , // = FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l_c2_filt(3, 0.0) , // = FC_FILTERED.subVector(3,5)  ;
  fc_l_c3_filt(3, 0.0) , // = FC_FILTERED.subVector(6,8)  ;
  fc_l_c4_filt(3, 0.0) , // = FC_FILTERED.subVector(9,11)  ;

  fc_r_c1_filt(3, 0.0) , // = FC_FILTERED.subVector(12,14)  ; 
  fc_r_c2_filt(3, 0.0) , // = FC_FILTERED.subVector(15,17)  ; 
  fc_r_c3_filt(3, 0.0) , // = FC_FILTERED.subVector(18,20)  ; 
  fc_r_c4_filt(3, 0.0) , // = FC_FILTERED.subVector(21,23)  ; 
  
  fc_l1_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l2_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(3,5)  ;
  fc_l3_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(6,8)  ;
  fc_l4_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(9,11)  ;

  fc_r1_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(12,14)  ; 
  fc_r2_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(15,17)  ; 
  fc_r3_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(18,20)  ; 
  fc_r4_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(21,23)  ;   
    
  d_EE_r_des(6, 0.0 ) ,
  d_EE_l_des(6 , 0.0) ,  
    
  T_w_aw_0(4,4) , // = locoman::utils::AW_world_posture(model, robot) ;
  T_aw_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_aw_0) ;    

  T_w_waist_0(4,4) , //    = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0(4,4) , //  = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0(4,4) , //     = model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0(4,4) , //     = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0(4,4) , //     = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0(4,4) , //     = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0(4,4) , //  = model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0(4,4) , //     = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0(4,4) , //     = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0(4,4) , //     = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0(4,4) , //     = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0(4,4) , //   = model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0(4,4) , //   = model.iDyn3_model.getPosition( r_hand_index ) ;   

  T_w_l_wrist_0(4,4) , //  = model.iDyn3_model.getPosition(l_wrist_index) ;
  T_w_l1_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  T_w_l2_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  T_w_l3_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  T_w_l4_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  T_w_r_wrist_0(4,4) , //  = model.iDyn3_model.getPosition(r_wrist_index) ;
  T_w_r1_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  T_w_r2_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  T_w_r3_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  T_w_r4_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
  
  T_waist_w_0(4,4) , //    = locoman::utils::iHomogeneous(T_w_waist_0)  ;
  T_l_ankle_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  T_l_c1_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  T_l_c2_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  T_l_c3_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  T_l_c4_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c4_0) ;    
    
  T_r_ankle_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  T_r_c1_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  T_r_c2_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  T_r_c3_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  T_r_c4_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  T_l_wrist_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
  T_l1_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  T_l2_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  T_l3_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  T_l4_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  T_r_wrist_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  T_r1_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  T_r2_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  T_r3_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  T_r4_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  T_l_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  T_r_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
  
  T_aw_l_c1_0(4,4) , //  = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  T_aw_l_c2_0(4,4) , //  = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l_c3_0(4,4) , //  = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  T_aw_l_c4_0(4,4) , //  = T_aw_w_0 * T_w_l_c4_0 ;

  T_aw_r_c1_0(4,4) , //  = T_aw_w_0 * T_w_r_c1_0 ;
  T_aw_r_c2_0(4,4) , //  = T_aw_w_0 * T_w_r_c2_0 ;
  T_aw_r_c3_0(4,4) , //  = T_aw_w_0 * T_w_r_c3_0 ;
  T_aw_r_c4_0(4,4) , //  = T_aw_w_0 * T_w_r_c4_0 ; 

  T_aw_l1_hand_0(4,4) , //  = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  T_aw_l2_hand_0(4,4) , //  = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l3_hand_0(4,4) , //  = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  T_aw_l4_hand_0(4,4) , //  = T_aw_w_0 * T_w_l4_hand_0 ;

  T_aw_r1_hand_0(4,4) , //  = T_aw_w_0 * T_w_r1_hand_0 ;
  T_aw_r2_hand_0(4,4) , //  = T_aw_w_0 * T_w_r2_hand_0 ;
  T_aw_r3_hand_0(4,4) , //  = T_aw_w_0 * T_w_r3_hand_0 ;
  T_aw_r4_hand_0(4,4) , //  = T_aw_w_0 * T_w_r4_hand_0 ;   
    
  //-----------------------------------------------------
  J_l_c1_mix_0( 6, ( size_q + 6 ) ) , //
  J_l_c2_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l_c3_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l_c4_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_r_c1_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c2_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c3_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c4_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_l_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_l1_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l2_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l3_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l4_hand_mix_0( 6, ( size_q + 6 ) ) ,//robot.getNumberOfKinematicJoints() + 6 ) ) ;

  J_r1_hand_mix_0( 6, ( size_q + 6 ) ) , // robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r2_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r3_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r4_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  //-------------------------------------------
  
  J_l_c1_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  J_l_c2_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  J_l_c3_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  J_l_c4_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  J_r_c1_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  J_r_c2_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  J_r_c3_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  J_r_c4_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  J_l_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  J_r_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  J_l1_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  J_l2_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  J_l3_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  J_l4_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  J_r1_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  J_r2_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  J_r3_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  J_r4_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------  
  J_aw_l_c1_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l_c2_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l_c3_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l_c4_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  J_aw_r_c1_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r_c2_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r_c3_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r_c4_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  J_aw_l1_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l2_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l3_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l4_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;
 
  J_aw_r1_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r2_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r3_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r4_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
  
  //------------------------------------------------
  Q_aw_l_c1(size_q+ 6, size_q + 6)   , //= Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  Q_aw_l_c2(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  Q_aw_l_c3(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l_c4(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  Q_aw_r_c1(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c2(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c3(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r_c4(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  Q_aw_r1_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r2_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r3_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r4_hand(size_q+ 6, size_q + 6)   ,

  Q_aw_l_tot(size_q+ 6, size_q + 6)   , // = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  Q_aw_r_tot(size_q+ 6, size_q + 6)   , // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  Q_aw_r_hand_tot(size_q+ 6, size_q + 6)   , // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  Q_aw_c(size_q+ 6, size_q + 6)   , // =  Q_aw_l_tot + Q_aw_r_tot ;  
  U_aw_s_cont( 6 , 6) , // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_cont( 6 , size_q ) , //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
  Q_aw_c_f_rh(size_q+ 6, size_q + 6)   , // =  Q_aw_l_tot + Q_aw_r_tot ;  
  U_aw_s_c_f_rh( 6 , 6) , // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_c_f_rh( 6 , size_q ) ,  //  = Q_aw_c.submatrix( 0  , 5,  6,
  
  //------------------------------------------------------------------
  d_fc_des_to_world(size_fc, 0.0)  ,

  T_l_c1_r_c1_loop(4,4) ,
  T_r_c1_l_c1_loop(4,4) ,
//   T_l_c1_r_c1_loop.zero() ,
//   T_r_c1_l_c1_loop.zero() ,  
  J_com_w( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_com_w_redu( 3,  ( size_q + 6 )  ) , //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_aw( 3,  ( size_q + 6 ) ) , //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_waist( 3,  ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_r_c1_aw( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_l_c1_aw( 6, ( size_q + 6 ) )  //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  {
    // TODO: skeleton constructor
}

bool locoman_service_1_thread::custom_init()
{    
    struct sched_param thread_param; 
    thread_param.sched_priority = 99;
    
    model.setFloatingBaseLink("Waist");
    yarp::sig::Vector q_current(robot.getNumberOfKinematicJoints(),0.0) ; // = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q_current, true);
    
    
  //------------------------------------------------------------------------------------------------------------------

  // Defining Various Parameters
  //yarp::sig::Vector zero_3(3, 0.0) ;
  //yarp::sig::Matrix Zeros_6_6(6,6) ;
  Zeros_6_6.zero();
  //yarp::sig::Matrix Eye_6(6,6) ;
  Eye_6.eye() ;
  //yarp::sig::Matrix Eye_3(3,3) ;
  Eye_3.eye() ;
  Eye_4.eye() ;
  //yarp::sig::Matrix B( 6 , 3 ) ;
  B.zero();
  B.setSubmatrix( Eye_3 , 0 , 0 ) ;
  
  /*size_q = robot.getNumberOfKinematicJoints()*/ ; // getNumberOfKinematicJoints = 31 (29 + 2 for the hands)
  size_u = 6 ;
  size_fc = 24 ; // number of fc components on the feet
  kc = 1E6 ;
  Kq = locoman::utils::getKq(robot) ;
  Kc.eye() ;
  Kc = kc*Kc ;    //   ;// 1E6*Kc ;    1E8*Kc ;  
  Kc_f_rh.eye() ;
  Kc_f_rh = (kc/10)*Kc_f_rh ;
  
  waist_index   =  model.iDyn3_model.getLinkIndex("Waist");  
  l_ankle_index = model.iDyn3_model.getLinkIndex("l_leg_ft") ; // sensors are placed in *_ankle in the model

  l_c1_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link") ;
  l_c2_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
  l_c3_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link") ;
  l_c4_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");

  r_ankle_index = model.iDyn3_model.getLinkIndex("r_leg_ft") ;

  r_c1_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
  r_c2_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
  r_c3_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
  r_c4_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");

  l_hand_index  = model.iDyn3_model.getLinkIndex("LSoftHand");
  r_hand_index  = model.iDyn3_model.getLinkIndex("RSoftHand");    

  l_wrist_index  = model.iDyn3_model.getLinkIndex("l_arm_ft") ;
  l_hand_c1_index = model.iDyn3_model.getLinkIndex("l_hand_upper_right_link"); 
  l_hand_c2_index = model.iDyn3_model.getLinkIndex("l_hand_lower_right_link"); 
  l_hand_c3_index = model.iDyn3_model.getLinkIndex("l_hand_upper_left_link");  
  l_hand_c4_index = model.iDyn3_model.getLinkIndex("l_hand_lower_left_link");  
//     
  r_wrist_index   = model.iDyn3_model.getLinkIndex("r_arm_ft") ;
  r_hand_c1_index = model.iDyn3_model.getLinkIndex("r_hand_upper_right_link"); 
  r_hand_c2_index = model.iDyn3_model.getLinkIndex("r_hand_lower_right_link"); 
  r_hand_c3_index = model.iDyn3_model.getLinkIndex("r_hand_upper_left_link");  
  r_hand_c4_index = model.iDyn3_model.getLinkIndex("r_hand_lower_left_link");  
// //    
  
  map_l_fcToSens =  locoman::utils::fConToSens( l_ankle_index, 
                                                l_c1_index  , 
                                                l_c2_index  ,                                
                                                l_c3_index  , 
                                                l_c4_index,
                                                model
                                                ) ;
                            
  map_r_fcToSens =  locoman::utils::fConToSens( r_ankle_index, 
                                                r_c1_index, 
                                                r_c2_index,
                                                r_c3_index, 
                                                r_c4_index,
                                                model
                                                ) ;
                      // 
  map_l_hand_fcToSens = locoman::utils::fConToSens( l_wrist_index, 
                                                    l_hand_c1_index, 
                                                    l_hand_c2_index,
                                                    l_hand_c3_index, 
                                                    l_hand_c4_index,
                                                    model
                                                    ) ;
    
  map_r_hand_fcToSens = locoman::utils::fConToSens( r_wrist_index, 
                                                    r_hand_c1_index, 
                                                    r_hand_c2_index,
                                                    r_hand_c3_index, 
                                                    r_hand_c4_index ,
                                                    model
                                                    ) ;
                                                                        
  map_l_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_l_fcToSens, 1E-10 ) ; //*  ft_l_ankle  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  map_r_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_r_fcToSens, 1E-10 ) ; // *  ft_r_ankle  ;  */// yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
  map_l_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_l_hand_fcToSens, 1E-10 ) ; //*  ft_l_wrist  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  map_r_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_r_hand_fcToSens, 1E-10 ) ; // 
    
      T_w_aw_0.zero()     ; //= locoman::utils::AW_world_posture(model, robot) ;
  T_aw_w_0.zero()  ; //= locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  T_w_waist_0.zero()   ; // = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0.zero()   ; // = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0.zero()    ; //= model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0.zero()    ; // = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0.zero()   ; // = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0.zero()    ; // = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0.zero()  ; //= model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0.zero()    ; // = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0.zero()    ; // = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0.zero()   ; // = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0.zero()    ; // = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0.zero()  ; //= model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0.zero() ;   

  T_w_l_wrist_0.zero()  ;
  T_w_l1_hand_0.zero()  ;
  T_w_l2_hand_0.zero()  ;
  T_w_l3_hand_0.zero()  ;
  T_w_l4_hand_0.zero()  ;
    
  T_w_r_wrist_0.zero()  ;
  T_w_r1_hand_0.zero()  ;
  T_w_r2_hand_0.zero()  ;
  T_w_r3_hand_0.zero()  ;
  T_w_r4_hand_0.zero()  ;
  
  // -----------------------------------------------------------------------
  T_waist_w_0.zero()  ;
  T_l_ankle_w_0.zero()  ;
  T_l_c1_w_0.zero()  ;
  T_l_c2_w_0.zero()  ;
  T_l_c3_w_0.zero()  ;
  T_l_c4_w_0.zero()  ;
    
  T_r_ankle_w_0.zero()  ;
  T_r_c1_w_0.zero()  ;
  T_r_c2_w_0.zero()  ;
  T_r_c3_w_0.zero()  ;
  T_r_c4_w_0.zero()  ;

  T_l_wrist_w_0.zero()  ;
  T_l1_hand_w_0.zero()  ;
  T_l2_hand_w_0.zero()  ;
  T_l3_hand_w_0.zero()  ;
  T_l4_hand_w_0.zero()  ;
      
  T_r_wrist_w_0.zero()  ;
  T_r1_hand_w_0.zero()  ;
  T_r2_hand_w_0.zero()  ;
  T_r3_hand_w_0.zero()  ;
  T_r4_hand_w_0.zero()  ;

  //---------------------------------------------------------------------
  
  T_l_hand_w_0.zero()  ;
  T_r_hand_w_0.zero()  ;
  
  T_aw_l_c1_0.zero()  ;
  T_aw_l_c2_0.zero()  ;
  T_aw_l_c3_0.zero()  ;
  T_aw_l_c4_0.zero()  ;

  T_aw_r_c1_0.zero()  ;
  T_aw_r_c2_0.zero()  ;
  T_aw_r_c3_0.zero()  ;
  T_aw_r_c4_0.zero()  ;

  T_aw_l1_hand_0.zero()  ;
  T_aw_l2_hand_0.zero()  ;
  T_aw_l3_hand_0.zero()  ;
  T_aw_l4_hand_0.zero()  ;

  T_aw_r1_hand_0.zero()  ;
  T_aw_r2_hand_0.zero()  ;
  T_aw_r3_hand_0.zero()  ;
  T_aw_r4_hand_0.zero()  ;
    
   //  Jacobian Matrices 
  J_l_c1_mix_0.zero()  ;
  J_l_c2_mix_0.zero()  ;
  J_l_c3_mix_0.zero()  ;
  J_l_c4_mix_0.zero()  ;
  
  J_r_c1_mix_0.zero()  ;
  J_r_c2_mix_0.zero()  ;
  J_r_c3_mix_0.zero()  ;
  J_r_c4_mix_0.zero()  ;
  
  J_l_hand_mix_0.zero()  ;
  J_r_hand_mix_0.zero()  ;  
  J_l1_hand_mix_0.zero()  ;
  J_l2_hand_mix_0.zero()  ;
  J_l3_hand_mix_0.zero()  ;
  J_l4_hand_mix_0.zero()  ;

  J_r1_hand_mix_0.zero()  ;
  J_r2_hand_mix_0.zero()  ;
  J_r3_hand_mix_0.zero()  ;
  J_r4_hand_mix_0.zero()  ;

 // -------------------------------------------
  
  J_l_c1_body_0.zero()  ;
  J_l_c2_body_0.zero()  ;
  J_l_c3_body_0.zero()  ;
  J_l_c4_body_0.zero()  ;

  J_r_c1_body_0.zero()  ;
  J_r_c2_body_0.zero()  ;
  J_r_c3_body_0.zero()  ;
  J_r_c4_body_0.zero()  ;

  J_l_hand_body_0.zero()  ;
  J_r_hand_body_0.zero()  ;

  J_l1_hand_body_0.zero()  ;
  J_l2_hand_body_0.zero()  ;
  J_l3_hand_body_0.zero()  ;
  J_l4_hand_body_0.zero()  ;

  J_r1_hand_body_0.zero()  ;
  J_r2_hand_body_0.zero()  ;
  J_r3_hand_body_0.zero()  ;
  J_r4_hand_body_0.zero()  ;
  
  ////---------------------------------------------------------------------------------------------------------------------------------------------------------------  
  J_aw_l_c1_spa_0.zero()  ;
  J_aw_l_c2_spa_0.zero()  ;
  J_aw_l_c3_spa_0.zero()  ;
  J_aw_l_c4_spa_0.zero()  ;

  J_aw_r_c1_spa_0.zero()  ;
  J_aw_r_c2_spa_0.zero()  ;
  J_aw_r_c3_spa_0.zero()  ;
  J_aw_r_c4_spa_0.zero()  ;

  J_aw_l1_hand_spa_0.zero()  ;
  J_aw_l2_hand_spa_0.zero()  ;
  J_aw_l3_hand_spa_0.zero()  ;
  J_aw_l4_hand_spa_0.zero()  ;
 
  J_aw_r1_hand_spa_0.zero()  ;
  J_aw_r2_hand_spa_0.zero()  ;
  J_aw_r3_hand_spa_0.zero()  ;
  J_aw_r4_hand_spa_0.zero()  ;
  
  ////------------------------------------------------
  Q_aw_l_c1.zero()  ;
  Q_aw_l_c2.zero()  ;
  Q_aw_l_c3.zero()  ;
  Q_aw_l_c4.zero()  ;

  Q_aw_r_c1.zero()  ;
  Q_aw_r_c2.zero()  ;
  Q_aw_r_c3.zero()  ;
  Q_aw_r_c4.zero()  ;
  
  Q_aw_r1_hand.zero()  ;
  Q_aw_r2_hand.zero()  ;
  Q_aw_r3_hand.zero()  ;
  Q_aw_r4_hand.zero()  ;
    
  Q_aw_l_tot.zero()  ;
  Q_aw_r_tot.zero()  ;
  Q_aw_r_hand_tot.zero()  ;
  Q_aw_c.zero()  ;
  U_aw_s_cont.zero()  ;
  Q_aw_s_cont.zero()  ;
  Q_aw_c_f_rh.zero()  ;
  U_aw_s_c_f_rh.zero()  ;
  Q_aw_s_c_f_rh.zero()  ;
  // ---------------------------------------------------------
// 
  d_fc_des_to_world.zero();  ;
// 
  T_l_c1_r_c1_loop.zero()  ;
  T_r_c1_l_c1_loop.zero()  ;
  T_l_c1_r_c1_loop.zero() ;
  T_r_c1_l_c1_loop.zero() ;  
  J_com_w.zero()  ;
  J_com_w_redu.zero()  ;
  J_com_aw.zero()  ;
  J_com_waist.zero()  ;
  J_r_c1_aw.zero()  ;
  J_l_c1_aw.zero()  ;
  
  
    
  return true;
}

void locoman_service_1_thread::run()
{   
    // First Block
    //-------------------------------------------------------------------------------------------------------
    // TODO: In this first block we will read yarp ports obtaining
    // the values of teh kineto-static configuration variables
    // joint configuration, contct force, etc...
    // robot name, flag simulator
    // 
    //--------------------------------------------------------------------------------------------------------
    
    // Second Block
    //-------------------------------------------------------------------
    // Here we wil compute Jacobians and other useful matrices
    //
    //-------------------------------------------------------------------
    
    
    //Third Block
    //-------------------------------------------------------------------
    //Sent the results trought ports
    
}    
