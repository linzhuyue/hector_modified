#include "../../include/common/LowLevelController.h"
#include <eigen3/Eigen/Core>

// upper level of joint controller 
// send data to joint controller
// using namespace pinocchio;
void LowLevelControllerCommand::zero(){
    tau = Vec5<double>::Zero();
    qDes = Vec5<double>::Zero();
    qdDes = Vec5<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    hiptoeforce = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat5<double>::Zero();
    kdJoint = Mat5<double>::Zero();
    double kptoe = 0;
    double kdtoe = 0;
}

/*!
 * Zero leg data
 */ 
void LowLevelControllerData::zero(){
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J_force_moment = Mat65<double>::Zero();
    J_force = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}

void LowLevelController::zeroCommand(){
    for (int i = 0; i < 2; i++){
        commands[i].zero();
    }
}

void LowLevelController::updateData(const LowlevelState* state){
    for (int leg = 0; leg < 2; leg++){
        for(int j = 0; j < 5; j++){
            data[leg].q(j) = state->motorState[leg*5+j].q;
            data[leg].qd(j) = state->motorState[leg*5+j].dq;
            data[leg].tau(j) = state->motorState[leg*5+j].tauEst;
            // std::cout << "motor joint data" << leg*5+j << ": "<< data[leg].q(j) << std::endl;
        }

        // computeLegJacobianAndPosition(_biped, data[leg].q, &(data[leg].J_force_moment), &(data[leg].J_force), &(data[leg].p), leg);
        computeLegJacobianAndPositionPIno(_biped, data[leg].q, &(data[leg].J_force_moment), &(data[leg].J_force), &(data[leg].p), leg);
        
        data[leg].v = data[leg].J_force * data[leg].qd;
    }

}

void LowLevelController::updateCommand(LowlevelCmd* cmd){

    for (int i = 0; i < 2; i++){
        Vec6<double> footForce = commands[i].feedforwardForce;
        Vec5<double> legtau = data[i].J_force_moment.transpose() * footForce; // force moment from stance leg
        
        for(int j = 0; j < 5; j++){
            std::cout << "legtau" << j << ": "<< legtau(j) << std::endl;
        }

        // cartesian PD control for swing foot
        if(commands[i].kpCartesian(0,0) != 0 || commands[i].kdCartesian(0,0) != 0)
        {
            Vec3<double> footForce_3d = commands[i].kpCartesian * (commands[i].pDes - data[i].p) +
                                        commands[i].kdCartesian * (commands[i].vDes - data[i].v);
          
            Vec5<double> swingtau = data[i].J_force.transpose() * footForce_3d ;

            // maintain hip angle tracking
            double kphip1 = 15;
            double kdhip1 = 1;
            swingtau(0) = kphip1*(0-data[i].q(0)) + kdhip1*(0-data[i].qd(0));
            // make sure foot is parallel with the ground
            swingtau(4) = commands[i].kptoe * (-data[i].q(3)-data[i].q(2)-data[i].q(4))+commands[i].kdtoe*(0-data[i].qd(4));

            for(int j = 0; j < 5; j++)
            {
                legtau(j) += swingtau(j);
            }
        }

        commands[i].tau += legtau;

        for (int j = 0; j < 5; j++){
            cmd->motorCmd[i*5+j].tau = commands[i].tau(j);
            cmd->motorCmd[i*5+j].q = commands[i].qDes(j);
            cmd->motorCmd[i*5+j].dq = commands[i].qdDes(j);
            cmd->motorCmd[i*5+j].Kp = commands[i].kpJoint(j,j);
            cmd->motorCmd[i*5+j].Kd = commands[i].kdJoint(j,j);
            std::cout << Side[i] << " " << limbName[j] <<" torque cmd  =  " << cmd->motorCmd[i*5+j].tau << std::endl;            
        }
        
        //To access left  arm controllers use motorCmd 10 through 12
        //To access right arm controllers use motorCmd 13 through 15

        commands[i].tau << 0, 0, 0, 0, 0; // zero torque command to prevent interference
        
        
   
    }
    //std::cout << "cmd sent" << std::endl;
   
}
void LowLevelController::computeFootRotationPIno(Vec10<float>& q, Mat3<float>* R_foot_L, Mat3<float>* R_foot_R){
    Eigen::VectorXd qq =  Eigen::VectorXd::Zero(model.nv);
    for (int i = 0; i < 10; ++i) {
        if (i<5)
        {
            qq(i) = q(i);
        }else{
            qq(i + 3) = q(i);
        }   
    }
    pinocchio::Data data(model);
    framesForwardKinematics(model, data, qq);
    updateFramePlacements(model,data);
    // Use placements directly from data.oMf
    const pinocchio::SE3 &placementL = data.oMf[frame_idL_foot];
    const pinocchio::SE3 &placementR = data.oMf[frame_idR_foot];
    *R_foot_L = placementL.rotation().template cast<float>();
    *R_foot_R = placementR.rotation().template cast<float>();
}
void LowLevelController::computeLegJacobianAndPositionPIno(Biped& _biped, Vec5<double>& q, Mat65<double>* J_f_m, Mat35<double>* J_f, 
                                       Vec3<double>* p, int leg)
{

  Eigen::VectorXd qq =Eigen::VectorXd::Zero(joint_num);
  pinocchio::Data::Matrix6x J(6,joint_num);
  J.setZero(); 
  pinocchio::Data::Matrix6x JJ(6,5);
  JJ.setZero(); 
  pinocchio::Data data(model);
  Eigen::Matrix4d T_B_RHIP,T_B_toe;
  T_B_RHIP.setIdentity();T_B_toe.setIdentity();
  Eigen::Matrix4d T_B_inv;
  Eigen::VectorXd foottrans =Eigen::VectorXd::Zero(3);
    if (leg==0)//left
    {
        for (int i = 0; i < 5; i++)
        {
            qq(i) = q(i);
        }
        // std::cout << "L q: " << q.transpose()<< std::endl;
        // std::cout << "L qq: " << qq.transpose()<< std::endl;
        framesForwardKinematics(model,data,qq);
        updateFramePlacements(model,data); 
        pinocchio::computeFrameJacobian(model,data,qq,frame_idL_foot,pinocchio::LOCAL_WORLD_ALIGNED,J);
        // framesForwardKinematics(model,data,qq);
        const Frame &frame = model.frames[frame_idL_foot];
        const SE3 &placement = data.oMi[frame.parent];
        const Frame &framehip = model.frames[model.getFrameId("L_hip")];
        const SE3 &placementhip = data.oMi[framehip.parent];
        T_B_RHIP.block(0,0,3,3)=placementhip.rotation();
        T_B_RHIP.block(0,3,3,1)=placementhip.translation();
        T_B_toe.block(0,0,3,3)=placement.rotation();
        T_B_toe.block(0,3,3,1)=placement.translation();
        T_B_inv.setIdentity();
        T_B_inv.block(0,0,3,3)=T_B_RHIP.block(0,0,3,3).transpose();
        T_B_inv.block(0,3,3,1)=-T_B_RHIP.block(0,0,3,3).transpose()*T_B_RHIP.block(0,3,3,1);
        foottrans = (T_B_inv*T_B_toe).block(0,3,3,1);
        foottrans(2)+=0.03;

        foottrans = placement.translation();
        foottrans(2)+=0.03;
        JJ = J.block(0,0,6,5);
    }else{ //right
        for (int i = 8; i < 13; i++)
        {
            qq(i) = q(i-8);
        }
        // std::cout << "R q: " << q.transpose()<< std::endl;
        // std::cout << "R qq: " << qq.transpose()<< std::endl;
        framesForwardKinematics(model,data,qq);
        updateFramePlacements(model,data); 
        pinocchio::computeFrameJacobian(model,data,qq,frame_idR_foot,pinocchio::LOCAL_WORLD_ALIGNED,J);
        
        const Frame &frame = model.frames[frame_idR_foot];
        const SE3 &placement = data.oMi[frame.parent];
        const Frame &framehip = model.frames[model.getFrameId("R_hip")];
        const SE3 &placementhip = data.oMi[framehip.parent];
        T_B_RHIP.block(0,0,3,3)=placementhip.rotation();
        T_B_RHIP.block(0,3,3,1)=placementhip.translation();
        T_B_toe.block(0,0,3,3)=placement.rotation();
        T_B_toe.block(0,3,3,1)=placement.translation();
        T_B_inv.setIdentity();
        T_B_inv.block(0,0,3,3)=T_B_RHIP.block(0,0,3,3).transpose();
        T_B_inv.block(0,3,3,1)=-T_B_RHIP.block(0,0,3,3).transpose()*T_B_RHIP.block(0,3,3,1);
        foottrans = (T_B_inv*T_B_toe).block(0,3,3,1);
        foottrans(2)+=0.03;
        foottrans = placement.translation();
        foottrans(2)+=0.03;
        JJ = J.block(0,8,6,5);
    }
    // std::cout << "Out qq: " << qq.transpose()<< std::endl;
    // std::cout<<"foottrans: "<<foottrans.transpose()<<std::endl;
    // std::cout<<"JJ:\n"<<JJ<<std::endl;
    if(J_f_m){
        J_f_m->operator()(0, 0) =  JJ(0,0);
        J_f_m->operator()(1, 0) =  JJ(1,0);
        J_f_m->operator()(2, 0) =  0.0;
        J_f_m->operator()(3, 0) =  0.0;
        J_f_m->operator()(4, 0) =  0.0;
        J_f_m->operator()(5, 0) =  1.0;

        J_f_m->operator()(0, 1) =  JJ(0,1);
        J_f_m->operator()(1, 1) =  JJ(1,1);
        J_f_m->operator()(2, 1) =  JJ(2,1);
        J_f_m->operator()(3, 1) =  JJ(3,1);
        J_f_m->operator()(4, 1) =  JJ(4,1);
        J_f_m->operator()(5, 1) = 0.0;

        J_f_m->operator()(0, 2) =  JJ(0,2);
        J_f_m->operator()(1, 2) =  JJ(1,2);
        J_f_m->operator()(2, 2) =  JJ(2,2);
        J_f_m->operator()(3, 2) =  JJ(3,2);
        J_f_m->operator()(4, 2) =  JJ(4,2);
        J_f_m->operator()(5, 2) =  JJ(5,2);

        J_f_m->operator()(0, 3) =  JJ(0,3);
        J_f_m->operator()(1, 3) =  JJ(1,3);
        J_f_m->operator()(2, 3) =  JJ(2,3);
        J_f_m->operator()(3, 3) =  JJ(3,3);
        J_f_m->operator()(4, 3) =  JJ(4,3);
        J_f_m->operator()(5, 3) =  JJ(5,3);

        J_f_m->operator()(0, 4) =  JJ(0,4);
        J_f_m->operator()(1, 4) =  JJ(1,4);
        J_f_m->operator()(2, 4) =  JJ(2,4);
        J_f_m->operator()(3, 4) =  JJ(3,4);
        J_f_m->operator()(4, 4) =  JJ(4,4);
        J_f_m->operator()(5, 4) =  JJ(5,4);
    }
      if(J_f){
        J_f->operator()(0, 0) =  JJ(0,0);
        J_f->operator()(1, 0) =  JJ(1,0);
        J_f->operator()(2, 0) =  0.0;

        J_f->operator()(0, 1) =  JJ(0,1);
        J_f->operator()(1, 1) =  JJ(1,1);
        J_f->operator()(2, 1) =  JJ(2,1);

        J_f->operator()(0, 2) =  JJ(0,2);
        J_f->operator()(1, 2) =  JJ(1,2);
        J_f->operator()(2, 2) =  JJ(2,2);

        J_f->operator()(0, 3) =  JJ(0,3);
        J_f->operator()(1, 3) =  JJ(1,3);
        J_f->operator()(2, 3) =  JJ(2,3);

        J_f->operator()(0, 4) =  JJ(0,4);
        J_f->operator()(1, 4) =  JJ(1,4);
        J_f->operator()(2, 4) =  JJ(2,4);
      }
        // p in hip joint frame not in body frame
    if(p){
        p->operator()(0) = foottrans(0);
        p->operator()(1) = foottrans(1);
        p->operator()(2) = foottrans(2);
    }
}
// 
// J_f_m: J_force_moment
void computeLegJacobianAndPosition(Biped& _biped, Vec5<double>& q, Mat65<double>* J_f_m, Mat35<double>* J_f, 
                                       Vec3<double>* p, int leg)
{
    q(2) = q(2) + 0.3*3.14159;
    q(3) = q(3) - 0.6*3.14159;
    q(4) = q(4) + 0.3*3.14159;

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);

    
    double side = -1.0; // 1 for Left legs; -1 for right legs
    // TODO different from matlab side = -1 is left in matlab
    // the C code is right Matlab code is in another direction
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        side = 1.0;
    }
    // std::cout<< "Leg Sign" << side << std::endl;
    

    if(J_f_m){
    J_f_m->operator()(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    J_f_m->operator()(2, 0) =  0.0;
    J_f_m->operator()(3, 0) = 0.0;
    J_f_m->operator()(4, 0) = 0.0;
    J_f_m->operator()(5, 0) = 1.0;

    J_f_m->operator()(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f_m->operator()(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);
    J_f_m->operator()(3, 1) = cos(q0);
    J_f_m->operator()(4, 1) = sin(q0);
    J_f_m->operator()(5, 1) = 0.0;

    J_f_m->operator()(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    J_f_m->operator()(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f_m->operator()(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f_m->operator()(3, 2) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 2) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 2) = sin(q1);

    J_f_m->operator()(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    J_f_m->operator()(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f_m->operator()(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f_m->operator()(3, 3) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 3) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 3) = sin(q1);

    J_f_m->operator()(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    J_f_m->operator()(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f_m->operator()(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    J_f_m->operator()(3, 4) = -cos(q1)*sin(q0);
    J_f_m->operator()(4, 4) = cos(q0)*cos(q1);
    J_f_m->operator()(5, 4) = sin(q1);
   }

   if(J_f){
    J_f->operator()(0, 0) =  sin(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135) + cos(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(1, 0) =  sin(q0)*(0.015*side + cos(q1)*(0.018*side + 0.0025) - 1.0*sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2))) - 1.0*cos(q0)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2) + 0.0135);
    J_f->operator()(2, 0) =  0.0;

    J_f->operator()(0, 1) =  -1.0*sin(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(1, 1) =  cos(q0)*(sin(q1)*(0.018*side + 0.0025) + cos(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)));
    J_f->operator()(2, 1) =  sin(q1)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q1)*(0.018*side + 0.0025);

    J_f->operator()(0, 2) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2));
    J_f->operator()(1, 2) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3) + 0.22*cos(q2)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));
    J_f->operator()(2, 2) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3) + 0.22*sin(q2));

    J_f->operator()(0, 3) =  sin(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3)) - 1.0*cos(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3));
    J_f->operator()(1, 3) =  - 1.0*sin(q0)*(0.04*cos(q2 + q3 + q4) + 0.22*cos(q2 + q3)) - 1.0*cos(q0)*sin(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));
    J_f->operator()(2, 3) =  cos(q1)*(0.04*sin(q2 + q3 + q4) + 0.22*sin(q2 + q3));

    J_f->operator()(0, 4) =  0.04*sin(q2 + q3 + q4)*sin(q0)*sin(q1) - 0.04*cos(q2 + q3 + q4)*cos(q0);
    J_f->operator()(1, 4) =  - 0.04*cos(q2 + q3 + q4)*sin(q0) - 0.04*sin(q2 + q3 + q4)*cos(q0)*sin(q1);
    J_f->operator()(2, 4) =  0.04*sin(q2 + q3 + q4)*cos(q1);
    
    }
    // p in hip joint frame not in body frame
   if(p){
    p->operator()(0) = - (3*cos(q0))/200 - (9*sin(q4)*(cos(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))))/250 - (11*cos(q0)*sin(q2))/50 - ( (side)*sin(q0))/50 - (11*cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)))/50 - (11*sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2))))/250 - (23*cos(q1)* (side)*sin(q0))/1000 - (11*cos(q2)*sin(q0)*sin(q1))/50;
    p->operator()(1) = (cos(q0)* (side))/50 - (9*sin(q4)*(cos(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1))))/250 - (3*sin(q0))/200 - (11*sin(q0)*sin(q2))/50 - (11*cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)))/50 - (11*sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2))))/250 + (23*cos(q0)*cos(q1)* (side))/1000 + (11*cos(q0)*cos(q2)*sin(q1))/50;
    p->operator()(2) = (23*(side)*sin(q1))/1000 - (11*cos(q1)*cos(q2))/50 - (9*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/250 + (9*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/250 - (11*cos(q1)*cos(q2)*cos(q3))/50 + (11*cos(q1)*sin(q2)*sin(q3))/50 - 3.0/50.0;
   }
    std::cout<<"leg:"<<leg << "Out qq: " << q.transpose()<< std::endl;
    std::cout<<"J_f_m: "<<J_f_m<<std::endl;
    std::cout<<"P:\n"<<p<<std::endl;
}
