#include "unitree_joint_control_tool.h"

double clamp(double& val, double min_val, double max_val)
{
  if ( val < min_val )
    val = min_val;
  if ( max_val < val )
    val = max_val;
  return min_val;
}
double computeVel(double current_position, double last_position, double last_velocity, double duration)
{
  double filter=0.35; 
  return (current_position - last_position) *(1-filter) / duration + last_velocity * filter;
}
double  computeTorque(double current_position, double current_velocity, ServoCmd& cmd)
{
  double pos = cmd.pos;
  double posStiffness = cmd.posStiffness;
  double velStiffness = cmd.velStiffness;
  double torque = cmd.torque;
  double d_value = std::abs(cmd.vel - velStopF);
  if ( std::abs(pos - posStopF) < RESOLUTION )
    posStiffness = 0.0;
  double targetVel = cmd.vel;
  if ( d_value < RESOLUTION )
    velStiffness = 0.0;
  clamp(targetVel, -MAX_JVEL, MAX_JVEL);
  double calcTorque = (targetVel - current_velocity) * velStiffness + (pos - current_position) * posStiffness + torque;
  clamp(calcTorque, -MAX_TAU, MAX_TAU);
  return calcTorque;
}