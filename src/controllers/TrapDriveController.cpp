#include "controllers/TrapDriveController.h"
#include "lib/TrapProfile.h"
#include "RobotInfo.h"
#include "lib/util/Util.h"

namespace frc973 {

using namespace Constants;

TrapDriveController::TrapDriveController(DriveStateProvider *state):
    m_state(state),
    m_dist(0.0),
    m_angle(0.0),
    m_dist_offset(0.0),
    m_angle_offset(0.0),
    m_time_offset(0.0),
    m_start_halt(true),
    m_end_halt(true),
    m_l_pos_pid(1.0, 0.0, 0.0),
    m_l_vel_pid(1.0, 0.0, 0.0),
    m_a_pos_pid(1.0, 0.0, 0.0),
    m_a_vel_pid(1.0, 0.0, 0.0),
    m_done(false),
    m_needSetControlMode(false) {
    ;
}

TrapDriveController::~TrapDriveController() {
    ;
}

void TrapDriveController::SetTarget(DriveBase::RelativeTo relativeTo,
        double dist, double angle,
        bool start_halt, bool end_halt) {
    m_time_offset = GetSecTime();

    switch (relativeTo) {
    case DriveBase::RelativeTo::Absolute:
        m_dist_offset = 0.0;
        m_angle_offset = 0.0;
        break;
    case DriveBase::RelativeTo::Now:
        m_dist_offset = m_state->GetDist();
        m_angle_offset = m_state->GetAngle();
        break;
    case DriveBase::RelativeTo::SetPoint:
        m_dist_offset += m_dist;
        m_angle_offset += m_angle;
        break;
    }

    m_dist = dist;
    m_angle = angle;

    m_start_halt = start_halt;
    m_end_halt = end_halt;
}

void TrapDriveController::CalcDriveOutput(DriveStateProvider *state,
        DriveControlSignalReceiver *out) {
	if(m_needSetControlMode == true){
		out->SetDriveControlMode(CANSpeedController::ControlMode::kSpeed);
		m_needSetControlMode = false;
	}

    double time = GetSecTime() - m_time_offset;

    Profiler::Waypoint goal = Profiler::TrapProfileUnsafe(time,
            m_dist, m_angle,
            MAX_VELOCITY, MAX_ACCELERATION,
            m_start_halt, m_end_halt);

    m_l_pos_pid.SetTarget(goal.linear_dist);
    m_l_vel_pid.SetTarget(goal.linear_vel);
    m_a_pos_pid.SetTarget(goal.angular_dist);
    m_a_vel_pid.SetTarget(goal.angular_vel);

    /* vel feed forward for linear term */
    double right_l_vel_ff = goal.linear_vel;
    double left_l_vel_ff = goal.linear_vel;

    /* vel feed forward for angular term */
    double right_a_vel_ff = DRIVE_WIDTH / 2.0 * RAD_PER_DEG * goal.angular_vel;
    double left_a_vel_ff = -right_a_vel_ff;

    /* correction terms for error in {linear,angular} {position,velocioty */
    double linear_dist_term = m_l_pos_pid.CalcOutput(DistFromStart());
    double linear_vel_term = m_l_vel_pid.CalcOutput(state->GetRate());
    double angular_dist_term = m_a_pos_pid.CalcOutput(AngleFromStart());
    double angular_vel_term = m_a_vel_pid.CalcOutput(state->GetAngularRate());


    /* right side receives positive angle correction */
    double right_output = right_l_vel_ff + right_a_vel_ff
         + linear_dist_term + linear_vel_term
         + angular_dist_term + angular_vel_term;
    /* left side receives negative angle correction */
    double left_output = left_l_vel_ff + left_a_vel_ff
         + linear_dist_term + linear_vel_term
         - angular_dist_term - angular_vel_term;

    out->SetDriveOutput(left_output, right_output);

    m_done = goal.done;
}

void TrapDriveController::Start() {
    m_needSetControlMode = true;
}

void TrapDriveController::Stop() {
}

double TrapDriveController::DistFromStart() {
    return m_state->GetDist() - m_dist_offset;
}

double TrapDriveController::AngleFromStart() {
    return m_state->GetAngle() - m_angle_offset;
}

}
