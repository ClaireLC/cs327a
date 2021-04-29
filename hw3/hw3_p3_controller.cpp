#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}


// Convenience functions for converting degrees to radians and vice-versa
#define DEG2RAD(deg) ((double)(deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((double)(rad) * 180.0 / M_PI)

#define SQRT2 0.707107

// Location of URDF files specifying world and robot information
const string world_file = "./resources/world_hw3_p3.urdf";
const string robot_file = "../../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA"; 
const string ee_link_name = "link6";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read (from simulation/robot):
const std::string JOINT_ANGLES_KEY  = "cs327a::hw3::robot::Kuka-IIWA::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs327a::hw3::robot::Kuka-IIWA::sensors::dq";

// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs327a::hw3::robot::Kuka-IIWA::actuators::fgc";

int main(int argc, char** argv) {
	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	RedisClient redis_client = RedisClient();
	redis_client.connect();

    // set up redis read + write callbacks
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

    // load robots, read current state and update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();

    // end effector tip position in link frame
    Eigen::Vector3d ee_pos_local;
    ee_pos_local << 0.0, 0.0, 0.0;

    // final control torques to send to the robot
    VectorXd command_torques = VectorXd::Zero(robot->dof());

    // Suggested controller variables
    Eigen::VectorXd g(robot->dof()); //joint space gravity vector
    Eigen::MatrixXd J0(6, robot->dof()); //end effector basic Jacobian
    Eigen::MatrixXd L0(6, 6); //Lambda_0 at end effector
    Eigen::VectorXd p_bar(6); //gravity vector at end-effector
    const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
    const Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3); // 3 x 3 identity matrix
    Eigen::MatrixXd J_bar(robot->dof(), 6);  //A-weighted generalized inverse of J0
    Eigen::MatrixXd N_bar(robot->dof(), robot->dof()); //I - Jbar*J0, null space projection matrix for Jbar
    Eigen::Vector3d ee_pos; //end effector position
    Eigen::Matrix3d ee_rot_mat; //end effector rotation
    robot->rotation(ee_rot_mat, ee_link_name); // initialize
    Eigen::Quaterniond ee_rot_lambda(ee_rot_mat); // end effector rotation in quaternion form
    Eigen::VectorXd ee_error(6); //end effector operational space instantaneous error
    Eigen::VectorXd v0(6); //end effector velocity
    Eigen::MatrixXd select_motion(6,6); // selection matrix for motion, Omega
    Eigen::MatrixXd select_forces(6,6); // selection matrix for forces, Omega_bar
    Eigen::VectorXd F_des(6); // desired end effector force

    // suggested starting gains
    double op_task_kp = 50; //operational space task proportional gain
    double op_task_kv = 20; //operational space task velocity gain
    double joint_damping_kv = 20; // joint damping velocity gain

    // Additional controller variables
    VectorXd q(robot->dof());
    VectorXd dq(robot->dof());
    Vector3d ee_pos_des; // end effector desired position
    VectorXd v0_des(6); //end effector desired velocity
    VectorXd dv0_des(6);  //end effector desired acceleration
    MatrixXd E_base(4,3);      // E base matrix (unscaled)
    MatrixXd E(4,3);           // E
    MatrixXd E_inv(3,4);       // E inverse
    VectorXd ee_pos_error(3);  // ee position error
    Quaterniond ee_quat_des;   // end effector desired quaternion
    Vector3d ee_ori_error;     // delta_phi
    VectorXd ee_dquat_des(4);  // desired quaternion first derivative
    VectorXd ee_ddquat_des(4); // desired quaternion second derivative
    VectorXd ee_vang_des(3);   // desired angular velocity
    VectorXd ee_dvang_des(3);  // desired angular acceleration

    VectorXd F_star(6);
    VectorXd F_motion(6);
    VectorXd F_force(6);
    VectorXd F_total(6);
    MatrixXd A(robot->dof(), robot->dof());

    Eigen::MatrixXd sigma_f(3,3); // motion specification matrix, linear
    Eigen::MatrixXd sigma_m(3,3); // motion specification matrix, angular
    sigma_f << 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0;
    sigma_m << 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0,
               0.0, 0.0, 0.0;

    Eigen::MatrixXd sigma_bar_f(3,3); // force specification matrix
    Eigen::MatrixXd sigma_bar_m(3,3); // moment specification matrix
    sigma_bar_f = I3 - sigma_f; // force specification matrix
    sigma_bar_m = I3 - sigma_m; // moment specification matrix

    VectorXd joint_damping(6);

    // initialize redis keys

    // prepare redis read callback
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    // prepare redis write callback
    redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// create a loop timer
    double control_freq = 200;
    LoopTimer timer;
    timer.setLoopFrequency(control_freq);
    double last_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;
    timer.initializeTimer(2000);
    double start_time = timer.elapsedTime();

	unsigned long long counter = 0;

    cout << "Starting control loop" << endl;

	runloop = true;
	while (runloop) 
	{ 
		fTimerDidSleep = timer.waitForNextLoop();
        double curr_time = timer.elapsedTime() - start_time;
        double loop_dt = curr_time - last_time;

        // read controller parameters from redis
        redis_client.executeReadCallback(0);

        // update robot model
        robot->updateModel();

        // reset command torques
        command_torques.setZero();

		/* ------------------------------------------------------------------------------------
            FILL ME IN: set joint torques
        -------------------------------------------------------------------------------------*/

        // Get current EEF position, rotation, and velocity
        robot->position(ee_pos, ee_link_name, ee_pos_local);
        robot->rotation(ee_rot_mat, ee_link_name);
        Quaterniond quat(ee_rot_mat); // end effector rotation in quaternion form
        robot->velocity6d(v0, ee_link_name, ee_pos_local);
        q = robot->_q;
        dq = robot->_dq;

        // Update dynamics quantities
        A = robot->_M; // Mass matrix
        robot->gravityVector(g); // Update gravity vectory
        robot->J_0(J0,ee_link_name,ee_pos_local); // Full Jacobian
        L0 = (J0 * A.inverse() * J0.transpose()).inverse();

        // Compute J_bar and N_bar
        J_bar = A.inverse() * J0.transpose() * L0;
        p_bar = J_bar.transpose() * g;
        N_bar = In - J_bar * J0;

        // Set E and E_inv matrices
        E_base << -quat.x(), -quat.y(), -quat.z(),
              quat.w(),  quat.z(), -quat.y(),
              -quat.z(), quat.w(), quat.x(),
               quat.y(), -quat.x(), quat.w();
        E = 0.5 * E_base;
        E_inv = 2.0 * E_base.transpose();

        // Compute selection matrices
        select_motion.topLeftCorner(3,3) = sigma_f;
        select_motion.bottomRightCorner(3,3) = sigma_m;
        select_forces.topLeftCorner(3,3) = sigma_bar_f;
        select_forces.bottomRightCorner(3,3) = sigma_bar_m;

        // --------------------------------------------------------------------
        // (2) Compute desired operational space trajectory values and errors 
        //---------------------------------------------------------------------
        // Desired position and quaternion
        ee_pos_des(0) = 0;
        ee_pos_des(1) = 0.5 + 0.1 * cos(0.4 * M_PI * curr_time);
        ee_pos_des(2) = 0.65 - 0.05 * cos(0.8 * M_PI * curr_time);
        ee_quat_des.w() = (1/SQRT2) * sin(0.25 * M_PI * cos(0.4 * M_PI * curr_time));
        ee_quat_des.x() = (1/SQRT2) * sin(0.25 * M_PI * cos(0.4 * M_PI * curr_time));
        ee_quat_des.y() = (1/SQRT2) * cos(0.25 * M_PI * cos(0.4 * M_PI * curr_time));
        ee_quat_des.z() = (1/SQRT2) * cos(0.25 * M_PI * cos(0.4 * M_PI * curr_time));
        
        ee_pos_error = ee_pos - ee_pos_des;
        Sai2Model::orientationError(ee_ori_error, ee_quat_des, quat);
    
        // Compute ee pose error
        ee_error(0) = ee_pos_error(0);
        ee_error(1) = ee_pos_error(1);
        ee_error(2) = ee_pos_error(2);
        ee_error(3) = ee_ori_error(0);
        ee_error(4) = ee_ori_error(1);
        ee_error(5) = ee_ori_error(2);
        //cout << "EE pos error" << endl;
        //cout << ee_error << endl;

        // Desired velocity
        v0_des(0) = 0;
        v0_des(1) = (-0.2/5) * M_PI * sin(0.4 * M_PI * curr_time);
        v0_des(2) = (0.2/5) * M_PI * sin(0.8 * M_PI * curr_time);
        ee_dquat_des(0) = M_PI*M_PI*(-1/SQRT2)*(0.1)*sin(0.4*M_PI*curr_time)*
                            cos(0.25*M_PI*cos(0.4*M_PI*curr_time));
        ee_dquat_des(1) = M_PI*M_PI*(1/SQRT2)*(0.1)*sin(0.4*M_PI*curr_time)*
                            sin(0.25*M_PI*cos(0.4*M_PI*curr_time));
        ee_dquat_des(2) = M_PI*M_PI*(-1/SQRT2)*(0.1)*sin(0.4*M_PI*curr_time)*
                            cos(0.25*M_PI*cos(0.4*M_PI*curr_time));
        ee_dquat_des(3) = M_PI*M_PI*(1/SQRT2)*(0.1)*sin(0.4*M_PI*curr_time)*
                            sin(0.25*M_PI*cos(0.4*M_PI*curr_time));
        // multiply by E_inv to get desired angular velocity
        ee_vang_des = E_inv * ee_dquat_des;
        v0_des(3) = ee_vang_des(0);
        v0_des(4) = ee_vang_des(1);
        v0_des(5) = ee_vang_des(2);
        //cout << "EE v_des" << endl;
        //cout << v0_des << endl;

        // Desired acceleration
        dv0_des(0) = 0;
        dv0_des(1) = (-0.4*M_PI*M_PI/25) * cos(0.4*M_PI*curr_time);
        dv0_des(2) = (-0.8*M_PI*M_PI/25) * cos(0.8*M_PI*curr_time);
        ee_ddquat_des(0) = (-1/SQRT2)*0.01*M_PI*M_PI*M_PI*(
                          4*cos(0.4*M_PI*curr_time)*cos(0.25*M_PI*cos(0.4*M_PI*curr_time)) + 
                          M_PI*sin(0.4*M_PI*curr_time)*sin(0.4*M_PI*curr_time)*
                          sin(0.25*M_PI*cos(0.4*M_PI*curr_time))
                          );
        ee_ddquat_des(1) = (-1/SQRT2)*0.01*M_PI*M_PI*M_PI*(
                          -4*cos(0.4*M_PI*curr_time)*sin(0.25*M_PI*cos(0.4*M_PI*curr_time)) + 
                          M_PI*sin(0.4*M_PI*curr_time)*sin(0.4*M_PI*curr_time)*
                          cos(0.25*M_PI*cos(0.4*M_PI*curr_time))
                          );
        ee_ddquat_des(2) = (-1/SQRT2)*0.01*M_PI*M_PI*M_PI*(
                          4*cos(0.4*M_PI*curr_time)*cos(0.25*M_PI*cos(0.4*M_PI*curr_time)) + 
                          M_PI*sin(0.4*M_PI*curr_time)*sin(0.4*M_PI*curr_time)*
                          sin(0.25*M_PI*cos(0.4*M_PI*curr_time))
                          );
        ee_ddquat_des(3) = (-1/SQRT2)*0.01*M_PI*M_PI*M_PI*(
                          -4*cos(0.4*M_PI*curr_time)*sin(0.25*M_PI*cos(0.4*M_PI*curr_time)) + 
                          M_PI*sin(0.4*M_PI*curr_time)*sin(0.4*M_PI*curr_time)*
                          cos(0.25*M_PI*cos(0.4*M_PI*curr_time))
                          );
        // multiply by E_inv to get desired angular acceleration
        ee_dvang_des = E_inv * ee_ddquat_des;
        dv0_des(3) = ee_dvang_des(0);
        dv0_des(4) = ee_dvang_des(1);
        dv0_des(5) = ee_dvang_des(2);

        // Desired forces
        F_des << 10, 0, 0, 0, 0, 0;
        
        // ---------------------------------------------------------------------------------
        // (3) Compute joint torques
        //----------------------------------------------------------------------------------
        F_star = dv0_des - op_task_kv*(v0 - v0_des) - op_task_kp*ee_error;

        F_motion = L0 * select_motion * F_star + p_bar;
        F_force = L0 * select_forces * (-op_task_kp * v0) + F_des;
        F_total = F_motion + F_force;

        cout << "F_motion" << endl;
        cout << F_motion.transpose() << endl;
        cout << "F_force" << endl;
        cout << F_force.transpose() << endl;
        cout << "F_total" << endl;
        cout << F_total.transpose() << endl;

        joint_damping = N_bar.transpose() * (A * -joint_damping_kv * dq + g);

        command_torques = J0.transpose() * F_total + joint_damping;

       
		/* ------------------------------------------------------------------------------------
            END OF FILL ME IN
        -------------------------------------------------------------------------------------*/

        // send torques to redis
        redis_client.executeWriteCallback(0);

		counter++;
		last_time = curr_time;
	}

    command_torques.setZero();
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

	double end_time = timer.elapsedTime();
    cout << "\n";
    cout << "Control Loop run time  : " << end_time << " seconds\n";
    cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
