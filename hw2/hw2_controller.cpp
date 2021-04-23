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
const string world_file = "./resources/world_hw2.urdf";
const string robot_file = "../../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA"; 
const string ee_link_name = "link6";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read (from simulation/robot):
const std::string JOINT_ANGLES_KEY  = "cs327a::robot::Kuka-IIWA::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs327a::robot::Kuka-IIWA::sensors::dq";
// - read (from user defined task space control):
const std::string TASK_KP_GAINS_KEY = "cs327a::robot::Kuka-IIWA::inputs::op_task_kp";
const std::string TASK_KV_GAINS_KEY = "cs327a::robot::Kuka-IIWA::inputs::op_task_kv";

// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "cs327a::robot::Kuka-IIWA::actuators::fgc";
const std::string PLOT_CURRENT_EE_POSITION_KEY = "cs327a::plot::Kuka-IIWA::sensors::ee_pos";
const std::string PLOT_DESIRED_EE_POSITION_KEY = "cs327a::plot::Kuka-IIWA::inputs::ee_pos_des";

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

    // Suggested controller variables
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd g(robot->dof()); //joint space gravity vector
    MatrixXd J0(6, robot->dof()); //end effector Basic Jacobian
    MatrixXd L_hat(6, 6); //Kinetic Energy Matrix at end effector (op. space)
    VectorXd p_hat(6); //gravity vector at end-effector (op. space)
    const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
    Eigen::MatrixXd J_pseudo(robot->dof(), 6);  //Right Psuedoinverse of J0
    Eigen::MatrixXd _JJT(robot->dof(), 6);  //Right Psuedoinverse of J0
    Eigen::MatrixXd N_proj(robot->dof(), robot->dof()); //I - J_pseudo*J0, null space projection matrix
    Vector3d ee_pos; //end effector position
    Matrix3d ee_rot_mat; //end effector rotation, in matrix form
    robot->rotation(ee_rot_mat, ee_link_name); // initialize the end effector rotation
    Quaterniond ee_rot_lambda(ee_rot_mat); // end effector rotation in quaternion form
    VectorXd v0(6); //end effector velocity
    Vector3d ee_pos_des; // end effector desired position
    VectorXd v0_des(6); //end effector desired velocity
    VectorXd dv0_des(6);  //end effector desired acceleration
    VectorXd ee_error(6); //ee operational space instantaneous error (position, orientation)

    // Additional controller variables
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
    VectorXd F(6);
    MatrixXd A;

    VectorXd joint_damping(6);

    // suggested starting gains
    double op_task_kp = 50; //operational space task proportional gain
    double op_task_kv = 20; //operational space task velocity gain
    double damping_kv = 20; // joint damping velocity gain

    // Position of Task Frame in relation to Link Frame
    Vector3d ee_pos_in_link = Vector3d(0.0, 0.0, 0.0);

    // initialize redis keys

    // op task
    // redis_client.setEigenMatrixJSON(DESIRED_TASK_POSITION_KEY, desired_position);
    redis_client.set(TASK_KP_GAINS_KEY, std::to_string(op_task_kp));
    redis_client.set(TASK_KV_GAINS_KEY, std::to_string(op_task_kv));

    // prepare redis read callback
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.addDoubleToReadCallback(0, TASK_KP_GAINS_KEY, op_task_kp);
    redis_client.addDoubleToReadCallback(0, TASK_KV_GAINS_KEY, op_task_kv);

    // prepare redis write callback
    redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);
    redis_client.addEigenToWriteCallback(0, PLOT_CURRENT_EE_POSITION_KEY, ee_pos);
    redis_client.addEigenToWriteCallback(0, PLOT_DESIRED_EE_POSITION_KEY, ee_pos_des);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop
    double start_time = timer.elapsedTime();
  double current_time = timer.elapsedTime();
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
        robot->gravityVector(g);

        // reset command torques
        command_torques.setZero();

		/* ------------------------------------------------------------------------------------
            FILL ME IN: set joint torques
        -------------------------------------------------------------------------------------*/

        // ---------------------------------------------------------------------------------
        // (1) Update current robot configuration parameters and desired trajectory values 
        //----------------------------------------------------------------------------------

        // Get current EEF position, rotation, and velocity
        robot->position(ee_pos, ee_link_name, ee_pos_in_link);
        robot->rotation(ee_rot_mat, ee_link_name);
        Quaterniond q(ee_rot_mat); // end effector rotation in quaternion form
        robot->velocity6d(v0, ee_link_name, ee_pos_in_link);

        // Update more robot values
        robot->gravityVector(g); // Update gravity vectory
        robot->J_0(J0,ee_link_name,ee_pos_in_link); // Full Jacobian
        // TODO maybe I need to compute these by hand?
        robot->operationalSpaceMatrices(L_hat, J_pseudo, N_proj, J0);

        // Compute pseudo inverse
        _JJT = J0 * J0.transpose();
        J_pseudo = J0.transpose() * _JJT.inverse();

        cout << "Op space matrices" << endl;
        cout << J0 << endl;
        cout << L_hat << endl;
        cout << J_pseudo << endl;

        //// Set E and E_inv matrices
        E_base << -q.x(), -q.y(), -q.z(),
              q.w(),  q.z(), -q.y(),
              -q.z(), q.w(), q.x(),
               q.y(), -q.x(), q.w();
        E = 0.5 * E_base;
        E_inv = 2.0 * E_base.transpose();
        cout << "E matrices" << endl;
        cout << E_base << endl;
        cout << E_inv << endl;

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
        Sai2Model::orientationError(ee_ori_error, ee_quat_des, q);
    
        // Compute ee pose error
        ee_error(0) = ee_pos_error(0);
        ee_error(1) = ee_pos_error(1);
        ee_error(2) = ee_pos_error(2);
        ee_error(3) = ee_ori_error(0);
        ee_error(4) = ee_ori_error(1);
        ee_error(5) = ee_ori_error(2);
        cout << "EE pos error" << endl;
        cout << ee_error << endl;

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
        cout << "EE v_des" << endl;
        cout << v0_des << endl;

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
        cout << "EE dv_des" << endl;
        cout << dv0_des << endl;
        
        // ---------------------------------------------------------------------------------
        // (3) Compute joint torques
        //----------------------------------------------------------------------------------
        F_star = dv0_des - op_task_kv*(v0 - v0_des) - op_task_kp*ee_error;
        p_hat = J_pseudo.transpose() * g;

        //L_hat = MatrixXd::Identity(6,6); // Part d

        F = L_hat * F_star - p_hat;
        command_torques = J0.transpose() * F;

        // Part c) Joint damping
        A = robot->_M; // Mass matrix
        joint_damping = N_proj.transpose() * (A * -damping_kv * robot->_dq + g);
        command_torques = J0.transpose() * F + joint_damping;


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
