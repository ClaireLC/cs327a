#include <iostream>
#include <string>
#include <csignal>
#include <thread>
#include <chrono>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <dynamics3d.h>
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h> 

using namespace std;

const double time_slowdown_factor = 10;

constexpr const char *sim_title = "cs327a - HW 5 Problem 2";
const string world_file = "./resources/world_hw5_p2.urdf";
const string robot_file = "../../resources/puma/puma_gripper.urdf";
const string robot1_name = "Puma1";
const string robot2_name = "Puma2";
const string object_name = "CoordObject";
const string object_file = "./resources/object.urdf";
const string object_link_name = "object";
const string ee_link_name = "end-effector";
const string gripper_joint_name = "gripper";

const string camera_name = "camera_front";

RedisClient redis_client;

const std::string OBJECT_CARTESIAN_POSITION_KEY = "cs327a::hw5_p2::object::cartesian_pos";

/* ----------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/
// state machine setup
enum ControlMode {
	CONTROL_GRASP_STABILIZE = 0,
	CONTROL_AUGMENTED_OBJECT
};

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim);
void simulation(Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

/* =======================================================================================
   MAIN LOOP
========================================================================================== */
int main (int argc, char** argv) {

	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

 	// set up redis callbacks
    redis_client.createWriteCallback(0);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);

	// load robots
	auto robot1 = new Sai2Model::Sai2Model(robot_file, false);
	auto robot2 = new Sai2Model::Sai2Model(robot_file, false);

	// load object
	auto coobject = new Sai2Model::Sai2Model(object_file, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	// set co-efficient of restition to zero to avoid bounce
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(0.0);
    sim->setCoeffFrictionStatic(0.5);
    sim->setCoeffFrictionDynamic(0.5);

    // set joint damping on grippers: 
    auto base_1 = sim->_world->getBaseNode(robot1_name);
    auto gripper_1 = base_1->getJoint(gripper_joint_name);
    gripper_1->setDamping(10.0);
    gripper_1->setJointLimits(-0.005, 0.068, 0.005);
    auto base_2 = sim->_world->getBaseNode(robot2_name);
    auto gripper_2 = base_2->getJoint(gripper_joint_name);
    gripper_2->setDamping(10.0);
    gripper_2->setJointLimits(-0.005, 0.068, 0.005);

    // set initial conditions
	robot1->_q << 90/180.0*M_PI,
				-22.5/180.0*M_PI,
				212/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot1_name, robot1->_q);
	robot1->updateModel();
	robot2->_q << 90/180.0*M_PI,
				202.5/180.0*M_PI,
				-28/180.0*M_PI,
				-90.0/180.0*M_PI,
				97/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot2_name, robot2->_q);
	robot2->updateModel();
	Eigen::Affine3d ee_trans;

	// set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor *primary = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
    int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow *window = glfwCreateWindow(windowW, windowH, sim_title, NULL, NULL);
    glfwSetWindowPos(window, windowPosX, windowPosY);
    glfwShowWindow(window);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetKeyCallback(window, keySelect);

    // cache variables
    double last_cursorx, last_cursory;

	// start the simulation thread first
    fSimulationRunning = true;
	thread sim_thread(simulation, sim);

	// next start the control thread
	thread ctrl_thread(control, robot1, robot2, coobject, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot1_name, robot1);
		graphics->updateGraphics(robot2_name, robot2);
		graphics->updateGraphics(object_name, coobject);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

/* ----------------------------------------------------------------------------------
	Utility functions
-------------------------------------------------------------------------------------*/
// Calculate the cross product matrix
Eigen::Matrix3d getCrossProductMat(const Eigen::Vector3d& t) {
	Eigen::Matrix3d ret;
	ret <<  0, -t(2), t(1),
			t(2), 0, -t(0),
			-t(1), t(0), 0;
	return ret;
}

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim) {
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs

	// load robot global frame to robot base transformations: todo: move out to a function
	Eigen::Affine3d robot1_base_frame = sim->getRobotBaseTransform(robot1_name);
	Eigen::Affine3d robot2_base_frame = sim->getRobotBaseTransform(robot2_name);

	// cache variables
	bool fTimerDidSleep = true;
	bool fTorqueUseDefaults = false; // set true when torques are overriden for the first time
	Eigen::VectorXd tau1 = Eigen::VectorXd::Zero(robot1->dof());
	Eigen::VectorXd tau2 = Eigen::VectorXd::Zero(robot2->dof());

	Eigen::Affine3d object_com_frame;
	Eigen::Vector3d object_current_pos;
	Eigen::MatrixXd object_inertia(6,6);
	Eigen::MatrixXd object_j(6,6);
	Eigen::VectorXd object_p(6);

	Eigen::VectorXd robot1_g(robot1->dof());
	Eigen::VectorXd robot2_g(robot2->dof());

	Eigen::VectorXd robot1_p(6);
	Eigen::VectorXd robot2_p(6);

	// **  Other sugested variables and gains **
	Eigen::Vector3d object_com_in_robot1_ee_frame;
	Eigen::Vector3d object_com_in_robot2_ee_frame;
	Eigen::MatrixXd robot1_j0_objcom(6, robot1->dof());
	Eigen::MatrixXd robot2_j0_objcom(6, robot2->dof());
	Eigen::MatrixXd robot1_j0_objcom_bar(robot1->dof(), 6);
	Eigen::MatrixXd robot2_j0_objcom_bar(robot2->dof(), 6);
	Eigen::MatrixXd robot1_objcom_inertia(6,6);
	Eigen::MatrixXd robot2_objcom_inertia(6,6);
  Eigen::Affine3d robot1_base_to_ee; // Transformation from base frame to ee link frame
  Eigen::Affine3d robot2_base_to_ee; // Transformation from base frame to ee link frame

	Eigen::MatrixXd augmented_object_inertia(6,6); Eigen::VectorXd augmented_object_p(6);

	Eigen::MatrixXd G(2*6, 2*6);
	Eigen::MatrixXd W(6, 2*6);
	Eigen::MatrixXd W_f(6, 6);
	Eigen::MatrixXd W_m(6, 6);
  Eigen::MatrixXd E_bar(1,6);
  Eigen::MatrixXd e12(1,3); // Vector from r1 ee to r2 ee
  Eigen::MatrixXd Itilde(5,6);

  const Eigen::MatrixXd I3 = MatrixXd::Identity(3, 3); // 3 x 3 identity matrix
  const Eigen::MatrixXd I9 = MatrixXd::Identity(9,9);
  const Eigen::Matrix3d zero3 = MatrixXd::Zero(3,3);
  const Eigen::MatrixXd zero5_6 = MatrixXd::Zero(5,6);
  const Eigen::MatrixXd zero1_6 = MatrixXd::Zero(1,6);

  Eigen::Vector3d robot1_ee_pos_r1f; // robot1 ee position in robot1 base frame
  Eigen::Vector3d robot2_ee_pos_r2f; // robot2 ee position in robot2 base frame
  Eigen::Vector3d robot1_ee_pos_wf; // robot1 ee position in world frame
  Eigen::Vector3d robot2_ee_pos_wf; // robot2 ee position in world frame

  Eigen::Vector3d r_1; // Vector from com to r1 ee
  Eigen::Vector3d r_2; // Vector from com to r2 ee
  Eigen::Matrix3d r_1_hat; // cross product matrix r_1
  Eigen::Matrix3d r_2_hat; // cross product matrix of r_2

	Eigen::Vector3d obj_des_pos;
	Eigen::VectorXd obj_des_vel(6);
	Eigen::VectorXd obj_cur_vel(6);
	Eigen::VectorXd obj_des_acc(6);

	Eigen::Vector3d obj_ori_error;
	Eigen::VectorXd obj_task_err(6);
	Eigen::VectorXd force_des_vec(12);
	Eigen::VectorXd force_ee_vec(12);

  Eigen::VectorXd F_star(6);
  Eigen::VectorXd F_motion(6);
  Eigen::VectorXd robot1_force_ee(6);
  Eigen::VectorXd robot2_force_ee(6);

  Quaterniond obj_quat_des;   // object desired quaternion
  Quaterniond obj_quat_cur;   // object current quat

  // Desired internal forces and moments
  Vector3d int_f_des;
  Vector3d int_m_des;

	double kp = 3;
	double kv = 1;

	// ** Control Mode **
	// 		0 = grasp stabilizing controller
	//		1 = augmented object controller
	ControlMode control_mode = CONTROL_GRASP_STABILIZE; 

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time;

        // read joint positions, velocities
        sim->getJointPositions(robot1_name, robot1->_q);
        sim->getJointVelocities(robot1_name, robot1->_dq);
        robot1->updateModel();
        sim->getJointPositions(robot2_name, robot2->_q);
        sim->getJointVelocities(robot2_name, robot2->_dq);
        robot2->updateModel();

        // read object position
        sim->getJointPositions(object_name, object_model->_q);
        sim->getJointVelocities(object_name, object_model->_dq);
        object_model->updateModel();

        // update object dynamics
		// - find object COM frame in global frame
		object_model->transform(object_com_frame, object_link_name);
		object_current_pos << object_com_frame.translation();
		redis_client.addEigenToWriteCallback(0, OBJECT_CARTESIAN_POSITION_KEY, object_current_pos);
		// - obtain object inertia matrix in world frame
		object_model->J_0(object_j, object_link_name, Eigen::Vector3d::Zero());
		object_inertia = (object_j*object_model->_M_inv*object_j.transpose()).inverse();
		// - obtain object p
		object_p << 0, 0, 9.8*0.5, 0, 0, 0;

        // ---------------------------------------------------------------------------------
        /* ---------------------- FILL ME IN ----------------------------------------------- */
		
		// --------------------------------------------------------------------
		// (1) Manipulator Jacobians
		//---------------------------------------------------------------------
		// ** FILL ME IN **

    // Get transforms from base frame to world frame
	  Eigen::Affine3d robot1_base_frame = sim->getRobotBaseTransform(robot1_name);
	  Eigen::Affine3d robot2_base_frame = sim->getRobotBaseTransform(robot2_name);
    
    // Get transforms from base frame to ee link frame
    robot1->transform(robot1_base_to_ee, ee_link_name);
    robot2->transform(robot2_base_to_ee, ee_link_name);

    // Get object com in ee frame
    object_com_in_robot1_ee_frame = robot1_base_to_ee.inverse() * robot1_base_frame.inverse() * object_current_pos;
    object_com_in_robot2_ee_frame = robot2_base_to_ee.inverse() * robot2_base_frame.inverse() * object_current_pos;

    cout << "Object com in robot 1 ee frame" << endl;
    cout << object_com_in_robot1_ee_frame << endl;
    cout << "Object com in robot 2 ee frame" << endl;
    cout << object_com_in_robot2_ee_frame << endl;
    
    // Get Jacobians in eef frame at object com location
    robot1->J_0(robot1_j0_objcom, ee_link_name, object_com_in_robot1_ee_frame);
    robot2->J_0(robot2_j0_objcom, ee_link_name, object_com_in_robot2_ee_frame);

		// --------------------------------------------------------------------
		// (2) Augmented Object Model
		//---------------------------------------------------------------------
		// ** FILL ME IN **
    
    // Compute augmented object inertia
    robot1_objcom_inertia=(robot1_j0_objcom*(robot1->_M_inv)*robot1_j0_objcom.transpose()).inverse();
    robot2_objcom_inertia=(robot2_j0_objcom*(robot2->_M_inv)*robot2_j0_objcom.transpose()).inverse();

    augmented_object_inertia = robot1_objcom_inertia + robot2_objcom_inertia + object_inertia;

    // Compute J_bar
    robot1_j0_objcom_bar=(robot1->_M_inv)*robot1_j0_objcom.transpose()*robot1_objcom_inertia;
    robot2_j0_objcom_bar=(robot2->_M_inv)*robot2_j0_objcom.transpose()*robot2_objcom_inertia;
    // Get gravity
    robot1->gravityVector(robot1_g);
    robot2->gravityVector(robot2_g);

    // Compute augmented object gravity
    robot1_p = robot1_j0_objcom_bar.transpose() * robot1_g;
    robot2_p = robot2_j0_objcom_bar.transpose() * robot2_g;
    augmented_object_p = object_p + robot1_p + robot2_p;
    
		// --------------------------------------------------------------------
		// (3) Grasp Matrix
		//---------------------------------------------------------------------
		// ** FILL ME IN **

    // Get robot eef positions in world frame
    robot1->position(robot1_ee_pos_r1f, ee_link_name);
    robot2->position(robot2_ee_pos_r2f, ee_link_name);
    robot1_ee_pos_wf = robot1_base_frame * robot1_ee_pos_r1f;
    robot2_ee_pos_wf = robot2_base_frame * robot2_ee_pos_r2f;

    r_1 = robot1_ee_pos_wf - object_current_pos; 
    r_2 = robot2_ee_pos_wf - object_current_pos; 
    r_1_hat = getCrossProductMat(r_1);
    r_2_hat = getCrossProductMat(r_2);

    e12 = robot2_ee_pos_wf - robot1_ee_pos_wf;
    //e12 << 0, 1, 0;
    E_bar.block(0,0,0,3) << -e12 / 2.0;
    E_bar.block(0,3,0,6) << e12 / 2.0;

    cout << "r1_ee_rf" << endl;
    cout << robot1_ee_pos_r1f << endl;
    cout << "r1_ee_wf" << endl;
    cout << robot1_ee_pos_wf << endl;
    cout << "r2_ee_rf" << endl;
    cout << robot2_ee_pos_r2f << endl;
    cout << "r2_ee_wf" << endl;
    cout << robot2_ee_pos_wf << endl;
    cout << "e12" << endl;
    cout << e12 << endl;
    cout << "E_bar" << endl;
    cout << E_bar << endl;
    cout << "r1" << endl;
    cout << r_1 << endl;
    cout << "r2" << endl;
    cout << r_2 << endl;
    cout << "object_current_pos" << endl;
    cout << object_current_pos << endl;
  
    W_f << I3, I3, r_1_hat, r_2_hat;
    W_m << zero3, zero3, I3, I3;
    W << W_f, W_m;
    Itilde << 0, -0.5, 0, 0, 0.5, 0,
              1, 0, 0, 0, 0, 0,
              0, 0, -1, 0, 0, 0,
              0, 0, 0, -1, 0, 0,
              0, 0, 0, 0, 0, -1;

    G << W_f, W_m, E_bar, zero1_6, zero5_6, Itilde;
    cout << G << endl;

		// --------------------------------------------------------------------
		// (4) Force Control 
		//---------------------------------------------------------------------

		if (control_mode == CONTROL_AUGMENTED_OBJECT) {
		// ** FILL ME IN ** Compute tau1 and tau2 	

    // Object desired positions, velocities, accelerations
    obj_des_pos(0) = 0;
    obj_des_pos(1) = 0.15 * sin(2*M_PI*curr_time/3);
    obj_des_pos(2) = 0.4;
    obj_quat_des.w() = 1.0;
    obj_quat_des.x() = 0.0;
    obj_quat_des.y() = 0.0;
    obj_quat_des.z() = 0.0;

    obj_des_vel(0) = 0;
    obj_des_vel(1) = 0.15*(2*M_PI/3)*cos(2*M_PI*curr_time/3);
    obj_des_vel(2) = 0.0;
    obj_des_vel(3) = 0.0;
    obj_des_vel(4) = 0.0;
    obj_des_vel(5) = 0.0;

    obj_des_acc(0) = 0;
    obj_des_acc(1) = -0.15*(2*M_PI/3)*(2*M_PI/3)*sin(2*M_PI*curr_time/3);
    obj_des_acc(2) = 0.0;
    obj_des_acc(3) = 0.0;
    obj_des_acc(4) = 0.0;
    obj_des_acc(5) = 0.0;

    // Get object orientation error
    Quaterniond obj_quat_cur(object_com_frame.linear()); // Current object quat
    Sai2Model::orientationError(obj_ori_error, obj_quat_des, obj_quat_cur);

    object_model->velocity6dInWorld(obj_cur_vel, object_link_name);

    obj_task_err << (object_current_pos - obj_des_pos), obj_ori_error;
    F_star = obj_des_acc - kv*(obj_cur_vel - obj_des_vel) - kp * obj_task_err;
    F_motion = augmented_object_inertia * F_star + object_p;

    // Set desired internal forces and moments
    int_f_des << 0, -15, 0;
    int_m_des << 0, 0, 0;

    // Compute desired forces on object
    force_des_vec << F_motion, int_f_des, int_m_des;

    // Compute desired ee forces
    force_ee_vec = G.inverse() * force_des_vec;
    
    // Parse out robot 1 and robot 2 ee forces
    robot1_force_ee(0) = force_ee_vec(0);
    robot1_force_ee(1) = force_ee_vec(1);
    robot1_force_ee(2) = force_ee_vec(2);
    robot1_force_ee(3) = force_ee_vec(6);
    robot1_force_ee(4) = force_ee_vec(7);
    robot1_force_ee(5) = force_ee_vec(8);

    robot2_force_ee(0) = force_ee_vec(3);
    robot2_force_ee(1) = force_ee_vec(4);
    robot2_force_ee(2) = force_ee_vec(5);
    robot2_force_ee(3) = force_ee_vec(9);
    robot2_force_ee(4) = force_ee_vec(10);
    robot2_force_ee(5) = force_ee_vec(11);

		Eigen::MatrixXd robot1_j0_ee(6, robot1->dof());
		Eigen::MatrixXd robot2_j0_ee(6, robot2->dof());
		robot1->J_0(robot1_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
		robot2->J_0(robot2_j0_ee, ee_link_name, Eigen::Vector3d::Zero());

    tau1 = robot1_j0_ee.transpose()*robot1_force_ee + robot1_g;
    tau2 = robot2_j0_ee.transpose()*robot2_force_ee + robot2_g;

		} else if (control_mode == CONTROL_GRASP_STABILIZE) { // initial grasp stabilization
			
			Eigen::MatrixXd robot1_j0_ee(6, robot1->dof());
			Eigen::MatrixXd robot2_j0_ee(6, robot2->dof());
			robot1->J_0(robot1_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			robot2->J_0(robot2_j0_ee, ee_link_name, Eigen::Vector3d::Zero());

			// Joint Torques
			tau1 = robot1_j0_ee.transpose()*(object_p/2) + robot1->_M*(-10.0*robot1->_dq) + robot1_g;
			tau2 = robot2_j0_ee.transpose()*(object_p/2) + robot2->_M*(-10.0*robot2->_dq) + robot2_g;
			
			// Grasp stabilization
			static uint grasp1Counter = 0;
			static uint grasp2Counter = 0;
			if (robot1->_dq[6] < 0.1) {
				grasp1Counter += 1;
			} else {
				grasp1Counter = 0;
			}
			if (robot2->_dq[6] < 0.1) {
				grasp2Counter += 1;
			} else {
				grasp2Counter = 0;
			}
			if (grasp1Counter > 40 && grasp2Counter > 40) {
				cout << " ** Switch Control Mode to Augmented Object Model ** " << endl;
				control_mode = CONTROL_AUGMENTED_OBJECT;
			}
		}

		/* ----------------------------------------------------------------------------------
			Safety torques 
		-------------------------------------------------------------------------------------*/ 
		
		// Set constant gripper forces
		tau1[6] = 15;
		tau2[6] = 15;

        // Default values if torques are exceeded:
        bool fTorqueOverride = false; // to avoid robot blow-ups
        const double tau1_max = 200;
        const double tau2_max = 200;
        if (!fTorqueUseDefaults) {
        	if (tau1.cwiseAbs().maxCoeff() > tau1_max || tau2.cwiseAbs().maxCoeff() > tau2_max) {
	        	fTorqueOverride = true;
	        	cerr << "Torque overriden. User asked torques beyond safe limit: \n";
	        	cerr << "Robot 1: " << tau1.transpose() << "\n";
	        	cerr << "Robot 2: " << tau2.transpose() << "\n";
	        	fTorqueUseDefaults = true;
	        }
	        // Also default values if object is dropped
	        const double object_thickness = 0.05;
	        bool fRobot1DroppedObject = robot1->_q[6] > object_thickness/2;
	        bool fRobot2DroppedObject = robot2->_q[6] > object_thickness/2;
	        if (fRobot1DroppedObject || fRobot2DroppedObject) {
	        	cerr << "Torque overriden. Robot 1 or 2 dropped object. \n";
	        	fTorqueUseDefaults = true;
	        }
        }
        else {
        	robot1->gravityVector(tau1);
			tau1 = tau1 + robot1->_M*(-10.0*robot1->_dq);
			tau1 = (tau1.array() >= tau1_max).select(tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			tau1 = (tau1.array() <= -tau1_max).select(-tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			robot2->gravityVector(tau2);
			tau2 = tau2 + robot2->_M*(-10.0*robot2->_dq);
			tau2 = (tau2.array() >= tau2_max).select(tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
			tau2 = (tau2.array() <= -tau2_max).select(-tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
        }

        /* ----------------------------------------------------------------------------------
			Send robot torques from controller
		-------------------------------------------------------------------------------------*/ 
		sim->setJointTorques(robot1_name, tau1);
		sim->setJointTorques(robot2_name, tau2);
		
		// write object location key out to redis
		redis_client.executeWriteCallback(0);

		// update last time
		last_time = curr_time;
	}
}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Select trajectory
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(10000); //10000Hz timer

	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		// integrate forward
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		// sim->integrate(loop_dt);
		sim->integrate(loop_dt);
		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
