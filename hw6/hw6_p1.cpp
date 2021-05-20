#include <iostream>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "../../../core/sai2-graphics/src/chai_extension/CRobotBase.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include <chai3d.h>

#include "timer/LoopTimer.h"

#include "force_sensor/ForceSensorSim.h"
#include "geometry/CapsuleDistanceHull.h"

#include <GLFW/glfw3.h> 

using namespace std;

const double time_slowdown_factor = 1;

constexpr const char *sim_title = "cs327a - HW 6 Problem 1";

const string world_fname = "./resources/world_hw6_p1.urdf";
const string robot_fname = "../../resources/puma/puma.urdf";
const string robot_name = "Puma";
const string camera_name = "camera_front";
const string ee_link_name = "end-effector";

// Desired joint posture
Eigen::VectorXd q_des;

// Capsule early collision detector
CapsuleDistanceHull* coll_detector;
// Object in collision course
chai3d::cShapeSphere* coll_sphere;
const double sphere_radius = 0.06;

// Flags to control UI movement of sphere
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;

/* ----------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
// void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);
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
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);

    // set initial conditions
    q_des.setZero(robot->dof());
	q_des << 0/180.0*M_PI,
				-22.5/180.0*M_PI,
				212/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
				180/180.0*M_PI;
	robot->_q = q_des;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();
	Eigen::Affine3d ee_trans;

	// add keyboard controlled sphere
	coll_sphere = new chai3d::cShapeSphere(sphere_radius);
	coll_sphere->m_material->setRedFireBrick();
	coll_sphere->m_material->setShininess(10);
	coll_sphere->m_material->m_specular.set(0.7f, 0.7f, 0.7f, 1.0f);
	graphics->_world->addChild(coll_sphere);
	coll_sphere->setLocalPos(chai3d::cVector3d(Eigen::Vector3d(0.4, 0.55, 0.71)));

	// add capsules for cheap distance computation. 
	// TODO: replace with direct convex geometry functions
	coll_detector = new CapsuleDistanceHull(robot_name);
	chai3d::cRobotBase* chai_robot;
	for (unsigned int i = 0; i < graphics->_world->getNumChildren(); ++i) {
		auto object = graphics->_world->getChild(i);
		if (object->m_name == robot_name) {
			chai_robot = dynamic_cast<chai3d::cRobotBase*>(object);
			if (chai_robot != NULL) {
				break;
			}
		}
	}
	coll_detector->addLink("upper_arm", Eigen::Vector3d(-0.06, 0.0, 0.01), Eigen::Vector3d(0.40, 0.0, 0.01), 0.16, chai_robot);
	coll_detector->addLink("lower_arm", Eigen::Vector3d(-0.02, 0.03, 0.0), Eigen::Vector3d(-0.02, -0.38, 0.0), 0.10, chai_robot);	

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
    fSimulationRunning = true;
	thread sim_thread(simulation, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

		// move sphere
		Eigen::Vector3d obj_pos = coll_sphere->getLocalPos().eigen();
		if (fTransYp) {
			obj_pos[1] += 0.01;
		} else if (fTransYn) {
			obj_pos[1] -= 0.01;
		}
		if (fTransZp) {
			obj_pos[2] += 0.01;
		} else if (fTransZn) {
			obj_pos[2] -= 0.01;
		}
		coll_sphere->setLocalPos(chai3d::cVector3d(obj_pos));

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


/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs

	// cache variables
	bool fTimerDidSleep = true;
	bool fTorqueUseDefaults = false; // set true when torques are overriden for the first time
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	
	// end effector tip position in link frame
	Eigen::Vector3d ee_pos_local;
	ee_pos_local << -0.2, 0.0, 0.0;

	// ** Distance computation variables ** 
	// center of sphere in robot base frame
	Eigen::Vector3d sphere_center;
	// position of the closest point on the robot to the sphere, 
	// in robot base frame
	Eigen::Vector3d closest_point;
	// distance from closest point to the surface of the sphere
	double closest_distance;
	// name of the link on which the closest point is located
	string closest_link_name;
	// linear velocity Jacobian and full jacobian at the point closest to the 
	// surface of the sphere
	Eigen::MatrixXd Jv_closest_point(3, robot->dof());
	Eigen::MatrixXd Jconst_full_6(6, robot->dof());
	// desired end effector positions
	Eigen::Vector3d ee_pos_des;
	Eigen::Vector3d ee_vel_des;
	Eigen::Vector3d ee_acc_des;
	// transform from base to closest link 
	Eigen::Affine3d T_closest_link;

	// ** Other suggested variables **
	bool constraint_active = false;
	// double constraint_force;
	Eigen::Vector3d n_c;
	Eigen::Vector3d xci_to_c;
	Eigen::MatrixXd Jc(1, robot->dof());
	Eigen::VectorXd tau_c(robot->dof());
	// Eigen::Vector3d constraint_acc_in_task;
	Eigen::MatrixXd Lambda_c(1,1);
	Eigen::MatrixXd Jc_bar(robot->dof(), 1);
	Eigen::MatrixXd Nc(robot->dof(), robot->dof());

	Eigen::Vector3d ee_pos;
	Eigen::Vector3d ee_vel;
	Eigen::MatrixXd Jv(3, robot->dof());
	Eigen::MatrixXd Jv_bar(robot->dof(), 3);
	Eigen::MatrixXd Nv(robot->dof(), robot->dof());
	Eigen::Matrix3d Lambda_v;
	Eigen::VectorXd tau_tc(robot->dof());

  // Posture control related variables
	Eigen::VectorXd tau_p(robot->dof()); // Posture torque

  // Additional variables
  Eigen::MatrixXd A(robot->dof(), robot->dof());
  Eigen::VectorXd q(robot->dof());
  Eigen::VectorXd dq(robot->dof());
	Eigen::VectorXd g(robot->dof());

  double Uc_grad; // Gradient of constraint artificial potential field
  Eigen::Vector3d F_star;
  Eigen::Vector3d F_t;
  Eigen::Vector3d p; //gravity vector at end-effector

	// ** Suggested gains ** 
	const double constraint_dist_thresh = 0.095;
	const double eta = 0.3;
	const double kvc = 20;
	const double kpx = 20;
	const double kvx = 10;
	const double kpj = 30;
	const double kvj = 20;

  const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time;

        // read joint position, velocity
        sim->getJointPositions(robot_name, robot->_q);
        sim->getJointVelocities(robot_name, robot->_dq);
        robot->updateModel();

        // update distance from sphere
        CapsuleDistanceInfo retinfo = coll_detector->computeDistanceSphere (coll_sphere->getLocalPos().eigen(), sphere_radius, robot);
        closest_link_name = retinfo._link_name;
        closest_distance = retinfo._distance;
        robot->transform(T_closest_link, closest_link_name);
		robot->J_0(Jconst_full_6, closest_link_name, retinfo._closest_point_self);
        Jv_closest_point = Jconst_full_6.topRows(3);
		closest_point = T_closest_link*retinfo._closest_point_self;
        sphere_center = coll_sphere->getLocalPos().eigen();			

        // update desired ee position
        ee_pos_des << 0.7, 0.2 + 0.4*sin(2*M_PI*curr_time/6), 0.5;

		/* --------------------------------------------------------------------------------------
			FILL ME IN: compute joint torques
		-------------------------------------------------------------------------------------*/

      // Update 
      robot->Jv(Jv, ee_link_name, ee_pos_local);
      robot->gravityVector(g); // Update gravity vector
      A  = robot->_M; // Mass matrix
      q  = robot->_q;
      dq = robot->_dq;
      robot->position(ee_pos, ee_link_name, ee_pos_local);
      robot->linearVelocity(ee_vel, ee_link_name, ee_pos_local);
  
      // Constraint controller
      xci_to_c = sphere_center - closest_point; 
      n_c = xci_to_c / xci_to_c.norm();

      Lambda_c = (Jc * A.inverse() * Jc.transpose()).inverse();
      Jc = n_c.transpose() * Jv_closest_point; // Constraint Jacobian
      Jc_bar = A.inverse() * Jc.transpose() * Lambda_c;
      Nc = In - Jc_bar * Jc;

      if (closest_distance <= constraint_dist_thresh){
        // Constraint potential field gradient
        Uc_grad = eta * ((1/constraint_dist_thresh) - (1/closest_distance))
                    * (1/closest_distance) * (1/closest_distance);

        constraint_active = true;
      }
      else{
        Uc_grad = 0;
        constraint_active = false;
      }

      // Set constraint torques
      if (constraint_active == true){
        tau_c = Jc.transpose() * Uc_grad + Jc.transpose() * Lambda_c*(-kvc * Jc * dq);
      }
      else{
        tau_c = Eigen::VectorXd::Zero(robot->dof());
      }
    
      // Task controller
      // Set desired ee positions, velocities, accelerations
      ee_pos_des << 0.7, 0.2 + 0.4 * sin(2*M_PI*curr_time/6), 0.5;
      ee_vel_des << 0.0, 0.4 * (M_PI/3) * cos(2*M_PI*curr_time/6), 0.0;
      ee_acc_des << 0.0, -0.4 * (M_PI/3) * (M_PI/3) * sin(2*M_PI*curr_time/6), 0.0;

      Lambda_v = (Jv * A.inverse() * Jv.transpose()).inverse();
      //Lambda_v = (Jv * Nc * A.inverse() * Nc.transpose() * Jv.transpose()).inverse(); // L_tc
      Jv_bar = A.inverse() * Jv.transpose() * Lambda_v;
      p = Jv_bar.transpose() * g;
      Nv = In - Jv_bar * Jv;

      F_star = ee_acc_des - kpx * (ee_pos - ee_pos_des) - kvx *(ee_vel - ee_vel_des);
      F_t = Lambda_v * F_star + p;

      // Set task torques
      if (constraint_active == true){
        // Project task tau into constraint nullspace
        tau_tc = Nc.transpose() * Jv.transpose() * F_t;
      }
      else{
        tau_tc = Jv.transpose() * F_t;
      }

      // Posture controller
      tau_p = Nv.transpose() * (A * (-kpj * (q-q_des) - kvj * dq) + g);
      if (constraint_active == true){
        // Project posture tau into constraint nullspace
        tau_p = Nc.transpose() * tau_p;
      }
        
    tau = tau_c + tau_tc + tau_p;
		/* --------------------------------------------------------- */
		sim->setJointTorques(robot_name, tau);

		// update last time
		last_time = curr_time;
	}
}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer

	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		// integrate forward
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);
		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() { //--- Window initialization ---//

    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 1.0 * screenH;
    int windowH = 0.8 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, sim_title, NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) { //--- Window error ---//
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods) //--- Mouse click commands ---//
{
    bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransYp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransYn = set;
			break;
		case GLFW_KEY_UP:
			fTransZp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransZn = set;
			break;
		default:
			break;
    }
}
