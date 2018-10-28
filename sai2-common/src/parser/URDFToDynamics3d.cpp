/*
 * URDFToDynamics3d.cpp
 *
 *  Created on: Dec 23, 2016
 *      Author: Shameek Ganguly
 */

#include "URDFToDynamics3d.h"

#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>
#include <urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h>

typedef my_shared_ptr<urdf::Link> LinkPtr;
typedef const my_shared_ptr<const urdf::Link> ConstLinkPtr;
typedef my_shared_ptr<urdf::Joint> JointPtr;
typedef my_shared_ptr<urdf::ModelInterface> ModelPtr;
typedef my_shared_ptr<urdf::World> WorldPtr;

#include <Eigen/Core>
using namespace Eigen;


#include <assert.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <stack>
using namespace std;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr > URDFLinkMap;
typedef map<string, JointPtr > URDFJointMap;

using namespace chai3d;
#define CDYN_ERROR_EPSILON 0.0001
#define CDYN_DEFAULT_MAT_RESTITUTION 0.9

namespace Parser {

// load link collision properties from urdf link to dynamics3d link
// TODO: working dir default should be "", but this requires checking
// to make sure that the directory path has a trailing backslash
static void loadLinkCollision(cDynamicLink* link, const my_shared_ptr<urdf::Collision>& collision_ptr, const std::string& working_dirname = "./") {
	auto tmp_mmesh = new cMultiMesh();
	tmp_mmesh->m_name = std::string("sai_dyn3d_link_mesh");
	auto tmp_mesh = new cMesh();
	const auto geom_type = collision_ptr->geometry->type;
	if (geom_type == urdf::Geometry::MESH) {
		// downcast geometry ptr to mesh type
		const auto mesh_ptr = dynamic_cast<const urdf::Mesh*>(collision_ptr->geometry.get());
		assert(mesh_ptr);
		// load object
		if(false == cLoadFileOBJ(tmp_mmesh, working_dirname+"/"+mesh_ptr->filename)) {
			if(false == cLoadFile3DS(tmp_mmesh, working_dirname+"/"+mesh_ptr->filename)) {
				cerr << "Couldn't load obj/3ds robot link file: " << working_dirname+"/"+mesh_ptr->filename << endl;
				abort();
			}
	    }
	    // apply scale
	    tmp_mmesh->scaleXYZ(
			mesh_ptr->scale.x,
			mesh_ptr->scale.y,
			mesh_ptr->scale.z
		);
	} else if (geom_type == urdf::Geometry::BOX) {
		// downcast geometry ptr to box type
		const auto box_ptr = dynamic_cast<const urdf::Box*>(collision_ptr->geometry.get());
		assert(box_ptr);
		// create chai box mesh
		chai3d::cCreateBox(tmp_mesh, box_ptr->dim.x, box_ptr->dim.y, box_ptr->dim.z);
		tmp_mmesh->addMesh(tmp_mesh);
	} else if (geom_type == urdf::Geometry::SPHERE) {
		// downcast geometry ptr to sphere type
		const auto sphere_ptr = dynamic_cast<const urdf::Sphere*>(collision_ptr->geometry.get());
		assert(sphere_ptr);
		// create chai sphere mesh
		chai3d::cCreateSphere(tmp_mesh, sphere_ptr->radius);
		tmp_mmesh->addMesh(tmp_mesh);
	} else if (geom_type == urdf::Geometry::CYLINDER) {
		// downcast geometry ptr to cylinder type
		const auto cylinder_ptr = dynamic_cast<const urdf::Cylinder*>(collision_ptr->geometry.get());
		assert(cylinder_ptr);
		// create chai sphere mesh
		chai3d::cCreateCylinder(tmp_mesh, cylinder_ptr->length, cylinder_ptr->radius);
		tmp_mmesh->addMesh(tmp_mesh);
	}

	// transfer meshes to combined collision mesh along with local position and orientation info
	auto urdf_q = collision_ptr->origin.rotation;
	Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
	cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

	//-- New code to combine meshes -- This results in multiple collision meshes on the same link
	tmp_mmesh->setLocalPos(cVector3d(
		collision_ptr->origin.position.x,
		collision_ptr->origin.position.y,
		collision_ptr->origin.position.z
		));
	tmp_mmesh->setLocalRot(tmp_cmat3);
	link->setCollisionModel(tmp_mmesh);
	// build collision model (currently, only convex hull and box are supported!)
	link->buildCollisionHull(CDYN_ERROR_EPSILON, CDYN_ERROR_EPSILON);
}

// load inertial properties from urdf link to dynamics3d link
static void loadLinkInertial(cDynamicLink* link, ConstLinkPtr& urdf_link) {
	Vector3d link_inertial_rpy; // inertial frame rotation in Euler XYZ moving angles (Roll, Pitch, Yaw)
	Vector3d link_inertial_position; // COM
	Matrix3d link_inertial_inertia; // inertia
	double link_inertial_mass; // link mass

	if (urdf_link->inertial) {
		// copy root mass
		link_inertial_mass = urdf_link->inertial->mass;

		// copy root com
		link_inertial_position <<
			urdf_link->inertial->origin.position.x,
			urdf_link->inertial->origin.position.y,
			urdf_link->inertial->origin.position.z;

		// copy root COM inertia
		link_inertial_inertia(0,0) = urdf_link->inertial->ixx;
		link_inertial_inertia(0,1) = urdf_link->inertial->ixy;
		link_inertial_inertia(0,2) = urdf_link->inertial->ixz;

		link_inertial_inertia(1,0) = urdf_link->inertial->ixy;
		link_inertial_inertia(1,1) = urdf_link->inertial->iyy;
		link_inertial_inertia(1,2) = urdf_link->inertial->iyz;

		link_inertial_inertia(2,0) = urdf_link->inertial->ixz;
		link_inertial_inertia(2,1) = urdf_link->inertial->iyz;
		link_inertial_inertia(2,2) = urdf_link->inertial->izz;

		// TODO: abort if link inertia is not diagonal

		// copy root inertia rotation (Euler XYZ moving notation)
		// root_inertial_rpy is not used anywhere. It is not supported currently.
		urdf_link->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

		if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
			cerr << "Error while processing body '" << urdf_link->name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
			abort();
		}
	} // else use default

	// add inertial properties to link
	link->setMassProperties(
		link_inertial_mass,
	    cMatrix3d(link_inertial_inertia),
	    cVector3d(link_inertial_position)
	);
}

void URDFToDynamics3dWorld(const std::string& filename, cDynamicWorld* world, bool verbose) {
	// load world urdf file
	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

	model_file.close();

	// parse xml to URDF world model
	assert(world);
	WorldPtr urdf_world = urdf::parseURDFWorld(model_xml_string);
	world->m_name = urdf_world->name_;
	if (verbose) {
		cout << "URDFToDynamics3dWorld: Starting model conversion to dynamics3d world." << endl;
		cout << "+ add world: " << world->m_name << endl;
	}

	// parse gravity
	world->setGravity(cVector3d(
		urdf_world->gravity_.x,
		urdf_world->gravity_.y,
		urdf_world->gravity_.z));

	// load parsed robots specifications into the dynamics3d world
	for (const auto robot_spec_pair: urdf_world->models_) {
		const auto robot_spec = robot_spec_pair.second;

		// get translation
		auto tmp_cvec3 = cVector3d(
			robot_spec->origin.position.x,
			robot_spec->origin.position.y,
			robot_spec->origin.position.z);
		// get rotation
		auto urdf_q = robot_spec->origin.rotation;
		Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

		// create new robot base
		cDynamicBase* robot = world->newBaseObject(
			tmp_cvec3,
			tmp_cmat3);
		assert(robot);
		robot->setLocalPos(tmp_cvec3);
		robot->setLocalRot(tmp_cmat3);

		// load robot from file
		URDFToDynamics3dRobot(robot_spec->model_filename, robot, verbose, robot_spec->model_working_dir);
		assert(robot->m_name == robot_spec->model_name);

		// overwrite robot name with custom name for this instance
		robot->m_name = robot_spec->name;
	}

	// parse static meshes
	for (const auto object_pair: urdf_world->graphics_.static_objects) {
		const auto object_ptr = object_pair.second;
		// get translation
		auto tmp_cvec3 = cVector3d(
			object_ptr->origin.position.x,
			object_ptr->origin.position.y,
			object_ptr->origin.position.z);
		// get rotation
		auto urdf_q = object_ptr->origin.rotation;
		Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d tmp_cmat3; tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

		// initialize a cGenericObject to represent this object in the world
		cDynamicBase* object = world->newBaseObject(
			tmp_cvec3,
			tmp_cmat3);
		assert(object);
		object->m_name = object_ptr->name;

		// add a link for collision
		auto default_mat = new cDynamicMaterial();
		//TODO: parse material property from file
		default_mat->setEpsilon(CDYN_DEFAULT_MAT_RESTITUTION); //default epsilon
		cDynamicLink* link = object->newLink(default_mat);
		link->m_name = "static_object_link";
		
		// load object graphics, must have atleast one
		for (const auto collision_ptr: object_ptr->collision_array) {
			loadLinkCollision(link, collision_ptr);
		}

		// add link to object base
		object->linkChild(link, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
	}
}

void URDFToDynamics3dRobot(const std::string& filename, cDynamicBase* model, bool verbose, const std::string& working_dirname) {
	// load file
	string filepath = working_dirname + "/" + filename;
	ifstream model_file (filepath);
	if (!model_file) {
		cerr << "Error opening file '" << filepath << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

	model_file.close();

	// read and parse xml string to urdf model
	assert(model);
	ModelPtr urdf_model = urdf::parseURDF (model_xml_string);
	model->m_name = urdf_model->getName();
	if (verbose) {
		cout << "URDFToDynamics3dRobot: Starting model conversion to dynamics3d." << endl;
		cout << "+ add robot: " << model->m_name << endl;
	}

	// load urdf model to dynamics3D link tree
	LinkPtr urdf_root_link;

	URDFLinkMap link_map; //map<string, LinkPtr >
	link_map = urdf_model->links_;

	URDFJointMap joint_map; //map<string, JointPtr >
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr> link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	// push the root LinkPtr to link stack. link stack height = 1
	// NOTE: depth first search happens due to use of stack
	link_stack.push (link_map[(urdf_model->getRoot()->name)]);

	// allocate a default material shared by all links
	// TODO: load material properties from file
	auto default_mat = new cDynamicMaterial();
	default_mat->setEpsilon(CDYN_DEFAULT_MAT_RESTITUTION); //default epsilon

	// add the root body
	ConstLinkPtr& root = urdf_model->getRoot();
	
	// TODO: Not sure if this is ever expected to be true or not
	// 	Mikael's example URDF has it set to false
	if (root->inertial || root->collision) {
		// initialize a cDynamics link
		auto root_link = model->newLink(default_mat);
		if (NULL == root_link) {
			cerr << "Robot dynamic root link failed to create." << endl;
			abort();
		}

		// set name
		root_link->m_name = root->name;

		// add inertial properties
		loadLinkInertial(root_link, root);

		// add collision properties
		for (const auto collision_ptr: root->collision_array) {
			loadLinkCollision(root_link, collision_ptr, working_dirname);
		}

		// Root assumed to be fixed base, so no joint needed in dynamics3d
		// NOTE: to use a floating base, explicitly specify 6 additional joints

		if (verbose) {
			cout << "+ Adding Root Body " << endl;
			cout << "  joint position: " << cVector3d(0.0, 0.0, 0.0) << endl;
			cout << "  joint type : fixed" << endl;
			cout << "  body inertia: " << endl;
			cout << root_link->getInertia().getRow(0) << endl;
			cout << root_link->getInertia().getRow(1) << endl;
			cout << root_link->getInertia().getRow(2) << endl;
			cout << "  body com   : " << root_link->getCenterOfMass() << endl;
			cout << "  body mass   : " << root_link->getMass() << endl;
			cout << "  body name   : " << root_link->m_name << endl;
		}

		// append root (joint, frame and link) to the base model.
		// NOTE: by default, root is added to origin in model frame.
		model->linkChild(
			root_link,
			cVector3d(0.0, 0.0, 0.0),
			cIdentity3d()
		);
	} //endif (root->inertial || root->collision)

	if (link_stack.top()->child_joints.size() > 0) {
		joint_index_stack.push(0); // SG: what does this do??
	} else {
		cerr << "Base link has no associated joints!" << endl;
		abort();
	}

	// this while loop is to enumerate all joints in the tree structure by name
	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top(); 

		// if there are unvisited child joints on current link:
		// 	then add link to stack
		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push (joint_idx + 1);

			// SG: the URDF model structure is:
			//	every non-terminal link has child joint(s)
			// 	every joint has child link (else, we would get an exception right below)
			link_stack.push (link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			if (verbose) {
				for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
					cout << "  ";
				}
				cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
			}

			joint_names.push_back(cur_joint->name); 
			// SG: this is the only data structure of interest it seems
			// all joints are processed in the for loop below
		} else { // else this link has been processed, so pop link
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	// iterate over all joints
	for (unsigned int j = 0; j < joint_names.size(); j++) {
		JointPtr urdf_joint = joint_map[joint_names[j]];
		LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
		LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

		// determine where to add the current joint and child body
		cDynamicLink* parent_link = NULL;
		parent_link = model->getLink(urdf_parent->name); // returns NULL if link does not exist

		//cout << "joint: " << urdf_joint->name << "\tparent = " << urdf_parent->name << " child = " << urdf_child->name << " parent_id = " << rbdl_parent_id << endl;

		// create a new link
		auto dyn_link = model->newLink(default_mat);
		if (NULL == dyn_link) {
			cerr << "Robot dynamic link failed to create." << endl;
			abort();
		}

		// set name
		dyn_link->m_name = urdf_child->name;

		// add inertial properties
		loadLinkInertial(dyn_link, urdf_child);

		// load collision properties
		for (const auto collision_ptr: urdf_child->collision_array) {
			loadLinkCollision(dyn_link, collision_ptr, working_dirname);
		}

		// create the joint
		// NOTE: dynamics3d currently only supports X, Y and Z aligned axes with prismatic
		// TODO: fix this problem
		// or revolute joints.
		auto urdf_joint_type = urdf_joint->type;
		cDynamicJoint* dyn_joint = NULL;
		int axis_type;
		switch (urdf_joint_type) {
			case urdf::Joint::REVOLUTE:
				if (urdf_joint->axis.x > 0.9) {
					axis_type = DYN_AXIS_X;
				} else if (urdf_joint->axis.y > 0.9) {
					axis_type = DYN_AXIS_Y;
				} else if (urdf_joint->axis.z > 0.9) {
					axis_type = DYN_AXIS_Z;
				} else {
					cerr << "Unsupported joint axis " << joint_names[j] << endl;
					abort();
				}
				dyn_joint = dyn_link->newJoint(DYN_JOINT_REVOLUTE, axis_type);
				break;
			case urdf::Joint::PRISMATIC:
				if (urdf_joint->axis.x > 0.9) {
					axis_type = DYN_AXIS_X;
				} else if (urdf_joint->axis.y > 0.9) {
					axis_type = DYN_AXIS_Y;
				} else if (urdf_joint->axis.z > 0.9) {
					axis_type = DYN_AXIS_Z;
				} else {
					cerr << "Unsupported joint axis " << joint_names[j] << endl;
					abort();
				}
				dyn_joint = dyn_link->newJoint(DYN_JOINT_PRISMATIC, axis_type);
				break;
			// currently unsupported joint types:
			case urdf::Joint::FLOATING:
			case urdf::Joint::PLANAR:
			case urdf::Joint::CONTINUOUS:
				cerr << "Unsupported joint type on joint " << joint_names[j] << endl;
				abort();
				break;
			case urdf::Joint::FIXED: //This is true for ground links
			default:
				break;
		}
		if (NULL != dyn_joint) {
			// set joint name
			dyn_joint->m_name = joint_names[j];
			// set joint home position
			dyn_joint->setPos(0.0); // TODO: does URDF support a joint home position?
			// TODO: set joint limits once we figure out how to get that info from the 
			// urdf model
			// TODO: set joint damping
		}

		// compute the joint transformation which acts as the child link transform
		// with respect to the parent
		Vector3d joint_rpy;
		Vector3d joint_translation;
		urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
		joint_translation <<
				urdf_joint->parent_to_joint_origin_transform.position.x,
				urdf_joint->parent_to_joint_origin_transform.position.y,
				urdf_joint->parent_to_joint_origin_transform.position.z;
		auto urdf_q = urdf_joint->parent_to_joint_origin_transform.rotation;
		Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d rot_in_parent;
		rot_in_parent.copyfrom(tmp_q.toRotationMatrix());

		if (verbose) {
			cout << "+ Adding Body " << endl;
			if (NULL == parent_link) {
				cout << "  parent_link_name  : NULL" << endl;
			} else {
				cout << "  parent_link_name  : " << parent_link->m_name << endl;
			}
			cout << "  position in parent: " << joint_translation.transpose() << endl;
			cout << "  orientation in parent: " << tmp_q.coeffs().transpose() << endl;
			if (NULL == dyn_link->getJoint(0)) {
				cout << "  joint type: fixed " << endl;
			} else {
				cout << "  joint type (0/P,1/R,2/S): " << dyn_link->getJoint(0)->getJointType() << endl;
				cout << "  joint axis (0/x,1/y,2/z): " << dyn_link->getJoint(0)->getJointAxis() << endl;
				cout << "  joint name: " << dyn_link->m_name << endl;
			}
			cout << "  body inertia: " << endl;
			cout << dyn_link->getInertia().getRow(0) << endl;
			cout << dyn_link->getInertia().getRow(1) << endl;
			cout << dyn_link->getInertia().getRow(2) << endl;
			cout << "  body com   : " << dyn_link->getCenterOfMass() << endl;
			cout << "  body mass   : " << dyn_link->getMass() << endl;
			cout << "  body name   : " << dyn_link->m_name << endl;
		}

		// add link to model
		if (NULL == parent_link) {
			// link is located at root
			model->linkChild(
				dyn_link,
				cVector3d(joint_translation),
				rot_in_parent
			);
		} else {
			parent_link->linkChild(
				dyn_link,
				cVector3d(joint_translation),
				rot_in_parent
			);
		}
	}

	if (verbose) {
		cout << "URDFToDynamics3dRobot: Finished model conversion to dynamics3d." << endl;
	}
}

} // namespace Parser
