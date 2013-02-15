//
//  Creature.cpp
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "Creature.h"
#define INITIAL_POS 2.0

Creature::Creature(World *myWorld)
{
    init(myWorld);
    curPos = Eigen::Vector3f(0.0,0.0,0.0);
    prePos = Eigen::Vector3f(0.0,0.0,0.0);
}

Creature::~Creature(){
    //destroyCreature();
}

void Creature::set_joint_velocity(int muscle_num, int horizontal_torque, int verticle_torque, float dt, float alpha)
{
    switch (muscle_num) {
        case 0:
            deform1->muscleController(horizontal_torque, verticle_torque, dt, alpha);
            break;
        case 1:
            deform2->muscleController(horizontal_torque, verticle_torque, dt, alpha);
            break;
        default:
            break;
    }
    
}

void Creature::init(World *myWorld){
    Eigen::Vector3i deform_res(5,5,5);double youngs_modulus = 2000;
	Eigen::Vector3f temp_position;Eigen::Vector3f rigid_size(2,2,2); double deform_length = 2;
	std::vector<Node*> temp_nodes;
    
    //create the first rigid skeleton
	cube1 = new RigidCube;
	cube1->Init(1.0, Eigen::Vector3f(0,INITIAL_POS,0), rigid_size, Eigen::Vector3f(1,0,0));
	cube1->m_fixed = false;
    
	//create the first deformable section;
	deform1 = new Deformable3D;
	temp_position = cube1->m_Center
    + Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
	deform1->Init(deform_res,1.0,youngs_modulus,0.4,100.0,temp_position,
                  Eigen::Vector3f(deform_length,rigid_size[1],rigid_size[2]),Eigen::Vector3f(1,1,1));
    
	//attach the previous rigid part and deformable part
	temp_nodes = deform1->m_Mesh->GetLeftNodes();
	cube1->AttachNodes(temp_nodes);
    
	//create the second rigid skeleton
	cube2 = new RigidCube;
	cube2->Init(1.0, Eigen::Vector3f(cube1->m_Center[0] + deform_length + rigid_size[0], INITIAL_POS, 0), rigid_size, Eigen::Vector3f(1,0,0));
	cube2->m_fixed = false;
    
	//attach the previous deformable part with the rigid part
	temp_nodes = deform1->m_Mesh->GetRightNodes();
	cube2->AttachNodes(temp_nodes);
    
	//create the second deformable section;
	deform2 = new Deformable3D;
	temp_position = cube2->m_Center
    + Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
	deform2->Init(deform_res,1.0,youngs_modulus,0.4,100.0,temp_position,
                  Eigen::Vector3f(deform_length,rigid_size[1],rigid_size[2]),Eigen::Vector3f(1,1,1));
    
	//attach the previous rigid part and deformable part
	temp_nodes = deform2->m_Mesh->GetLeftNodes();
	cube2->AttachNodes(temp_nodes);
    
	//create the first rigid skeleton
	cube3 = new RigidCube;
	cube3->Init(1.0, Eigen::Vector3f(cube2->m_Center[0] + deform_length + rigid_size[0],INITIAL_POS, 0), rigid_size, Eigen::Vector3f(1,0,0));
	cube3->m_fixed = false;
    
	//attach the previous deformable part with the rigid part
	temp_nodes = deform2->m_Mesh->GetRightNodes();
	cube3->AttachNodes(temp_nodes);
    
	myWorld->Add_Object(cube1);
	myWorld->Add_Object(deform1);
	myWorld->Add_Object(cube2);
	myWorld->Add_Object(deform2);
	myWorld->Add_Object(cube3);
}

float Creature::getDistance()
{
    curPos = cube1->m_Center;
    Eigen::Vector3f dis = curPos;
    float distance = dis.norm();
    
    return distance;
}

void Creature::destroyCreature()
{
    delete cube1;
    delete cube2;
    delete cube3;
    delete deform1;
    delete deform2;
}

void Creature::to_center(){

    Eigen::Vector3i deform_res(5,5,5);double youngs_modulus = 2000;
	Eigen::Vector3f temp_position;Eigen::Vector3f rigid_size(2,2,2); double deform_length = 2;
	std::vector<Node*> temp_nodes;
    
	cube1->Init(1.0, Eigen::Vector3f(0,INITIAL_POS,0), rigid_size, Eigen::Vector3f(1,0,0));
	cube1->m_fixed = false;
    
	//create the first deformable section;
	temp_position = cube1->m_Center
    + Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
	deform1->Init(deform_res,1.0,youngs_modulus,0.4,100.0,temp_position,
                  Eigen::Vector3f(deform_length,rigid_size[1],rigid_size[2]),Eigen::Vector3f(1,1,1));
    
	//attach the previous rigid part and deformable part
	temp_nodes = deform1->m_Mesh->GetLeftNodes();
	cube1->AttachNodes(temp_nodes);
    
	//create the second rigid skeleton
	cube2->Init(1.0, Eigen::Vector3f(cube1->m_Center[0] + deform_length + rigid_size[0],INITIAL_POS, 0), rigid_size, Eigen::Vector3f(1,0,0));
	cube2->m_fixed = false;
    
	//attach the previous deformable part with the rigid part
	temp_nodes = deform1->m_Mesh->GetRightNodes();
	cube2->AttachNodes(temp_nodes);
    
	//create the second deformable section;
	temp_position = cube2->m_Center
    + Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
	deform2->Init(deform_res,1.0,youngs_modulus,0.4,100.0,temp_position,
                  Eigen::Vector3f(deform_length,rigid_size[1],rigid_size[2]),Eigen::Vector3f(1,1,1));
    
	//attach the previous rigid part and deformable part
	temp_nodes = deform2->m_Mesh->GetLeftNodes();
	cube2->AttachNodes(temp_nodes);
    
	//create the first rigid skeleton
	cube3->Init(1.0, Eigen::Vector3f(cube2->m_Center[0] + deform_length + rigid_size[0],INITIAL_POS, 0), rigid_size, Eigen::Vector3f(1,0,0));
	cube3->m_fixed = false;
    
	//attach the previous deformable part with the rigid part
	temp_nodes = deform2->m_Mesh->GetRightNodes();
	cube3->AttachNodes(temp_nodes);
    
    curPos = Eigen::Vector3f(0.0,0.0,0.0);
    prePos = Eigen::Vector3f(0.0,0.0,0.0);
    
}
