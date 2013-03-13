//
//  Snake.cpp
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "Snake.h"
#define INITIAL_POS 10.0
#define INITIAL_Y_OFFSET 20
#include <iostream>




void Muscle::muscleController(int horizontal_torque, int verticle_torque , float dt, float alpha1, float alpha2, int segment_num)
{
	
    if(horizontal_torque==2){
       UpdateRestShape(dt, alpha1, SHRINK_RIGHT);

        //std::cout<<"Segment #"<<segment_num<<": Shrink Right"<<std::endl;
    }
    else if(horizontal_torque==1)
    {
        UpdateRestShape(dt, alpha1, RELEASE_RIGHT);

        //std::cout<<"Segment #"<<segment_num<<": Release Right"<<std::endl;
      
    }
    else if(horizontal_torque == -1)
    {
       UpdateRestShape(dt, alpha1, SHRINK_LEFT);

        //std::cout<<"Segment #"<<segment_num<<": Shrink Left"<<std::endl;
    }
    else if(horizontal_torque== -2){
        UpdateRestShape(dt, alpha1, RELEASE_LEFT);

        //std::cout<<"Segment #"<<segment_num<<": Release Left"<<std::endl;
    }
    
   
    if(verticle_torque==2){
        UpdateRestShape(dt, alpha2, SHRINK_UP);
        
        //std::cout<<"Segment #"<<segment_num<<": Shrink Right"<<std::endl;
    }
    else if(verticle_torque==1)
    {
        UpdateRestShape(dt, alpha2, RELEASE_UP);
        
        //std::cout<<"Segment #"<<segment_num<<": Release Right"<<std::endl;
        
    }
    else if(verticle_torque == -1)
    {
        UpdateRestShape(dt, alpha2, SHRINK_DOWN);
        
        //std::cout<<"Segment #"<<segment_num<<": Shrink Left"<<std::endl;
    }
    else if(verticle_torque== -2){
        UpdateRestShape(dt, alpha2, RELEASE_DOWN);
        
        //std::cout<<"Segment #"<<segment_num<<": Release Left"<<std::endl;
    }
   
}


Snake::Snake(int a_num_seg, int snake_num)
{
	m_num_segment = a_num_seg;

    curPos = Eigen::Vector3f(0.0,INITIAL_POS,0.0);
    prePos = Eigen::Vector3f(0.0,INITIAL_POS,0.0);

	init(snake_num);
    
    static int count = 0;
    std::cout<<"Snake #" <<count++<<std::endl;
}

Snake::~Snake(){
}

void Snake::SetWorld(World* a_world){
	
	for(int i = 0; i < m_num_segment; i++)
	{
		m_bones[i].SetWorld(a_world);
	}

	for(int i = 0; i < m_num_segment-1; i++)
	{
		m_muscles[i].SetWorld(a_world);
	}

}

void Snake::set_joint_velocity(int muscle_num, int horizontal_torque, int verticle_torque, float dt, float alpha1, float alpha2)
{
    m_muscles[muscle_num].muscleController(horizontal_torque, verticle_torque, dt, alpha1, alpha2, 0);
    
}

void Snake::init(int snake_num){
    
	m_bones = new Bone[m_num_segment];
	m_muscles = new Muscle[m_num_segment-1];
	
	//connect the nueronet;
	m_bones[0].m_prev = NULL;
	for(int i = 0; i < m_num_segment-1; i++){
		m_bones[i].m_next = m_muscles + i;
		m_bones[i+1].m_prev = m_muscles + i;
		m_muscles[i].m_prev = m_bones + i;
		m_muscles[i].m_next = m_bones + (i+1);
        m_muscles[i].setBones(m_bones+i, m_bones + (i+1));
	}
	m_bones[m_num_segment - 1].m_next = NULL;

	initPhysics(snake_num);
}

void Snake::initPhysics(int snake_num){
	
	Eigen::Vector3i deform_res(3,2,2);double youngs_modulus = 2000;
	Eigen::Vector3f temp_position;Eigen::Vector3f rigid_size(0.5,2,2); double deform_length = 3;
	std::vector<Node*> temp_nodes;

	//create the head bone
	m_bones[0].Init(1.0, Eigen::Vector3f(0,INITIAL_POS,snake_num*INITIAL_Y_OFFSET), rigid_size, Eigen::Vector3f(1,0,0));
    
    float density = 1.0;

	for(int i = 0; i < m_num_segment - 1; i++)
	{
		temp_position = m_bones[i].m_Center
				+ Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
		//create the i-th muscle
		m_muscles[i].Init(deform_res,density,youngs_modulus,0.4,100.0,temp_position,
                  Eigen::Vector3f(deform_length,rigid_size[1],rigid_size[2]),Eigen::Vector3f(1,1,1));
		//create the i+1-th bone
		temp_position = m_bones[i].m_Center + Eigen::Vector3f(deform_length + rigid_size[0],0,0);
		m_bones[i+1].Init(1.0, temp_position, rigid_size, Eigen::Vector3f(1,0,0));

		//attach the previous bone to current muscle
		temp_nodes = m_muscles[i].m_Mesh->GetLeftNodes();
		m_bones[i].AttachNodes(temp_nodes);

		//attach the next bone to current muscle
		temp_nodes = m_muscles[i].m_Mesh->GetRightNodes();
		m_bones[i+1].AttachNodes(temp_nodes);

	}
    
    original_pos = m_bones[m_num_segment/2].m_Center[0];

	//m_bones[0].m_fixed = true;
	//m_bones[m_num_segment-1].m_fixed = true;

}

void Snake::Reinit(int snake_num){

	Eigen::Vector3f temp_position;Eigen::Vector3f rigid_size(0.5,2,2); double deform_length = 3;
	std::vector<Node*> temp_nodes;

	//create the head bone
	m_bones[0].Reinit(Eigen::Vector3f(0,INITIAL_POS,snake_num*INITIAL_Y_OFFSET));

	for(int i = 0; i < m_num_segment - 1; i++)
	{
		temp_position = m_bones[i].m_Center
				+ Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
		//create the i-th muscle
		m_muscles[i].Reinit(temp_position);
                  
		//create the i+1-th bone
		temp_position = m_bones[i].m_Center + Eigen::Vector3f(deform_length + rigid_size[0],0,0);
		m_bones[i+1].Reinit(temp_position);

		//attach the previous bone to current muscle
		temp_nodes = m_muscles[i].m_Mesh->GetLeftNodes();
		m_bones[i].AttachNodes(temp_nodes);

		//attach the next bone to current muscle
		temp_nodes = m_muscles[i].m_Mesh->GetRightNodes();
		m_bones[i+1].AttachNodes(temp_nodes);

	}

}

float Snake::getDistance(int segment_ID)
{
	//TODO: seems too simple
    
    //std::cout<<"Head position = "<<m_bones[0].m_Center<<std::endl;
    //std::cout<<"Tail Position = "<<m_bones[7].m_Center<<std::endl;

    //we take negative because snake is moving toward negative x direction
    //also take average of all segmentds so that we have whole body moving forward
    float distance = 0;
    for (int i=0; i<m_num_segment; i++) {
        
        //snake is moving toward negative x axis. there is direcional friction so should not chnage. or change both here and direction of friction
        distance += -(m_bones[i].m_Center[0]);
        distance += (m_bones[i].m_Center[2]-segment_ID*INITIAL_Y_OFFSET);
    }
    distance = distance/m_num_segment+original_pos;
   return distance;

}

void Snake::destroySnake()
{

}

void Snake::Draw(int type, const Camera& camera, const Light& light){

	for(int i = 0; i < m_num_segment; i++)
	{
		m_bones[i].Draw(type, camera, light);
	}

	for(int i = 0; i < m_num_segment-1; i++)
	{
		m_muscles[i].Draw(type, camera, light);
	}

}

void Snake::UpdateAll(double dt){
	
	for(int i = 0; i < m_num_segment; i++)
	{
		m_bones[i].UpdateAll(dt);
	}
	
	for(int i = 0; i < m_num_segment-1; i++)
	{
		m_muscles[i].UpdateAll(dt);
		m_muscles[i].m_Direction = m_bones[i].m_Center - m_bones[i+1].m_Center;
	}

}