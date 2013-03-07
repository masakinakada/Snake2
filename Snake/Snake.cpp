//
//  Snake.cpp
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#include "Snake.h"
#define INITIAL_POS 5.5
#include <iostream>




void Muscle::muscleController(int horizontal_torque, int verticle_torque , float dt, float alpha, int segment_num)
{
	/*
    if(horizontal_torque==2){
       UpdateRestShape(dt, alpha, SHRINK_RIGHT);

        //std::cout<<"Segment #"<<segment_num<<": Shrink Right"<<std::endl;
    }
    else if(horizontal_torque==1)
    {
        UpdateRestShape(dt, alpha, RELEASE_RIGHT);

        //std::cout<<"Segment #"<<segment_num<<": Release Right"<<std::endl;
      
    }
    else if(horizontal_torque == -1)
    {
       UpdateRestShape(dt, alpha, SHRINK_LEFT);

        //std::cout<<"Segment #"<<segment_num<<": Shrink Left"<<std::endl;
    }
    else if(horizontal_torque== -2){
        UpdateRestShape(dt, alpha, RELEASE_LEFT);

        //std::cout<<"Segment #"<<segment_num<<": Release Left"<<std::endl;
    }
    
   */
    if(verticle_torque==2){
        UpdateRestShape(dt, alpha, SHRINK_UP);
        
        //std::cout<<"Segment #"<<segment_num<<": Shrink Right"<<std::endl;
    }
    else if(verticle_torque==1)
    {
        UpdateRestShape(dt, alpha, RELEASE_UP);
        
        //std::cout<<"Segment #"<<segment_num<<": Release Right"<<std::endl;
        
    }
    else if(verticle_torque == -1)
    {
        UpdateRestShape(dt, alpha, SHRINK_DOWN);
        
        //std::cout<<"Segment #"<<segment_num<<": Shrink Left"<<std::endl;
    }
    else if(verticle_torque== -2){
        UpdateRestShape(dt, alpha, RELEASE_DOWN);
        
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

void Snake::set_joint_velocity(int muscle_num, int horizontal_torque, int verticle_torque, float dt, float alpha)
{
    m_muscles[muscle_num].muscleController(horizontal_torque, verticle_torque, dt, alpha, 0);
    
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
	}
	m_bones[m_num_segment - 1].m_next = NULL;

	initPhysics(snake_num);
}

void Snake::initPhysics(int snake_num){
	
	Eigen::Vector3i deform_res(2,2,2);double youngs_modulus = 2000;
	Eigen::Vector3f temp_position;Eigen::Vector3f rigid_size(0.5,2,2); double deform_length = 2;
	std::vector<Node*> temp_nodes;

	//create the head bone
	m_bones[0].Init(1.0, Eigen::Vector3f(0,INITIAL_POS,snake_num*20), rigid_size, Eigen::Vector3f(1,0,0));
    
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

	//m_bones[0].m_fixed = true;
	//m_bones[m_num_segment-1].m_fixed = true;

}

void Snake::Reinit(int snake_num){

	Eigen::Vector3f temp_position;Eigen::Vector3f rigid_size(0.5,2,2); double deform_length = 2;
	std::vector<Node*> temp_nodes;

	//create the head bone
	m_bones[0].Reinit(Eigen::Vector3f(0,INITIAL_POS,snake_num*20));

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

float Snake::getDistance()
{
	//TODO: seems too simple
    
    //std::cout<<"Head position = "<<m_bones[0].m_Center<<std::endl;
    //std::cout<<"Tail Position = "<<m_bones[7].m_Center<<std::endl;

    //we take negative because snake is moving toward negative x direction
    //also take average of all segmentds so that we have whole body moving forward
    float distance = 0;
    for (int i=0; i<m_num_segment; i++) {
        distance-=m_bones[i].m_Center[0];
    }
    distance = distance/m_num_segment;
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