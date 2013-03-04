//
//  World.cpp
//  FEMCloth2D-FEM
//
//  Created by Jingyi Fang on 2/15/12.
//  Copyright 2012 UCLA. All rights reserved.
//


#include <string>
#include "World.h"
#include "Object.h"
#include "3DDeformable.h"
#include "RigidCube.h"

#ifndef Object_Types_

#define NoType 0
#define TypeCube 1
#define TypeSphere 2
#define TypeCloth 3
#define TypeDeformable3D 4
#define TypeRide 5
#define TypeRigidCube 6

#endif

World::World(double hardness){
    m_hardness = hardness;
}

World::~World(){
    
}

void World::Add_Object(Object* new_object)
{
    List_of_Object.push_back(new_object);
	new_object->SetWorld(this);
}

void World::Remove_Object(std::string name){
    for (unsigned int i = 0; i < List_of_Object.size(); i++) {
        if(List_of_Object[i]->GetName() == name)
        {
            List_of_Object.erase(List_of_Object.begin()+i);
            return;
        }
    }
}

void World::Draw(int type, const Camera& camera, const Light& light){
	for (unsigned int i = 0; i<List_of_Object.size(); i++) {
       List_of_Object[i]->Draw(type,camera, light);
    }
}

void World::Update(double dt){
    int num_division = dt/(1/2000.0);
	//parallize
#ifdef WIN32
#pragma omp parallel for
#endif
	for ( int i = 0; i<List_of_Object.size(); i++) {
	   for ( int j = 0; j < num_division; j++)
			List_of_Object[i]->UpdateAll(1/2000.0);
    }
}

void World::Clear(){
	List_of_Object.clear();	
}