//
//  Craeture.h
//  test
//
//  Created by Masaki Nakada on 2/10/13.
//  Copyright (c) 2013 Masaki Nakada. All rights reserved.
//

#ifndef test_Craeture_h
#define test_Craeture_h

#include "World.h"
#include "Object.h"
#include "3DDeformable.h"
#include "RigidCube.h"
#include "Eigen/Dense"

class Creature
{
public:
    Creature(World *myWorld);
    ~Creature();
    void init(World *myWorld);
    void to_center();
    void set_joint_velocity(int muscle_num, int horizontal_torque, int verticle_torque, float dt, float alpha);
    void deactivate_friction();
    float getDistance();
    void destroyCreature();
    
private:
    RigidCube* cube1;
    RigidCube* cube2;
    RigidCube* cube3;
    Deformable3D* deform1;
    Deformable3D* deform2;
    Eigen::Vector3f curPos;
    Eigen::Vector3f prePos;
    
};


#endif
