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

class Bone;
class Muscle;
class Bone:public RigidCube{
public: 
	Bone(){};
	~Bone(){};
public:
	Muscle* m_prev;
	Muscle* m_next;
}
;
class Muscle:public Deformable3D{
public:
	Muscle(){};
	~Muscle(){};
	void muscleController(int horizontal_torque, int verticle_torque, float dt, float alpha, int segment_num);

public:
	Bone* m_prev;
	Bone* m_next;
};

class Snake:public Object
{
public:
    Snake(int a_num_segments);
    ~Snake();
    void init();
	void initPhysics();
    struct links get_root();
    void set_joint_velocity(int muscle_num, int horizontal_torque, int verticle_torque, float dt, float alpha);
    void deactivate_friction();
    float getDistance();
    void destroySnake();
    void Draw(int type, const Camera& camera, const Light& light);//Update data on GPU's buffer and draw the vertexs, rotate clockwise around z with speed
	void UpdateAll(double dt);
	void SetWorld(World *a_world);
private:
	Eigen::Vector3f curPos;
    Eigen::Vector3f prePos;
	int m_num_segment;
	Bone* m_bones;
	Muscle* m_muscles;
};


#endif
