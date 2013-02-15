//
//  3DDeformable.h
//
//  Created by Jingyi Fang on 3/13/12.
//  All rights reserved.
//

#ifndef FVM_3D_3DDEFORMABLE_h
#define FVM_3D_3DDEFORMABLE_h

#include <vector>
#include "Camera.h"
#include "Light.h"
#include "Eigen/Dense"
#include "Object.h"

enum ACTUATE_TYPE{SHRINK_LEFT, SHRINK_RIGHT, RELEASE_LEFT, RELEASE_RIGHT,
				  SHRINK_UP, SHRINK_DOWN, RELEASE_UP, RELEASE_DOWN,
	              STATIC};

class Deformable3D:public Object{
    
private:
    //real data
    Eigen::Vector3i m_Num;
	Eigen::Vector3f m_Color;
	Eigen::Vector3f m_Position;
	Eigen::Vector3f m_Size;

    float m_Density;//mass density over the volume
    float m_Mu;//coefficient for neo-hookean
    float m_Lambda;
    float m_Gamma;//damping coefficient
	bool m_is_init;
    
    //below are for drawing
    GLuint m_vertexArrayObject;                      
    GLuint m_vertexBufferObject;  
    GLuint m_shader;
    
    //data for drawing, will be sent to GPU
    int m_NTrianglePoints;
    Eigen::Vector4f *m_TrianglePoints;
    Eigen::Vector4f *m_TriangleColors;
    Eigen::Vector3f *m_TPointNormals;
    
    bool m_Manipulated;
    Eigen::Vector3f m_Maniposition;//the position of the manipulator in 3D
    Node *m_MadNode;//the manipulated node
    Camera m_Eye;

	double m_accum_shrink_left;
	double m_accum_shrink_right;
	double m_accum_shrink_up;
	double m_accum_shrink_down;

	double m_timer;

public:
    Mesh3D* m_Mesh;
    Mesh3D* m_Rest_Mesh;
	Mesh3D* m_Init_Mesh;
public:
     Deformable3D();
    ~Deformable3D();
	 
	 void Init(Eigen::Vector3i Num, float density, float youngs, float poisson, float gamma, Eigen::Vector3f position, Eigen::Vector3f size,Eigen::Vector3f color);//position is the center of the bottom plane
	 
	 void UpdateAll(double dt);//Update the position, velocity, force, based on Newton Mechanics, Using Implicit Euler Method, Update the vertexs for drawing
	 void Draw(int type,const Camera& camera, const Light& light);//Update data on GPU's buffer and draw the vertexs
	 void MouseLeft(float cursorX, float cursorY, const Camera& camera);//react to left mouse click
     void MouseRight(float cursorX, float cursorY, const Camera& camera);//react to right mouse click
	 void MouseMove(const Camera& camera, double cursor_x, double cursor_y);

	 void Output2File(std::ofstream* filestream);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    void muscleController(int horizontal_torque, int verticle_torque, float dt, float alpha);

private:
    void UpdateRestShape(double dt, double alpha, ACTUATE_TYPE type);
	void InitDraw();//Init the vertexs and color data on GPU, Init the shader program, link the shader program with the buffer data
    void UpdatePhysics(double dt);
	void UpdateForce();
	void UpdatePosition(double dt);
	void UpdateDraw();//update the data for drawing
    void Select(double cursorX,double cursorY,double range, const Camera& camera);//Give the mouse selection position (screen coordinate)
    void Deselect(){m_Manipulated = false; std::cout<<"Node Dismanipulated!"<<std::endl;}
    void Anchor(){m_MadNode->m_Fixed = 1; std::cout<<"Node Fixed!"<<std::endl;};//Anchor the current Manipualated Node
    void Deanchor(){m_MadNode->m_Fixed = 0; std::cout<<"Node Released!"<<std::endl;}
	void HandleCollision(Node& a_node);
};

#endif
