//
//  3DDeformable.cpp
//
//  Created by Jingyi Fang on 3/13/12.
//  All rights reserved.
//


#include <cmath>
#include <queue>

#include "3DDeformable.h"
#include "Object.h"
#include "Terrain.h"

typedef std::queue<Node*> NodeQueue;

Deformable3D::Deformable3D(){
	m_is_init = false;
	m_Manipulated = false;
    m_type = TypeDeformable3D;
}

Deformable3D::~Deformable3D(){

}

void Deformable3D::Init(Eigen::Vector3i Num, float density,float youngs, float poisson, float gamma, Eigen::Vector3f position, Eigen::Vector3f size, Eigen::Vector3f color){
    
    m_Num = Num;
    m_Mu = youngs/(2*(1+poisson));
    m_Lambda = youngs*poisson/((1+poisson)*(1-2*poisson));
    m_Gamma = gamma;
    m_Density = density;//default mass for nodes
	m_Position = position;
	m_Color = color;
	m_Size = size;

	Eigen::Vector3f mesh_size;
	mesh_size[0] = m_Size[0]/m_Num[0];
	mesh_size[1] = m_Size[1]/m_Num[1];
	mesh_size[2] = m_Size[2]/m_Num[2];
    m_Mesh = new Mesh3D(Num, mesh_size, position);//generate the mesh of tetras and nodes and triangles, down, left, back corner at position
    m_Rest_Mesh = new Mesh3D(Num, mesh_size, -0.5*m_Size);
	m_Init_Mesh =new Mesh3D(Num, mesh_size, -0.5*m_Size);

	m_accum_shrink_left = 0;
	m_accum_shrink_right = 0;
	m_accum_shrink_up = 0;
	m_accum_shrink_down = 0;

	UpdateRestShape(0,0,STATIC);
    
	//Init Node Mass
	Tetrahedron* temp_tetra;
    for(int id = 0; id < m_Mesh->m_Num_Tetra; id++){
        temp_tetra = &m_Mesh->m_Tetras[id];
        //distribute mass for each tetra
        temp_tetra->m_node_1->m_Mass +=  0.25*temp_tetra->m_volume*m_Density;
        temp_tetra->m_node_2->m_Mass +=  0.25*temp_tetra->m_volume*m_Density;
        temp_tetra->m_node_3->m_Mass +=  0.25*temp_tetra->m_volume*m_Density;
        temp_tetra->m_node_4->m_Mass +=  0.25*temp_tetra->m_volume*m_Density;
    }

	InitDraw();

	m_is_init = true;
    
}



void Deformable3D::UpdateRestShape(double dt, double alpha, ACTUATE_TYPE type){
	
	double threshold = 0.2;
	//experiment 
	switch(type){
	case SHRINK_LEFT:
		m_accum_shrink_left += alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_left > threshold){
			m_accum_shrink_left = threshold;
			break;//shrink threshold
		}
		else{
				double z_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					z_ratio = (0.5 + m_Rest_Mesh->m_Nodes[i].m_Position[2]/m_Rest_Mesh->m_Size[2]);
					//z_ratio is 1 on the left most nodes, 0 on the right most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 - z_ratio*alpha*dt;
				}
		}
		break;
	case SHRINK_RIGHT:
		m_accum_shrink_right += alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_right > threshold){
			m_accum_shrink_right = threshold;
			break;//shrink threshold
		}
		else{
				double z_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					z_ratio = (0.5 - m_Rest_Mesh->m_Nodes[i].m_Position[2]/m_Rest_Mesh->m_Size[2]);
					//z_ratio is 1 on the right most nodes, 0 on the left most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 - z_ratio*alpha*dt; 
				}
		}
		break;
	case SHRINK_UP:
		m_accum_shrink_up += alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_up > threshold){
			m_accum_shrink_up = threshold;
			break;//shrink threshold
		}
		else{
				double y_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					y_ratio = (0.5 + m_Rest_Mesh->m_Nodes[i].m_Position[1]/m_Rest_Mesh->m_Size[1]);
					//y_ratio is 1 on the up most nodes, 0 on the down most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 - y_ratio*alpha*dt; 
				}
		}
		break;
	case SHRINK_DOWN:
		m_accum_shrink_down += alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_up > threshold){
			m_accum_shrink_up = threshold;
			break;//shrink threshold
		}
		else{
				double y_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					y_ratio = (0.5 - m_Rest_Mesh->m_Nodes[i].m_Position[1]/m_Rest_Mesh->m_Size[1]);
					//y_ratio is 1 on the up most nodes, 0 on the down most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 - y_ratio*alpha*dt; 
				}
		}
		break;
	case RELEASE_LEFT:
		m_accum_shrink_left -= alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_left < 0)
		{
			m_accum_shrink_left = 0;
			for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					m_Rest_Mesh->m_Nodes[i].m_Position = m_Init_Mesh->m_Nodes[i].m_Position;
				}
		}
		else{
				double z_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					z_ratio = (0.5 + m_Rest_Mesh->m_Nodes[i].m_Position[2]/m_Rest_Mesh->m_Size[2]);
					//z_ratio is 1 on the left most nodes, 0 on the right most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 + z_ratio*alpha*dt;
				}
		}
		break;
	case RELEASE_RIGHT:
		m_accum_shrink_right -= alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_right < 0)
		{
			m_accum_shrink_right = 0;
			for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					m_Rest_Mesh->m_Nodes[i].m_Position = m_Init_Mesh->m_Nodes[i].m_Position;
				}
		}
		else{
				double z_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					z_ratio = (0.5 - m_Rest_Mesh->m_Nodes[i].m_Position[2]/m_Rest_Mesh->m_Size[2]);
					//z_ratio is 1 on the right  most nodes, 0 on the left most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 + z_ratio*alpha*dt; 
				}
		}
		break;
	case RELEASE_UP:
		m_accum_shrink_up -= alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_up < 0)
		{
			m_accum_shrink_up = 0;
			for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					m_Rest_Mesh->m_Nodes[i].m_Position = m_Init_Mesh->m_Nodes[i].m_Position;
				}
		}
		else{
				double y_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					y_ratio = (0.5 + m_Rest_Mesh->m_Nodes[i].m_Position[1]/m_Rest_Mesh->m_Size[1]);
					//y_ratio is 1 on the up most nodes, 0 on the down most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 + y_ratio*alpha*dt; 
				}
		}
		break;
	case RELEASE_DOWN:
		m_accum_shrink_down -= alpha*dt;//total shrink decay term x(t) = x(0)*exp(-shrink_decay);
		if(m_accum_shrink_down < 0)
		{
			m_accum_shrink_down = 0;
			for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					m_Rest_Mesh->m_Nodes[i].m_Position = m_Init_Mesh->m_Nodes[i].m_Position;
				}
		}
		else{
				double y_ratio;
				for(int i =0; i < m_Rest_Mesh->m_Num_Node; i++)
				{
					y_ratio = (0.5 - m_Rest_Mesh->m_Nodes[i].m_Position[1]/m_Rest_Mesh->m_Size[1]);
					//y_ratio is 1 on the up most nodes, 0 on the down most nodes
					m_Rest_Mesh->m_Nodes[i].m_Position[0] *= 1 + y_ratio*alpha*dt; 
				}
		}
		break;
	case STATIC:
	default:
		//donothing
		break;
	
	}

	//create the Dm matrix for the tetra

	Eigen::Vector3f X_2_1;
	Eigen::Vector3f X_3_1;	
	Eigen::Vector3f X_4_1;
	Tetrahedron* material_tetra, *real_tetra;
	Eigen::Matrix3f M_D_m;

	 for(int id = 0; id < m_Mesh->m_Num_Tetra; id++){
        material_tetra = &m_Rest_Mesh->m_Tetras[id];
		real_tetra = &m_Mesh->m_Tetras[id];
        X_2_1 = material_tetra->m_node_2->m_Position -  material_tetra->m_node_1->m_Position;
        X_3_1 = material_tetra->m_node_3->m_Position -  material_tetra->m_node_1->m_Position;
        X_4_1 = material_tetra->m_node_4->m_Position -  material_tetra->m_node_1->m_Position;
        
        M_D_m << X_2_1[0],X_3_1[0], X_4_1[0], 
        X_2_1[1],X_3_1[1], X_4_1[1],
        X_2_1[2],X_3_1[2], X_4_1[2];

        real_tetra->m_volume = 0.5*fabs(M_D_m.determinant());
        real_tetra->m_D_m_Inverse = M_D_m.inverse();
        real_tetra->m_D_m_Inverse_Transpose = M_D_m.inverse().transpose();

	 }
}


<<<<<<< HEAD
void Deformable3D::muscleController(int horizontal_torque, int verticle_torque , float dt, float alpha, int segment_num)
{
    if(horizontal_torque==2){
        UpdateRestShape(dt, alpha, SHRINK_RIGHT);
        //m_accum_shrink_left -= alpha*dt;
        //std::cout<<"Segment #"<<segment_num<<": Shrink Right"<<std::endl;
    }
    else if(horizontal_torque==1)
    {
        UpdateRestShape(dt, alpha, RELEASE_RIGHT);
        //m_accum_shrink_right -= alpha*dt;
        //std::cout<<"Segment #"<<segment_num<<": Release Right"<<std::endl;
      
    }
    else if(horizontal_torque == -1)
    {
       UpdateRestShape(dt, alpha, SHRINK_LEFT);
       //m_accum_shrink_right -= alpha*dt;
        
        //std::cout<<"Segment #"<<segment_num<<": Shrink Left"<<std::endl;
    }
    else if(horizontal_torque== -2){
        UpdateRestShape(dt, alpha, RELEASE_LEFT);
        //m_accum_shrink_left -= alpha*dt;
        
        //std::cout<<"Segment #"<<segment_num<<": Release Left"<<std::endl;
    }

    
    if(verticle_torque==1)
    {
        UpdateRestShape(dt, alpha, SHRINK_UP);
        m_accum_shrink_down -= alpha*dt;
    }
    else if(verticle_torque ==-1)
    {
        UpdateRestShape(dt, alpha, SHRINK_DOWN);
        m_accum_shrink_up -= alpha*dt;
    }
}
=======
>>>>>>> 6ff2e95d2e84aa57e0fdf32070fa8b66098d0561

void Deformable3D::InitDraw(){
    
    // Initialize the data array on CPU
       
    m_NTrianglePoints = 3*20*m_Num[0]*m_Num[1]*m_Num[2];
    m_TrianglePoints = new Eigen::Vector4f[m_NTrianglePoints];
    m_TPointNormals = new Eigen::Vector3f[m_NTrianglePoints];
    m_TriangleColors = new Eigen::Vector4f[m_NTrianglePoints];

	for(int id = 0; id < m_NTrianglePoints; id++)
		m_TriangleColors[id] = Eigen::Vector4f(0,1.0,0,1.0);

    //Create the Vertex Array and Buffers, bind them
#ifdef __APPLE__
	glGenVertexArraysAPPLE(1, &m_vertexArrayObject);
#else
	 glGenVertexArrays(1, &m_vertexArrayObject);
#endif
   
    glGenBuffers(1, &m_vertexBufferObject);//generate buffer for current vertex array
    
    //load and compile shaders on GPU, use current shader program
    m_shader = Util::InitShader( "vSmoothPhong.vert", "fSmoothPhong.frag" );
    glUseProgram(m_shader);
}

void Deformable3D::UpdateDraw(){

	//init the normals as zero
	for(int id = 0; id< m_Num[0]*m_Num[1]*m_Num[2]*5; id++){

		m_Mesh->m_Tetras[id].m_node_1->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_1->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_1->m_Normal *= 0;

		m_Mesh->m_Tetras[id].m_node_2->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_2->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_2->m_Normal *= 0;

		m_Mesh->m_Tetras[id].m_node_3->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_3->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_3->m_Normal *= 0;

		m_Mesh->m_Tetras[id].m_node_4->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_4->m_Normal *= 0;
		m_Mesh->m_Tetras[id].m_node_4->m_Normal *= 0;  

	}

	Tetrahedron* temp_tetra;
	for(int id = 0; id< m_Num[0]*m_Num[1]*m_Num[2]*5; id++){
		temp_tetra = &m_Mesh->m_Tetras[id];
        //update the position for drawing
        m_TrianglePoints[12*id] = Eigen::Vector4f(temp_tetra->m_node_1->m_Position[0],temp_tetra->m_node_1->m_Position[1],temp_tetra->m_node_1->m_Position[2],1.0);
        m_TrianglePoints[12*id+1] = Eigen::Vector4f(temp_tetra->m_node_2->m_Position[0],temp_tetra->m_node_2->m_Position[1],temp_tetra->m_node_2->m_Position[2],1.0);
        m_TrianglePoints[12*id+2] = Eigen::Vector4f(temp_tetra->m_node_3->m_Position[0],temp_tetra->m_node_3->m_Position[1],temp_tetra->m_node_3->m_Position[2],1.0);

		m_TrianglePoints[12*id+3] = Eigen::Vector4f(temp_tetra->m_node_1->m_Position[0],temp_tetra->m_node_1->m_Position[1],temp_tetra->m_node_1->m_Position[2],1.0);
        m_TrianglePoints[12*id+4] = Eigen::Vector4f(temp_tetra->m_node_2->m_Position[0],temp_tetra->m_node_2->m_Position[1],temp_tetra->m_node_2->m_Position[2],1.0);
        m_TrianglePoints[12*id+5] = Eigen::Vector4f(temp_tetra->m_node_4->m_Position[0],temp_tetra->m_node_4->m_Position[1],temp_tetra->m_node_4->m_Position[2],1.0);
		
		m_TrianglePoints[12*id+6] = Eigen::Vector4f(temp_tetra->m_node_1->m_Position[0],temp_tetra->m_node_1->m_Position[1],temp_tetra->m_node_1->m_Position[2],1.0);
        m_TrianglePoints[12*id+7] = Eigen::Vector4f(temp_tetra->m_node_4->m_Position[0],temp_tetra->m_node_4->m_Position[1],temp_tetra->m_node_4->m_Position[2],1.0);
        m_TrianglePoints[12*id+8] = Eigen::Vector4f(temp_tetra->m_node_3->m_Position[0],temp_tetra->m_node_3->m_Position[1],temp_tetra->m_node_3->m_Position[2],1.0);
		 
		m_TrianglePoints[12*id+9] = Eigen::Vector4f(temp_tetra->m_node_4->m_Position[0],temp_tetra->m_node_4->m_Position[1],temp_tetra->m_node_4->m_Position[2],1.0);
        m_TrianglePoints[12*id+10] = Eigen::Vector4f(temp_tetra->m_node_2->m_Position[0],temp_tetra->m_node_2->m_Position[1],temp_tetra->m_node_2->m_Position[2],1.0);
        m_TrianglePoints[12*id+11] = Eigen::Vector4f(temp_tetra->m_node_3->m_Position[0],temp_tetra->m_node_3->m_Position[1],temp_tetra->m_node_3->m_Position[2],1.0);
       
		Eigen::Vector3f v_1_2 = temp_tetra->m_node_2->m_Position - temp_tetra->m_node_1->m_Position;
		Eigen::Vector3f v_1_3 = temp_tetra->m_node_3->m_Position - temp_tetra->m_node_1->m_Position;
		Eigen::Vector3f v_1_4 = temp_tetra->m_node_4->m_Position - temp_tetra->m_node_1->m_Position;
		Eigen::Vector3f v_2_3 = temp_tetra->m_node_3->m_Position - temp_tetra->m_node_2->m_Position;
		Eigen::Vector3f v_2_4 = temp_tetra->m_node_4->m_Position - temp_tetra->m_node_2->m_Position;

		//update the normal on each node for drawing
		//the normals of shared closed faces are canceled during addition, leaving only normals for open faces
		m_Mesh->m_Tetras[id].m_face_normal_123 = v_1_3.cross(v_1_2);
		m_Mesh->m_Tetras[id].m_face_normal_124 = v_1_2.cross(v_1_4);
		m_Mesh->m_Tetras[id].m_face_normal_134 = v_1_4.cross(v_1_3);
		m_Mesh->m_Tetras[id].m_face_normal_234 = v_2_3.cross(v_2_4);

		m_Mesh->m_Tetras[id].m_node_1->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_123;
		m_Mesh->m_Tetras[id].m_node_1->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_124;
		m_Mesh->m_Tetras[id].m_node_1->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_134;

		m_Mesh->m_Tetras[id].m_node_2->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_123;
		m_Mesh->m_Tetras[id].m_node_2->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_124;
		m_Mesh->m_Tetras[id].m_node_2->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_234;

		m_Mesh->m_Tetras[id].m_node_3->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_123;
		m_Mesh->m_Tetras[id].m_node_3->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_134;
		m_Mesh->m_Tetras[id].m_node_3->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_234;

		m_Mesh->m_Tetras[id].m_node_4->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_124;
		m_Mesh->m_Tetras[id].m_node_4->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_134;
		m_Mesh->m_Tetras[id].m_node_4->m_Normal += m_Mesh->m_Tetras[id].m_face_normal_234;  
    }

	//update the normals for each triangle point
	for(int id = 0; id< m_Num[0]*m_Num[1]*m_Num[2]*5; id++){

		m_TPointNormals[12*id] = m_Mesh->m_Tetras[id].m_node_1->m_Normal;
        m_TPointNormals[12*id+1] = m_Mesh->m_Tetras[id].m_node_2->m_Normal;
        m_TPointNormals[12*id+2] = m_Mesh->m_Tetras[id].m_node_3->m_Normal;

		m_TPointNormals[12*id+3] = m_Mesh->m_Tetras[id].m_node_1->m_Normal;
        m_TPointNormals[12*id+4] = m_Mesh->m_Tetras[id].m_node_2->m_Normal;
        m_TPointNormals[12*id+5] = m_Mesh->m_Tetras[id].m_node_4->m_Normal;
		
		m_TPointNormals[12*id+6] = m_Mesh->m_Tetras[id].m_node_1->m_Normal;
        m_TPointNormals[12*id+7] = m_Mesh->m_Tetras[id].m_node_4->m_Normal;
        m_TPointNormals[12*id+8] = m_Mesh->m_Tetras[id].m_node_3->m_Normal;
		 
		m_TPointNormals[12*id+9] = m_Mesh->m_Tetras[id].m_node_4->m_Normal;
        m_TPointNormals[12*id+10] = m_Mesh->m_Tetras[id].m_node_2->m_Normal;
        m_TPointNormals[12*id+11] = m_Mesh->m_Tetras[id].m_node_3->m_Normal;

	}
}

void Deformable3D::UpdateForce(){

    //init with gravity
	for(int i = 0; i< m_Mesh->m_Num_Node; i++){
		m_Mesh->m_Nodes[i].m_Force = Eigen::Vector3f(0,-GRAVITY_CONSTANT*m_Mesh->m_Nodes[i].m_Mass,0);
    }   
    
	
    //Loop through each tetra element, calculate the elastic force for each of the three nodes; add the force to current node force;
    Eigen::Vector3f x_2_1;
    Eigen::Vector3f x_3_1;
	Eigen::Vector3f x_4_1;
    Eigen::Vector3f f_1;
    Eigen::Vector3f f_2;
    Eigen::Vector3f f_3;
	Eigen::Vector3f f_4;
    Eigen::Vector3f e_velocity;
    Eigen::Matrix3f M_Ds;
    Eigen::Matrix3f M_F;
    Eigen::Matrix3f M_F_Inverse_Transpose;
    Eigen::Matrix3f M_P;
    Eigen::Matrix3f F_2_F_3_F_4;
    Tetrahedron* temp_tetra;

    for(int i = 0; i < m_Mesh->m_Num_Tetra; i++){//loop through all the elements to calculate elastic force as well as damping force
            
		temp_tetra = &m_Mesh->m_Tetras[i];

        x_2_1 = temp_tetra->m_node_2->m_Position - temp_tetra->m_node_1->m_Position;
		x_3_1 = temp_tetra->m_node_3->m_Position - temp_tetra->m_node_1->m_Position;
		x_4_1 = temp_tetra->m_node_4->m_Position - temp_tetra->m_node_1->m_Position;

        //First calculate Ds 
        M_Ds << x_2_1[0],x_3_1[0], x_4_1[0], 
			    x_2_1[1],x_3_1[1], x_4_1[1],
				x_2_1[2],x_3_1[2], x_4_1[2];
        //then calculate F
        M_F = M_Ds*temp_tetra->m_D_m_Inverse;
        //then calculate P, need mu and lambda as parameters
        M_F_Inverse_Transpose = M_F.inverse().transpose();
        M_P = m_Mu*(M_F - M_F_Inverse_Transpose) + m_Lambda*log(M_F.determinant())*M_F_Inverse_Transpose;
        //the calculate force

		// this might be wrong
        F_2_F_3_F_4 = -temp_tetra->m_volume*M_P*temp_tetra->m_D_m_Inverse_Transpose;
        f_2 = Eigen::Vector3f(F_2_F_3_F_4(0,0),F_2_F_3_F_4(1,0),F_2_F_3_F_4(2,0));
        f_3 = Eigen::Vector3f(F_2_F_3_F_4(0,1),F_2_F_3_F_4(1,1),F_2_F_3_F_4(2,1));
		f_4 = Eigen::Vector3f(F_2_F_3_F_4(0,2),F_2_F_3_F_4(1,2),F_2_F_3_F_4(2,2));
        f_1 = -f_2 - f_3 - f_4;
            
        //distribute the force on this element to the nodes
        temp_tetra->m_node_1->m_Force += f_1;
        temp_tetra->m_node_2->m_Force += f_2;
        temp_tetra->m_node_3->m_Force += f_3;
        temp_tetra->m_node_4->m_Force += f_4;

        //add damping for velocity
        e_velocity = (temp_tetra->m_node_1->m_Velocity + temp_tetra->m_node_2->m_Velocity + temp_tetra->m_node_3->m_Velocity + temp_tetra->m_node_4->m_Velocity)/4.0;
        temp_tetra->m_node_1->m_Force += -m_Gamma*temp_tetra->m_volume*(temp_tetra->m_node_1->m_Velocity - e_velocity);
        temp_tetra->m_node_2->m_Force += -m_Gamma*temp_tetra->m_volume*(temp_tetra->m_node_2->m_Velocity - e_velocity);
        temp_tetra->m_node_3->m_Force += -m_Gamma*temp_tetra->m_volume*(temp_tetra->m_node_3->m_Velocity - e_velocity);
		temp_tetra->m_node_4->m_Force += -m_Gamma*temp_tetra->m_volume*(temp_tetra->m_node_4->m_Velocity - e_velocity);
        //now force is updated for all nodes
    }   

}

void Deformable3D::HandleCollision(Node& a_node){

	Terrain* terrain;
	Eigen::Vector3f material_point = a_node.m_Position;
	Eigen::Vector3f surface_normal;
	Eigen::Vector3f prev_momentom_n, prev_momentom_v, new_momentom_n, new_momentom_v;
	Eigen::Vector3f new_velocity_n_vector;
	double friction_ness = 1.0;//from 0~2 is enough
	double rebouce_ness = 0.2;
    for (unsigned int i = 0; i< m_world->List_of_Object.size(); i++) {
		if(m_world->List_of_Object[i] == this)
			continue;//this one is itself, no self-collision
        //find the first intersection and calculate the force, return the force
        switch (m_world->List_of_Object[i]->GetType()) {
			case TypeTerrain:
			{
				terrain = dynamic_cast<Terrain*>(m_world->List_of_Object[i]);
				double dist_y = material_point[1] - terrain->GetHeight(Eigen::Vector2f(material_point[0],material_point[2]));
				if(dist_y < 0){//penetration
					surface_normal = terrain->GetNormal(Eigen::Vector2f(material_point[0],material_point[2]));
					surface_normal.normalize();
					
					prev_momentom_n = a_node.m_Mass*a_node.m_Velocity.dot(surface_normal)*surface_normal;
					prev_momentom_v = a_node.m_Mass*a_node.m_Velocity - prev_momentom_n;
					new_velocity_n_vector = prev_momentom_v.normalized();

					new_momentom_n = -rebouce_ness* prev_momentom_n;
#ifdef __APPLE__
                    new_momentom_v = new_velocity_n_vector*fmax(prev_momentom_v.norm() - friction_ness*(new_momentom_n.norm() + prev_momentom_n.norm()),0);
                    
#else
                    new_momentom_v = new_velocity_n_vector*max(prev_momentom_v.norm() - friction_ness*(new_momentom_n.norm() + prev_momentom_n.norm()),0);
#endif
					a_node.m_Velocity = (new_momentom_n + new_momentom_v)/a_node.m_Mass;

					return;
				}
			}
			break;
            default:
                break;
        }
    }
    
    return;
	
}

void Deformable3D::UpdatePosition(double dt){
	   
    //update position and velocity
   
   for(int i = 0; i< m_Mesh->m_Num_Node; i++){

       if(!m_Mesh->m_Nodes[i].m_Fixed){//check weather fixed or not
            m_Mesh->m_Nodes[i].m_Velocity += dt*m_Mesh->m_Nodes[i].m_Force/m_Mesh->m_Nodes[i].m_Mass;
            m_Mesh->m_Nodes[i].m_Position += dt*m_Mesh->m_Nodes[i].m_Velocity;
        }
       else{
            m_Mesh->m_Nodes[i].m_Velocity = Eigen::Vector3f(0,0,0);
       }              
   
   }
   
   //collision detection and resolution. Friction also handled here
   for(int i = 0; i< m_Mesh->m_Num_Node; i++){
	   if(!m_Mesh->m_Nodes[i].m_Fixed)
			HandleCollision(m_Mesh->m_Nodes[i]);
   }

  //deal with manipulated node
    if(m_Manipulated){
        
        m_MadNode->m_Position = m_Maniposition;//x,y,z is the position of the manipulator
        m_MadNode->m_Velocity = Eigen::Vector3f(0,0,0);
        m_MadNode->m_Force = Eigen::Vector3f(0,0,0);
        
    }

}

void Deformable3D::UpdatePhysics(double dt){

	UpdateForce();
	UpdatePosition(dt);
    
}
 
void Deformable3D::UpdateAll(double dt){

	
	m_timer += dt;
    /*
	if(m_timer < 2)
		UpdateRestShape(dt, 1, SHRINK_LEFT);
	else if(m_timer <4)
		UpdateRestShape(dt, 1, RELEASE_LEFT);
    else if(m_timer <6)
		UpdateRestShape(dt, 1, SHRINK_RIGHT);
    else if(m_timer <8)
		UpdateRestShape(dt, 1, RELEASE_RIGHT);
    else if(m_timer <10)
		UpdateRestShape(dt, 1, SHRINK_UP);
    else if(m_timer <12)
		UpdateRestShape(dt, 1, RELEASE_UP);
*/
	UpdatePhysics(dt);
    //update drawing buffer: Points and Colors, Normals, etc
    UpdateDraw();
                    
}

void Deformable3D::MouseMove(const Camera& camera, double cursor_x, double cursor_y){

	float zoom_factor = (camera.m_position[2] - m_Maniposition[2])/(camera.m_position[2] - camera.m_znear);
        
    m_Maniposition[0] = zoom_factor*cursor_x*(camera.m_position[2] - camera.m_znear)*tan(DegreesToRadians*camera.m_fovy/2);
    m_Maniposition[1] = zoom_factor*cursor_y*(camera.m_position[2] - camera.m_znear)*tan(DegreesToRadians*camera.m_fovy*camera.m_aspect/2);

}
 
void Deformable3D::Draw(int type, const Camera& camera, const Light& light){
    

    //dt is the timestep, x and y are the cursor's input, camera defines the view atmrix and projection matrix
    //update the position and color of vertexesfor drawing
    if(!m_is_init)
		return;

    glUseProgram(m_shader);
#ifdef __APPLE__
    glBindVertexArrayAPPLE(m_vertexArrayObject);//use as current vertex array    
#else
	glBindVertexArray(m_vertexArrayObject);//use as current vertex array    
#endif
   
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);

    //Get new position of the cube and update the model view Angel::Angel::Angel::matrix
    Eigen::Affine3f cMw;
    Eigen::Affine3f proj;
    
    GLint world2camera = glGetUniformLocation(m_shader, "cMw"); 
	GLint projection = glGetUniformLocation(m_shader, "proj");
    GLint kAmbient = glGetUniformLocation(m_shader,"kAmbient");
    GLint kDiffuse = glGetUniformLocation(m_shader,"kDiffuse");
    GLint kSpecular = glGetUniformLocation(m_shader,"kSpecular");
    GLint shininess = glGetUniformLocation(m_shader,"shininess");
    GLint camera_position = glGetUniformLocation(m_shader, "cameraPosition");
    GLint light_position = glGetUniformLocation(m_shader, "lightPosition");
    
    //generate the Angel::Angel::Angel::matrixes
    proj = Util::Perspective( camera.m_fovy, camera.m_aspect, camera.m_znear, camera.m_zfar );
	cMw = camera.m_cMw;//LookAt(camera.position,camera.lookat, camera.up );
    
    Eigen::Vector4f v4color(m_Color[0],m_Color[1],m_Color[2],1.0);
    Eigen::Vector4f Ambient;
    Ambient = 0.3*v4color;
    Eigen::Vector4f Diffuse;
    Diffuse = 0.5*v4color;
    Eigen::Vector4f Specular(0.3,0.3,0.3,1.0);
    
    glUniformMatrix4fv( world2camera, 1, GL_FALSE, cMw.data() );
    glUniformMatrix4fv( projection, 1, GL_FALSE, proj.data() );
    
    glUniform4fv(kAmbient, 1, Ambient.data());
    glUniform4fv(kDiffuse, 1, Diffuse.data()); 
    glUniform4fv(kSpecular, 1, Specular.data());
    glUniform4fv(camera_position, 1, camera.m_position.data());
    glUniform4fv(light_position, 1, light.m_position.data());
    glUniform1f(shininess, 10);

    
    GLuint position;
    GLuint color;
    GLuint normal;
    
    //send the updated data to buffer
    glBufferData(GL_ARRAY_BUFFER, (sizeof(m_TrianglePoints[0]) + sizeof(m_TriangleColors[0]) + sizeof(m_TPointNormals[0]))*m_NTrianglePoints, NULL, GL_STATIC_DRAW);//send data to current buffer
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_TrianglePoints[0])*m_NTrianglePoints, m_TrianglePoints);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_TrianglePoints[0])*m_NTrianglePoints, sizeof(m_TriangleColors[0])*m_NTrianglePoints, m_TriangleColors);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_TriangleColors[0])*m_NTrianglePoints+sizeof(m_TrianglePoints[0])*m_NTrianglePoints,sizeof(m_TPointNormals[0])*m_NTrianglePoints, m_TPointNormals);
    
    position = glGetAttribLocation( m_shader, "vPosition" );
    glEnableVertexAttribArray( position );
    glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0)); 
    
    color = glGetAttribLocation(m_shader, "vColor");
    glEnableVertexAttribArray(color);
    glVertexAttribPointer(color, 4, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET(sizeof(m_TrianglePoints[0])*m_NTrianglePoints));
    
    normal = glGetAttribLocation(m_shader, "vNormal");
    glEnableVertexAttribArray(normal);
    glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET((sizeof(m_TriangleColors[0])+sizeof(m_TrianglePoints[0]))*m_NTrianglePoints));
    
    switch (type) {
        case DRAW_MESH:
            glUniform1i(glGetUniformLocation(m_shader, "renderType"), 1);
            glDrawArrays(GL_LINES, 0, m_NTrianglePoints);
            break;
        case DRAW_PHONG:
            glUniform1i(glGetUniformLocation(m_shader, "renderType"), 2);
            glDrawArrays(GL_TRIANGLES, 0, m_NTrianglePoints);
            break;
    }
    
    //remember the current camera for selction
    m_Eye = camera;//shadow copy, but okay since Camera Class has no pointers
    
	
}

void Deformable3D::Select(double x, double y, double range, const Camera& camera){
	
    //Based on the mouse clicked position, shot ray into the 3D scene,
    //for each node, search for the nearest node within range to the shoted ray
    
    //equation of the ray: (0,0,Camera.Position) + t*(tan(Camera.fovy/2*x),tan(Camera.fovy*Camera.aspect/2*y),1);
    Eigen::Vector4f CameraPosition(camera.m_position);
    Eigen::Vector3f Ray(tan(DegreesToRadians*x*camera.m_fovy/2),tan(DegreesToRadians*y*camera.m_fovy*camera.m_aspect/2),-1.0);//vector
    Ray.normalize();
    Eigen::Vector3f Camera2Node;
    double temp;
    //Search for the node closest to the ray within the range
    double mindist = 1000;
    double tempdist = 1000;
    
    for(int id = 0; id< m_Mesh->m_Num_Node; id++){
		m_Mesh->m_Nodes[id].m_Force = Eigen::Vector3f(0,-GRAVITY_CONSTANT*m_Mesh->m_Nodes[id].m_Mass,0);
            Camera2Node = m_Mesh->m_Nodes[id].m_Position - CameraPosition.head(3);
            temp = Camera2Node.dot(Ray);
            tempdist = sqrt(Camera2Node.norm()*Camera2Node.norm() - temp*temp);
            //distance of current node to the ray
            
            if(tempdist < mindist){
                m_MadNode = &(m_Mesh->m_Nodes[id]);
                mindist = tempdist;
            }
    }
        
    if(mindist < range){
        m_Manipulated = true;
        std::cout<<"Node Manipulated!"<<std::endl;
        //determine the position of the manipulator here
        m_Maniposition = Eigen::Vector3f(m_MadNode->m_Position);
    }
    else{
        Deselect();
    } 
    
    //printf("Left Button Clicked: %f, %f\n",x,y);

	
}

void Deformable3D::MouseLeft(float x, float y, const Camera& camera){

    //for testing the click return value:
    
    printf("Cursor Clicked: x: %f, y: %f\n",x,y);
    
    //Search for selected point
    if(this->m_Manipulated){//if already manipulated, anchor when left click again
        this->Anchor();
        this->Deselect();
    }
    else{
        this->Select(x,y, 0.5, camera);
    }

}

void Deformable3D::MouseRight(float x, float y, const Camera& camera){
    if(this->m_Manipulated){
        this->Deanchor();
        this->Deselect();
    }
}

void Deformable3D::Output2File(std::ofstream* filestream){

	(*filestream)<<"//BEGIN DEFORMABLE"<<std::endl;
	(*filestream)<<"mesh2{"<<std::endl;
	std::vector<Node*> nodes;
	std::vector<int> faces;

	m_Mesh->GetSurface(nodes, faces);

    for(int i=0;i<nodes.size();i++){
        nodes[i]->m_Normal.normalize();
    }

	(*filestream)<<"vertex_vectors{"<<nodes.size()<<","<<std::endl;
	for(int i = 0; i < nodes.size()-1; i++){
            (*filestream)<<"<"<<nodes[i]->m_Position[0]<<","<<nodes[i]->m_Position[1]<<","<<nodes[i]->m_Position[2]<<">,"<<std::endl;
	}
        int i = nodes.size()-1;
        (*filestream)<<"<"<<nodes[i]->m_Position[0]<<","<<nodes[i]->m_Position[1]<<","<<nodes[i]->m_Position[2]<<">,"<<std::endl;
	(*filestream)<<"}"<<std::endl; // end vertex_vectors

	(*filestream)<<"normal_vectors{"<<nodes.size()<<","<<std::endl;
	for(int i = 0; i < nodes.size()-1; i++){
            (*filestream)<<"<"<<nodes[i]->m_Normal[0]<<","<<nodes[i]->m_Normal[1]<<","<<nodes[i]->m_Normal[2]<<">,"<<std::endl;
	}
        i = nodes.size()-1;
        (*filestream)<<"<"<<nodes[i]->m_Normal[0]<<","<<nodes[i]->m_Normal[1]<<","<<nodes[i]->m_Normal[2]<<">,"<<std::endl;
	(*filestream)<<"}"<<std::endl; // end normal_vectors

	(*filestream)<<"face_indices{"<<faces.size()/3<<","<<std::endl;
	for(int i = 0; i < faces.size()/3-1; i++)
            (*filestream)<<"<"<<faces[3*i]<<","<<faces[3*i + 1]<<","<<faces[3*i + 2]<<">,"<<std::endl;
        i = faces.size()/3-1;
        (*filestream)<<"<"<<faces[3*i]<<","<<faces[3*i + 1]<<","<<faces[3*i + 2]<<">,"<<std::endl;
	(*filestream)<<"}"<<std::endl; // end face_indices

	(*filestream)<<"}"<<std::endl; // end mesh2
	(*filestream)<<"//END DEFORMABLE"<<std::endl<<std::endl;
}
