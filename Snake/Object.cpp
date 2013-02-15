//
//  Object.cpp
//  174A template
//
//  Created by Jingyi Fang on 4/2/12.
//  Copyright JingyiFang. All rights reserved.
//

#include "Object.h"

Mesh3D::Mesh3D(Eigen::Vector3i a_num, Eigen::Vector3f a_size, Eigen::Vector3f a_position){
    Meshialize(a_num, a_size, a_position);
}

void Mesh3D::Meshialize(Eigen::Vector3i a_num, Eigen::Vector3f a_size, Eigen::Vector3f a_position){
    m_Num = a_num;
    m_Size = a_size;
    
    m_Num_Node = (m_Num[0]+1)*(m_Num[1]+1)*(m_Num[2]+1);
    //m_Num_Trian;
    m_Num_Tetra = m_Num[0]*m_Num[1]*m_Num[2]*5;
    //create a 3D mesh of a_num and each cube of a_size
    
    m_Nodes = new Node[m_Num_Node];
 //   m_Surface = new Triangle[2*(m_Num[0])*(m_Num[1]) + 2*(m_Num[1])*(m_Num[2]) + 2*(m_Num[0])*(m_Num[2])];
    m_Tetras = new Tetrahedron[m_Num_Tetra];
    
    int idx,idy,idz;
    int idc;//id for current node
    
    //the array of nodes
	for(idx = 0; idx< (m_Num[0]+1); idx++){
        for(idy = 0; idy < (m_Num[1]+1); idy++){
            for(idz = 0; idz < (m_Num[2]+1); idz++){
                
                idc = idx*(m_Num[1]+1)*(m_Num[2]+1) + idy*(m_Num[2]+1) + idz;
                
                m_Nodes[idc].m_Position = a_position + Eigen::Vector3f(m_Size[0]*idx,m_Size[1]*idy,m_Size[2]*idz);
                m_Nodes[idc].m_Velocity = Eigen::Vector3f(0,0,0);
                m_Nodes[idc].m_Mass = 0;
                m_Nodes[idc].m_Fixed = false;
            } 
        }
	}

	Node *a, *b, *c, *d, *e, *f, *g, *h;
    //generate the tetrahedrons
	int i_dx, i_dy, i_dz;
	i_dx = (m_Num[1] + 1)*(m_Num[2] + 1);
	i_dy = (m_Num[2] + 1);
	i_dz = 1;
	int idn;
	for(idx = 0; idx< m_Num[0]; idx++){
        for(idy = 0; idy < m_Num[1]; idy++){
            for(idz = 0; idz < m_Num[2]; idz++){
                
                idc = idx*(m_Num[1])*(m_Num[2]) + idy*(m_Num[2]) + idz;//the id of cube cell
				
                //locate the eight nodes wrapping the cube cell
				idn = idx*i_dx + idy*i_dy + idz*i_dz;//the id of a node on the cell
                a = &m_Nodes[idn + i_dz];
                b = &m_Nodes[idn + i_dx + i_dz]; 
                c = &m_Nodes[idn + i_dx]; 
                d = &m_Nodes[idn];
                e = &m_Nodes[idn + i_dz + i_dy]; 
                f = &m_Nodes[idn + i_dx + i_dz + i_dy]; 
                g = &m_Nodes[idn + i_dx + i_dy]; 
                h = &m_Nodes[idn + i_dy];
        
				//cut the cube cell into five tetras.
                m_Tetras[5*idc].m_node_1 = a;
                m_Tetras[5*idc].m_node_2 = d;
                m_Tetras[5*idc].m_node_3 = e;
                m_Tetras[5*idc].m_node_4 = b;
                
                m_Tetras[5*idc + 1].m_node_1 = f;
                m_Tetras[5*idc + 1].m_node_2 = g;
                m_Tetras[5*idc + 1].m_node_3 = b;
                m_Tetras[5*idc + 1].m_node_4 = e;
                
                m_Tetras[5*idc + 2].m_node_1 = c;
                m_Tetras[5*idc + 2].m_node_2 = g;
                m_Tetras[5*idc + 2].m_node_3 = d;
                m_Tetras[5*idc + 2].m_node_4 = b;
                
                m_Tetras[5*idc + 3].m_node_1 = h;
                m_Tetras[5*idc + 3].m_node_2 = g;
                m_Tetras[5*idc + 3].m_node_3 = e;
                m_Tetras[5*idc + 3].m_node_4 = d;
                
                m_Tetras[5*idc + 4].m_node_1 = e;
                m_Tetras[5*idc + 4].m_node_2 = g;
                m_Tetras[5*idc + 4].m_node_3 = b;
                m_Tetras[5*idc + 4].m_node_4 = d;
        
            }
        }
    }
}

std::vector<Node*> Mesh3D::GetTopNodes(){
	//return the nodes on the top surface of the cubic meshnet
	std::vector<Node*> top_nodes;
	top_nodes.clear();
	int idx, idy, idz, idc;
	idy = m_Num[1];
	for(idx = 0; idx< (m_Num[0]+1); idx++){
        for(idz = 0; idz < (m_Num[2]+1); idz++){
			idc = idx*(m_Num[1]+1)*(m_Num[2]+1) + idy*(m_Num[2]+1) + idz;
			top_nodes.push_back(&m_Nodes[idc]);

		}
	}
	
	return top_nodes;
}


void Mesh3D::GetTopSurface(std::vector<Node*>& nodes, std::vector<int>& faces){
	//return the nodes on the top surface of the cubic meshnet
	nodes = GetTopNodes();

	faces.clear();
	int a,b,c,d, idx, idz;
	for(idx = 0; idx< (m_Num[0]); idx++){
        for(idz = 0; idz < (m_Num[2]); idz++){
			a = idx*(m_Num[2]+1) + idz;
			b = (idx+1)*(m_Num[2]+1) + idz;
			c = idx*(m_Num[2]+1) + idz + 1;
			d = (idx+1)*(m_Num[2]+1) + idz + 1;

			faces.push_back(a);
			faces.push_back(b);
			faces.push_back(c);
			faces.push_back(a);
			faces.push_back(c);
			faces.push_back(d);
		}
	}
}



std::vector<Node*> Mesh3D::GetBottomNodes(){
	//return the nodes on the bottom surface of the cubic meshnet
	//return the nodes on the top surface of the cubic meshnet
	std::vector<Node*> bottom_nodes;
	bottom_nodes.clear();
	int idx, idy, idz, idc;
	idy = 0;
	for(idx = 0; idx< (m_Num[0]+1); idx++){
        for(idz = 0; idz < (m_Num[2]+1); idz++){

			idc = idx*(m_Num[1]+1)*(m_Num[2]+1) + idy*(m_Num[2]+1) + idz;
			bottom_nodes.push_back(&m_Nodes[idc]);

		}
	}
	
	return bottom_nodes;
}


void Mesh3D::GetBottomSurface(std::vector<Node*>& nodes, std::vector<int>& faces){
	//return the nodes on the top surface of the cubic meshnet
	nodes = GetBottomNodes();

	faces.clear();
	int a,b,c,d, idx, idz;
	for(idx = 0; idx< (m_Num[0]); idx++){
        for(idz = 0; idz < (m_Num[2]); idz++){
			a = idx*(m_Num[2]+1) + idz;
			b = (idx+1)*(m_Num[2]+1) + idz;
			c = idx*(m_Num[2]+1) + idz + 1;
			d = (idx+1)*(m_Num[2]+1) + idz + 1;

			faces.push_back(a);
			faces.push_back(b);
			faces.push_back(c);
			faces.push_back(a);
			faces.push_back(c);
			faces.push_back(d);
		}
	}
}


std::vector<Node*> Mesh3D::GetFrontNodes(){
	//return the nodes on the bottom surface of the cubic meshnet
	//return the nodes on the top surface of the cubic meshnet
	std::vector<Node*> front_nodes;
	front_nodes.clear();
	int idx, idy, idz, idc;
	idz = m_Num[2];
	for(idx = 0; idx< (m_Num[0]+1); idx++){
        for(idy = 0; idy < (m_Num[1]+1); idy++){
            
			idc = idx*(m_Num[1]+1)*(m_Num[2]+1) + idy*(m_Num[2]+1) + idz;
			front_nodes.push_back(&m_Nodes[idc]);
            
		}
	}
	
	return front_nodes;
}


void Mesh3D::GetFrontSurface(std::vector<Node*>& nodes, std::vector<int>& faces){
	//return the nodes on the top surface of the cubic meshnet
	nodes = GetFrontNodes();

	faces.clear();
	int a,b,c,d, idx, idy;
	for(idx = 0; idx< (m_Num[0]); idx++){
        for(idy = 0; idy < (m_Num[1]); idy++){
			a = idx*(m_Num[1]+1) + idy;
			b = (idx+1)*(m_Num[1]+1) + idy;
			c = idx*(m_Num[1]+1) + idy + 1;
			d = (idx+1)*(m_Num[1]+1) + idy + 1;

			faces.push_back(a);
			faces.push_back(b);
			faces.push_back(c);
			faces.push_back(a);
			faces.push_back(c);
			faces.push_back(d);
		}
	}
}



std::vector<Node*> Mesh3D::GetBackNodes(){
	//return the nodes on the bottom surface of the cubic meshnet
	//return the nodes on the top surface of the cubic meshnet
	std::vector<Node*> back_nodes;
	back_nodes.clear();
	int idx, idy, idz, idc;
	idz = 0;
	for(idx = 0; idx< (m_Num[0]+1); idx++){
        for(idy = 0; idy < (m_Num[1]+1); idy++){
            
			idc = idx*(m_Num[1]+1)*(m_Num[2]+1) + idy*(m_Num[2]+1) + idz;
			back_nodes.push_back(&m_Nodes[idc]);
        
		}
	}
	
	return back_nodes;
}

void Mesh3D::GetBackSurface(std::vector<Node*>& nodes, std::vector<int>& faces){
	//return the nodes on the top surface of the cubic meshnet
	nodes = GetBackNodes();

	faces.clear();
	int a,b,c,d, idx, idy;
	for(idx = 0; idx< (m_Num[0]); idx++){
        for(idy = 0; idy < (m_Num[1]); idy++){
			a = idx*(m_Num[1]+1) + idy;
			b = (idx+1)*(m_Num[1]+1) + idy;
			c = idx*(m_Num[1]+1) + idy + 1;
			d = (idx+1)*(m_Num[1]+1) + idy + 1;

			faces.push_back(a);
			faces.push_back(b);
			faces.push_back(c);
			faces.push_back(a);
			faces.push_back(c);
			faces.push_back(d);
		}
	}
}



std::vector<Node*> Mesh3D::GetLeftNodes(){
	//return the nodes on the bottom surface of the cubic meshnet
	//return the nodes on the top surface of the cubic meshnet
	std::vector<Node*> left_nodes;
	left_nodes.clear();
	int idx, idy, idz, idc;
	idx = 0;
	for(idy = 0; idy< (m_Num[1]+1); idy++){
        for(idz = 0; idz < (m_Num[2]+1); idz++){
            
			idc = idx*(m_Num[1]+1)*(m_Num[2]+1) + idy*(m_Num[2]+1) + idz;
			left_nodes.push_back(&m_Nodes[idc]);
            
		}
	}
	
	return left_nodes;
}

void Mesh3D::GetLeftSurface(std::vector<Node*>& nodes, std::vector<int>& faces){
	//return the nodes on the top surface of the cubic meshnet
	nodes = GetLeftNodes();

	faces.clear();
	int a,b,c,d, idz, idy;
	for(idy = 0; idy< (m_Num[1]); idy++){
        for(idz = 0; idz < (m_Num[2]); idz++){
			a = idy*(m_Num[2]+1) + idz;
			b = (idy+1)*(m_Num[2]+1) + idz;
			c = idy*(m_Num[2]+1) + idz + 1;
			d = (idy+1)*(m_Num[2]+1) + idz + 1;

			faces.push_back(a);
			faces.push_back(b);
			faces.push_back(c);
			faces.push_back(a);
			faces.push_back(c);
			faces.push_back(d);
		}
	}
}



std::vector<Node*> Mesh3D::GetRightNodes(){
	//return the nodes on the bottom surface of the cubic meshnet
	//return the nodes on the top surface of the cubic meshnet
	std::vector<Node*> right_nodes;
	right_nodes.clear();
	int idx, idy, idz, idc;
	idx = m_Num[0];
	for(idy = 0; idy< (m_Num[1]+1); idy++){
        for(idz = 0; idz < (m_Num[2]+1); idz++){
            
			idc = idx*(m_Num[1]+1)*(m_Num[2]+1) + idy*(m_Num[2]+1) + idz;
			right_nodes.push_back(&m_Nodes[idc]);
            
		}
	}
	
	return right_nodes;
}

void Mesh3D::GetRightSurface(std::vector<Node*>& nodes, std::vector<int>& faces){
	//return the nodes on the top surface of the cubic meshnet
	nodes = GetRightNodes();

	faces.clear();
	int a,b,c,d, idz, idy;
	for(idy = 0; idy< (m_Num[1]); idy++){
        for(idz = 0; idz < (m_Num[2]); idz++){
			a = idy*(m_Num[2]+1) + idz;
			b = (idy+1)*(m_Num[2]+1) + idz;
			c = idy*(m_Num[2]+1) + idz + 1;
			d = (idy+1)*(m_Num[2]+1) + idz + 1;

			faces.push_back(a);
			faces.push_back(b);
			faces.push_back(c);
			faces.push_back(a);
			faces.push_back(c);
			faces.push_back(d);
		}
	}
}

void Mesh3D::GetSurface(std::vector<Node*>& nodes, std::vector<int>& faces){

	std::vector<Node*> nodes1, nodes2, nodes3, nodes4, nodes5, nodes6;
	std::vector<int> faces1, faces2, faces3, faces4, faces5, faces6;
	GetTopSurface(nodes1, faces1);
	GetBottomSurface(nodes2, faces2);
	GetFrontSurface(nodes3, faces3);
	GetBackSurface(nodes4, faces4);
	GetRightSurface(nodes5, faces5);
	GetLeftSurface(nodes6, faces6);

	int off_set = 0;

	faces = faces1;
	nodes = nodes1;
	off_set += nodes.size(); 
	
	
	std::transform(faces2.begin(), faces2.end(), faces2.begin(), bind2nd(std::plus<int>(), off_set));
	faces.insert(faces.end(), faces2.begin(), faces2.end());
	nodes.insert(nodes.end(), nodes2.begin(), nodes2.end()); 
	off_set += nodes.size(); 

	std::transform(faces3.begin(), faces3.end(), faces3.begin(), bind2nd(std::plus<int>(), off_set));
	faces.insert(faces.end(), faces3.begin(), faces3.end());
	nodes.insert(nodes.end(), nodes3.begin(), nodes3.end()); 
	off_set += nodes.size();

	std::transform(faces4.begin(), faces4.end(), faces4.begin(), bind2nd(std::plus<int>(), off_set));
	faces.insert(faces.end(), faces4.begin(), faces4.end());
	nodes.insert(nodes.end(), nodes4.begin(), nodes4.end()); 
	off_set += nodes.size();
	
	std::transform(faces5.begin(), faces5.end(), faces5.begin(), bind2nd(std::plus<int>(), off_set));
	faces.insert(faces.end(), faces5.begin(), faces5.end());
	nodes.insert(nodes.end(), nodes5.begin(), nodes5.end()); 
	off_set += nodes.size();

	std::transform(faces6.begin(), faces6.end(), faces6.begin(), bind2nd(std::plus<int>(), off_set));
	faces.insert(faces.end(), faces6.begin(), faces6.end());
	nodes.insert(nodes.end(), nodes6.begin(), nodes6.end()); 
	
}

Object::Object(){

    m_type = NoType; 
    m_name = "No Name";
    m_world = NULL;
    
}

void Object::SetWorld(World *a_world){
	m_world = a_world; 
}



Cube::Cube(){
    m_type = TypeCube;
    Init(Eigen::Vector3f(0.0,0.0,0.0),Eigen::Vector3f(1.0,1.0,1.0),Eigen::Vector3f(1.0,0.0,0.0));
}

Cube::Cube(Eigen::Vector3f center, Eigen::Vector3f scale, Eigen::Vector3f color){
    Init(center, scale, color);
}

void Cube::Init(Eigen::Vector3f center, Eigen::Vector3f scale, Eigen::Vector3f color){

    m_Center = center;
    m_Size = scale;
	m_Color = color;
    
	m_Trans.setIdentity();
	m_Trans.translate(m_Center);
    m_Trans.scale(m_Size);
    
    m_TransBack = m_Trans.inverse();

	InitDraw();
}

void Cube::GenFace(int colorid, int a, int b, int c, int d )
{//generate two triangles for each face and assign colors to the vertices
   
	//face normal
	Eigen::Vector3f temp1 = m_Points[a].head(3) - m_Points[b].head(3);
	Eigen::Vector3f temp2 = m_Points[c].head(3) - m_Points[b].head(3);
	Eigen::Vector3f temp_norm = temp2.cross(temp1);
	//first triangle
    m_Vertices[m_Index] = m_Points[a];  m_Normals[m_Index] = temp_norm; m_Index++;
    m_Vertices[m_Index] = m_Points[b];  m_Normals[m_Index] = temp_norm; m_Index++; 
	m_Vertices[m_Index] = m_Points[c];  m_Normals[m_Index] = temp_norm; m_Index++;
	//second triangle
	m_Vertices[m_Index] = m_Points[a]; m_Normals[m_Index] = temp_norm; m_Index++; 
    m_Vertices[m_Index] = m_Points[c]; m_Normals[m_Index] = temp_norm; m_Index++;
    m_Vertices[m_Index] = m_Points[d]; m_Normals[m_Index] = temp_norm; m_Index++;
	
}

void Cube::InitDraw(){

    // Initialize the data array on CPU

    m_Points[0] = Eigen::Vector4f( -0.5, -0.5,  0.5, 1.0 );
    m_Points[1] = Eigen::Vector4f( -0.5,  0.5,  0.5, 1.0 );
    m_Points[2] = Eigen::Vector4f(  0.5,  0.5,  0.5, 1.0 );
    m_Points[3] = Eigen::Vector4f(  0.5, -0.5,  0.5, 1.0 );
    m_Points[4] = Eigen::Vector4f( -0.5, -0.5, -0.5, 1.0 );
    m_Points[5] = Eigen::Vector4f( -0.5,  0.5, -0.5, 1.0 );
    m_Points[6] = Eigen::Vector4f(  0.5,  0.5, -0.5, 1.0 );
    m_Points[7] = Eigen::Vector4f(  0.5, -0.5, -0.5, 1.0 );
    
    m_Vertices = new Eigen::Vector4f[36];
	m_Normals = new Eigen::Vector3f[36];
    m_Index = 0;
    
    //generate the 6 faces with distinct colors
    GenFace( 5, 1, 0, 3, 2 );
    GenFace( 1, 2, 3, 7, 6 );
    GenFace( 2, 3, 0, 4, 7 );
    GenFace( 3, 6, 5, 1, 2 );
    GenFace( 4, 4, 5, 6, 7 );
    GenFace( 0, 5, 4, 0, 1 );

    //Create the Vertex Array and Buffers, bind them
#ifdef __APPLE__
    glGenVertexArraysAPPLE(1, &m_vertexArrayObject);
    glBindVertexArrayAPPLE(m_vertexArrayObject);//use as current vertex array
#else
	glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);//use as current vertex array
#endif
    glGenBuffers(1, &m_vertexBufferObject);//generate buffer for current vertex array
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer

     //Send data from CPU to GPU
    glBufferData(GL_ARRAY_BUFFER, (sizeof(m_Vertices[0]) + sizeof(m_Normals[0]))*36, NULL, GL_STATIC_DRAW);//send data to current buffer
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_Vertices[0])*36, m_Vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_Vertices[0])*36, sizeof(m_Normals[0])*36, m_Normals);
    
    
    //load and compile shaders on GPU, use current shader program
    m_shader = Util::InitShader( "vPhong.vert", "fPhong.frag" );
    glUseProgram(m_shader);
    
    
    // Link the Shader with the buffer data
    // initialize the vertex position attribute from the vertex shader
    GLuint position = glGetAttribLocation( m_shader, "vPosition" );
    glEnableVertexAttribArray( position );
    glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0)); 

	GLuint normal = glGetAttribLocation(m_shader, "vNormal");
    glEnableVertexAttribArray(normal);
    glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET(sizeof(m_Vertices[0])*36));
    
}

void Cube::Draw(int type, const Camera& camera,const Light& light){
	
    //Get new position of the cube and update the model view matrix
    Eigen::Affine3f wMo;//object to world matrix
    Eigen::Affine3f cMw;
    Eigen::Affine3f proj;

    glUseProgram(m_shader);
#ifdef __APPLE__
    glBindVertexArrayAPPLE(m_vertexArrayObject); 
#else
	glBindVertexArray(m_vertexArrayObject);
#endif
    
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer
  
	GLuint camera_position = glGetUniformLocation(m_shader, "cameraPosition");
    GLuint light_position = glGetUniformLocation(m_shader, "lightPosition");
	GLuint color = glGetUniformLocation(m_shader, "Color");

    GLuint object2world = glGetUniformLocation(m_shader, "wMo");
    GLuint world2camera = glGetUniformLocation(m_shader, "cMw"); 
	GLuint projection = glGetUniformLocation(m_shader, "proj");

    wMo = m_Trans;

    proj = Util::Perspective( camera.m_fovy, camera.m_aspect, camera.m_znear, camera.m_zfar );
	cMw = camera.m_cMw;//Angel::LookAt(camera.position,camera.lookat, camera.up );
 
    glUniformMatrix4fv( object2world , 1, GL_FALSE, wMo.data() );
    glUniformMatrix4fv( world2camera, 1, GL_FALSE, cMw.data());
    glUniformMatrix4fv( projection, 1, GL_FALSE, proj.data());
	glUniform4fv(camera_position, 1, camera.m_position.data());
    glUniform4fv(light_position, 1, light.m_position.data());
	Eigen::Vector4f l_color(m_Color[0],m_Color[1],m_Color[2],1.0);
  	glUniform4fv(color,1,l_color.data());
	
	switch (type) {
        case DRAW_MESH:
            glDrawArrays(GL_LINES, 0, 36);
            break;
        case DRAW_PHONG:
            glDrawArrays(GL_TRIANGLES, 0, 36);
            break;
    }
    

}

void Cube::Output2File(std::ofstream* filestream){
	
	(*filestream)<<"//BEGIN CUBE"<<std::endl;
	Eigen::Vector4f current_vertex;
	for(int i =0 ; i < 8; i++){
		current_vertex = m_Trans*m_Points[i];
		(*filestream)<<current_vertex[0]<<" "<<current_vertex[1]<<" "<<current_vertex[2]<<std::endl;
	}

	(*filestream)<<"//END CUBE"<<std::endl;
}


Cylinder::Cylinder(){
    m_type = TypeCylinder;
    Init(Eigen::Vector3f(0.0,0.0,0.0),Eigen::Vector3f(1.0,1.0,1.0),Eigen::Vector3f(1.0,0.0,0.0));
}


void Cylinder::makeCircle()
{
    for (int i = 0; i < 64; i++)
    {
        float a = i * 2.0f * M_PI / 64;
        m_CirclePoints[i] = 0.5*Eigen::Vector2f(cosf(a), sinf(a));
    }
}


void Cylinder::makeConeWall(float z1, float z2, int dir)
{
	Eigen::Vector3f n;
    for (int i = 0; i < 64; i++)
    {
        Eigen::Vector3f p1(m_CirclePoints[i][0], m_CirclePoints[i][1], z1);
        Eigen::Vector3f p2(0.0f, 0.0f, z2);
        Eigen::Vector3f p3(m_CirclePoints[(i+1)%64][0], m_CirclePoints[(i+1)%64][1], z1);
        if (dir == -1)
        {
            Eigen::Vector3f temp = p1;
            p1 = p3;
            p3 = temp;
        }
        n = (p1-p2).cross(p3-p2);
		
        m_Vertices[m_Index] = Eigen::Vector4f(p1[0],p1[1],p1[2],1); m_Normals[m_Index] = n; m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p2[0],p2[1],p2[2],1); m_Normals[m_Index] = n; m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p3[0],p3[1],p3[2],1); m_Normals[m_Index] = n; m_Index++;
    }

}

void Cylinder::GenerateCylinder()
{
    makeCircle();
    m_Index = 0;
    makeConeWall(0.5f, 0.5f, 1);
    makeConeWall(-0.5f, -0.5f, -1);
    
    for (int i = 0; i < 64; i++)
    {
        int i2 = (i+1)%64;
        Eigen::Vector3f  p1(m_CirclePoints[i2][0], m_CirclePoints[i2][1], -0.5);
        Eigen::Vector3f  p2(m_CirclePoints[i2][0], m_CirclePoints[i2][1], 0.5);
        Eigen::Vector3f  p3(m_CirclePoints[i][0],  m_CirclePoints[i][1],  0.5);
        //point3 n = cross(p3-p2, p1-p2);
        m_Vertices[m_Index] = Eigen::Vector4f(p1[0],p1[1],p1[2],1); m_Normals[m_Index] = Eigen::Vector3f(p1[0], p1[1], 0.0f); m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p2[0],p2[1],p2[2],1); m_Normals[m_Index] = Eigen::Vector3f(p2[0], p2[1], 0.0f); m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p3[0],p3[1],p3[2],1); m_Normals[m_Index] = Eigen::Vector3f(p3[0], p3[1], 0.0f); m_Index++;
        p1 = Eigen::Vector3f(m_CirclePoints[i2][0], m_CirclePoints[i2][1], -0.5);
        p2 = Eigen::Vector3f(m_CirclePoints[i][0],  m_CirclePoints[i][1],  0.5);
        p3 = Eigen::Vector3f(m_CirclePoints[i][0],  m_CirclePoints[i][1],  -0.5);
        //n = cross(p3-p2, p1-p2);
        m_Vertices[m_Index] = Eigen::Vector4f(p1[0],p1[1],p1[2],1); m_Normals[m_Index] = Eigen::Vector3f(p1[0], p1[1], 0.0f); m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p2[0],p2[1],p2[2],1); m_Normals[m_Index] = Eigen::Vector3f(p2[0], p2[1], 0.0f); m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p3[0],p3[1],p3[2],1); m_Normals[m_Index] = Eigen::Vector3f(p3[0], p3[1], 0.0f); m_Index++;
    }
    
}

void Cylinder::Init(Eigen::Vector3f center,Eigen::Vector3f size, Eigen::Vector3f color){
	
	m_Center = center;
    m_Size = size;
	m_Color = color;

	InitDraw();
}


void Cylinder::InitDraw(){
    // Initialize the data array on CPU
    m_Vertices = new Eigen::Vector4f[64*12];//64 is the number of division
	m_Normals = new Eigen::Vector3f[64*12];//6 is the number of verteces for each division(two triangle)
	m_CirclePoints = new Eigen::Vector2f[64];
	
	m_Index = 0;
	GenerateCylinder();
    
    //Create the Vertex Array and Buffers, bind them
#ifdef __APPLE__
    glGenVertexArraysAPPLE(1, &m_vertexArrayObject);
    glBindVertexArrayAPPLE(m_vertexArrayObject);//use as current vertex array
#else
	glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);//use as current vertex array
#endif
    glGenBuffers(1, &m_vertexBufferObject);//generate buffer for current vertex array
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer
    
    //Send data from CPU to GPU
    glBufferData(GL_ARRAY_BUFFER, (sizeof(m_Vertices[0]) + sizeof(m_Normals[0]))*64*12, NULL, GL_STATIC_DRAW);//send data to current buffer
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_Vertices[0])*64*12, m_Vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_Vertices[0])*64*12, sizeof(m_Normals[0])*64*12, m_Normals);
    
    
    //load and compile shaders on GPU, use current shader program
    m_shader = Util::InitShader( "vPhong.glsl", "fPhong.glsl" );
    glUseProgram(m_shader);
    
    
    // Link the Shader with the buffer data
    // initialize the vertex position attribute from the vertex shader
    GLuint position = glGetAttribLocation( m_shader, "vPosition" );
    glEnableVertexAttribArray( position );
    glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0)); 

	GLuint normal = glGetAttribLocation(m_shader, "vNormal");
    glEnableVertexAttribArray(normal);
    glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET(sizeof(m_Vertices[0])*64*12));
    
}

void Cylinder::Draw(int type, const Camera& camera, const Light& light){
    //Get new position of the Cylinder and update the model view matrix
   Eigen::Affine3f wMo;//object to world matrix
    Eigen::Affine3f cMw;
    Eigen::Affine3f proj;

    glUseProgram(m_shader);
#ifdef __APPLE__
    glBindVertexArrayAPPLE(m_vertexArrayObject); 
#else
	glBindVertexArray(m_vertexArrayObject);
#endif
    
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer
    
	GLuint camera_position = glGetUniformLocation(m_shader, "cameraPosition");
    GLuint light_position = glGetUniformLocation(m_shader, "lightPosition");
	GLuint object_color = glGetUniformLocation(m_shader, "Color");

    GLuint object2world = glGetUniformLocation(m_shader, "wMo");
    GLuint world2camera = glGetUniformLocation(m_shader, "cMw"); 
	GLuint projection = glGetUniformLocation(m_shader, "proj");
   
    wMo = m_Trans;

    proj = Util::Perspective( camera.m_fovy, camera.m_aspect, camera.m_znear, camera.m_zfar );

	//the world to camera matrix is read from camera
	cMw = camera.m_cMw;
    
    glUniformMatrix4fv( object2world , 1, GL_FALSE, wMo.data() );
    glUniformMatrix4fv( world2camera, 1, GL_FALSE, cMw.data() );
    glUniformMatrix4fv( projection, 1, GL_FALSE, proj.data() );
	glUniform4fv(camera_position, 1, camera.m_position.data());
    glUniform4fv(light_position, 1, light.m_position.data());
	Eigen::Vector4f l_color(m_Color[0],m_Color[1],m_Color[2],1.0);
	glUniform4fv(object_color,1,l_color.data());

    switch (type) {
        case DRAW_MESH:
            glDrawArrays(GL_LINES, 0, 64*12);
            break;
        case DRAW_PHONG:
            glDrawArrays(GL_TRIANGLES, 0, 64*12);
            break;
    }
}



Sphere::Sphere(){
    m_type = TypeSphere;
	m_count = 0;
    Init(Eigen::Vector3f(0.0,0.0,0.0), Eigen::Vector3f(1.0,1.0,1.0),Eigen::Vector3f(1.0,0.0,0.0));
}

Sphere::Sphere(Eigen::Vector3f center, Eigen::Vector3f scale, Eigen::Vector3f color){
    Init(center, scale, color);
}

void Sphere::Init(Eigen::Vector3f center, Eigen::Vector3f scale, Eigen::Vector3f color){
    m_Center = center;
    m_Size = scale;
	m_Color = color;
   
	m_Trans.setIdentity();
	m_Trans.translate(m_Center);
    m_Trans.scale(m_Size);
    
    m_TransBack = m_Trans.inverse();

	InitDraw();
}

Eigen::Vector4f Sphere::Unit(const Eigen::Vector4f &p)
{
    Eigen::Vector4f c;
    double d=0.0;
    for(int i=0; i<3; i++) d+=p[i]*p[i];
    d=sqrt(d);
    if(d > 0.0) for(int i=0; i<3; i++) c[i] = p[i]/d;
    c[3] = 1.0;
    return c;
}

void Sphere::DividTriangle(const Eigen::Vector4f& a, const Eigen::Vector4f& b, const Eigen::Vector4f& c, int n){
	Eigen::Vector4f v1, v2, v3;
    if(n>0)
    {
        v1 = Unit(a + b);
        v2 = Unit(a + c);
        v3 = Unit(b + c);   
        DividTriangle(a , v2, v1, n-1);
        DividTriangle(c , v3, v2, n-1);
        DividTriangle(b , v1, v3, n-1);
        DividTriangle(v1, v2, v3, n-1);
    }
	else{
		m_Vertices[m_count] = a;
		m_Normals[m_count] = a.head(3);
		m_count++;
		m_Vertices[m_count] = b;
		m_Normals[m_count] = b.head(3);
		m_count++;
		m_Vertices[m_count] = c;
		m_Normals[m_count] = c.head(3);
		m_count++;
	};
}

void Sphere::GenerateSphere(){
	m_count = 0;
	Eigen::Vector4f v[4];
    v[0] = Eigen::Vector4f(0.0, 0.0, 1.0, 1.0);
    v[1] = Eigen::Vector4f(0.0, 0.942809, -0.333333, 1.0);
    v[2] = Eigen::Vector4f(-0.816497, -0.471405, -0.333333, 1.0);
    v[3] = Eigen::Vector4f(0.816497, -0.471405, -0.333333, 1.0);
	
	DividTriangle(v[0], v[1], v[2], 5);
    DividTriangle(v[3], v[2], v[1], 5);
    DividTriangle(v[0], v[3], v[1], 5);
    DividTriangle(v[0], v[3], v[2], 5);

}

void Sphere::InitDraw(){
    // Initialize the data array on CPU
    m_Vertices = new Eigen::Vector4f[16*256*3];
	m_Normals = new Eigen::Vector3f[16*256*3];
    m_Index = 0;
   
	GenerateSphere();
    
    //Create the Vertex Array and Buffers, bind them
#ifdef __APPLE__
    glGenVertexArraysAPPLE(1, &m_vertexArrayObject);
    glBindVertexArrayAPPLE(m_vertexArrayObject);//use as current vertex array
#else
	glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);//use as current vertex array
#endif
    glGenBuffers(1, &m_vertexBufferObject);//generate buffer for current vertex array
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer
    
    //Send data from CPU to GPU
    glBufferData(GL_ARRAY_BUFFER, (sizeof(m_Vertices[0]) + sizeof(m_Normals[0]))*16*256*3, NULL, GL_STATIC_DRAW);//send data to current buffer
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_Vertices[0])*16*256*3, m_Vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_Vertices[0])*16*256*3, sizeof(m_Normals[0])*16*256*3, m_Normals);
    
    
    //load and compile shaders on GPU, use current shader program
    m_shader = Util::InitShader( "vPhong.glsl", "fPhong.glsl" );
    glUseProgram(m_shader);
    
    
    // Link the Shader with the buffer data
    // initialize the vertex position attribute from the vertex shader
    GLuint position = glGetAttribLocation( m_shader, "vPosition" );
    glEnableVertexAttribArray( position );
    glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0)); 

	GLuint normal = glGetAttribLocation(m_shader, "vNormal");
    glEnableVertexAttribArray(normal);
    glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET(sizeof(m_Vertices[0])*16*256*3));
    
}

void Sphere::Draw(int type, const Camera& camera, const Light& light){

    //Get new position of the Sphere and update the model view matrix
    Eigen::Affine3f wMo;//object to world matrix
    Eigen::Affine3f cMw;
    Eigen::Affine3f proj;

    glUseProgram(m_shader);
#ifdef __APPLE__
    glBindVertexArrayAPPLE(m_vertexArrayObject); 
#else
	glBindVertexArray(m_vertexArrayObject);
#endif
    
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer
    
	GLuint camera_position = glGetUniformLocation(m_shader, "cameraPosition");
    GLuint light_position = glGetUniformLocation(m_shader, "lightPosition");
	GLuint color = glGetUniformLocation(m_shader, "Color");

    GLuint object2world = glGetUniformLocation(m_shader, "wMo");
    GLuint world2camera = glGetUniformLocation(m_shader, "cMw"); 
	GLuint projection = glGetUniformLocation(m_shader, "proj");

    wMo = m_Trans;
    proj = Util::Perspective( camera.m_fovy, camera.m_aspect, camera.m_znear, camera.m_zfar );
	cMw = camera.m_cMw;//Angel::LookAt(camera.position,camera.lookat, camera.up );
    
    glUniformMatrix4fv( object2world , 1, GL_FALSE, wMo.data() );
    glUniformMatrix4fv( world2camera, 1, GL_FALSE, cMw.data() );
    glUniformMatrix4fv( projection, 1, GL_FALSE, proj.data() );
	glUniform4fv(camera_position, 1, camera.m_position.data());
    glUniform4fv(light_position, 1, light.m_position.data());

	Eigen::Vector4f l_color(m_Color[0],m_Color[1],m_Color[2],1.0);
  	glUniform4fv(color,1,l_color.data());

	switch (type) {
        case DRAW_MESH:
            glDrawArrays(GL_LINES, 0, 16*256*3);
            break;
        case DRAW_PHONG:
            glDrawArrays(GL_TRIANGLES, 0, 16*256*3);
            break;
    }
}



Cone::Cone(){
    m_type = TypeCone;
    Init(Eigen::Vector3f(0.0,0.0,0.0),Eigen::Vector3f(1.0,1.0,1.0),Eigen::Vector3f(1.0,0.0,0.0));
}


void Cone::makeCircle()
{
    for (int i = 0; i < 64; i++)
    {
        float a = i * 2.0f * M_PI / 64;
        m_CirclePoints[i] = 0.5*Eigen::Vector2f(cosf(a), sinf(a));
    }
}

void Cone::makeConeWall(float z1, float z2, int dir)
{
	
	Eigen::Vector3f n;
    for (int i = 0; i < 64; i++)
    {
        Eigen::Vector3f p1(m_CirclePoints[i][0], m_CirclePoints[i][1], z1);
        Eigen::Vector3f p2(0.0f, 0.0f, z2);
        Eigen::Vector3f p3(m_CirclePoints[(i+1)%64][0], m_CirclePoints[(i+1)%64][1], z1);
        if (dir == -1)
        {
            Eigen::Vector3f temp = p1;
            p1 = p3;
            p3 = temp;
        }
        n = (p1-p2).cross(p3-p2);
		
        m_Vertices[m_Index] = Eigen::Vector4f(p1[0],p1[1],p1[2],1); m_Normals[m_Index] = n; m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p2[0],p2[1],p2[2],1); m_Normals[m_Index] = n; m_Index++;
        m_Vertices[m_Index] = Eigen::Vector4f(p3[0],p3[1],p3[2],1); m_Normals[m_Index] = n; m_Index++;
    }
	
}

void Cone::GenerateCone()
{
    makeCircle();
    makeConeWall(0.5f, 0.5f, 1);
    makeConeWall(0.5f, -0.5f, -1);
}

void Cone::Init(Eigen::Vector3f center,Eigen::Vector3f size, Eigen::Vector3f color){
	
	m_Center = center;
    m_Size = size;
	m_Color = color;

	InitDraw();
}

void Cone::InitDraw(){
    
	// Initialize the data array on CPU
    m_Vertices = new Eigen::Vector4f[64*6];//64 is the number of division
	m_Normals = new Eigen::Vector3f[64*6];//6 is the number of verteces for each division(two triangle)
	m_CirclePoints = new Eigen::Vector2f[64];
	
	m_Index = 0;
	GenerateCone();
    
    //Create the Vertex Array and Buffers, bind them
#ifdef __APPLE__
    glGenVertexArraysAPPLE(1, &m_vertexArrayObject);
    glBindVertexArrayAPPLE(m_vertexArrayObject);//use as current vertex array
#else
	glGenVertexArrays(1, &m_vertexArrayObject);
    glBindVertexArray(m_vertexArrayObject);//use as current vertex array
#endif
    glGenBuffers(1, &m_vertexBufferObject);//generate buffer for current vertex array
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer
    
    //Send data from CPU to GPU
    glBufferData(GL_ARRAY_BUFFER, (sizeof(m_Vertices[0]) + sizeof(m_Normals[0]))*64*6, NULL, GL_STATIC_DRAW);//send data to current buffer
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_Vertices[0])*64*6, m_Vertices);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_Vertices[0])*64*6, sizeof(m_Normals[0])*64*6, m_Normals);
    
    
    //load and compile shaders on GPU, use current shader program
    m_shader = Util::InitShader( "vPhong.glsl", "fPhong.glsl" );
    glUseProgram(m_shader);
    
    
    // Link the Shader with the buffer data
    // initialize the vertex position attribute from the vertex shader
    GLuint position = glGetAttribLocation( m_shader, "vPosition" );
    glEnableVertexAttribArray( position );
    glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0)); 

	GLuint normal = glGetAttribLocation(m_shader, "vNormal");
    glEnableVertexAttribArray(normal);
    glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET(sizeof(m_Vertices[0])*64*6));
    
}

void Cone::Draw(int type, const Camera& camera, const Light& light){
	
    //Get new position of the Cone and update the model view matrix
    Eigen::Affine3f wMo;//object to world matrix
    Eigen::Affine3f cMw;
    Eigen::Affine3f proj;

    glUseProgram(m_shader);
#ifdef __APPLE__
    glBindVertexArrayAPPLE(m_vertexArrayObject); 
#else
	glBindVertexArray(m_vertexArrayObject);
#endif
    
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);//use as current buffer
    
	GLuint camera_position = glGetUniformLocation(m_shader, "cameraPosition");
    GLuint light_position = glGetUniformLocation(m_shader, "lightPosition");
	GLuint object_color = glGetUniformLocation(m_shader, "Color");

    GLuint object2world = glGetUniformLocation(m_shader, "wMo");
    GLuint world2camera = glGetUniformLocation(m_shader, "cMw"); 
	GLuint projection = glGetUniformLocation(m_shader, "proj");
   
    //generate the object to world translate matrix

    wMo = m_Trans;

    proj = Util::Perspective( camera.m_fovy, camera.m_aspect, camera.m_znear, camera.m_zfar );

	//the world to camera matrix is read from camera
	cMw = camera.m_cMw;
    
    glUniformMatrix4fv( object2world , 1, GL_FALSE, wMo.data() );
    glUniformMatrix4fv( world2camera, 1, GL_FALSE, cMw.data() );
    glUniformMatrix4fv( projection, 1, GL_FALSE, proj.data() );
	glUniform4fv(camera_position, 1, camera.m_position.data());
    glUniform4fv(light_position, 1, light.m_position.data());
	Eigen::Vector4f l_color(m_Color[0],m_Color[1],m_Color[2],1.0);
	glUniform4fv(object_color,1,l_color.data());

	switch (type) {
        case DRAW_MESH:
            glDrawArrays(GL_LINES, 0, 64*6);
            break;
        case DRAW_PHONG:
            glDrawArrays(GL_TRIANGLES, 0, 64*6);
            break;
    }
	
}





