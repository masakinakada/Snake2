#include "Terrain.h"
#include "Drawer.h"
#include "Object.h"

extern Drawer* myDrawer;

Terrain::Terrain(Eigen::Vector2f a_size, Eigen::Vector2i a_res, int n_hill, TerrainType a_type){
	m_type = TypeTerrain;
	m_frictness = 10;
	
	InitBase(a_size.x(), a_size.y(), a_res.x(), a_res.y(), n_hill, a_type);

	InitDraw();
}

void Terrain::InitBase(double a_size_x, double a_size_z, int a_res_x, int a_res_z, int n_hill, TerrainType a_type)
{
	
	m_size_x = a_size_x;
	m_size_z = a_size_z;
	m_res_x = a_res_x;
	m_res_z = a_res_z;
	m_dx = m_size_x/m_res_x;
	m_dz = m_size_z/m_res_z;

	m_height_data = new double[(a_res_x+1)*(a_res_z+1)];
	m_normal_data = new Eigen::Vector3f[(a_res_x+1)*(a_res_z+1)];
	for(int i = 0; i < (a_res_x+1)*(a_res_z+1); i++)
		m_height_data[i] = 0.0;//clear to 0
	
	if(a_type == TERRAIN_FLAT){
		for(int ix = 0; ix < (a_res_x+1); ix++)
			for(int iz = 0; iz < (a_res_z+1); iz++){
				m_height_data[ix*(a_res_z+1) + iz] = 0.0;
			}
	}

	else if(a_type == TERRAIN_RANDOM){
		double hill_height_max = 60;
		double hill_center_x, hill_center_z, hill_height, hill_narrowness_x, hill_narrowness_z;
		double dev_x, dev_z;

		for(int i = 0; i < n_hill; i++){	

			hill_center_x = (Util::getRand()-0.5)*a_size_x;
			hill_center_z = (Util::getRand()-0.5)*a_size_z;
			hill_height = Util::getRand()*(hill_height_max - 5) + 5;
			hill_narrowness_x = Util::getRand()*30 + 10; //10~40
			hill_narrowness_z = Util::getRand()*30 + 10; //10~40

			//add the hill to current height map
			for(int ix = 0; ix < (a_res_x+1); ix++)
				for(int iz = 0; iz < (a_res_z+1); iz++){
					dev_x = ix*m_dx - 0.5*a_size_x - hill_center_x;
					dev_z = iz*m_dz - 0.5*a_size_z - hill_center_z;
					m_height_data[ix*(a_res_z+1) + iz] += //guassian hill
						hill_height*exp(-dev_x*dev_x/(2*hill_narrowness_x*hill_narrowness_x)-dev_z*dev_z/(2*hill_narrowness_z*hill_narrowness_z));
				}
		}
	
		//normalize
		double temp_largest = m_height_data[0];
		double temp_smallest = m_height_data[0];
		for(int i = 0; i < (a_res_x+1)*(a_res_z+1); i++)
		{
			if(m_height_data[i] > temp_largest)
				temp_largest = m_height_data[i];
			if(m_height_data[i] < temp_smallest)
				temp_smallest = m_height_data[i];
		}

		for(int i = 0; i < (a_res_x+1)*(a_res_z+1); i++)
		{
			//eventually the range would be from 0~m_hill_height_max
			m_height_data[i] = hill_height_max*(m_height_data[i] - temp_smallest)/temp_largest;
		}


	}else if(a_type == TERRAIN_TEST){
		
		for(int ix = 0; ix < (a_res_x+1); ix++)
			for(int iz = 0; iz < (a_res_z+1); iz++){
				m_height_data[ix*(a_res_z+1) + iz] = 0.0;
			}
    }else if(a_type == TERRAIN_UPHILL){
    
        float slope_coef = -0.5;
        for(int ix = 0; ix < (a_res_x+1); ix++)
            for(int iz = 0; iz < (a_res_z+1); iz++){
                m_height_data[ix*(a_res_z+1) + iz] = slope_coef*ix - slope_coef*(a_res_x+1)/2;
            }
    }

	GenerateNormals();
	
}

void Terrain::GenerateNormals(){
//generate normal

	Eigen::Vector3f v1,v2;
	Eigen::Vector3f current_normal, temp_face_normal;
	for(int ix = 0; ix < m_res_x + 1; ix++)
		for(int iz = 0; iz < m_res_z + 1; iz++){
			current_normal *= 0.0;
			//upper left face normal
			if(ix > 0 && iz < m_res_z)
			{
				v1.x() = 0;
				v1.z() = m_dz;
				v1.y() = m_height_data[ix*(m_res_z+1) + iz + 1] - m_height_data[ix*(m_res_z+1) + iz];
				v1.normalize();

				v2.x() = -m_dx;
				v2.z() = 0;
				v2.y() = m_height_data[(ix-1)*(m_res_z+1) + iz] - m_height_data[ix*(m_res_z+1) + iz];
				v2.normalize();

				temp_face_normal = v1.cross(v2);
				current_normal += temp_face_normal;
			}
			//upper right face normal
			if(ix < m_res_x - 1 && iz < m_res_z)
			{
				v1.x() = 0;
				v1.z() = m_dz;
				v1.y() = m_height_data[ix*(m_res_z+1) + iz + 1] - m_height_data[ix*(m_res_z+1) + iz];
				v1.normalize();

				v2.x() = m_dx;
				v2.z() = 0;
				v2.y() = m_height_data[(ix+1)*(m_res_z+1) + iz] - m_height_data[ix*(m_res_z+1) + iz];
				v2.normalize();

				temp_face_normal = v2.cross(v1);
				current_normal += temp_face_normal;
			}
			//lower left face normal
			if(ix > 0 && iz > 0)
			{
				v1.x() = 0;
				v1.z() = -m_dz;
				v1.y() = m_height_data[ix*(m_res_z+1) + iz - 1] - m_height_data[ix*(m_res_z+1) + iz];
				v1.normalize();

				v2.x() = -m_dx;
				v2.z() = 0;
				v2.y() = m_height_data[(ix-1)*(m_res_z+1) + iz] - m_height_data[ix*(m_res_z+1) + iz];
				v2.normalize();

				temp_face_normal = v2.cross(v1);
				current_normal += temp_face_normal;
			}
			//lower right face normal
			if(ix < m_res_x  && iz > 0)
			{
				v1.x() = 0;
				v1.z() = -m_dz;
				v1.y() = m_height_data[ix*(m_res_z+1) + iz - 1] - m_height_data[ix*(m_res_z+1) + iz];
				v1.normalize();

				v2.x() = m_dx;
				v2.z() = 0;
				v2.y() = m_height_data[(ix+1)*(m_res_z+1) + iz] - m_height_data[ix*(m_res_z+1) + iz];
				v2.normalize();

				temp_face_normal = v1.cross(v2);
				current_normal += temp_face_normal;
			}

			m_normal_data[ix*(m_res_z+1) + iz] = -current_normal.normalized();//average
		}
	

}

double Terrain::GetHeight(Eigen::Vector2f xy) const{
	
	Cube* temp_cube;
	Cylinder* temp_cylinder;

	double x = xy.x();
	double z = xy.y();
	//check if x y lands on a surface object
	for(int i = 0; i < m_surface_objects.size(); i++){
		
		switch (m_surface_objects[i]->m_type)
		{
		case TypeCube:
			temp_cube = dynamic_cast<Cube*>(m_surface_objects[i]);
			if(x < temp_cube->m_Center[0] + temp_cube->m_Size[0]*0.5
				&& x > temp_cube->m_Center[0] - temp_cube->m_Size[0]*0.5
				&& z < temp_cube->m_Center[2] + temp_cube->m_Size[2]*0.5
				&& z > temp_cube->m_Center[2] - temp_cube->m_Size[2]*0.5)
				return temp_cube->m_Size[1];
			break;
		case TypeCylinder:
			temp_cylinder = dynamic_cast<Cylinder*>(m_surface_objects[i]);
			if(x < temp_cylinder->m_Center[0] + temp_cylinder->m_Size[0]*0.5
				&&x > temp_cylinder->m_Center[0] - temp_cylinder->m_Size[0]*0.5
				&&z < temp_cylinder->m_Center[2] + temp_cylinder->m_Size[2]*0.5
				&&z > temp_cylinder->m_Center[0] - temp_cylinder->m_Size[2]*0.5)
				return sqrt(0.25*temp_cylinder->m_Size[1]*temp_cylinder->m_Size[1] - (x - temp_cylinder->m_Center[0])*(x - temp_cylinder->m_Center[0]));
			break;
		default:
			break;
		}
		
	
	}


	int idx, idz;
	idx = (xy.x() + 0.5*m_size_x)/m_dx;
	idz = (xy.y() + 0.5*m_size_z)/m_dz;

	//interpolation
	double upleft, upright, downleft, downright;
	downleft = m_height_data[idx*(m_res_z+1) + idz];
	downright = m_height_data[(idx+1)*(m_res_z+1) + idz];
	upleft = m_height_data[idx*(m_res_z+1) + idz+1];
	upright = m_height_data[(idx+1)*(m_res_z+1) + idz+1];

	double alpha, beta;
	alpha = 1 - (xy.x() + 0.5*m_size_x - idx*m_dx)/m_dx;
	beta = 1 - (xy.y() + 0.5*m_size_z - idz*m_dz)/m_dz;

	return alpha*beta*downleft + (1-alpha)*beta*downright + (1-beta)*alpha*upleft + (1-beta)*(1-alpha)*upright;
}

Eigen::Vector3f Terrain::GetNormal(Eigen::Vector2f xy) const{

	//COULD HAVE BUG HERE

	int idx, idz;
	idx = (xy.x() + 0.5*m_size_x)/m_dx;
	idz = (xy.y() + 0.5*m_size_z)/m_dz;

	//interpolation
	Eigen::Vector3f upleft, upright, downleft, downright;
	downleft = m_normal_data[idx*(m_res_z+1) + idz];
	downright = m_normal_data[(idx+1)*(m_res_z+1) + idz];
	upleft = m_normal_data[idx*(m_res_z+1) + idz+1];
	upright = m_normal_data[(idx+1)*(m_res_z+1) + idz+1];

	double alpha, beta;
	alpha = 1 - (xy.x() + 0.5*m_size_x - idx*m_dx)/m_dx;
	beta = 1 - (xy.y() + 0.5*m_size_z - idz*m_dz)/m_dz;

	return alpha*beta*downleft + (1-alpha)*beta*downright + (1-beta)*alpha*upleft + (1-beta)*(1-alpha)*upright;

}

bool Terrain::TestIntersection(Millipede* a_bug, Eigen::Vector3f a_o, Eigen::Vector3f a_p){

	Cube* temp_cube;
	Sphere* temp_sphere;
	Cylinder* temp_cylinder;
	//generate sample points on the line segment for intersection test
	int num_sample_points = 5; 
	assert(num_sample_points > 2);
	double d_alpha = 1.0/(num_sample_points-1);
	std::vector<Eigen::Vector3f> sample_points;
	for(int i = 0; i < num_sample_points; i++){
		sample_points.push_back(a_o*i*d_alpha+ a_p*(1 - i*d_alpha));
	}
	
	Eigen::Vector4f temp_point;
	//the intersection test of line agains geometrical primitives are now 
	//implemented with point-inside-outside test

	for(int i = 0; i < m_obstacles.size(); i++)
	{
		//test if part of the line segment inside the object
		switch (m_obstacles[i]->m_type)
		{
		case TypeCube:
			temp_cube = dynamic_cast<Cube*>(m_obstacles[i]);
			for(int j = 0; j < num_sample_points; j++){
				temp_point = Eigen::Vector4f(sample_points[j][0],sample_points[j][1],sample_points[j][2],1.0);
				temp_point = temp_cube->m_TransBack*temp_point;
				if(fabs(temp_point[0]) < 0.5&&fabs(temp_point[1]) < 0.5 && fabs(temp_point[2]) < 0.5){
					return true; 
					
				}
			}
		break;
		case TypeSphere:
			temp_sphere = dynamic_cast<Sphere*>(m_obstacles[i]);
			for(int j = 0; j < num_sample_points; j++){
				temp_point = Eigen::Vector4f(sample_points[j][0],sample_points[j][1],sample_points[j][2],1.0);
				temp_point = temp_sphere->m_TransBack*temp_point;
				temp_point[3] = 0.0;
				if(temp_point.norm() < 1.0){
					return true; 
					
				}
			}
		break;
		case TypeCylinder:
			temp_cylinder = dynamic_cast<Cylinder*>(m_obstacles[i]);
			for(int j = 0; j < num_sample_points; j++){
				temp_point = Eigen::Vector4f(sample_points[j][0],sample_points[j][1],sample_points[j][2],1.0);
				temp_point = temp_cylinder->m_TransBack*temp_point;
				if(temp_point[0]*temp_point[0] + temp_point[1]*temp_point[1] < 0.5*0.5){
					return true; 
				}
			}

		break;
		default:
			break;
		}
	}

	return false;
}

void Terrain::InitDraw(){

	// Initialize the data array on CPU
    
    m_NTrianglePoints = 3*2*m_res_x*m_res_z;
    m_TrianglePoints = new Eigen::Vector4f[m_NTrianglePoints];
    m_TPointNormals = new Eigen::Vector3f[m_NTrianglePoints];
    m_TriangleColors = new Eigen::Vector4f[m_NTrianglePoints];

	int box_id;
	Eigen::Vector4f a,b,c,d;
	Eigen::Vector3f na,nb,nc,nd;
	for(int ix = 0; ix < m_res_x; ix++)
		for(int iz = 0; iz < m_res_z; iz++){

		box_id = ix*m_res_z + iz;
		
		a.x() = m_dx*ix - 0.5*m_size_x;a.z() = m_dz*iz - 0.5*m_size_z;a.y() = m_height_data[ix*(m_res_z+1) + iz];a.w() = 1.0;
		b.x() = m_dx*(ix+1) - 0.5*m_size_x;b.z() = m_dz*iz - 0.5*m_size_z;b.y() = m_height_data[(ix+1)*(m_res_z+1) + iz];b.w() = 1.0;
		c.x() = m_dx*(ix+1) - 0.5*m_size_x;c.z() = m_dz*(iz+1) - 0.5*m_size_z;c.y() = m_height_data[(ix+1)*(m_res_z+1) + iz + 1];c.w() = 1.0;
		d.x() = m_dx*ix - 0.5*m_size_x;d.z() = m_dz*(iz+1) - 0.5*m_size_z;d.y() = m_height_data[ix*(m_res_z+1) + iz + 1];d.w() = 1.0;
		
		m_TrianglePoints[6*box_id] = a;
		m_TrianglePoints[6*box_id + 1] = b;
		m_TrianglePoints[6*box_id + 2] = c;
		m_TrianglePoints[6*box_id + 3] = c;
		m_TrianglePoints[6*box_id + 4] = d;
		m_TrianglePoints[6*box_id + 5] = a;

		na = m_normal_data[ix*(m_res_z+1) + iz];
		nb = m_normal_data[(ix+1)*(m_res_z+1) + iz];
		nc = m_normal_data[(ix+1)*(m_res_z+1) + iz+1];
		nd = m_normal_data[ix*(m_res_z+1) + iz+1];
		
		m_TPointNormals[6*box_id] = na;
		m_TPointNormals[6*box_id + 1] = nb;
		m_TPointNormals[6*box_id + 2] = nc;
		m_TPointNormals[6*box_id + 3] = nc;
		m_TPointNormals[6*box_id + 4] = nd;
		m_TPointNormals[6*box_id + 5] = na;
		
		for(int ii = 0; ii < 6; ii++)
			m_TriangleColors[box_id*6+ii] = Eigen::Vector4f(0,0.8,1.0,1.0);	
	}

	
    //Create the Vertex Array and Buffers, bind them
#ifdef __APPLE__
	glGenVertexArraysAPPLE(1, &m_vertexArrayObject);
	glBindVertexArrayAPPLE(m_vertexArrayObject);//use as current vertex array   
#else
	 glGenVertexArrays(1, &m_vertexArrayObject);
	 glBindVertexArray(m_vertexArrayObject);//use as current vertex array 
#endif
   
    glGenBuffers(1, &m_vertexBufferObject);//generate buffer for current vertex array
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
    
	 //send the updated data to buffer
    glBufferData(GL_ARRAY_BUFFER, (sizeof(m_TrianglePoints[0]) + sizeof(m_TriangleColors[0]) + sizeof(m_TPointNormals[0]))*m_NTrianglePoints, NULL, GL_STATIC_DRAW);//send data to current buffer
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_TrianglePoints[0])*m_NTrianglePoints, m_TrianglePoints);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_TrianglePoints[0])*m_NTrianglePoints, sizeof(m_TriangleColors[0])*m_NTrianglePoints, m_TriangleColors);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(m_TriangleColors[0])*m_NTrianglePoints+sizeof(m_TrianglePoints[0])*m_NTrianglePoints,sizeof(m_TPointNormals[0])*m_NTrianglePoints, m_TPointNormals);
    
    //load and compile shaders on GPU, use current shader program
    m_shader = Util::InitShader( "vSmoothPhong.vert", "fSmoothPhong.frag" );
    glUseProgram(m_shader);
   
	 
    GLuint position;
    GLuint color;
    GLuint normal;
    

	position = glGetAttribLocation( m_shader, "vPosition" );
    glEnableVertexAttribArray( position );
    glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0)); 
    
    color = glGetAttribLocation(m_shader, "vColor");
    glEnableVertexAttribArray(color);
    glVertexAttribPointer(color, 4, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET(sizeof(m_TrianglePoints[0])*m_NTrianglePoints));
    
    normal = glGetAttribLocation(m_shader, "vNormal");
    glEnableVertexAttribArray(normal);
    glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE,0, BUFFER_OFFSET((sizeof(m_TriangleColors[0])+sizeof(m_TrianglePoints[0]))*m_NTrianglePoints));
	
}

void Terrain::Draw(int type, const Camera& camera, const Light& light){


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
    
    Eigen::Vector4f v4color(0.55,0.25,0.08,1.0);
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

	//draw the obstacles
	for(int i = 0; i < m_obstacles.size(); i++)
	{
		m_obstacles[i]->Draw(type,camera, light);
	}

	for(int i = 0; i < m_foods.size(); i++)
	{
		m_foods[i]->Draw(type,camera, light);
	}

	//draw the obstacles
	for(int i = 0; i < m_surface_objects.size(); i++)
	{
		m_surface_objects[i]->Draw(type,camera, light);
	}

}