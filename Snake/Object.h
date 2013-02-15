//
//  Object.h
//  174A Template
//
//  Created by Jingyi Fang on 2/15/12.
//  Copyright Jingyi Fang. All rights reserved.
//

#ifndef _Object_h
#define _Object_h

#include"Eigen/Dense"
#include "Utility.h"
#include "Camera.h"
#include "Light.h"

#include <vector>

#define GRAVITY_CONSTANT 9.8

#ifndef M_PI
#  define M_PI  3.14159265358979323846
#endif

#ifndef DegreesToRadians
# define DegreesToRadians 0.01745329251994329576922222222222
#endif

enum ObjectType{NoType, TypeCube, TypeSphere, TypeCone, TypeCylinder, TypeCloth, TypeDeformable3D,
				TypeRide, TypeRigidCube, TypeMixed, TypeTerrain };

class Node{
    //the Node class contains the mass, position, velocity and neighbor of the node
    //currently one Dimension
public:
    float m_Mass;
    bool m_Fixed;
    int m_Updated; //-1 or 1; alternate
    Eigen::Vector3f m_Velocity;
    Eigen::Vector3f m_Position;
    Eigen::Vector3f m_Force;
    Eigen::Vector3f m_Normal;//the normal is averaged over the neighboring face

public:
    Node()
    {m_Mass = 0.0; m_Fixed = false; m_Updated  = -1; m_Velocity = Eigen::Vector3f(0.0,0.0,0.0); 
        m_Position = Eigen::Vector3f(0.0,0.0,0.0); m_Force = Eigen::Vector3f(0.0,-m_Mass*GRAVITY_CONSTANT,0.0);
    }//inline Default Constructor
    ~Node(){};

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Triangle{
    
public:
    Triangle(){}
    ~Triangle(){}
    
    Node* m_node_1;
    Node* m_node_2;
    Node* m_node_3;
    
    Eigen::Vector3f m_face_normal;
    
    double m_area;
    double m_density;
    Eigen::Matrix2f m_D_m_Inverse;
    Eigen::Matrix2f m_D_m_Inverse_Transpose;
    
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Tetrahedron{
    
public:
    Tetrahedron(){m_node_1 = 0; m_node_2 = 0; m_node_3 = 0; m_node_4 = 0 ;}
	~Tetrahedron(){}
    
    Node* m_node_1;
    Node* m_node_2;
    Node* m_node_3;
	Node* m_node_4;
    
    double m_volume;
    double m_density;
    
	Eigen::Vector3f m_face_normal_123;
	Eigen::Vector3f m_face_normal_124;
	Eigen::Vector3f m_face_normal_134;
	Eigen::Vector3f m_face_normal_234;
    
    
    Eigen::Matrix3f m_D_m_Inverse;
    Eigen::Matrix3f m_D_m_Inverse_Transpose;
    
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Mesh3D{
//store pure gemoetry
public:
    Mesh3D(Eigen::Vector3i a_num, Eigen::Vector3f a_size, Eigen::Vector3f a_position);
    virtual ~Mesh3D(){}
    void Meshialize(Eigen::Vector3i a_num, Eigen::Vector3f a_size, Eigen::Vector3f a_position);
    
	std::vector<Node*> GetTopNodes();//return the vector nodes on the +y face of the 3D cubic mesh
	std::vector<Node*> GetBottomNodes();//-y
    std::vector<Node*> GetFrontNodes();//+z
    std::vector<Node*> GetBackNodes();//-z
    std::vector<Node*> GetLeftNodes();//-x
    std::vector<Node*> GetRightNodes();//+x
    
	void GetTopSurface(std::vector<Node*>& nodes, std::vector<int>& faces);
	void GetBottomSurface(std::vector<Node*>& nodes, std::vector<int>& faces);
	void GetBackSurface(std::vector<Node*>& nodes, std::vector<int>& faces);
	void GetFrontSurface(std::vector<Node*>& nodes, std::vector<int>& faces);
	void GetLeftSurface(std::vector<Node*>& nodes, std::vector<int>& faces);
	void GetRightSurface(std::vector<Node*>& nodes, std::vector<int>& faces);
	void GetSurface(std::vector<Node*>& nodes, std::vector<int>& faces);

    Eigen::Vector3f m_Size;
    Eigen::Vector3i m_Num;
    
    Node* m_Nodes;
    Triangle* m_Surface;
    Tetrahedron* m_Tetras;
    
    int m_Num_Node;
    int m_Num_Trian;
    int m_Num_Tetra;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class World;
class Camera;
class Light;
class Object{
public:
    Object();
    virtual ~Object(){};
    virtual void SetName(std::string name){m_name = name;}
    virtual void SetType(ObjectType type){m_type = type;}
    std::string GetName(){return m_name;}
    virtual ObjectType GetType(){return m_type;}
	virtual void SetWorld(World *a_world);
	virtual void Draw(int type, const Camera& camera, const Light& light) = 0;
	virtual void UpdateAll(double dt) = 0;
    World* m_world;

public:
    std::string m_name;
    ObjectType m_type;
};


class Cube:public Object{
    
protected:
   
    GLuint m_vertexArrayObject;                      
    GLuint m_vertexBufferObject;  
    GLuint m_shader;
    Eigen::Vector4f m_Points[8];//8 Vertices

    Eigen::Vector4f* m_Vertices;
    Eigen::Vector3f *m_Normals;
    int m_Index;
    
public:
    Eigen::Vector3f m_Center;//For generating translation Angel::matrix
    Eigen::Vector3f m_Size;//For generating scaling Angel::matrix
	Eigen::Vector3f m_Color;//6 Face Colors

    Eigen::Affine3f m_Trans;//the transformation from object to world
    Eigen::Affine3f m_TransBack;//the tranformation from world to object
    
public:
    Cube();//Default constructor create a unit cube in center of screen
	virtual ~Cube(){}
    Cube(Eigen::Vector3f, Eigen::Vector3f,Eigen::Vector3f );//constructor creating a cube with size and center
    void GenFace(int colorid, int a, int b, int c, int d);
    virtual void Init(Eigen::Vector3f center,Eigen::Vector3f size, Eigen::Vector3f color);//Init the data
    virtual void InitDraw();//Init the vertexs and color data on GPU, Init the shader program, link the shader program with the buffer data
    virtual void Draw(int type, const Camera& camera, const Light& light);//Update data on GPU's buffer and draw the vertexs, rotate clockwise around z with speed
	virtual void UpdateAll(double dt){};
	virtual void Output2File(std::ofstream* filestream);
	void SetColor(const Eigen::Vector3f& a_color){m_Color = a_color;}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



class Cylinder:public Object{

public:
    Cylinder();//Default constructor create a unit cube in center of screen
	virtual ~Cylinder(){}
	void Init(Eigen::Vector3f center,Eigen::Vector3f size, Eigen::Vector3f color);
	void InitDraw();
    void Draw(int type, const Camera& camera, const Light& light);
	void UpdateAll(double dt){};
    
protected:
   
    GLuint m_vertexArrayObject;                      
    GLuint m_vertexBufferObject;  
    GLuint m_shader;
    Eigen::Vector4f *m_Vertices;
    Eigen::Vector3f *m_Normals;
	Eigen::Vector2f *m_CirclePoints;

	int m_Index;

	void makeCircle();
	void makeConeWall(float z1, float z2, int dir);
    void GenerateCylinder();
    
public:
	Eigen::Vector3f m_Center;//For generating translation Angel::matrix
    Eigen::Vector3f m_Size;
	Eigen::Vector3f m_Color;

	Eigen::Affine3f m_Trans;//the transformation from object to world
    Eigen::Affine3f m_TransBack;//the tranformation from world to object

   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class Sphere:public Object{
    
protected:
   
    GLuint m_vertexArrayObject;                      
    GLuint m_vertexBufferObject;  
    GLuint m_shader;

	
    Eigen::Vector4f* m_Vertices;
    Eigen::Vector3f *m_Normals;
    int m_Index;
	int m_count;
    
public:    
    Eigen::Vector3f m_Center;//For generating translation Angel::matrix
    Eigen::Vector3f m_Size;//For generating scaling Angel::matrix
	Eigen::Vector3f m_Rotation;
    Eigen::Affine3f m_Trans;
    Eigen::Affine3f m_TransBack;
    Eigen::Vector3f m_Color;

public:
    Sphere();//Default constructor create a unit cube in center of screen
    Sphere(Eigen::Vector3f, Eigen::Vector3f,Eigen::Vector3f );//constructor creating a cube with size and center
    void GenerateSphere();
	Eigen::Vector4f Unit(const Eigen::Vector4f &p);
	void DividTriangle(const Eigen::Vector4f& a,const Eigen::Vector4f& b, const Eigen::Vector4f& c, int n);//for iterative generation of sphere
    void Init(Eigen::Vector3f center,Eigen::Vector3f size, Eigen::Vector3f color);//Init the data
    void InitDraw();//Init the vertexs and color data on GPU, Init the shader program, link the shader program with the buffer data
    void Draw(int type, const Camera& camera,const Light& light);//Update data on GPU's buffer and draw the vertexs, rotate clockwise around z with speed
	virtual void UpdateAll(double dt){};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class Cone:public Object{
    
protected:
   
    GLuint m_vertexArrayObject;                      
    GLuint m_vertexBufferObject;  
    GLuint m_shader;
    Eigen::Vector4f *m_Vertices;
    Eigen::Vector3f *m_Normals;
	Eigen::Vector2f *m_CirclePoints;

	int m_Index;

	void makeCircle();
	void makeConeWall(float z1, float z2, int dir);
    void GenerateCone();
   
public:
    Eigen::Vector3f m_Center;//For generating translation Angel::matrix
    Eigen::Vector3f m_Size;
	Eigen::Vector3f m_Color;

	Eigen::Affine3f m_Trans;//the transformation from object to world
    Eigen::Affine3f m_TransBack;//the tranformation from world to object

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Cone();//Default constructor create a unit cube in center of screen
	void Init(Eigen::Vector3f center,Eigen::Vector3f size, Eigen::Vector3f color);
	void InitDraw();
    void Draw(int type, const Camera& camera, const Light& light);//Update data on GPU's buffer and draw the vertexs, rotate clockwise around z with speed
	void UpdateAll(double dt){}
};

#endif
