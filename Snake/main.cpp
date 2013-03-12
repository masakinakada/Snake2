//
//  main.cpp
//  Millipede
//
//  Created by Jingyi Fang on 2/10/11.
//  Copyright 2011 Jingyi Fang. All rights reserved.
//

#include "main.h"
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>


void initScene(){
    
	std::cout<<"Initiallizing The System...."<<std::endl;
    
	std::cout<<"Setting up Light, Camera and Clock...."<<std::endl;
    //set up the camera
    Pentax.Init(Eigen::Vector4f(0,250,250.0,1.0),Eigen::Vector4f(0,0,0.0,1.0), Eigen::Vector4f(0,1.0,0,0),
                60, Window_Width/Window_Height , 1.0, 1000);
    
    //set up the light
    Lumia.m_position = Pentax.m_position;
    Lumia.m_color = Eigen::Vector4f(1.0,1.0,1.0,1.0);//white light
    
	//set up the world
	myWorld = new World(50000);
    
	//set up the drawer
	myDrawer = new Drawer;
	myDrawer->PushMatrix();
    
    std::cout<<"Setting up the World..."<<std::endl;
    
	myTerrain = new Terrain(Eigen::Vector2f(500,500), Eigen::Vector2i(100,100), 100, TERRAIN_DOWNHILL);
    
	reinitScene();
    
    //change initial camera position
    Pentax.m_zoom  = Pentax.m_zoom  * 0.25;
    
	std::cout<<"Starting Animation..."<<std::endl;
    
    
}

void reinitScene(){
    
	//press SpaceBar to trigger
    
	myWorld->Clear();//clear everything
	myWorld->Add_Object(myTerrain);//add back the terrain
    

    ga = new GA(myWorld);
    
    TIME_LAST = TM.GetElapsedTime() ;
	DTIME = 0.0;
	FRAME_TIME = 0.0;
    
    
}

void drawScene(){
    
    glEnable( GL_DEPTH_TEST );
    glClearColor(0.0, 0.0, 0.0, 0.0);//Black background
    
	Pentax.Update(DTIME);
    
    Lumia.m_position = Pentax.m_position;//the light is attached to the camera
	
	myWorld->Draw(DRAW_TYPE, Pentax, Lumia);
    TIME = TM.GetElapsedTime() ;
    
	DTIME = TIME - TIME_LAST;
	TIME_LAST = TIME;
}

void keyboardCallback(unsigned char key, int x, int y){
    
	if(CONTROL == 1){
		switch(key)
		{
			case '7'://down
                break;
		}
	}
    
	if ( key == EscKey || key == 'q' || key == 'Q' )
    {
        writeBestGenome();
        exit(0);
    }
    if( key == 's'|| key == 'S')
    {
        STOP *= -1;
    }
	if( key == 'p'|| key == 'P' )
	{
		PICK *= -1;
		if(PICK == 1)
			std::cout<<"Picking Mode"<<std::endl;
		if(PICK == -1)
			std::cout<<"Rotating Mode"<<std::endl;
	}
    
    if( key == '1')
        DRAW_TYPE = DRAW_MESH;
    if( key == '2')
        DRAW_TYPE = DRAW_PHONG;
    if( key == '3')
        DRAW_TYPE = DRAW_TEXTURE;
    
    
    //reset the scene and camera
    if ( key == SpaceKey) {
        reinitScene();
        glutSwapBuffers();
    }
    
    if (key =='d'){
        drawFlag = !drawFlag;
        std::cout<<"Stop Drawing on display. Keep iteration for learning. this should run fater"<<std::endl;
    }
}

void writeBestGenome(){
    ga->writeBest();
}

void displayCallback(){
    if(drawFlag){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        drawScene();
        glutSwapBuffers();
    }

}

void reshapeCallback(int w, int h){
    Window_Width = w;
    Window_Height = h;
    
    glViewport(0, 0, w, h);
	Pentax.m_aspect = (float)w/(float)h;
	glutPostRedisplay() ;
    
}

void motionCallback(int x, int y){
    
    if( Button == GLUT_LEFT_BUTTON )
    {
		CursorX = double(2*x-Window_Width)/(Window_Width);
		CursorY = double(Window_Height-2*y)/(Window_Height);
        
		Pentax.MouseDrag(CursorX, CursorY);
		
        glutPostRedisplay() ;
    }
    else if( Button == GLUT_RIGHT_BUTTON )
    {
        if( y - PrevY > 0 )
            Pentax.m_zoom  = Pentax.m_zoom  * 1.03 ;
        else
            Pentax.m_zoom   = Pentax.m_zoom  * 0.97 ;
        PrevY = y ;
        glutPostRedisplay() ;
    }
}

void mouseCallback(int button, int state, int x, int y){
    
	CursorX = double(2*x-Window_Width)/(Window_Width);
	CursorY = double(Window_Height-2*y)/(Window_Height);
    
	Button = button;
	if( Button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		if(PICK == -1){
			Pentax.MouseLeftDown(CursorX,CursorY);
		}
        
	}
	if( Button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{
		if(PICK == -1){
			Pentax.MouseLeftUp(CursorX,CursorY);
			Button = -1 ;
		}
        
		
	}
	if( button == GLUT_RIGHT_BUTTON && state == GLUT_UP )
	{
		if(PICK == 1){
			//World.Pick(x,y);
		}
	}
    
    if( Button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN )
    {
        PrevY = y ;
    }
	glutPostRedisplay() ;
}

void cursorCallback(int x, int y){
    
	CursorX = double(2*x-Window_Width)/(Window_Width);
	CursorY = double(Window_Height-2*y)/(Window_Height);
}

void idleCallback(){
    
	TIME = TM.GetElapsedTime() ;
    
	DTIME = TIME - TIME_LAST;
	TIME_LAST = TIME;
    
	
	FRAME_TIME += DTIME;
    
	if(STOP == -1){

		myWorld->Update(0.015);//real dt for physics is 1/2000, look inside.
		ga->iterate(TIME, 0.015);//dt for ga is 1/100
	   
	}
    
	if(FRAME_TIME > 0.05)//20 frames per second
	{
		glutPostRedisplay(); //draw new frame
		FRAME_TIME = 0;
		FRAME_COUNT++;
		//OUTPUT_ONE_FRAME();
	}
//printf("Physics Rate %f\r", 1.0/DTIME) ;
}



int main (int argc, char ** argv){
    // init GLUT
#ifdef __APPLE__
 std::srand ( unsigned ( std::time(0) ) );
#endif // _APPLE

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    
	glutInitWindowSize(Window_Width,Window_Height);
    glutCreateWindow("Evalutionary Learning with Rigid-Deformable - Masaki Nakada");
    // init GLUT callbacks
    glutIdleFunc(idleCallback);
	glutReshapeFunc (reshapeCallback);
    glutKeyboardFunc(keyboardCallback);
    glutMouseFunc(mouseCallback) ;
    glutMotionFunc(motionCallback);
	glutPassiveMotionFunc(cursorCallback);
    glutDisplayFunc(displayCallback);
#ifndef __APPLE__
	glewInit();
#endif
    initScene();
    glutMainLoop();
    
    return 0;
}


