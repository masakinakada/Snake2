#include "Utility.h"
#include "Camera.h"
#include "Light.h"
#include "Object.h"
#include "3DDeformable.h"
#include "RigidCube.h"
#include "Terrain.h"
#include "World.h"
#include "Ball.h"
#include "Timer.h"
#include "Drawer.h"
#include "Creature.h"
#include "Monkey.h"
#include "GA.h"

//Global Variables
int Window_Width = 1600;
int Window_Height = 1080;

int Button = -1 ;
int PrevY = 0 ;


const int EscKey               = 27;
const int SpaceKey             = 32;

Camera  Pentax;
Light Lumia;
World * myWorld = NULL;
Terrain* myTerrain = NULL;
Drawer* myDrawer = NULL;
Creature* myCreature = NULL;
GA* ga = NULL;

int STOP = -1;//stop the animation
int CONTROL = -1;//control the millipede
int PICK = -1;//pick mode or not
int DRAW_TYPE = DRAW_PHONG;

Timer TM ;
double TIME;
double TIME_LAST;
double DTIME;
double FRAME_TIME = 0;
int FRAME_COUNT = 0;
GLfloat CursorX;
GLfloat CursorY;//Curosr Position

//declaration of function

void initScene();
void reinitScene();
void drawScene();
void keyboardCallback(unsigned char key, int x, int y);
void specialCallback(int key, int x, int y);
void displayCallback();
void reshapeCallback(int w, int h);
void cursorCallback(int x, int y);
void cursorCallback(int x, int y);
void mouseCallback(int button, int state, int x, int y);
void idleCallback();
