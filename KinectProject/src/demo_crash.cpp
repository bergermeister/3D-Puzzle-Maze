/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// This is a demo of the QuickStep and StepFast methods,
// originally by David Whittaker.

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "testApp.h"
#include <pthread.h>
#include <iostream>

using namespace std;

char maze[100][100];
int rows=0;
int cols=0;

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif


// select the method you want to test here (only uncomment *one* line)
#define QUICKSTEP 1
//#define STEPFAST 1

// some constants

#define LENGTH 3.5 //3.5		// chassis length
#define WIDTH 2.5 //2.5		// chassis width
#define HEIGHT 1.0 //1.0		// chassis height
#define RADIUS 0.5 //0.5		// wheel radius
#define STARTZ 1.0 //1.0		// starting height of chassis
#define CMASS 5			                   // chassis mass
#define WMASS 5			                   // wheel mass
#define COMOFFSET -10		               // center of mass offset
#define WALLMASS 100	                   // wall box mass
#define PAPERWALLMASS 10	                 // wall box mass
#define BALLMASS 1		                      // ball mass
#define FMAX 25			                     // car engine fmax
#define ROWS 1			                     // rows of cars
#define COLS 1			                    // columns of cars
#define ITERS 20	                      	// number of iterations
#define WBOXSIZE 10.0	                	// size of wall boxes
#define WALLWIDTH 12		               // width of wall
#define WALLHEIGHT 10		               // height of wall
#define DISABLE_THRESHOLD 0.008	          // maximum velocity (squared) a body can have and be disabled
#define DISABLE_STEPS 10	             // number of steps a box has to have been disable-able before it will be disabled


#define PERSON
#define MAZE
#define PAPERW

// dynamics and collision objects (chassis, 3 wheels, environment)
static dWorldID world;
static dSpaceID space;
static dBodyID body[10000];
static int bodies;
static dJointID joint[100000];
static int joints;
static dJointGroupID contactgroup;
static dGeomID ground;
static dGeomID box[10000];
static int boxes;
static dGeomID sphere[10000];
static int spheres;
static dGeomID wall_boxes[10000];
static dBodyID wall_bodies[10000];
static dGeomID paper_boxes[10000];
static dBodyID paper_bodies[10000];
static dGeomID arm_boxes[4];
static dBodyID arm_bodies[4];
static int wb_stepsdis[10000];
static int wp_stepsdis[10000];
static int wb;
static int wp;
static bool doFast;
static dBodyID b;
static dMass m;
static bool collide = false;
static float xyz[3] = {10.0f, 15.0f, 4.0f};
static float hpr[3] = {215.0f, -12.5f, 0.25f};


// things that the user control
static dReal turn = 0, speed = 0;	// user commands
static dReal cannon_angle=0,cannon_elevation=-1.2;

dVector3 bodypos = {0, 0, 0};  //saving body pos for collision camera

//THREADS!
static int done[2]={0,1};
static int argc_data;
static char **argv_data;
static dsFunctions fn_data;
struct shared_data * shm2 = (struct shared_data *)malloc(sizeof(struct shared_data*));


//-----------------------------------------------------------------------------------------------------------
// this is called by dSpaceCollide when two objects in space are potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	int i,n;
	
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnected(b1, b2))
		return;
	
	const int N = 4;
	dContact contact[N];
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	if (n > 0) {
		for (i=0; i<n; i++) {
			contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
			if (dGeomGetClass(o1) == dSphereClass || dGeomGetClass(o2) == dSphereClass)
				contact[i].surface.mu = 20;
			else
				contact[i].surface.mu = 0.5;	//0.5
			contact[i].surface.slip1 = 0.0;
			contact[i].surface.slip2 = 0.0;
			contact[i].surface.soft_erp = 0.8; // 0.8
			contact[i].surface.soft_cfm = 0.01; // 0.01
			dJointID c = dJointCreateContact (world,contactgroup,contact+i);
			dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
			collide = true;
		}
	}
	collide = false;
}
//-----------------------------------------------------------------------------------------------------------
// Read Maze
void readMaze (){
	ifstream infile;
	
	rows = 0;
	cols = 0;

	infile.open ("maze.txt");

	if (!infile.is_open()){
		cout << "Error reading file" << endl;
		return;
	}

	infile >> rows;											//reads number of rows
	infile >> cols;											//reads number of columns

	cout << "Number of rows is " << rows << endl;
	cout << "Number of columns is " << cols << endl << endl;
	
	char nxt;
	infile.get(nxt);										//stores the first character

	for (int k = 0; k < rows; k++) {						//reads and stores rest of maze
		for (int i = 0; i < cols; i ++){
			infile.get(nxt);
			cout << nxt;									//prints original maze
			maze[k][i] = nxt;
		}
		infile.get(nxt);
		cout << endl;
	}

	return;
}
//-----------------------------------------------------------------------------------------------------------
// Create Maze
void createMaze(){
	for (int i = 0; i < rows; i++){
		for(int j = 0; j < cols; j++){
			if(maze[i][j] == '*'){
				wall_bodies[wb] = dBodyCreate (world);
				dBodySetPosition (wall_bodies[wb],-WBOXSIZE*j,-WBOXSIZE*i,10);
				dMassSetBox (&m,1,WBOXSIZE,WBOXSIZE,WBOXSIZE);
				dMassAdjust (&m, WALLMASS);
				dBodySetMass (wall_bodies[wb],&m);
				wall_boxes[wb] = dCreateBox (space,WBOXSIZE,WBOXSIZE,WBOXSIZE*2);
				dGeomSetBody (wall_boxes[wb],wall_bodies[wb]);
				wb++;
			}
			else if (maze[i][j] == '#') {
				paper_bodies[wp] = dBodyCreate (world);
				dBodySetPosition (paper_bodies[wp],-WBOXSIZE*j,-WBOXSIZE*i,10);
				dMassSetBox (&m,1,WBOXSIZE,WBOXSIZE,WBOXSIZE);
				dMassAdjust (&m, PAPERWALLMASS);
				dBodySetMass (paper_bodies[wp],&m);
				paper_boxes[wp] = dCreateBox (space,WBOXSIZE/2,WBOXSIZE/2,WBOXSIZE/2);
				dGeomSetBody (paper_boxes[wp],wall_bodies[wp]);
				//dBodyDisable(wall_bodies[wb++]);
				wp++;
			}
		
		}
	}

	return;
}
//-----------------------------------------------------------------------------------------------------------
// start simulation - set viewpoint
static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);
	dsSetViewpoint(xyz, hpr);
	//dsSetViewpoint (xyz,hpr);
	printf ("Press:\t'a' to increase speed.\n"
			"\t'z' to decrease speed.\n"
			"\t',' to steer left.\n"
			"\t'.' to steer right.\n"
			"\t' ' to reset speed and steering.\n"
			"\t'[' to turn the cannon left.\n"
			"\t']' to turn the cannon right.\n"
			"\t'1' to raise the cannon.\n"
			"\t'2' to lower the cannon.\n"
			"\t'x' to shoot from the cannon.\n"
			"\t'f' to toggle fast step mode.\n"
			"\t'+' to increase AutoEnableDepth.\n"
			"\t'-' to decrease AutoEnableDepth.\n"
			"\t'r' to reset simulation.\n");
}

//-----------------------------------------------------------------------------------------------------------
// Make boxPerson
void makeBoxPerson(dReal x, dReal y, int &bodyI, int &jointI, int &boxI, int &sphereI)
{
	int i;
	dMass m;
	
	// chassis body
	body[bodyI] = dBodyCreate (world);
	dBodySetPosition (body[bodyI],shm2->x,shm2->y,STARTZ);
	dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
	dMassAdjust (&m,CMASS/2.0);
	dBodySetMass (body[bodyI],&m);
	box[boxI] = dCreateBox (space,LENGTH,WIDTH,HEIGHT);
	dGeomSetBody (box[boxI],body[bodyI]);
	
	bodypos[0] =shm2->x;
	bodypos[1] = shm2->y;
	bodypos[2] = STARTZ;
	
	// wheel bodies
	for (i=1; i<=4; i++) {
		body[bodyI+i] = dBodyCreate (world);
		dQuaternion q;
		dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
		dBodySetQuaternion (body[bodyI+i],q);
		dMassSetSphere (&m,1,RADIUS);
		dMassAdjust (&m,WMASS);
		dBodySetMass (body[bodyI+i],&m);
		sphere[sphereI+i-1] = dCreateSphere (space,RADIUS);
		dGeomSetBody (sphere[sphereI+i-1],body[bodyI+i]);
	}
	dBodySetPosition (body[bodyI+1],shm2->x+0.4*LENGTH-0.5*RADIUS ,shm2->y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
	dBodySetPosition (body[bodyI+2],shm2->x+0.4*LENGTH-0.5*RADIUS ,shm2->y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
	dBodySetPosition (body[bodyI+3],shm2->x-0.4*LENGTH+0.5*RADIUS,shm2->y+WIDTH*0.5,STARTZ-HEIGHT*0.5);
	dBodySetPosition (body[bodyI+4],shm2->x-0.4*LENGTH+0.5*RADIUS,shm2->y-WIDTH*0.5,STARTZ-HEIGHT*0.5);
	
	// front and back wheel hinges
	for (i=0; i<4; i++) {
		joint[jointI+i] = dJointCreateHinge2 (world,0);
		dJointAttach (joint[jointI+i],body[bodyI],body[bodyI+i+1]);
		const dReal *a = dBodyGetPosition (body[bodyI+i+1]);
		dJointSetHinge2Anchor (joint[jointI+i],a[0],a[1],a[2]);
		dJointSetHinge2Axis1 (joint[jointI+i],0,0,(i<2 ? 1 : -1));
		dJointSetHinge2Axis2 (joint[jointI+i],0,1,0);
		dJointSetHinge2Param (joint[jointI+i],dParamSuspensionERP,0.8);
		dJointSetHinge2Param (joint[jointI+i],dParamSuspensionCFM,1e-5);
		dJointSetHinge2Param (joint[jointI+i],dParamVel2,0);
		dJointSetHinge2Param (joint[jointI+i],dParamFMax2,FMAX);
	}
	
	//center of mass offset body. (hang another copy of the body COMOFFSET units below it by a fixed joint)
	dBodyID b = dBodyCreate (world);
	dBodySetPosition (b,shm2->x,shm2->y,STARTZ+COMOFFSET);
	dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
	dMassAdjust (&m,CMASS/2.0);
	dBodySetMass (b,&m);
	dJointID j = dJointCreateFixed(world, 0);
	dJointAttach(j, body[bodyI], b);
	dJointSetFixed(j);
	//box[boxI+1] = dCreateBox(space,LENGTH,WIDTH,HEIGHT);
	//dGeomSetBody (box[boxI+1],b);
	
	
	bodyI	+= 5;
	jointI	+= 4;
	boxI	+= 1;
	sphereI	+= 4;
}
//-----------------------------------------------------------------------------------------------------------
// Reset simulation
void resetSimulation()
{
	int i= 0;
	// destroy world if it exists
	if (bodies)
	{
		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
	}
	
	for (i = 0; i < 1000; i++){
		wb_stepsdis[i] = 0;
		wp_stepsdis[i] = 0;
	}

	// recreate world
	world = dWorldCreate();
	space = dSweepAndPruneSpaceCreate( 0, dSAP_AXES_XYZ );

	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (world,0,0,-1.5);
	dWorldSetCFM (world, 1e-5);
	dWorldSetERP (world, 0.8);
	dWorldSetQuickStepNumIterations (world,ITERS);
	ground = dCreatePlane (space,0,0,1,0);
	
	bodies = 0;
	joints = 0;
	boxes = 0;
	spheres = 0;
	wb = 0;
	wp = 0;

#ifdef PERSON
	for (dReal x = 0.0; x < COLS*(LENGTH+RADIUS); x += LENGTH+RADIUS)
		for (dReal y = -((ROWS-1)*(WIDTH/2+RADIUS)); y <= ((ROWS-1)*(WIDTH/2+RADIUS)); y += WIDTH+RADIUS*2)
			makeBoxPerson(x, y, bodies, joints, boxes, spheres);
#endif
#ifdef MAZE
	createMaze();

	dMessage(0,"wall boxes: %i", wb);
#endif

}
//-----------------------------------------------------------------------------------------------------------
// called when a key pressed
static void command (int cmd)
{
	switch (cmd) {
	case 'a': case 'A':
		shm2->h+=5;
		if(shm2->h >= 360)
				shm2->h = 0;
		break;
	case 'd': case 'D':
		shm2->h-=5;
		if(shm2->h < 0)
			shm2->h = 355;
		break;
	case 'w' : case 'W':
		speed +=0.3;
		if(speed >5)
			speed =5;
		shm2->x += cos(shm2->h * PI / 180);
		shm2->y += sin(shm2->h * PI / 180);
		break;
	case 's' : case 'S':
		speed -=0.3;
		if(speed < -5)
			speed = -5;
		shm2->x -= cos(shm2->h * PI / 180);
		shm2->y -= sin(shm2->h * PI / 180);
		break;
	case 'z': case 'Z':
		speed -= 0.3;
		break;
	case ',':
		turn += 0.1;
		if (turn > 0.3)
			turn = 0.3;
		break;
	case '.':
		turn -= 0.1;
		if (turn < -0.3)
			turn = -0.3;
		break;
	case ' ':
		speed = 0;
		turn = 0;
		break;
	case 'f': case 'F':
		doFast = !doFast;
		break;
	case '+':
		dWorldSetAutoEnableDepthSF1 (world, dWorldGetAutoEnableDepthSF1 (world) + 1);
		break;
	case '-':
		dWorldSetAutoEnableDepthSF1 (world, dWorldGetAutoEnableDepthSF1 (world) - 1);
		break;
	case 'r': case 'R':
		resetSimulation();
		break;
	case '[':
		cannon_angle += 0.1;
		break;
	case ']':
		cannon_angle -= 0.1;
		break;
	case '1':
		cannon_elevation += 0.1;
		break;
	case '2':
		cannon_elevation -= 0.1;
		break;
	default:
		speed -= 0.1;
		if (speed < 0)
			speed = 0;
		break;
	}

}
//-----------------------------------------------------------------------------------------------------------
// update camera
void updateCamera(const dVector3 f)
{
	bodypos[0] = f[0];
	bodypos[1] = f[1];
	bodypos[2] = f[2] + 2;
}
//-----------------------------------------------------------------------------------------------------------
// simulation loop
static void simLoop (int pause)
{
	int i, j;

	dsSetTexture (DS_WOOD);

	float xyz[3]={shm2->x, shm2->y, shm2->z};
	float hpr[3] = {shm2->h, shm2->p, shm2->r};

	if (!pause) {
#ifdef BOX
		dBodyAddForce(body[bodies-1],lspeed,0,0);
#endif
		
		for (j = 0; j < joints; j++)
		{
			dReal curturn = dJointGetHinge2Angle1 (joint[j]);
			//dMessage (0,"curturn %e, turn %e, vel %e", curturn, turn, (turn-curturn)*1.0);
			dJointSetHinge2Param(joint[j],dParamVel,(turn + shm2->turn - curturn)*1.0);
			dJointSetHinge2Param(joint[j],dParamFMax,dInfinity);
			dJointSetHinge2Param(joint[j],dParamVel2,speed + shm2->s);
			//dJointSetHinge2Param(joint[j],dParamFMax2,FMAX);
			dBodyEnable(dJointGetBody(joint[j],0));
			dBodyEnable(dJointGetBody(joint[j],1));
		}	
		
		if (doFast)
		{
			dSpaceCollide (space,0,&nearCallback);
#if defined(QUICKSTEP)
			dWorldQuickStep (world,0.05);
#elif defined(STEPFAST)
			dWorldStepFast1 (world,0.05,ITERS);
#endif
			dJointGroupEmpty (contactgroup);
		}
		else
		{
			dSpaceCollide (space,0,&nearCallback);
			dWorldStep (world,0.05);
			dJointGroupEmpty (contactgroup);
		}
		
		for (i = 0; i < wb; i++)
		{
			b = dGeomGetBody(wall_boxes[i]);
			if (dBodyIsEnabled(b)) 
			{
				bool disable = true;
				const dReal *lvel = dBodyGetLinearVel(b);
				dReal lspeed = lvel[0]*lvel[0]+lvel[1]*lvel[1]+lvel[2]*lvel[2];
				if (lspeed > DISABLE_THRESHOLD)
					disable = false;
				const dReal *avel = dBodyGetAngularVel(b);
				dReal aspeed = avel[0]*avel[0]+avel[1]*avel[1]+avel[2]*avel[2];
				if (aspeed > DISABLE_THRESHOLD)
					disable = false;
				
				if (disable)
					wb_stepsdis[i]++;
				else
					wb_stepsdis[i] = 0;
				
				if (wb_stepsdis[i] > DISABLE_STEPS)
				{
					dBodyDisable(b);
					dsSetColor(0.5,0.5,1);
				}
				else
					dsSetColor(1,1,1);

			}
			else
				dsSetColor(0.4,0.4,0.4);
			dVector3 ss;
			dGeomBoxGetLengths (wall_boxes[i], ss);
			dsDrawBox(dGeomGetPosition(wall_boxes[i]), dGeomGetRotation(wall_boxes[i]), ss);
		}
		
		dsSetTexture (DS_CHECKERED);
		for (i = 0; i < wp; i++)
		{
			b = dGeomGetBody(paper_boxes[i]);
			dVector3 ss;
			dGeomBoxGetLengths (paper_boxes[i], ss);
			dsDrawBox(dGeomGetPosition(paper_boxes[i]), dGeomGetRotation(paper_boxes[i]), ss);
		}

	}
	else
	{
		for (i = 0; i < wb; i++)
		{
			b = dGeomGetBody(wall_boxes[i]);
			if (dBodyIsEnabled(b))
				dsSetColor(1,1,1);
			else
				dsSetColor(0.4,0.4,0.4);
			dVector3 ss;
			dGeomBoxGetLengths (wall_boxes[i], ss);
			dsDrawBox(dGeomGetPosition(wall_boxes[i]), dGeomGetRotation(wall_boxes[i]), ss);
		}
		dsSetTexture (DS_CHECKERED);
		for (i = 0; i < wp; i++)
		{
			//dsSetTexture(DS_CHECKERED);
			b = dGeomGetBody(paper_boxes[i]);
			if (dBodyIsEnabled(b))
				dsSetColor(1,1,1);
			else
				dsSetColor(0.4,0.4,0.4);
			dVector3 ss;
			dGeomBoxGetLengths (paper_boxes[i], ss);
			dsDrawBox(dGeomGetPosition(paper_boxes[i]), dGeomGetRotation(paper_boxes[i]), ss);
		}
	}

	dsSetColor (0,1,1);



	dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
	//dGeomSetPosition(box[0],bodypos[0],bodypos[1],bodypos[2]);
	//dsDrawBox (dGeomGetPosition(box[0]),dGeomGetRotation(box[0]),sides);
	for (i = 0; i < boxes; i++)	
		dsDrawBox (dGeomGetPosition(box[i]),dGeomGetRotation(box[i]),sides);
	dsSetColor (1,1,1);
	for (i=0; i< spheres; i++) dsDrawSphere (dGeomGetPosition(sphere[i]),
		dGeomGetRotation(sphere[i]),RADIUS);

	updateCamera(dGeomGetPosition(box[0]));
	float cameraPos[3]={bodypos[0],bodypos[1],bodypos[2]};
	dsSetViewpoint(cameraPos, hpr);

}
//-----------------------------------------------------------------------------------------------------------
// Threads!
void *threadSim(void *arg)
{
	// run simulation
	dsSimulationLoop (argc_data,argv_data,352,288,&fn_data);
	done[0]=1;
	return NULL;
}
void *threadSimK(void *arg)
{
	// run simulation	
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 400, 850, OF_WINDOW);
	ofRunApp(new testApp(shm2));

	done[1]=1;
	return NULL;
}

//-----------------------------------------------------------------------------------------------------------
// MAIN
int main (int argc, char **argv)
{
	pthread_t tid[2];
	int count_thread=0;

	readMaze();

	shm2->x=10.0f;
	shm2->y=15.0f;
	shm2->z=7.5900f;
	shm2->h=215.0f;
	shm2->p=-12.5f;
	shm2->r=0.25f;
	shm2->s=0;

	doFast = true;
	
	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

	//assigning struct
	//d=new struct data; //(struct data*)malloc(sizeof(struct data));
	argc_data=argc;
	argv_data=argv;
	fn_data=fn; 

	dInitODE2(0);

	bodies = 0;
	joints = 0;
	boxes = 0;
	spheres = 0;
	
	resetSimulation();
	// run simulation
	int rc=pthread_create(&tid[0],NULL,threadSim,NULL);
	int rcK=pthread_create(&tid[1],NULL,threadSimK,NULL);

	while(count_thread<2)
	{
		if(done[0])
		{
		   pthread_join(tid[0],NULL);
		   count_thread++;
		   done[0]=0;
		}
		if(done[1])
		{
		   pthread_join(tid[1],NULL);
		   count_thread++;
		   done[1]=0;
		}
		Sleep(1);
	}
	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);
	dCloseODE();

	free(shm2);

	return 0;
}
