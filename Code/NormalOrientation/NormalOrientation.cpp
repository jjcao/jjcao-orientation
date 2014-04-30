// NormalOrientation.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "pclNormal.h"
//============================================================================
// Name        : pointlab.cpp
// Author      : Andrea Tagliasacchi
// Version     : 1.0
// Copyright   : (c) Andrea Tagliasacchi - All Rights Reserved
// Description : A point cloud viewer with stencil buffer to compensate for
//				 overlapping transparent splats
//============================================================================
#pragma warning(disable : 4244 4290 4305 4800 4996)
// General includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <numeric>
#include <boost/timer.hpp>
#ifdef __LINUX__
#include <GL/glu.h>
#include <GL/gl.h>
#include <GL/glut.h>
#endif
#ifdef __APPLE__
#include <OpenGL/glu.h>
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif
#ifdef _WIN32
#include <gl/gl/glut.h>
#endif
#include "myutil.h"
#include "MyHeaps.h"
#include <atlstr.h>
using namespace std;

// for picking and draging, added by jjcao
#define BUFSIZE 512
GLuint selectBuf[BUFSIZE];
double snapDist(0xffffffff);

// GUI Variables
float clear_color[3] = { 1, 1, 1 }; //white
int window_height = 800; //1100
int window_width  = 800; //1100
double eyepos[3] = {0,0,1}; // from what position I look at the object

// Lights
float light_ambient[4] = { 0.3f, 0.3f, 0.3f, 1.0f };
float light_diffuse[4] = { 0.6f, 0.6f, 0.6f, 1.0f };
float light0_position[4] = { 0.0f, 0.0f, 5.0f, 0.0f };
float light1_position[4] = { 1.0f, 1.0f, 1.0f, 0.0f };

// Colors
//float surfelcolor[4] = {1, .73, .0, 0.5f}; // gold
//float surfelcolor[4] = {0.275, .337, .60, 0.5f}; // blue
float surfelcolor[4] = {1.00, .65, .35, 0.5f}; // orange
//float surfelcolor[4] = {1.00, .55, .35, 0.5f};//pink
float pickcolor[4] = {1.00, .55, .95, 0.5f};//purple
//float surfelcolor[4] = {0.12, .35, .35, 0.5f};//cyan
float clustercolor[4] = {0.19, .87, .322, 0.5f};//green
float blue[3] = {0, 0, 1};
float red[3] = {1, 0, 0};
//float vertColor[3] = {1, .8, .8};//added by jjcao
float vertColor[3]={0,0,1};
float edgeColor[3] = {1, 0, 0};//added by jjcao
float edgePickColor[3] = {0, 0, 0.5}; // added by jjcao
float vertPickColor[3] = {0, 0, 1}; // added by jjcao 点击点的颜色
// Modelview
//double modelview_rot[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; //standard
double modelview_rot[16] = { 0.169773, 0.883897, 0.435759, 0,
-0.0581135, 0.45039, -0.890929, 0,
-0.983763, 0.125933, 0.127831, 0, 0, 0, 0, 1 }; //for Sharf's woman
//double modelview_rot[16] = { -0.998294, 0.0230628, -0.0535224, 0,
//                                -0.0174278, 0.758218, 0.651759, 0,
//								         0.0556148, 0.651586, -0.756529, 0, 0, 0, 0, 1 }; //for Pecaso
//double modelview_rot[16] = {-0.886703, -0.156514, -0.435038, 0,
//										-0.26335,   0.944216,  0.197471,  0,
//										0.379864,  0.289753, -0.878491,  0, 0, 0, 0, 1 };//for fertility_1n
//double modelview_rot[16] = {0.589297, -0.325771, 0.739324, 0,
//										-0.72799, 0.182738, 0.660784, 0,
//										-0.350367, -0.927621, -0.129471,0,0, 0, 0, 1};//for runman1s1 reconstruction
//double modelview_rot[16] = {0.751328, -0.352342, 0.557993, 0,
//										-0.597814, -0.00526998, 0.801616, 0,
//										-0.279503, -0.9355857, -0.214595, 0, 0, 0, 0, 1};//for runman1s1 skeleton edit
// modelview data
enum ModelViewStatus {NORMAL, ROTATING, TRANSLATING, ZOOMING, PICKING} modelview_status;
int pick_vid = -1;// added by jjcao
int pick_eid = -1;// added by jjcao
double modelview_sx = 0, modelview_sy = 0; // modelview I/O starting position
double modelview_sz = 0; // added by jjcao
double modelview_tx=0, modelview_ty=0; // x/y axis translation
double modelview_zoom = 1; // zoom

// Flow control variables
bool showsplats			= false;
bool isanimated         = false;
bool tranparencyEnabled = false;
bool show_skeleton     = true;
bool show_pcloud         = false; // added by jjcao
bool show_cylindrical_approximation = false; // added by jjcao
int  show_backpointing = 0;
int  colorwhat         = 0;
double skeldisksize    = .005;
double markersize      = 0.033241;
vector<int> pickPoint;
int choosePoint(0);
class Pcloud;
class Graph;
class NormalOrientation;
// Data models
Pcloud* pcloud(0); // the point cloud
Graph* graph(0);   // the skeleton graph
NormalOrientation* normalOrientation(0);
char* off_filename;
char* graph_filename;
//char* pcd_filename;
//char* npcd_filename;
char* cg_mst;
char* point_cloud_consistent;
int num_count(1);//记录交互次数
int num_argc(0);
vector<vector<vector<pair<int,int>>>>nff;
vector<vector<pair<int,int>>>minff;
vector<int>flip;
vector<int>unflip;
// Local classes
class Pcloud{
public:
	vector<double> points;	// point  coordinates, //changed by jjcao form vector<double> to vector<vector<double> >
	vector<double> normals; // curvature normals
	vector<double> skele;   // skeletal points
	vector<double> color;   // color of skeletal or cloud
	vector<int> c2sMaps; // correspondence from point to skeleton nodej, added by jjcao
	int 	npoints; 		// number of points
	int 	ndims;   		// dimensionality of the points
	vector<double> normals_consistent;
	// Constructor
	Pcloud(const char* cloud_filename,const char* cloud_filename_consistent){ 
		read_cloud(cloud_filename);
		read_cloud_consistent(cloud_filename_consistent);
	}
	// added by jjcao
	//Pcloud(const char* cloud_filename, const char* map_filename)
	Pcloud(const char* cloud_filename){ 
		read_cloud(cloud_filename);
		//read_map(map_filename);
	}
	// added by jjcao
	void read_map(const char* map_filename){
		// open file and check
		FILE* fid = fopen( map_filename, "r" );
		if( fid == NULL )
			cout << "ERROR: file %s does not exists\n" << map_filename;

		this->c2sMaps.resize(this->npoints);

		int j;
		for (int i = 0; i < this->npoints; ++i){
			fscanf(fid, "%d\n", &j);
			this->c2sMaps[i] = j-1;
		}
	}
	// Encapsulated by jjcao
	void read_cloud(const char* cloud_filename){
		// open file and check
		FILE* fid = fopen( cloud_filename, "r" );
		if( fid == NULL )
			cout << "ERROR: file %s does not exists\n" << cloud_filename;

		char buffer[40];
		fscanf( fid,"%s\n", buffer );

		int ignore;
		fscanf( fid,"%d %d\n", &npoints, &ignore);

		this->ndims = 3;
		this->points.resize(npoints*ndims);
		this->normals.resize(npoints*ndims);//added bu jjcao
		this->normals_consistent.resize(npoints*ndims);
		double a,b,c,d,e,f;
		this->color.resize((ndims+1)*npoints);
		for (int i = 0; i < npoints; i++){
			fscanf(fid, "%lf %lf %lf %lf %lf %lf\n", &a, &b, &c, &d, &e, &f );
			setPoint(i, a,b,c);
			setNormal(i,d,e,f);
		}

		//normalize();// added by jjcao
		if (!hasNormal()){ // added by jjcao
			this->normals.clear();
		}
		fclose(fid);// added by jjcao
	}
//////////////////////////////////////////////////////////////////////////
//read consistent orientation of point clouds
	void read_cloud_consistent(const char* consistent_point_filename)
	{
		FILE* fid = fopen( consistent_point_filename, "r" );
		if( fid == NULL )
			cout << "ERROR: file %s does not exists\n" << consistent_point_filename;

		char buffer[40];
		fscanf( fid,"%s\n", buffer );

		int ignore,nums;
		fscanf( fid,"%d %d\n", &nums, &ignore);
		double a,b,c,d,e,f;
		for (int i = 0; i < nums; i++){
			fscanf(fid, "%lf %lf %lf %lf %lf %lf\n", &a, &b, &c, &d, &e, &f );
			setNormal_consistent(i,d,e,f);
		}
		fclose(fid);// added by jjcao
	}
//////////////////////////////////////////////////////////////////////////
	/// Set coordinates of point "i" with x,y,z
	void setPoint( unsigned int i, double x, double y, double z ){
		assert( i<points.size() );
		points[i*ndims + 0] = x;
		points[i*ndims + 1] = y;
		points[i*ndims + 2] = z;
	}
	// added by jjcao
	vector<double> getPoint(int i){
		assert( i<npoints );
		vector<double> p(3);
		unsigned int j(i*ndims);
		p[0] = points[j];
		p[1] = points[j+1];
		p[2] = points[j+2];
		return p;
	}
	/// Set normal for point "i" with vx,vy,vz
	void setNormal( unsigned int i, double vx, double vy, double vz ){
		assert( i<normals.size() );
		normals[i*ndims + 0] = vx;
		normals[i*ndims + 1] = vy;
		normals[i*ndims + 2] = vz;
	}
	void setNormal_consistent(unsigned int i, double vx, double vy, double vz )
	{
		//assert( i<normals.size() );
		normals_consistent[i*ndims + 0] = vx;
		normals_consistent[i*ndims + 1] = vy;
		normals_consistent[i*ndims + 2] = vz;
	}
	// added by jjcao
	void normalize(){// scale to unitBox and move to origin		
		double minx(0xffffffff), maxx(-minx);
		double miny(0xffffffff), maxy(-miny);
		double minz(0xffffffff), maxz(-minz);
		double x,y,z;

		for (int i = 0; i < this->npoints; ++i){
			x = this->points[i*ndims];
			y = this->points[i*ndims+1];
			z = this->points[i*ndims+2];
			if (x<minx) minx = x;
			if (x>maxx) maxx = x;
			if (y<miny) miny = y;
			if (y>maxy) maxy = y;
			if (z<minz) minz = z;
			if (z>maxz) maxz = z;
		}

		vector<double> m1(3), m2(3), c(3), d(3);
		m1[0] = minx; m1[1] = miny; m1[2] = minz;
		m2[0] = maxx; m2[1] = maxy; m2[2] = maxz;

		sum(m1,m2,c);
		scale(c, 0.5);
		diff(m2,m1,d);
		double s = max(d[0], d[1]);
		s = max(s, d[2]);
		s = 1.6 / s;

		int tmp;
		for (int i = 0; i < this->npoints; ++i){
			tmp = i*ndims;
			this->points[tmp] = (this->points[tmp]-c[0])*s; ++tmp;
			this->points[tmp] = (this->points[tmp]-c[1])*s; ++tmp;
			this->points[tmp] = (this->points[tmp]-c[2])*s; 
		}
	}
	// added by jjcao
	bool hasNormal(){
		//std::vector<double>::iterator it = this->normals.begin();
		//++it; ++it; ++it;
		//double res = std::accumulate(this->normals.begin(), it, 0.0);
		double res = std::accumulate(this->normals.begin(), this->normals.end(), 0.0);
		return bool(res);
	}
	/// Set skeletal position for point "i" with skx,sky,skz
	void setSkele( unsigned int i, double skx, double sky, double skz ){
		assert( i<skele.size() );
		skele[i*ndims + 0] = skx;
		skele[i*ndims + 1] = sky;
		skele[i*ndims + 2] = skz;
	}
	void setColor( int i, double cx, double cy, double cz,double cl ){
		//assert( i<color.size() );
		color[i*ndims + 0] = cx;
		color[i*ndims + 1] = cy;
		color[i*ndims + 2] = cz;
		color[i*ndims + 3]=cl;
	}
	void toSysout(){
		for (int i = 0; i < npoints; ++i) {
			for (int j = 0; j < ndims; ++j)
				::printf("%+.3f ", points[i*ndims+j]);
			::printf(": ");


			if( hasNormals() )
				for (int j = 0; j < ndims; ++j)
					::printf("%+.3f ", points[i*ndims+j]);
			if( hasSkele() )
				for (int j = 0; j < ndims; ++j)
					::printf("%+.3f ", skele[i*ndims+j]);

			cout << endl;
		}
	}
	bool hasNormals() const{
		return normals.size();
	}
	bool hasSkele() const{
		return skele.size();
	}
	inline double operator()(int i, int dim){
		assert(i<npoints );
		return points[ i*ndims + dim ];
	}
	void draw_transp(double disksize, int drawbackpoint=0, int colorwhat=0){
		glEnable(GL_BLEND);

		/// create a sorting field for the points to be drawn
		MinHeap<double> heap( npoints );
		double mv[16], pj[16]; int mw[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, mv);
		glGetDoublev(GL_PROJECTION_MATRIX, pj);
		glGetIntegerv(GL_VIEWPORT, mw);
		vector<double> depth( npoints, 0 );
		vector<double> point(3,0);
		vector<double> wpoint(3,0);
		for (int i = 0; i < npoints; i++) {
			gluProject( points[ i*ndims + 0 ],
				points[ i*ndims + 1 ],
				points[ i*ndims + 2 ],
				mv, pj, mw,
				&wpoint[0], &wpoint[1], &wpoint[2]);
			depth[i] = -wpoint[2];
			heap.push( depth[i], i );
		}
		vector<int> srtidxs(npoints, 0);
		heap.heapsort( srtidxs );

		// extract camera vector
		float model[16];
		glGetFloatv (GL_MODELVIEW_MATRIX, model);
		vector<double> camera(3,0);
		camera[0] = model[2];
		camera[1] = model[6];
		camera[2] = model[10];

		// COLOR/QUADRIC SETUP
		//float diskfront[4] = {.81, .42, .11, 0.5f};
		float diskback[4] = {0,0,0,1};
		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_TRUE);
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, surfelcolor);
		glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, diskback);

		/// DRAW BACKFACING
		vector<double> zaxis (3,0); zaxis[2] = 1;
		vector<double> normal(3,0);
		vector<double> crossv(3,0);
		double theta = 0;
		for (int i=0,j; i < npoints; i++){
			j = srtidxs[i];
			normal[ 0 ] = normals[ j*ndims + 0 ];
			normal[ 1 ] = normals[ j*ndims + 1 ];
			normal[ 2 ] = normals[ j*ndims + 2 ];
			if( dot(camera, normal)>=0 )
				continue;

			// estimate rotation parameters
			theta = acos( dot( zaxis, normal ) ) * 180 / PI;
			cross( zaxis, normal, crossv );
			glPushMatrix();
			glTranslatef( (*this)(j,0), (*this)(j,1), (*this)(j,2) );
			glRotatef( theta, crossv[0], crossv[1], crossv[2] );
			gluDisk( q, 0, disksize, 20, 1 );
			glPopMatrix();
		}

		// setup stencil
		glEnable(GL_STENCIL_TEST);
		glStencilFunc( GL_EQUAL, 0, 1 );
		glStencilOp( GL_KEEP, GL_KEEP, GL_INCR );

		/// DRAW FRONTFACING
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);


		for (int i=0,j; i < npoints; i++){
			j = srtidxs[npoints-i-1];
			normal[ 0 ] = normals[ j*ndims + 0 ];
			normal[ 1 ] = normals[ j*ndims + 1 ];
			normal[ 2 ] = normals[ j*ndims + 2 ];
			if( dot(camera, normal)<0 )
				continue;

			// estimate rotation parameters
			theta = acos( dot( zaxis, normal ) ) * 180 / PI;
			cross( zaxis, normal, crossv );
			glPushMatrix();
			glTranslatef( (*this)(j,0), (*this)(j,1), (*this)(j,2) );
			glRotatef( theta, crossv[0], crossv[1], crossv[2] );
			gluDisk( q, 0, disksize/2, 20, 1 );
			glPopMatrix();
		}
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);

		// turn off stencil
		glDisable(GL_STENCIL_TEST);

		// turn off transparent display
		glDisable(GL_BLEND);
	}
	void draw4(double disksize, int drawbackpoint=0, int colorwhat=1, bool showsplats=0 )
	{
		float model[16];
		glGetFloatv (GL_MODELVIEW_MATRIX, model);
		vector<double> camera(3,0);
		camera[0] = model[2];
		camera[1] = model[6];
		camera[2] = model[10];
		// draw a disk
		// float diskfront[3] = {.81, .42, .11};
		float diskback[3] = {0,0,0};
		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_TRUE);
		if( drawbackpoint == 0 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, diskback);
		else if( drawbackpoint == 1 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, surfelcolor);

		vector<double> zaxis (3,0); zaxis[2] = 1;
		vector<double> normal(3,0);
		vector<double> crossv(3,0);
		double theta = 0;
		//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, surfelcolor);
		//可视化种子点定向情况。
		for (int i=0;i<flip.size();i++)
		{
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, pickcolor);//red
			if( hasNormals() && showsplats ){
				normal[ 0 ] = normals[ flip[i]*ndims + 0 ];
				normal[ 1 ] = normals[ flip[i]*ndims + 1 ];
				normal[ 2 ] = normals[ flip[i]*ndims + 2 ];
				theta = acos( dot( zaxis, normal ) ) * 180 / PI;
				cross( zaxis, normal, crossv );
				if( drawbackpoint==2 && dot(camera, normal)<0 )
					continue;

				glPushMatrix();
				glTranslatef( (*this)(flip[i],0), (*this)(flip[i],1), (*this)(flip[i],2) );
				glRotatef( theta, crossv[0], crossv[1], crossv[2] );
				gluDisk( q, 0, 2*disksize, 20, 1 );
				glPopMatrix();
			}
			// Are we using spheres?
			if( showsplats == false ){
				glPushMatrix();
				glTranslatef( (*this)(flip[i],0), (*this)(flip[i],1), (*this)(flip[i],2) );
				gluSphere (q, 2*disksize, 10, 10);
				glPopMatrix();
			}
		}
		for (int j=0;j<unflip.size();j++)
		{
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, clustercolor);//green
			if( hasNormals() && showsplats ){
				normal[ 0 ] = normals[ unflip[j]*ndims + 0 ];
				normal[ 1 ] = normals[ unflip[j]*ndims + 1 ];
				normal[ 2 ] = normals[ unflip[j]*ndims + 2 ];
				theta = acos( dot( zaxis, normal ) ) * 180 / PI;
				cross( zaxis, normal, crossv );
				if( drawbackpoint==2 && dot(camera, normal)<0 )
					continue;

				glPushMatrix();
				glTranslatef( (*this)(unflip[j],0), (*this)(unflip[j],1), (*this)(unflip[j],2) );
				glRotatef( theta, crossv[0], crossv[1], crossv[2] );
				gluDisk( q, 0, 2*disksize, 20, 1 );
				glPopMatrix();
			}
			// Are we using spheres?
			if( showsplats == false ){
				glPushMatrix();
				glTranslatef( (*this)(unflip[j],0), (*this)(unflip[j],1), (*this)(unflip[j],2) );
				gluSphere (q, 2*disksize, 10, 10);
				glPopMatrix();
			}
		}	
	}
	// draw a point cloud using OpenGLz
	void draw(double disksize, int drawbackpoint=0, int colorwhat=1, bool showsplats=0 ){
		// DEBUG: draw a simple sphere
		//		GLUquadricObj *q = gluNewQuadric();
		//		gluQuadricNormals (q,GLU_TRUE);
		//		glColor3f(1,1,1);

		// retrieve modelview matrix
		float model[16];
		glGetFloatv (GL_MODELVIEW_MATRIX, model);
		vector<double> camera(3,0);
		camera[0] = model[2];
		camera[1] = model[6];
		camera[2] = model[10];

		// draw a disk
		// float diskfront[3] = {.81, .42, .11};
		float diskback[3] = {0,0,0};
		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_TRUE);
		//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, surfelcolor);
		if( drawbackpoint == 0 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, diskback);
		else if( drawbackpoint == 1 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, surfelcolor);

		vector<double> zaxis (3,0); zaxis[2] = 1;
		vector<double> normal(3,0);
		vector<double> crossv(3,0);
		double theta = 0;
		int error_num_point(0);//记录定向错误点的个数
		for (int i = 0; i < npoints; i++){
			// Need to apply per splat color?
			//if( colorwhat==1 ){
				double normal1[3],normal2[3];
				float currcolor[4];
				normal1[0]=normals[i*ndims+0];
				normal1[1]=normals[i*ndims+1];
				normal1[2]=normals[i*ndims+2];
				normal2[0]=normals_consistent[i*ndims+0];
				normal2[1]=normals_consistent[i*ndims+1];
				normal2[2]=normals_consistent[i*ndims+2];
				if (dot(normal1,normal2)>0)
				{
					//法向正确的为橘色
					currcolor[0] = surfelcolor[0];
					currcolor[1] = surfelcolor[1];
					currcolor[2] = surfelcolor[2];
					currcolor[3] = surfelcolor[3];
					glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, currcolor);
				} 
				else
				{
					//错误为蓝色
					currcolor[0] = 0;
					currcolor[1] = 0;
					currcolor[2] = 1;
					currcolor[3] = 0;
					glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, currcolor);
					error_num_point++;
				}
				
				
			//}

			// Are we using splats?
			if( hasNormals() && showsplats ){
				normal[ 0 ] = normals[ i*ndims + 0 ];
				normal[ 1 ] = normals[ i*ndims + 1 ];
				normal[ 2 ] = normals[ i*ndims + 2 ];
				theta = acos( dot( zaxis, normal ) ) * 180 / PI;
				cross( zaxis, normal, crossv );
				if( drawbackpoint==2 && dot(camera, normal)<0)
					continue;

				glPushMatrix();
				glTranslatef( (*this)(i,0), (*this)(i,1), (*this)(i,2) );
				glRotatef( theta, crossv[0], crossv[1], crossv[2] );
				if (dot(normal1,normal2)>0)
				{
					gluDisk( q, 0, disksize/2, 20, 1 );
				} 
				else
				{
					gluDisk( q, 0, 2*disksize, 20, 1 );
				}
				glPopMatrix();
			}
			// Are we using spheres?
			if( showsplats == false ){
				glPushMatrix();
				glTranslatef( (*this)(i,0), (*this)(i,1), (*this)(i,2) );
				if (dot(normal1,normal2)>0)
				{
					gluSphere (q, disksize/2, 10, 10);
				} 
				else
				{
					gluSphere (q, 2*disksize, 10, 10);
				}
				glPopMatrix();
			}
		}
		cout<<"Ratio of incorrect normal of point: "<<error_num_point<<" / "<<npoints<<" = "<<(double)error_num_point/npoints<<endl;
	}
	void draw2(double disksize, int drawbackpoint=0, int colorwhat=1, bool showsplats=0 )
	{
		float model[16];
		glGetFloatv (GL_MODELVIEW_MATRIX, model);
		vector<double> camera(3,0);
		camera[0] = model[2];
		camera[1] = model[6];
		camera[2] = model[10];

		// draw a disk
		float diskback[3] = {0,0,0};
		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_TRUE);
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, surfelcolor);
		if( drawbackpoint == 0 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, diskback);
		else if( drawbackpoint == 1 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, surfelcolor);

		vector<double> zaxis (3,0); zaxis[2] = 1;
		vector<double> normal(3,0);
		vector<double> crossv(3,0);
		double theta = 0;
		// Are we using splats?
		for (int i = 0; i < npoints; i++){
			if( hasNormals() && showsplats ){
				normal[ 0 ] = normals[ i*ndims + 0 ];
				normal[ 1 ] = normals[ i*ndims + 1 ];
				normal[ 2 ] = normals[ i*ndims + 2 ];
				theta = acos( dot( zaxis, normal ) ) * 180 / PI;
				cross( zaxis, normal, crossv );
				if( drawbackpoint==2 && dot(camera, normal)<0 )
					continue;

				glPushMatrix();
				glTranslatef( (*this)(i,0), (*this)(i,1), (*this)(i,2) );
				glRotatef( theta, crossv[0], crossv[1], crossv[2] );
				gluDisk( q, 0, disksize/2, 20, 1 );
				glPopMatrix();
			}
			// Are we using spheres?
			if( showsplats == false ){
				glPushMatrix();
				glTranslatef( (*this)(i,0), (*this)(i,1), (*this)(i,2) );
				gluSphere (q, disksize/2, 10, 10);
				glPopMatrix();
			}
		}
	}
	void draw3(int pick,double disksize, int drawbackpoint=0, int colorwhat=1, bool showsplats=0)
	{
		float model[16];
		glGetFloatv (GL_MODELVIEW_MATRIX, model);
		vector<double> camera(3,0);
		camera[0] = model[2];
		camera[1] = model[6];
		camera[2] = model[10];
		// draw a disk
		float diskback[3] = {0,0,0};
		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_TRUE);
		if( drawbackpoint == 0 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, diskback);
		else if( drawbackpoint == 1 )
			glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, surfelcolor);

		vector<double> zaxis (3,0); zaxis[2] = 1;
		vector<double> normal(3,0);
		vector<double> crossv(3,0);
		double theta = 0;
		//可视化聚类情况，显示所选取的离我们最近的聚类点。
		int gz=0;
		for (vector<vector<pair<int,int>>>::iterator i=minff.begin();i!=minff.end();i++)
		{
			for (vector<pair<int,int>>::iterator j=i->begin();j!=i->end();j++)
			{
				int point_number=(*j).second;
				if (point_number==pick)
				{
					//只点击聚类点
					int gz_num=gz;//所在格子标号
					int gz_size(0);//记录格子点数
					vector<vector<pair<int,int>>> eee=nff[gz_num];
					for (vector<vector<pair<int,int>>>::iterator k11=eee.begin();k11!=eee.end();k11++)
					{
						for (vector<pair<int,int>>::iterator k22=k11->begin();k22!=k11->end();k22++)
						{
							++gz_size;
						}
					}
					int fl=0;//记录类标号
					char* filename="I:fandisk_gz.txt";
					FILE *file=fopen(filename,"w");
					fprintf(file,"%d %d\n",gz_num,eee.size());
					//fprintf(file,"%d %d\n",gz_num,gz_size);
					std::cout<<"第"<<gz_num<<"格子"<<"分成"<<eee.size()<<"类"<<std::endl;
					for (vector<vector<pair<int,int>>>::iterator k1=eee.begin();k1!=eee.end();k1++)
					{
						for (vector<pair<int,int>>::iterator k2=k1->begin();k2!=k1->end();k2++)
						{
							int point_gz=(*k2).second;
							if ((*k2).first!=(k1->size()-1))
							{
								fprintf(file,"%d ",point_gz);
							} 
							else
							{
								fprintf(file,"%d\n",point_gz);
							}
							glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, pickcolor);
							if( hasNormals() && showsplats ){
								normal[ 0 ] = normals[ point_gz*ndims + 0 ];
								normal[ 1 ] = normals[ point_gz*ndims + 1 ];
								normal[ 2 ] = normals[ point_gz*ndims + 2 ];
								theta = acos( dot( zaxis, normal ) ) * 180 / PI;
								cross( zaxis, normal, crossv );
								if( drawbackpoint==2 && dot(camera, normal)<0 )
										continue;

								glPushMatrix();
								glTranslatef( (*this)(point_gz,0), (*this)(point_gz,1), (*this)(point_gz,2) );
								glRotatef( theta, crossv[0], crossv[1], crossv[2] );
								gluDisk( q, 0, disksize/2, 20, 1 );
								glPopMatrix();
								}
								// Are we using spheres?
							if( showsplats == false ){
								glPushMatrix();
								glTranslatef( (*this)(point_gz,0), (*this)(point_gz,1), (*this)(point_gz,2) );
								gluSphere (q, disksize/2, 10, 10);
								glPopMatrix();
							}
						}
						fl++;
					}
					fclose(file);
				}else{
					glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, clustercolor);
					if( hasNormals() && showsplats ){
						normal[ 0 ] = normals[ point_number*ndims + 0 ];
						normal[ 1 ] = normals[ point_number*ndims + 1 ];
						normal[ 2 ] = normals[ point_number*ndims + 2 ];
						theta = acos( dot( zaxis, normal ) ) * 180 / PI;
						cross( zaxis, normal, crossv );
						if( drawbackpoint==2 && dot(camera, normal)<0 )
							continue;

						glPushMatrix();
						glTranslatef( (*this)(point_number,0), (*this)(point_number,1), (*this)(point_number,2) );
						glRotatef( theta, crossv[0], crossv[1], crossv[2] );
						gluDisk( q, 0, disksize/2, 20, 1 );
						glPopMatrix();
					}
					// Are we using spheres?
					if( showsplats == false ){
						glPushMatrix();
						glTranslatef( (*this)(point_number,0), (*this)(point_number,1), (*this)(point_number,2) );
						gluSphere (q, disksize/2, 10, 10);
						glPopMatrix();
					}
				}
				
			}
			gz++;
		}
	}
	
};
class Graph{
	//进行修改，只显示图中的点，不显示边
public:
	vector< vector<double> > vertices;
	vector<double> verts_radius; // added by jjcao
	vector<vector<int> > edges;
	Graph(char* filename){
		// open file and check
		FILE* fid = fopen( filename, "r" );
		if( fid == NULL )
			cout << "ERROR: file %s does not exists\n" << filename;

		int ndims, nvertices, nedges;
		fscanf( fid,"%*[#] D:%d NV:%d NE:%d\n", &ndims, &nvertices, &nedges );
		this->vertices.resize( nvertices, vector<double>(ndims,0) );		
		this->edges.resize( nedges, vector<int>(2,0));

		for (int i = 0; i < nvertices; i++)
			fscanf(fid, "%*[v] %lf %lf %lf\n", &vertices[i][0], &vertices[i][1], &vertices[i][2] );

		for (int i = 0; i < nedges; i++){
			fscanf(fid, "%*[e] %d %d\n", &edges[i][0], &edges[i][1]);//changed by jjcao
			//correct for using 1-indexing
			//edges[i][0]--; edges[i][1]--; // changed from ++ to -- by jjcao
		}

		// added by jjcao	
		int flag(0);
		this->verts_radius  = vector<double>(nvertices); 	
		for (int i = 0; i < nvertices; i++){
			flag = fscanf(fid, "%*[vr] %lf\n", &verts_radius[i]);
			if (flag!=1) break;
		}
		if (flag!=1) this->verts_radius.clear();

		fclose(fid);
	}
	// added by jjcao
	void save(const char* filename){
		ofstream ost(filename);
		ost << "# D:3 NV:" << vertices.size() << " NE:" << edges.size() << endl;
		for(vector<vector<double> >::iterator it=vertices.begin(); it!=vertices.end(); ++it){
			ost << "v " << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << endl;
		}
		for(vector<vector<int> >::iterator it=edges.begin(); it!=edges.end(); ++it){
			ost << "e " << (*it)[0]+1 << " " << (*it)[1]+1 << endl;
		}
		for(vector<double>::iterator it=verts_radius.begin(); it!=verts_radius.end(); ++it){
			ost << "vr " << (*it) << endl;
		}
		ost.close();
	}
	// added by jjcao
	void computeVertsRadius(){
		if (pcloud->c2sMaps.empty()) return;
		if (verts_radius.empty())	verts_radius  = vector<double>(vertices.size()); 

		unsigned int j;
		vector<double> p,v;
		double len;
		vector< list<double> > dists(this->vertices.size());
		for (int i = 0; i < pcloud->npoints; ++i){
			p = pcloud->getPoint(i);
			j = pcloud->c2sMaps[i];
			v = this->vertices[j]; 
			len = euclidean_distance(p,v);
			dists[j].push_back(len);
		}
		for (unsigned int i = 0; i < this->vertices.size(); ++i){
			//list<double>::iterator it = min_element(dists[i].begin(), dists[i].end());
			//this->verts_radius[i] = *it;
			this->verts_radius[i] = std::accumulate(dists[i].begin(), dists[i].end(), 0.0);
			this->verts_radius[i] = this->verts_radius[i]/dists[i].size();
		}
	}
	void toSysout(){
		for (unsigned int i = 0; i < vertices.size(); i++)
			cout << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2] << endl;
		for (unsigned int i = 0; i < edges.size(); i++)
			cout << edges[i][0] << " " << edges[i][1] << endl;
	}

//void draw(float spheresize=1, float cylindersize=.01)
	void draw(double cylindersize=.01){
		vector<double> zaxis (3,0); zaxis[2] = 1;
		vector<double> raxis (3,0); // rotation axis
		vector<double> a(3,0); // first point of edge
		vector<double> b(3,0); // second point of edge
		vector<double> DIFF(3,0); // difference
		vector<double> crossv(3,0); // cross product result;

		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_FALSE);

		// For every edge in the graph
		//float lightred[3] = {1, .8, .8};//commented by jjcao
		//float red[3] = {1, 0, 0};//commented by jjcao
		for (unsigned int i = 0; i < edges.size(); i++) {
			int e1Idx = edges[i][0];
			int e2Idx = edges[i][1];
			a = vertices[e1Idx];
			b = vertices[e2Idx];
			diff( b, a, DIFF ); // obtain direction
			normalize( DIFF );  // normalize

			// compute rotation & length data
			double length = euclidean_distance( a, b );
			double theta = acos( dot( zaxis, DIFF ) ) * 180 / PI;
			cross( zaxis, DIFF, crossv );

			// rotate and translate
			glPushMatrix();
			glTranslatef( a[0], a[1], a[2] );
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, edgeColor); // changed by jjcao
			glRotatef( theta, crossv[0], crossv[1], crossv[2] );
			glPopMatrix();

			glPushMatrix();
			glTranslatef( b[0], b[1], b[2] );

			if (show_cylindrical_approximation && !verts_radius.empty()){	// changed by jjcao	
				//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, edgeColor); 
				gluSphere(q, verts_radius[e2Idx], 10, 10);
			}else{
				glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, vertColor); 
				//gluSphere(q, 1.5*cylindersize, 10, 10);
				gluSphere(q, cylindersize/2, 10, 10);
			}
			glPopMatrix();
			glPushMatrix();
			glTranslatef( a[0], a[1], a[2] );
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, vertColor); // changed by jjcao
			if (show_cylindrical_approximation && !verts_radius.empty()){	// changed by jjcao
				//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, edgeColor); 
				gluSphere(q, verts_radius[e1Idx], 10, 10);
			}else{				
				glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, vertColor); 
				//gluSphere(q, 1.5*cylindersize, 10, 10);
				gluSphere(q, cylindersize/2, 10, 10);
			}
			glPopMatrix();
		}
	}	
	// added by jjcao, for picking edges
	void drawEdges(float cylindersize=.01){
		vector<double> zaxis (3,0); zaxis[2] = 1;
		vector<double> raxis (3,0); // rotation axis
		vector<double> a(3,0); // first point of edge
		vector<double> b(3,0); // second point of edge
		vector<double> DIFF(3,0); // difference
		vector<double> crossv(3,0); // cross product result;

		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_FALSE);

		// For every edge in the graph
		for (unsigned int i = 0; i < edges.size(); i++) {
			int e1Idx = edges[i][0];
			int e2Idx = edges[i][1];
			a = vertices[e1Idx];
			b = vertices[e2Idx];
			diff( b, a, DIFF ); // obtain direction
			normalize( DIFF );  // normalize

			// compute rotation & length data
			double length = euclidean_distance( a, b );
			double theta = acos( dot( zaxis, DIFF ) ) * 180 / PI;
			cross( zaxis, DIFF, crossv );

			// rotate and translate
			glPushMatrix();
			glPushName(i+vertices.size());
			glTranslatef( a[0], a[1], a[2] );
			glRotatef( theta, crossv[0], crossv[1], crossv[2] );
			gluCylinder(q, cylindersize, cylindersize, length, 5, 10 );
			glPopName();
			glPopMatrix();
		}
	}
	// added by jjcao
	//float cylindersize=.01
	void drawPickedVertices(double cylindersize){
		if (pick_vid < 0) return;

		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_FALSE);

		vector<double> v = vertices[pick_vid];
		glPushMatrix();
		glTranslatef( v[0], v[1], v[2] );	
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, vertPickColor);
		gluSphere(q, cylindersize/2, 10, 10);
		glPopMatrix();
	}	
	// added by jjcao, for picking vertices
//float cylindersize=.01
	void drawVertices(double cylindersize){
		vector<double> v(3,0); // each vertex

		GLUquadricObj *q = gluNewQuadric();
		gluQuadricNormals (q,GLU_FALSE);

		// For every vertices in the graph
		for (unsigned int i = 0; i < vertices.size(); ++i) {
			v = vertices[i];

			glPushMatrix();
			glPushName(i);
			glTranslatef( v[0], v[1], v[2] );	
			//gluSphere(q, 10.5*cylindersize, 10, 10);
			gluSphere(q, cylindersize/2, 10, 10);
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, vertColor);//添加颜色
			glPopName();
			glPopMatrix();
		}
	}

	// added by jjcao	
	vector<pair<vector<vector<int> >::iterator, int> > belong2edges(int vid){
		vector<pair<vector<vector<int> >::iterator, int> > result;
		pair<vector<vector<int> >::iterator, int> tmp;
		int i(0);
		for (vector<vector<int> >::iterator it = edges.begin(); it != edges.end(); ++it,++i){
			vector<int> edge = *it;
			if (edge[0] == vid){
				tmp.first = it;
				tmp.second = 0;
				result.push_back(tmp);
			}else if (edge[1] == vid){
				tmp.first = it;
				tmp.second = 1;
				result.push_back(tmp);
			}
		}
		return result;
	}
	// added by jjcao
	bool isNeighborVertex(int id1, int id2){
		bool result(false);
		vector<int> t1(2); t1[0]=id1; t1[1]=id2;
		vector<int> t2(2); t2[0]=id2; t2[1]=id1;
		vector<int> e;
		for (vector<vector<int> >::iterator it = edges.begin(); it != edges.end(); ++it){
			vector<int> e = *it;
			if ( length(diff(e,t1)) == 0 || length(diff(e,t2)) == 0){
				result = true;
				break;
			}
		}
		return result;
	}
	// added by jjcao
	void delVert(int vid){
		vector<vector<double> >::iterator vit = vertices.begin();
		vit += vid;		
		vertices.erase(vit);

		if (!verts_radius.empty())
		{
			vector<double>::iterator vrit = verts_radius.begin();
			vrit += vid;
			verts_radius.erase(vrit);
		}

		for (vector<vector<int> >::iterator it=edges.begin(); it!=edges.end(); ++it){
			if( (*it)[0] > vid){
				(*it)[0] -= 1;
			}
			if( (*it)[1] > vid){
				(*it)[1] -= 1;
			}
		}
	}
	// added by jjcao
	int nearestVertex(vector<double>& m, int mid, double snapDist){
		vector<double> v;
		double minlen(snapDist);
		int id(-1);int i(0);
		for (vector<vector<double> >::iterator it = vertices.begin(); it != vertices.end(); ++it,++i){
			if (mid == i) continue;
			v = *it;
			double len = euclidean_distance(v,m);
			if ( len < minlen){
				minlen = len;
				id = i;
			}
		}
		return id;
	}
};

//display
void draw_axis(){
	glScalef( .2, .2, .2 ); //make smaller
	glDisable(GL_LIGHTING);
	// Measurements
	float fAxisRadius = 0.025f;
	float fAxisHeight = 1.0f;
	float fArrowRadius = 0.06f;
	float fArrowHeight = 0.1f;
	// Setup the quadric object
	GLUquadricObj *quadric = gluNewQuadric();
	gluQuadricDrawStyle(quadric, GLU_FILL);
	gluQuadricNormals(quadric, GLU_SMOOTH);
	gluQuadricOrientation(quadric, GLU_OUTSIDE);
	gluQuadricTexture(quadric, false);

	// Draw the blue Z axis first, with arrowed head
	glColor3f(0.0f, 0.0f, 1.0f);
	gluCylinder(quadric, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(quadric, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	gluDisk(quadric, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();

	// Draw the Red X axis 2nd, with arrowed head
	glColor3f(1.0f, 0.0f, 0.0f);
	glPushMatrix();
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	gluCylinder(quadric, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(quadric, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	gluDisk(quadric, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	// Draw the Green Y axis 3rd, with arrowed head
	glColor3f(0.0f, 1.0f, 0.0f);
	glPushMatrix();
	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	gluCylinder(quadric, fAxisRadius, fAxisRadius, fAxisHeight, 10, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, 1.0f);
	gluCylinder(quadric, fArrowRadius, 0.0f, fArrowHeight, 10, 1);
	glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
	gluDisk(quadric, fAxisRadius, fArrowRadius, 10, 1);
	glPopMatrix();
	glPopMatrix();

	// White Sphere at origin
	glColor3f(1.0f, 1.0f, 1.0f);
	gluSphere(quadric, 0.05f, 15, 15);

	// Repritinate lights and colors
	glEnable(GL_LIGHTING);
}
void draw_axis_simple(){
	float oldLW;// added by jjcao
	glGetFloatv(GL_LINE_WIDTH, &oldLW);

	glLineWidth(2);
	glDisable(GL_LIGHTING);
	float al[4] = {1.0f, 1.0f, 1.0f, 1.0f};
	//glLightModelfv(GL_LIGHT_MODEL_AMBIENT, al); // comment by jjcao

	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(1,0,0);
	glEnd();


	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(0,1,0);
	glEnd();

	glColor3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(0,0,1);
	glEnd();

	glLineWidth(oldLW); // added by jjcao
	glEnable(GL_LIGHTING);
}

void displayCallback() {
	// clear background
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	// ANIMATION CODE: Here is best place to change view if you want to spin around
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//glOrtho(-1, 1, -1, 1, -10, 10);
	//gluLookAt( eyepos[0],eyepos[1],eyepos[2],0,0,0,0,1,0 );
	// END OF ANIMATION CODE

	// model drawing
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef( modelview_tx, modelview_ty, 0 );
	glScalef( modelview_zoom, modelview_zoom, modelview_zoom );

	// ANIMATION CODE: Here is best place to change view if you want to spin the object around
	gluLookAt( eyepos[0],eyepos[1],eyepos[2],0,0,0,0,1,0 );
	// END OF ANIMATION CODE

	glMultMatrixd( modelview_rot );

	// this is for the example of rotational symmetry video
	//gluLookAt( eyepos[0],eyepos[1],eyepos[2],0,0,0,0,1,0 );

	//draw_axis_simple(); // added by jjcao
	// Draw every curve of the skeleton
	//定量分析
	if( show_skeleton ){
		if( num_argc==4&&(strcmp(point_cloud_consistent,"detection")!=0) ){
			if( tranparencyEnabled )
				pcloud->draw_transp( markersize, show_backpointing, colorwhat );
			else
				pcloud->draw( markersize, show_backpointing, colorwhat, showsplats );
		}else if (num_argc==4&&(strcmp(point_cloud_consistent,"detection")==0))
		{
			if (1==2)
			{
				pcloud->draw3( choosePoint,markersize, show_backpointing, colorwhat, showsplats);
			}else
			{
				pcloud->draw4( markersize, show_backpointing, colorwhat, showsplats );
			}
		}
	}
	// Draw the point cloud
	if (show_pcloud){
		if( tranparencyEnabled )
			pcloud->draw_transp( markersize, show_backpointing, colorwhat );
		else
			pcloud->draw2( markersize, show_backpointing, colorwhat, showsplats );
	}
	glutSwapBuffers();
}
void startPicking(int cursorX, int cursorY) {
	GLint	viewport[4];
	glSelectBuffer(BUFSIZE,selectBuf);
	glRenderMode(GL_SELECT);
	// This sets  <viewport> to the Size and Location Of The Screen Relative To The Window
	glGetIntegerv(GL_VIEWPORT, viewport);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glGetIntegerv(GL_VIEWPORT,viewport);
	gluPickMatrix(cursorX,viewport[3]-cursorY,2,2,viewport);

	glMatrixMode(GL_MODELVIEW);	// Select The Modelview Matrix

	glInitNames();
}
void processHits (GLint hits, GLuint buffer[])
{
	GLuint names, *ptr, minZ,*ptrNames, numberOfNames;

	//cout << "hits =" << hits << endl;
	ptr = (GLuint *) buffer;
	minZ = 0xffffffff;
	for (int i = 0; i < hits; i++) {	
		names = *ptr;
		ptr++;
		if (*ptr < minZ) {
			numberOfNames = names;
			minZ = *ptr;
			ptrNames = ptr+2;
		}

		ptr += names+2;
	}
	//cout << "The closest hit names are ";
	ptr = ptrNames;
	if ( *ptr >= graph->vertices.size() ){
		pick_eid = *ptr;
		pickPoint.push_back(pick_eid);
		choosePoint=pick_eid;
		cout << "edge picked: " << pick_eid << " verts size: " << graph->vertices.size() << endl;
	}
	else{
		pick_vid = *ptr;
		pickPoint.push_back(pick_vid);
		choosePoint=pick_vid;
		cout << "vertex picked: " << pick_vid << " verts size: " << graph->vertices.size() << endl;
	}

	modelview_sz = minZ;

	if (numberOfNames<1){
		pick_eid = -1;
		pick_vid = -1;
	}
	if (strcmp(point_cloud_consistent,"detection")==0)
	{
		pcloud=new Pcloud(off_filename);
	}
	if (pickPoint.size()!=0)
	{
		if (pickPoint.size()==1&&strcmp(point_cloud_consistent,"detection")!=0)
		{
			double a;
			boost::timer time_count;
			normalOrientation=new NormalOrientation(off_filename,cg_mst,pickPoint[0]);
			a=time_count.elapsed();
			cout<<"cost of the "<<num_count<<" time of orientation: "<<a<<endl;
			//定量分析
			//pcloud=new Pcloud(off_filename,point_cloud_consistent);
			//一致定向+定性分析
			//pcloud=new Pcloud(off_filename);
			if (num_argc==2)
			{
				pcloud=new Pcloud(off_filename);
			} 
			else 
			{
				pcloud=new Pcloud(off_filename,point_cloud_consistent);
			}
		}else if(pickPoint.size()!=1&&strcmp(point_cloud_consistent,"detection")!=0)
		{
			double a;
			boost::timer time_count;
			normalOrientation=new NormalOrientation(off_filename,graph_filename,pickPoint);
			a=time_count.elapsed();
			num_count+=1;
			cout<<"cost of the "<<num_count<<"time of orientation: "<<a<<endl;
			//定量分析
			//pcloud=new Pcloud(off_filename,point_cloud_consistent);
			//一致定向+定性分析
			//pcloud=new Pcloud(off_filename);
			if (num_argc==2)
			{
				pcloud=new Pcloud(off_filename);
			} 
			else 
			{
				pcloud=new Pcloud(off_filename,point_cloud_consistent);
			}
		}
	}
	glutPostRedisplay();
}
// added by jjcao
void stopPicking() {
	int hits;

	// restoring the original projection matrix
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glFlush();

	// returning to normal rendering mode
	hits = glRenderMode(GL_RENDER);

	// if there are hits process them
	if (hits != 0)
		processHits(hits,selectBuf);
}

// added by jjcao
void picking()
{
	// Render The Targets To The Selection Buffer
	if( graph != NULL ){
		graph -> drawVertices(markersize);
		//graph -> drawEdges();
	}
}
void mouseCallback(int button, int state, int x, int y) {
	//cout << "mousestat is " << state <<  " down: " << GLUT_DOWN << endl;
	//cout << x << ", " << y << endl; // added by jjcao

	// change modelview status
	if (state == GLUT_UP){
		// begin of added by jjcao
		if (modelview_status == PICKING && pick_vid>-1){
			vector<double> v = graph->vertices[pick_vid];
			// drag, merge, or separate?
			int mod = glutGetModifiers();			
			if (mod == GLUT_ACTIVE_SHIFT){// merge				
				int nearvid = graph->nearestVertex(v,pick_vid, snapDist);
				if (nearvid > -1 && !graph->isNeighborVertex(pick_vid, nearvid)){
					vector<pair<vector<vector<int> >::iterator, int> > edges = graph->belong2edges(pick_vid);
					// set its edges to point to new vid; change pick_vid; keep the position of the vertex; update verts_flag.
					for (vector<pair<vector<vector<int> >::iterator, int> >::iterator it = edges.begin(); it!=edges.end();++it){
						vector<vector<int> >::iterator eit = it->first;
						int id = it->second;
						(*eit)[id] = nearvid;				
						//cout << (*eit)[id] << endl;
					}
					graph->delVert(pick_vid);
					if (pick_vid<nearvid){
						pick_vid = nearvid-1;
					}else{
						pick_vid = nearvid;
					}
					pick_eid -= 1;
				}
			}else if (mod== GLUT_ACTIVE_CTRL && pick_eid>-1){// separate, add a vertex
				int eid = pick_eid - graph->vertices.size();
				vector<vector<int> >::iterator eit = graph->edges.begin();
				eit += eid;

				if ( (*eit)[0] == pick_vid || (*eit)[1] == pick_vid ){
					// add a vertex, update verts_flag, set it as the end of the picked edge; change pick_vid and pick_eid.
					if (graph->verts_radius.size()==graph->vertices.size()){
						graph->verts_radius.push_back(graph->verts_radius[pick_vid]);
					}
					graph->vertices.push_back(v);					

					if ( (*eit)[0] == pick_vid )
					{
						(*eit)[0] = graph->vertices.size()-1;
					}else{
						(*eit)[1] = graph->vertices.size()-1;
					}
					pick_vid = graph->vertices.size()-1;
					pick_eid += 1;
				}
			}
		}
		// end of added by jjcao

		modelview_status = NORMAL;
	}
	else if (state == GLUT_DOWN) {
		//cout << "button down: " << button << endl;

		if (button == GLUT_RIGHT_BUTTON){
			//cout << "left button" << endl;
			modelview_status = TRANSLATING;
		}
		else if (button == GLUT_LEFT_BUTTON){
			//cout << "right button" << endl;
			modelview_status = ROTATING;
		}
		else{ // added by jjcao
			//cout << "middle button" << endl;
			modelview_status = PICKING;

			startPicking(x,y);
			picking();
			stopPicking();
		}
		// set starting point
		modelview_sx = x;
		modelview_sy = y;
	}
}

void motionCallback(int x, int y) {
	double dx, dy, ang, len, v[3], rot[3];
	double z_axis[3] = { 0, 0, 1 };

	if (modelview_status != NORMAL) {

		// Get movement differentials
		dx = x - modelview_sx;
		dy = y - modelview_sy;

		//cout << "motion with status: " << modelview_status << endl;

		// If difference is significant
		if ( (dx != 0.0) || (dy != 0.0)) {
			if (modelview_status == ROTATING) {
				v[0] = dx;
				v[1] = -dy;
				v[2] = 0.0;

				len = length(v);
				ang = -((len * PI *10 )/180.0);

				normalize(v);
				cross(v, z_axis, rot);
				modelview_sx = x;
				modelview_sy = y;

				glLoadIdentity();
				glRotated(ang, rot[0], rot[1], rot[2]);
				glMultMatrixd(modelview_rot);
				glGetDoublev(GL_MODELVIEW_MATRIX, modelview_rot);
			}
			if (modelview_status == TRANSLATING) {
				modelview_sx = x;
				modelview_sy = y;

				modelview_tx += dx / window_width;
				modelview_ty -= dy / window_height;
			}
			if (modelview_status == PICKING && pick_vid>-1) {
				GLdouble modelMatrix[16];
				GLdouble projMatrix[16];
				int viewport[4];		

				glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);				
				glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);				
				glGetIntegerv(GL_VIEWPORT,viewport);

				//////////////////////////////////////////////////////////////////////////
				// modelview_sx,modelview_sy are the previous mouse position. modelview_sz is the
				//   Z value of the selected object. Together, they form the window
				//   coordinate position of the mouse in 3D window coordinate space.
				//   Vack-transform them into object coordinates.
				vector<double>	sp(3), p(3);
				GLint projected;
				projected = gluUnProject( modelview_sx, viewport[3]-modelview_sy, modelview_sz/(double)0xffffffff,
					modelMatrix,projMatrix,viewport,&sp[0], &sp[1],&sp[2]);
				assert( projected == GL_TRUE );

				// So the same with the current window coordnate mouse position.
				projected = gluUnProject( x, viewport[3]-y, modelview_sz/(double)0xffffffff,
					modelMatrix,projMatrix,viewport,&p[0], &p[1],&p[2]);
				assert( projected == GL_TRUE );

				// (p-sp) is the object coordinate delta motion of the mouse.
				//   Set this in the scene to reposition the selected object.
				vector<double> v = graph->vertices[pick_vid];
				sum(v, p);
				diff(v,sp);
				graph->vertices[pick_vid] = v;

				//////////////////////////////////////////////////////////////////////////
				modelview_sx = x;
				modelview_sy = y;
			}
			glutPostRedisplay();
		}//if ( (dx != 0.0) || (dy != 0.0)) 
	}
}

void reshapeCallback(int x_new, int y_new) {
	// update window dimension (1/1 ratio)
	window_width  = x_new;
	window_height = y_new; //changed by jjcao from x_new to y_new

	// re-set viewport
	glViewport(0, 0, window_width, window_height);

	// refresh
	glutPostRedisplay();
}

double framerate = 100;
void animationcallback( int par ){
	if( !isanimated ){
		glutPostRedisplay();
		return;
	}

	static double alpha = 90; //vieangle in radiants
	alpha += 0.5; //half a degree a frame
	eyepos[0] = cos(alpha*PI/180);
	eyepos[1] = 0;
	eyepos[2] = sin(alpha*PI/180);
	glutPostRedisplay();
	glutTimerFunc(framerate, animationcallback, 0);
}

bool isxrotationdone = false;
double rotangle = 0.5;
int    numrotations = 0;
void animationcallback2( int par ){
	if( !isxrotationdone && isanimated ){
		numrotations++;
		glLoadIdentity();
		glRotated(-rotangle, 1, 0, 0);
		glMultMatrixd(modelview_rot);
		glGetDoublev(GL_MODELVIEW_MATRIX, modelview_rot);
		if( numrotations == 90.0/rotangle )
			isxrotationdone = true;
	}

	// regardlessy keep rotating around model's Z
	glLoadIdentity();
	glMultMatrixd(modelview_rot);
	glRotated(rotangle, 0, 0, 1);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview_rot);

	glutPostRedisplay();
	glutTimerFunc(framerate, animationcallback2, 0);
}

// keyboard
void keyboardCallback(unsigned char key, int x, int y) {
	switch (key) {
	case 'q':
		exit(0);
		printf("viewer quit.\n");
		break;
	case 's':
		showsplats = !showsplats;
		break;
	case 'a':
		isanimated = !isanimated;
		glutTimerFunc(framerate, animationcallback, 0);
		break;
	case '=':
		modelview_zoom *= 1.1;
		break;
	case 'm':
		cout << "modelview" << endl;
		for (int i = 0; i < 16; i++)
			cout << modelview_rot[i] << " ";
		cout << endl;
		break;
	case '-':
		modelview_zoom *= 0.9;
		break;
	case 't':
		tranparencyEnabled = !tranparencyEnabled;
		break;
	case 'c':
		colorwhat = (colorwhat+1)%3;
		break;
	case 'h':
		printf("EXAMPLE SYNTAX: normalorientation infile.off \n");
		printf("EXAMPLE SYNTAX: normalorientation infile.off detection infile.txt\n");
		printf("EXAMPLE SYNTAX: normal infile.off infile_consistent.off consistent\n");
		printf("h: show this help\n");
		printf("= / -: zoom/unzoom the model\n");
		printf("[ / ]: decrease/increase size of splats or spherelets\n");
		printf("b: switch between color specification (available only in spherelets mode)\n");
		printf("s: toggle splats/spheres\n");
		printf("1: show/source point or incorrect normal point\n");
		printf("2: show/point cloud or ground truth\n");// added by jjcao
		printf("middle button down: picking vertex or edge of the skeleton\n");// added by jjcao
		break;
	case 'b':
		show_backpointing = (show_backpointing+1)%3;
		break;
	case '[':
		markersize*=.95;
		cout << "markersize " << markersize << endl;
		break;
	case ']':
		markersize/=.95;
		cout << "markersize " << markersize << endl;
		break;
	case '1':
		show_skeleton = !show_skeleton;
		break;
	case '2': // added by jjcao
		show_pcloud = !show_pcloud;
		break;
	}
	glutPostRedisplay();
}

// added by jjvao
void specialKeyboardCallback(int key, int x, int y) {
	if (pick_vid<0) return;
	switch (key) {
	case GLUT_KEY_UP:		
		graph->verts_radius[pick_vid] /= .95;
		cout << "radius " << graph->verts_radius[pick_vid] << endl;
		break;
	case GLUT_KEY_DOWN:
		graph->verts_radius[pick_vid] *= .95;
		cout << "radius " << graph->verts_radius[pick_vid] << endl;
		break;
	}
	glutPostRedisplay();
}

int main(int argc, char* argv[])
{
	boost::timer time_count;
	num_argc=argc;
	printf("Welcome to pointlab.\n");
	if (argc<2){
		printf("you must provide an input file\n");
		return 0;
	} else if (argc>4) {
		return 0;
	}
	char* flag_detection="detection";
	char* vf=argv[3];//可视点
	off_filename=argv[1];
	string a(off_filename);
	string filename=a.substr(0,a.size()-3);
	string filename3=a.substr(0,a.size()-4);
	string file_cg=filename.append(string("cg"));
	string file_mst=filename3.append(string("_mst.cg"));
	graph_filename=(char*)file_cg.c_str();
	cg_mst=(char*)file_mst.c_str();
	if(argc==4)
	{
		point_cloud_consistent=argv[2];
		if (strcmp(point_cloud_consistent,flag_detection)!=0)
		{
			pcloud=new Pcloud(off_filename,point_cloud_consistent);
		}else
		{
			pcloud=new Pcloud(off_filename);
		}
	}else if (argc==2)
	{
		point_cloud_consistent=argv[1];
		pcloud=new Pcloud(off_filename);
	}
	//---------------------------- POINTLAB CODE ----------------------------//
	if (argc==2||argc==4)
	{
		if (argc==4&&strcmp(argv[2],flag_detection)==0)
		{
			//add detection of inconsistent
			cout<<"detection"<<endl;
			double time_a=time_count.elapsed();
			time_count.restart();
			normalOrientation=new NormalOrientation(off_filename,graph_filename);
			double time_b=time_count.elapsed();
			cout<<"the time of construction of the graph: "<<time_b<<" s"<<endl;
			normalOrientation=new NormalOrientation(off_filename,graph_filename,cg_mst);
			graph = new Graph( cg_mst );
			double time_c=time_count.elapsed();
			time_count.restart();
			normalOrientation=new NormalOrientation(off_filename,graph_filename,vf,1,flip,unflip);
			double time_d=time_count.elapsed();
			cout<<"the time of automatic multi-source normal propagation: "<<time_d<<" s"<<endl;
			pcloud=new Pcloud(off_filename);
		} 
		else
		{
			double time_a=time_count.elapsed();
			time_count.restart();
			normalOrientation=new NormalOrientation(off_filename,graph_filename);
			double time_b=time_count.elapsed();
			cout<<"the time of construction of the graph: "<<time_b<<" s"<<endl;
			normalOrientation=new NormalOrientation(off_filename,graph_filename,cg_mst);
			cout << "cg mode" << endl;
			graph = new Graph( cg_mst );
			if (graph->verts_radius.empty())//added by jjcao
				graph->computeVertsRadius();
		}
	}
	//---------------------------- POINTLAB CODE ----------------------------//

	// initialize glut stuff and create window
	glutInit( &argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL );
	// viewport is set automatically
	glutInitWindowSize(window_width, window_height);
	glutCreateWindow("pointlab");

	// register callbacks
	glutDisplayFunc(displayCallback);
	glutKeyboardFunc(keyboardCallback);
	glutSpecialFunc(specialKeyboardCallback);
	glutMotionFunc(motionCallback);
	glutMouseFunc(mouseCallback);
	glutReshapeFunc(reshapeCallback);

	// default drawing settings
	glClearColor(clear_color[0], clear_color[1], clear_color[2], 0.0);
	glClearStencil( 0x0 );

	glLineWidth(1.0);
	glEnable(GL_DEPTH_TEST); // delete covered objects

	// FIXED lights positions/setup
	glEnable(GL_LIGHTING);
	glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, true );
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightf(GL_LIGHT0, GL_POSITION, light0_position[0]);


	// setup blending options
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Corrects changes in shading during zoom
	glEnable(GL_NORMALIZE);

	// set the projection mode
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1, 1, -1, 1, -10, 10);
	// glFrustum(-1, 1, -1, 1, -10, 10);
	gluLookAt( eyepos[0],eyepos[1],eyepos[2],0,0,0,0,1,0 );

	// Shape material
	// glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red);

	// set animation callback and start animation
	glutTimerFunc(framerate, animationcallback, 0);
	glutMainLoop();
	printf("pointlab terminated correctly.\n");
	return 0;
}

