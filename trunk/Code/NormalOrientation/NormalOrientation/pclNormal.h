#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <wchar.h>
#include <cmath>
#include <pcl/impl/point_types.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/console/print.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/cstdint.hpp>
#include <shlwapi.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/config.hpp>
#include <boost/cstdint.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph//breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>
#include <stdlib.h>
#include <strstream>
#include <algorithm>
#include <utility>
#include <vector>
#include <queue>
#include <boost/graph/visitors.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/graph/graph_utility.hpp>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <Eigen/Core>
#include "myutil.h"
#include <boost/numeric/ublas/vector_expression.hpp>
using namespace std;
template < typename TimeMap > class bfs_time_visitor:public boost::default_bfs_visitor {
	typedef typename boost::property_traits < TimeMap >::value_type T;
public:
	bfs_time_visitor(TimeMap tmap, T & t):m_timemap(tmap), m_time(t) { }
	template < typename Vertex, typename Graph >
	void discover_vertex(Vertex u, const Graph & g) const
	{
		boost::put(m_timemap, u, m_time++);
	}
	TimeMap m_timemap;
	T & m_time;
};
template <class ParentDecorator>
struct print_parent {
	print_parent(const ParentDecorator& p_) : p(p_) { }
	template <class Vertex>
	void operator()(const Vertex& v) const {
		std::cout << "parent[" << v << "] = " <<  p[v]  << std::endl;
	}
	ParentDecorator p;
};
template <class NewGraph, class Tag>
struct graph_copier 
	: public boost::base_visitor<graph_copier<NewGraph, Tag> >
{
	typedef Tag event_filter;

	graph_copier(NewGraph& graph) : new_g(graph) { }

	template <class Edge, class Graph>
	void operator()(Edge e, Graph& g) {
		boost::add_edge(boost::source(e, g), boost::target(e, g), new_g);
	}
private:
	NewGraph& new_g;
};

template <class NewGraph, class Tag>
inline graph_copier<NewGraph, Tag>
copy_graph(NewGraph& g, Tag) {
	return graph_copier<NewGraph, Tag>(g);
}
struct edge_w
{
	pair<int,int>e;
	float w;
	bool operator<(const edge_w&x)const{return w>x.w;}
};
float ool=1.0/sqrt(3.0);
float ol=1.0/sqrt(2.0);
float view_point[18]={0.0,0.0,-1.0,1.0,0.0,0.0,0.0,0.0,1.0,-1.0,0.0,0.0,0.0,1.0,0.0,0.0,-1.0,0.0};//0.0,-1.0,0.0,1.0,0.0,0.0,-1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,-1.0,1.0,0.0,0.0,-1.0,0.0,0.0,0.0,1.0,0.0,0.0,-1.0,0.0,1.0,1.0,1.0,1.0,-1.0,1.0,1.0,1.0,-1.0,1.0,-1.0,-1.0,-1.0,1.0,1.0,-1.0,-1.0,1.0,-1.0,1.0,-1.0,-1.0,-1.0,-1.0
float view_normal[18]={0.0,0.0,1.0,-1.0,0.0,0.0,0.0,0.0,-1.0,1.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0};//0.0,1.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,-1.0,0.0,0.0,1.0,-1.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,-ool,-ool,-ool,-ool,ool,-ool,-ool,-ool,ool,-ool,ool,ool,ool,-ool,-ool,ool,ool,-ool,ool,-ool,ool,ool,ool,ool
//顶点8
//1.0,1.0,1.0,1.0,-1.0,1.0,1.0,1.0,-1.0,1.0,-1.0,-1.0,-1.0,1.0,1.0,-1.0,-1.0,1.0,-1.0,1.0,-1.0,-1.0,-1.0,-1.0
//-ool,-ool,-ool,-ool,ool,-ool,-ool,-ool,ool,-ool,ool,ool,ool,-ool,-ool,ool,ool,-ool,ool,-ool,ool,ool,ool,ool
//面6
//0.0,0.0,-1.0,1.0,0.0,0.0,0.0,0.0,1.0,-1.0,0.0,0.0,0.0,1.0,0.0,0.0,-1.0,0.0
//0.0,0.0,1.0,-1.0,0.0,0.0,0.0,0.0,-1.0,1.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0
//棱12
//1.0,1.0,0.0,-1.0,1.0,0.0,1.0,-1.0,0.0,-1.0,-1.0,0.0,1.0,0.0,1.0,1.0,0.0,-1.0,-1.0,0.0,1.0,-1.0,0.0,-1.0,0.0,1.0,1.0,0.0,-1.0,1.0,0.0,1.0,-1.0,0.0,-1.0,-1.0
//-ol,-ol,0.0,ol,ol,0.0,-ol,ol,0.0,ol,ol,0.0,-ol,0.0,-ol,-ol,0.0,ol,ol,0.0,-ol,ol,0.0,ol,0.0,-ol,-ol,0.0,ol,-ol,0.0,-ol,ol,0.0,ol,ol
class NormalOrientation
{
	typedef pair<int,int>E;
	public:
		//返回聚类情况
		vector<vector<vector<E>>>nff;
		vector<vector<E>>minff;
		vector<vector<vector<E>>>& getNff(){return nff;}
		vector<vector<E>>& getMinff(){return minff;}
		vector<double> vote;//记录点的投票数
		vector<double> getVote(){return vote;}
		//1.初始每个点的投票数为零
		//2.法向正确的（内积小于零）自加一
		//3.法向错误的（内积大于零）自减一
		//4.投票数非零的作为种子点
		//5.投票数小于零的法向需要翻转
		//6.投票数大于零的法向不需要翻转
		//detection inconsistent point cloud
		NormalOrientation(char* inputFileName,char* cg_filename,char* vf,int vote1,vector<int>&a,vector<int>&b)
		{
			//利用可视点的方法
			pcl::PointCloud<pcl::PointNormal>::Ptr en(new pcl::PointCloud<pcl::PointNormal>);
			vector<int> record_seedpoint;
			en=readNoff(inputFileName);
			en=normalizedPoint(en);
			vector<vector<int>>kNN_search;
			vector<vector<float>> adopdistance=kNSearch_distance(6,en,kNN_search);
			vote.resize(en->size());
			for (vector<double>::iterator p=vote.begin();p!=vote.end();p++)
			{
				*p=0.0;
			}
			//读取视点和可视点
			FILE *file=fopen(vf,"r");
			int num(0);//记录视点个数
			fscanf(file,"%d\n",&num);
			for (int i=0;i<num;i++)
			{
				pcl::PointNormal vote_point;//vote point
				pcl::PointCloud<pcl::PointNormal>::Ptr visiblePoints(new pcl::PointCloud<pcl::PointNormal>);
				int visible_num(0);//the number of visible points 
				fscanf(file,"%f %f %f\n",&vote_point.x,&vote_point.y,&vote_point.z);
				vote_point.normal_x=-vote_point.x;
				vote_point.normal_y=-vote_point.y;
				vote_point.normal_z=-vote_point.z;
				fscanf(file,"%d\n",&visible_num);
				vector<int>num_pn(visible_num);//记录可视点标号
				for (int j=0;j<visible_num;j++)
				{
					int pn(0);
					fscanf(file,"%d ",&pn);num_pn[j]=pn-1;
					visiblePoints->push_back(en->points[pn-1]);
				}
				visiblePoints->height=1;visiblePoints->width=visiblePoints->size();
				fscanf(file,"\n");
				boost::numeric::ublas::vector<double> bp(3);
				bp(0)=vote_point.normal_x;bp(1)=vote_point.normal_y;bp(2)=vote_point.normal_z;
				bp=bp/norm_2(bp);
				//voting
				for (int l=0;l<visible_num;l++)
				{
					boost::numeric::ublas::vector<double> ap(3);
					ap(0)=visiblePoints->points[l].normal_x;
					ap(1)=visiblePoints->points[l].normal_y;
					ap(2)=visiblePoints->points[l].normal_z;
						double app=(double)abs(acos(abs(inner_prod(ap,bp)))/(PI/6.0));///(PI/6.0)
						if (inner_prod(ap,bp)>0.0)
						{
							vote[num_pn[l]]=(double)(vote[num_pn[l]]-(double)exp(-app*app));//abs(inner_prod(ap,bp))
						}else if(inner_prod(ap,bp)<0.0)
						{
							vote[num_pn[l]]=(double)(vote[num_pn[l]]+(double)exp(-app*app));
						}
				}
			}
			fclose(file);
			double max=*max_element(vote.begin(),vote.end());
			double min=*min_element(vote.begin(),vote.end());
			for (int vote_i=0;vote_i<vote.size();vote_i++)
			{
				if (vote[vote_i]<=(min+0.01)&&vote[vote_i]>=min)//(min+1.5)
				{
					en->points[vote_i].normal_x=-en->points[vote_i].normal_x;
					en->points[vote_i].normal_y=-en->points[vote_i].normal_y;
					en->points[vote_i].normal_z=-en->points[vote_i].normal_z;
					record_seedpoint.push_back(vote_i);
					a.push_back(vote_i);
				}else if (vote[vote_i]>=(max-0.01)&&vote[vote_i]<=max)//=(max-1.5)
				{
					record_seedpoint.push_back(vote_i);
					b.push_back(vote_i);
				}
			}
			//输出种子点的个数
			std::cout<<"The number of source points: "<<record_seedpoint.size()<<std::endl;
			//输出投票结果
			FILE* file_vote=fopen("I:/record_vote.txt","w");
			fprintf(file_vote,"%d\n",en->size());
			for(int vi=0;vi<vote.size();vi++)
			{
				fprintf(file_vote,"%f\n",vote[vi]);
			}
			fclose(file_vote);
			multinormalPropagation(*en,cg_filename,record_seedpoint,1);
			writeNOffFile(en,inputFileName);
		}
		NormalOrientation(char* inputFileName,const char* graphFileName)
		{
			//给定法向。不用计算法向。生成cg图文件。
			vector<E> edge;
			pcl::PointCloud<pcl::PointXYZ> points;
			pcl::PointCloud<pcl::PointXYZ>::Ptr normal_off(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointNormal>::Ptr en(new pcl::PointCloud<pcl::PointNormal>);
			readOFFFile(inputFileName,edge);
			//generate gc file--product graph
			//直接读取法向。
			en=readNoff(inputFileName);
			FILE* file=fopen(graphFileName,"w");
			fprintf(file,"# D:3 NV:%d NE:%d\n",en->size(),edge.size());
			for (size_t i=0;i<en->size();i++)
			{
				fprintf(file,"v %f %f %f\n",en->points[i].x,en->points[i].y,en->points[i].z);
			}
			for (vector<E>::iterator it=edge.begin();it!=edge.end();it++)
			{
				fprintf(file,"e %d %d\n",it->first,it->second);
			}
			fclose(file);
		}
		NormalOrientation(char*noff_filename,char* cg_filename,char* cg_mst)
		{
			typedef pair<int,int>E;
			vector<E>triedge;
			vector<E>edge;
			vector<float>weight;
			pcl::PointCloud<pcl::PointNormal>::Ptr npcd(new pcl::PointCloud<pcl::PointNormal>);
			npcd=readNoff(noff_filename);
			triedge=load_cg(cg_filename);
			buildMST(npcd,weight,edge,triedge);
			save_cg(cg_mst,npcd,edge);
		}
		NormalOrientation(char* offnormal,char* cg_filename,int pick)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr pp(new pcl::PointCloud<pcl::PointNormal>);
			pp=readNoff(offnormal);
			normalPropagation(*pp,cg_filename,pick);
			writeNOffFile(pp,offnormal);
		}
		NormalOrientation(char* offnormal,char* cg_filename,vector<int> pick)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr pp(new pcl::PointCloud<pcl::PointNormal>);
			pp=readNoff(offnormal);
			multinormalPropagation(*pp,cg_filename,pick);
			writeNOffFile(pp,offnormal);
		}
		vector<pair<int,int>> load_cg(char* cg_filename)
		{
			typedef pair<int,int>E;
			vector<E>edge;
			FILE* file=fopen(cg_filename,"r");
			int ndims, nvertices, nedges;
			fscanf( file,"%*[#] D:%d NV:%d NE:%d\n", &ndims, &nvertices, &nedges );
			float a,b,c;
			for (int i = 0; i < nvertices; i++)
			{	
				fscanf(file, "%*[v] %lf %lf %lf\n", &a, &b, &c );

			}
			for (int i = 0; i < nedges; i++)
			{
				E e;
				fscanf(file, "%*[e] %d %d\n", &e.first, &e.second);
				edge.push_back(e);
			}
			fclose(file);
			return edge;
		}

		void save_cg(char* cg_filename,pcl::PointCloud<pcl::PointNormal>::Ptr pcd_normal,vector<pair<int,int>>edge)
		{
			FILE * filename=fopen(cg_filename,"w");
			fprintf(filename,"# D:3 NV:%d NE:%d\n",pcd_normal->size(),edge.size());
			for (size_t i=0;i<pcd_normal->size();i++)
			{
				fprintf(filename,"v %f %f %f\n",pcd_normal->points[i].x,pcd_normal->points[i].y,pcd_normal->points[i].z);
			}
			for (vector<E>::iterator it=edge.begin();it!=edge.end();it++)
			{
				fprintf(filename,"e %d %d\n",it->first,it->second);
			}
			fclose(filename);
			
		}		
		pcl::PointCloud<pcl::PointNormal>::Ptr normalizedPoint(pcl::PointCloud<pcl::PointNormal>::Ptr point)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr en(new pcl::PointCloud<pcl::PointNormal>);
			en->height=point->height;
			en->width=point->width;
			en->resize(en->height*en->width);
			double tmp1=0;
			double tmpx=0;
			double tmpy=0;
			double tmpz=0;
			//find centerpoint
			for (int i=0;i<en->width;i++)
			{
				tmpx+=point->points[i].x;
				tmpy+=point->points[i].y;
				tmpz+=point->points[i].z;
			}
			for (int j=0;j<en->width;j++)
			{
				en->points[j].x=point->points[j].x-(tmpx/en->width);
				en->points[j].y=point->points[j].y-(tmpy/en->width);
				en->points[j].z=point->points[j].z-(tmpz/en->width);
				en->points[j].normal_x=point->points[j].normal_x;
				en->points[j].normal_y=point->points[j].normal_y;
				en->points[j].normal_z=point->points[j].normal_z;
			}
			//normalized
			for (int i1=0;i1<en->width;i1++)
			{
				double tmp2=sqrt(pow(en->points[i1].x,2)+pow(en->points[i1].y,2)+pow(en->points[i1].z,2));
				if (tmp2>tmp1)
				{
					tmp1=tmp2;
				}
			}
			for (int j1=0;j1<en->width;j1++)
			{
				en->points[j1].x=en->points[j1].x/tmp1;
				en->points[j1].y=en->points[j1].y/tmp1;
				en->points[j1].z=en->points[j1].z/tmp1;
			}
			return en;
		}

		//read point from NOFF
		pcl::PointCloud<pcl::PointNormal>::Ptr readNoff(char* noff_filename)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr en(new pcl::PointCloud<pcl::PointNormal>);
			string  s;
			int pointsize=0;
			int flag=0;
			FILE * file_noff=fopen(noff_filename,"r");
			fscanf(file_noff,"%s\n",&s);
			fscanf(file_noff,"%d %d\n",&pointsize,&flag);
			en->height=1;
			en->width=pointsize;
			en->resize(en->height*en->width);
			for (int i=0;i<pointsize;i++)
			{
				fscanf(file_noff,
					   "%f %f %f %f %f %f\n",
					   &en->points[i].x,&en->points[i].y,&en->points[i].z,
					   &en->points[i].normal_x,&en->points[i].normal_y,&en->points[i].normal_z);
			}
			fclose(file_noff);
			return en;
		}
		
		//读取NOFF文件。计算k=6的kdtree为连通图。得到边。
		//void readOFFFile(char* inputFileName,string pcdFileName,vector<pair<int,int>>& edge)
		void readOFFFile(char* inputFileName,vector<pair<int,int>>& edge)
		{
			typedef pair<int,int> E;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ> cloud;
			vector<E> edgeflag;
			FILE * fileReader=fopen(inputFileName,"r");
			if (fileReader==NULL)
			{
				cout << "ERROR: file %s does not exists\n" << inputFileName;
			}
			string flag;
			fscanf(fileReader,"%s\n",&flag);
			int points,viewpoint;
			fscanf(fileReader,"%d %d\n",&points,&viewpoint);
			cloud.width=points;
			cloud.height=1;
			cloud.is_dense=false;
			cloud.resize(cloud.width*cloud.height);
			cloud1->height=1;
			cloud1->width=points;
			cloud1->resize(cloud1->height*cloud1->width);
			for (int i=0;i<points;i++)
			{
				float x,y,z;
				float x1,y1,z1;
				fscanf(fileReader,"%f %f %f %f %f %f\n",&x,&y,&z,&x1,&y1,&z1);
				cloud.points[i].x=x;
				cloud.points[i].y=y;
				cloud.points[i].z=z;
			}
			*cloud1=cloud;
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
			kdtree.setInputCloud(cloud1);
			for (int i=0;i<cloud.points.size();i++)
			{
				std::vector<int> pointIdxNKNSearch(6);
				std::vector<float> pointNKNSquaredDistance(6);
				kdtree.nearestKSearch(cloud.points[i],6,pointIdxNKNSearch,pointNKNSquaredDistance);
				for (int ii=1;ii<pointIdxNKNSearch.size();ii++)
				{
						edge.push_back(E(i,pointIdxNKNSearch[ii]));
				}
		}
	}
		//writeOffFile
		void writeNOffFile(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud,char* fileName)
		{
			FILE *file=fopen(fileName,"w");
			char s[]="NOFF";
			fprintf(file,"%s\n",s);
			int flag=0;
			fprintf(file,"%d %d\n",(int)cloud->size(),flag);
			for (size_t i=0;i<cloud->size();i++)
			{
				fprintf(file,"%f %f %f %f %f %f\n",
					cloud->points[i].x,
					cloud->points[i].y,
					cloud->points[i].z,
					cloud->points[i].normal_x,
					cloud->points[i].normal_y,
					cloud->points[i].normal_z
					);
			}
			fclose(file);
		}

		vector<vector<float>> kNSearch_distance(int k,const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,vector<vector<int>>&b)
		{
			vector<vector<float>> a;
			pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
			kdtree.setInputCloud(cloud);
			for (int i=0;i<cloud->size();i++)
			{
				std::vector<int> pointIdxNKNSearch(k);
				std::vector<float> pointNKNSquaredDistance(k);
				if (kdtree.nearestKSearch(cloud->points[i],k,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
				{
					a.push_back(pointNKNSquaredDistance);
					b.push_back(pointIdxNKNSearch);
				}
			}
			return a;
		}
		//construction of EMST of undirection of graph
		void buildMST(pcl::PointCloud<pcl::PointNormal>::Ptr p,vector<float>& mweight,vector<pair<int,int>> &medge,vector<pair<int,int>>triangleEdge)
		{
			using namespace boost;
			typedef adjacency_list<vecS,vecS,undirectedS,no_property,property<edge_weight_t,float>>Graph;
			typedef graph_traits<Graph>::edge_descriptor Edge;
			typedef graph_traits<Graph>::vertex_descriptor Vertex;
			typedef pair<int,int>E;
			const int num_nodes =p->size();
			vector<E> edge_array;
			vector<float> weights;
			Graph g(num_nodes);
			size_t num_edges=triangleEdge.size();
			property_map<Graph,edge_weight_t>::type weightmap=get(edge_weight,g);
			for(size_t i=0;i<num_edges;i++)
			{
				//compute Hoppe weight
				float weightEdge=1-abs(p->points[triangleEdge[i].first].normal_x*p->points[triangleEdge[i].second].normal_x
									   +p->points[triangleEdge[i].first].normal_y*p->points[triangleEdge[i].second].normal_y
									   +p->points[triangleEdge[i].first].normal_z*p->points[triangleEdge[i].second].normal_z);
				Edge e;bool inserted;
				boost::tie(e,inserted)=add_edge(triangleEdge[i].first,triangleEdge[i].second,g);
				weightmap[e]=weightEdge;
			}
			vector<Edge> spanning_tree;
			kruskal_minimum_spanning_tree(g,back_inserter(spanning_tree));
			for (vector<Edge>::iterator ei=spanning_tree.begin();ei!=spanning_tree.end();ei++)
			{
				medge.push_back(E(source(*ei,g),target(*ei,g)));
				mweight.push_back(weightmap[*ei]);
			}
		}
		//step1 propagation of normal by first seed point
		//获取BFS传播结果和每个点的父亲结点
		void normalPropagationPrepared(pcl::PointCloud<pcl::PointNormal>&point_normal,char* cg_filename,vector<int>&disorder,vector<int>&parent,int pick)
		{
			using namespace boost;
			typedef adjacency_list <vecS,vecS,undirectedS, 
									boost::property<boost::vertex_color_t,
													boost::default_color_type,
													boost::property<boost::vertex_degree_t, int,
																	boost::property<boost::vertex_in_degree_t, int,
																					boost::property<boost::vertex_out_degree_t, int> 
																					> 
																	> 
													>
									> graph_t;
			typedef std::pair <int,int>E;
			vector<E> edge=load_cg(cg_filename);
			const int n_edges = edge.size();
			int n_num=point_normal.size();
			graph_t gg(n_num);
			graph_t G_copy(n_num);
			for (vector<E>::iterator e=edge.begin();e!=edge.end();e++)
			{
				add_edge(e->first,e->second,gg);
			}
			// Typedefs
			typedef graph_traits < graph_t >::vertex_descriptor Vertex;
			typedef graph_traits < graph_t >::vertices_size_type Size;
			typedef Size* Iiter;
			// Array to store predecessor (parent) of each vertex. This will be
			// used as a Decorator (actually, its iterator will be).
			std::vector<Vertex> p(boost::num_vertices(gg));
			// VC++ version of std::vector has no ::pointer, so
			// I use ::value_type* instead.
				typedef std::vector<Vertex>::value_type* Piter;
			// a vector to hold the discover time property for each vertex
			vector < Size > dtime(num_vertices(gg));
			Size time = 0;
			bfs_time_visitor < Size * >vis(&dtime[0], time);
			/*breadth_first_search(gg, vertex(edge[0].first, gg), visitor(vis));*/
			breadth_first_search(gg, vertex(pick, gg), visitor(vis));
			vector<graph_traits<graph_t>::vertices_size_type> discover_order(n_num);
			integer_range < int >range(0, n_num);
			copy(range.begin(), range.end(), discover_order.begin());
			sort(discover_order.begin(), discover_order.end(),
				indirect_cmp < Iiter, std::less < Size > >(&dtime[0]));
			//std::cout << "order of discovery: ";
			for (int i = 0; i <n_num; ++i)
			{
				
				disorder.push_back(discover_order[i]);
				//std::cout << discover_order[i]<<" ";
			}
			std::cout << std::endl;
			// The source vertex
			Vertex s = vertex(pick, gg);
			p[s] = s;
			boost::breadth_first_search
				(	gg, s, 
					boost::visitor(
									boost::make_bfs_visitor
									(
										std::make_pair(
														boost::record_predecessors(&p[0], boost::on_tree_edge()),
														copy_graph(G_copy, boost::on_examine_edge())
													   )
									)
								  )
				);
			for (vector<Vertex>::iterator ei=p.begin();ei!=p.end();ei++)
			{
				parent.push_back(*ei);
			}
		}
		void normalPropagation(pcl::PointCloud<pcl::PointNormal>& plk,char* cg_filename,int pick)
		{
			vector<int> parent,disorder;
			normalPropagationPrepared(plk,cg_filename,disorder,parent,pick);
			for (int i=0;i!=disorder.size();i++)
			{
				int k=parent[disorder[i]];
				float result=plk.points[k].normal_x*plk.points[disorder[i]].normal_x
							 +plk.points[k].normal_y*plk.points[disorder[i]].normal_y
							 +plk.points[k].normal_z*plk.points[disorder[i]].normal_z;
				if (result<0)
				{
					plk.points[disorder[i]].normal_x=-plk.points[disorder[i]].normal_x;
					plk.points[disorder[i]].normal_y=-plk.points[disorder[i]].normal_y;
					plk.points[disorder[i]].normal_z=-plk.points[disorder[i]].normal_z;
				}
			}
		}
		void multinormalPropagation(pcl::PointCloud<pcl::PointNormal>& plk,char* cg_filename,vector<int> pick,int vote_flag=0)
		{
			//先建图
			using namespace boost;
			typedef adjacency_list<vecS,vecS,undirectedS,no_property,property<edge_weight_t,float>>Graph;
			typedef graph_traits<Graph>::edge_descriptor Edge;
			typedef graph_traits<Graph>::vertex_descriptor Vertex;
			typedef std::pair <int,int>E;
			typedef std::pair<Edge,bool>ED;
			pcl::PointCloud<pcl::PointNormal>::Ptr plk1(new pcl::PointCloud<pcl::PointNormal>);
			*plk1=plk;
			vector<E> edge1=load_cg(cg_filename);
			const int n_edges = edge1.size();
			int n_num=plk.width;
			Graph g(n_num);
			property_map<Graph,edge_weight_t>::type weightmap=get(edge_weight,g);
			for (int i1=0;i1<n_edges;i1++)
			{
				float w=1-abs(plk.points[edge1[i1].first].normal_x*plk.points[edge1[i1].second].normal_x
							  +plk.points[edge1[i1].first].normal_y*plk.points[edge1[i1].second].normal_y
							  +plk.points[edge1[i1].first].normal_z*plk.points[edge1[i1].second].normal_z);
				Edge e;bool inserted;
				boost::tie(e,inserted)=add_edge(edge1[i1].first,edge1[i1].second,g);
				weightmap[e]=w;
			}
			//多元区域各向异性增长
			vector<int> bflag(n_num,0);//记录是否被访问
			int num_seedpoint=pick.size();//记录种子点的个数
			//vector<int> flag_count(num_seedpoint);
			for (int i2=0;i2<num_seedpoint;i2++)
			{
				bflag[pick[i2]]=1;
				//flag_count.push_back(pick[i2]);
			}
			if (vote_flag==0)
			{
				if (num_seedpoint!=1)
				{
					plk.points[pick[num_seedpoint-1]].normal_x=-plk.points[pick[num_seedpoint-1]].normal_x;
					plk.points[pick[num_seedpoint-1]].normal_y=-plk.points[pick[num_seedpoint-1]].normal_y;
					plk.points[pick[num_seedpoint-1]].normal_z=-plk.points[pick[num_seedpoint-1]].normal_z;
				}
			}
			int nMinIndexA;
			int nMinIndexB;
			int j=0;//记录总遍历次数
			int sum=n_num-num_seedpoint;
			vector<E>flag_edge;
			priority_queue<edge_w>q;
			//优先队列初始化
			for (int i4=0;i4<bflag.size();i4++)
			{
				if (bflag[i4]==0)
				{
					continue;
				}
				Vertex v1=i4;
				for (int j2=i4*5;j2<i4*5+5;j2++)
				{
					edge_w el;
					Vertex v2=edge1[j2].second;
					ED e6=edge(v1,v2,g);
					el.e=edge1[j2];
					el.w=weightmap[e6.first];
					q.push(el);
				}
			}
			//MMST
			while(!q.empty())
			{
				edge_w ef=q.top();
				Vertex v11=ef.e.first;
				Vertex v22=ef.e.second;
				ED e7=edge(v11,v22,g);
				if (bflag[ef.e.second]==0&&e7.second==true)
				{
					bflag[ef.e.second]=1;
					flag_edge.push_back(ef.e);
					for (int j3=ef.e.second*5;j3<ef.e.second*5+5;j3++)
					{
						if (bflag[edge1[j3].second]==0)
						{
							edge_w ef1;Vertex v33=edge1[j3].second;
							ED e8=edge(v22,v33,g);
							ef1.e=edge1[j3];
							ef1.w=weightmap[e8.first];
							q.push(ef1);
						}
					}
				}else
				{
					q.pop();
				} 
			}
			for (int flag_i=0;flag_i<flag_edge.size();flag_i++)
			{
				float result=plk.points[flag_edge[flag_i].first].normal_x*plk.points[flag_edge[flag_i].second].normal_x
					+plk.points[flag_edge[flag_i].first].normal_y*plk.points[flag_edge[flag_i].second].normal_y
					+plk.points[flag_edge[flag_i].first].normal_z*plk.points[flag_edge[flag_i].second].normal_z;
				if (result<0)
				{
					plk.points[flag_edge[flag_i].second].normal_x=-plk.points[flag_edge[flag_i].second].normal_x;
					plk.points[flag_edge[flag_i].second].normal_y=-plk.points[flag_edge[flag_i].second].normal_y;
					plk.points[flag_edge[flag_i].second].normal_z=-plk.points[flag_edge[flag_i].second].normal_z;
				}
			}

		}
};