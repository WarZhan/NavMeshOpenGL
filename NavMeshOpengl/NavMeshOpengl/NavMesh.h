#pragma once
#ifndef _NAVMESH_H_
#define _NAVMESH_H_

#include "Recast\Recast.h"
#include "Recast\RecastAlloc.h"
#include "Recast\RecastAssert.h"


struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};


//读入轮廓
bool LoadConSet(fstream *f, rcContourSet *cset, int &sum);

// 获取多边形的顶点的数量
// static int countPolyVerts(const unsigned short* p, const int nvp);

static void pushBack(int v, int* arr, int& an);

static void pushFront(int v, int* arr, int& an);

//获取下一个索引
inline int next(int i, int n);
//获取前一个索引
inline int prev(int i, int n);
//叉乘
inline int area2(const int* a, const int* b, const int* c);


static bool vequal(const int* a, const int* b);

// 获取多边形的顶点的数量
int countPolyVerts(const unsigned short* p, const int nvp);


//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
// 异或
inline bool xorb(bool x, bool y);

//叉乘
inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c);

// Returns T if (a,b,c) are collinear and point c lies 
// on the closed segment ab.
//a b c 3点共线 且 c 在 ab 中间
static bool between(const int* a, const int* b, const int* c);


static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb,
							 const int nvp);


static void mergePolys(unsigned short* pa, unsigned short* pb, int ea, int eb,
					   unsigned short* tmp, const int nvp);


//计算顶点哈希
inline int computeVertexHash(int x, int y, int z);

// 添加顶点 
// out nv 多边形网格顶点的个数
// out verts 多边形网格的顶点
static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, int* firstVert, int* nextVert, int& nv);


static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly);



// Returns true iff segments ab and cd intersect, properly or improperly.
// ab 与 cd 相交 返回真
static bool intersect(const int* a, const int* b, const int* c, const int* d);

// Returns true iff c is strictly to the left of the directed
// line through a to b.
// 返回true，如果c是严格ab向量的左边
inline bool left(const int* a, const int* b, const int* c);

//左侧或共线
inline bool leftOn(const int* a, const int* b, const int* c);

//共线
inline bool collinear(const int* a, const int* b, const int* c);

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
// 返回true，当且仅当AB正确相交CD：它们共享一个点内部两个部分。交点的应确保通过严格leftness。
static bool intersectProp(const int* a, const int* b, const int* c, const int* d);

// Returns T if(v_i, v_j) is a proper internal
// diagonal of P.
// 返回T;如果是适当的内部对角线P.（V_I，v_j）
static bool diagonal(int i, int j, int n, const int* verts, int* indices);

// Returns true if the diagonal (i,j) is strictly internal to the 
// polygon P in the neighborhood of the i endpoint.
// 返回true，如果对角线（I，J）是严格的内部的i端点附近的多边形P
static bool	inCone(int i, int j, int n, const int* verts, int* indices);

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
// 返回T;如果（v_i，v_j）是适当的内部或外部的P对角线，忽略边缘事件（入射的边）到V_I v_j*。
static bool diagonalie(int i, int j, int n, const int* verts, int* indices);

// rem顶点索引
static bool canRemoveVertex(rcPolyMesh& mesh, const unsigned short rem);

static bool removeVertex(rcPolyMesh& mesh, const unsigned short rem, const int maxTris);

// 创建多边形网络
// nvp 允许最大的多边形网格
bool BuildPolyMesh(rcContourSet& cset, const int nvp, rcPolyMesh& mesh);

static int triangulate(int n, const int* verts, int* indices, int* tris);

//输出多边形数据
//#Debug test3 准确版 
void OutputPolyMesh(fstream *f, rcPolyMesh *rcPmesh, int &iSumOfIndexNum);

#endif