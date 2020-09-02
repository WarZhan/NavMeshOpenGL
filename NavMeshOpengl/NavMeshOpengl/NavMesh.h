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


//��������
bool LoadConSet(fstream *f, rcContourSet *cset, int &sum);

// ��ȡ����εĶ��������
// static int countPolyVerts(const unsigned short* p, const int nvp);

static void pushBack(int v, int* arr, int& an);

static void pushFront(int v, int* arr, int& an);

//��ȡ��һ������
inline int next(int i, int n);
//��ȡǰһ������
inline int prev(int i, int n);
//���
inline int area2(const int* a, const int* b, const int* c);


static bool vequal(const int* a, const int* b);

// ��ȡ����εĶ��������
int countPolyVerts(const unsigned short* p, const int nvp);


//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
// ���
inline bool xorb(bool x, bool y);

//���
inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c);

// Returns T if (a,b,c) are collinear and point c lies 
// on the closed segment ab.
//a b c 3�㹲�� �� c �� ab �м�
static bool between(const int* a, const int* b, const int* c);


static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb,
							 const int nvp);


static void mergePolys(unsigned short* pa, unsigned short* pb, int ea, int eb,
					   unsigned short* tmp, const int nvp);


//���㶥���ϣ
inline int computeVertexHash(int x, int y, int z);

// ��Ӷ��� 
// out nv ��������񶥵�ĸ���
// out verts ���������Ķ���
static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, int* firstVert, int* nextVert, int& nv);


static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly);



// Returns true iff segments ab and cd intersect, properly or improperly.
// ab �� cd �ཻ ������
static bool intersect(const int* a, const int* b, const int* c, const int* d);

// Returns true iff c is strictly to the left of the directed
// line through a to b.
// ����true�����c���ϸ�ab���������
inline bool left(const int* a, const int* b, const int* c);

//������
inline bool leftOn(const int* a, const int* b, const int* c);

//����
inline bool collinear(const int* a, const int* b, const int* c);

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
// ����true�����ҽ���AB��ȷ�ཻCD�����ǹ���һ�����ڲ��������֡������Ӧȷ��ͨ���ϸ�leftness��
static bool intersectProp(const int* a, const int* b, const int* c, const int* d);

// Returns T if(v_i, v_j) is a proper internal
// diagonal of P.
// ����T;������ʵ����ڲ��Խ���P.��V_I��v_j��
static bool diagonal(int i, int j, int n, const int* verts, int* indices);

// Returns true if the diagonal (i,j) is strictly internal to the 
// polygon P in the neighborhood of the i endpoint.
// ����true������Խ��ߣ�I��J�����ϸ���ڲ���i�˵㸽���Ķ����P
static bool	inCone(int i, int j, int n, const int* verts, int* indices);

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
// ����T;�����v_i��v_j�����ʵ����ڲ����ⲿ��P�Խ��ߣ����Ա�Ե�¼�������ıߣ���V_I v_j*��
static bool diagonalie(int i, int j, int n, const int* verts, int* indices);

// rem��������
static bool canRemoveVertex(rcPolyMesh& mesh, const unsigned short rem);

static bool removeVertex(rcPolyMesh& mesh, const unsigned short rem, const int maxTris);

// �������������
// nvp �������Ķ��������
bool BuildPolyMesh(rcContourSet& cset, const int nvp, rcPolyMesh& mesh);

static int triangulate(int n, const int* verts, int* indices, int* tris);

//������������
//#Debug test3 ׼ȷ�� 
void OutputPolyMesh(fstream *f, rcPolyMesh *rcPmesh, int &iSumOfIndexNum);

#endif