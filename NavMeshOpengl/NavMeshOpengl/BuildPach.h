#pragma once
#ifndef _BUILDPACH_H_
#define _BUILDPACH_H_

#include <vector>
#include "Angel.h"
#include "Recast\Recast.h"

using namespace std;

struct AStarNode
{
	int iIndex;
	int iF_score;
};

// �������ε����
void CalculateArea(rcPolyMesh *PolyMesh, int *PolyArea);

//��������
void CalculateCentre(rcPolyMesh *PolyMesh, int *PolyCentre);

// ��ȡ�����ڵĶ����
int GetPloyIndex(vec3 vPoint);

// �����ڽ�
void CalculateContiguous(rcPolyMesh *PolyMesh, int ** con);

// �����ڽӺ�����
void CalculateContiguousAndCentre(rcPolyMesh *PolyMesh, bool ** con, vec3* centre);

// �Ͽ�˹�����㷨
void Dijkstra(int **con, int nploys, int v0, int *distance, int *path);

// Ѱ�ҹյ�
bool FindNextPoint(rcPolyMesh *PolyMesh,int *StartPoint, int StartIndex, int *EndPoint, int EndIndex, 
				   int *Path, int *NextPoint, int &NextIndex);

// ��ȡ���·
bool FindPach(rcPolyMesh *PolyMesh, int **con, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect);

// A*�㷨
void AStar(bool **con, int npolys, int iStart, int iEnd, vec3 *centre, int *came_from);

// A*Ѱ·
bool FindPachOfAStar(rcPolyMesh *PolyMesh, bool **con, vec3 *centre, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect);

#endif