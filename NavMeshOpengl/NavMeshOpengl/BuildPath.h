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

// 估算多边形的面积
void CalculateArea(rcPolyMesh *PolyMesh, int *PolyArea);

//计算重心
void CalculateCentre(rcPolyMesh *PolyMesh, int *PolyCentre);

// 获取点所在的多边形
int GetPloyIndex(vec3 vPoint);

// 计算邻接
void CalculateContiguous(rcPolyMesh *PolyMesh, int ** con);

// 计算邻接和重心
void CalculateContiguousAndCentre(rcPolyMesh *PolyMesh, bool ** con, vec3* centre);

// 迪克斯特拉算法
void Dijkstra(int **con, int nploys, int v0, int *distance, int *path);

// 寻找拐点
bool FindNextPoint(rcPolyMesh *PolyMesh,int *StartPoint, int StartIndex, int *EndPoint, int EndIndex, 
				   int *Path, int *NextPoint, int &NextIndex);

// 获取最短路
bool FindPath(rcPolyMesh *PolyMesh, int **con, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect);

// A*算法
void AStar(bool **con, int npolys, int iStart, int iEnd, vec3 *centre, int *came_from);

// A*寻路
bool FindPachOfAStar(rcPolyMesh *PolyMesh, bool **con, vec3 *centre, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect);

#endif