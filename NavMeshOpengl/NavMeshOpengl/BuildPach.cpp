#pragma once
#include "BuildPach.h"
#include "NavMesh.h"
#include <cmath>
#include <queue>
using namespace std;

#ifndef MAXWEIGHT
#define MAXWEIGHT 0xfffffff
#endif

// ��ȡ�����ڵĶ����
int GetPloyIndex(rcPolyMesh *PolyMesh, vec3 vPoint)
{
	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		// ��ȡ�����
		unsigned short *ploy = &PolyMesh->polys[i * PolyMesh->nvp * 2];
		int n = countPolyVerts(ploy, PolyMesh->nvp);
		int j = 0;
		int pa[3] = {0}, pb[3] = {0}, pc[3] = {0};
		pc[0] = vPoint.x;
		pc[1] = vPoint.y;
		pc[2] = vPoint.z;
// 		pb[0] = PolyMesh->verts[p[0] * 3 + 0];
// 		pb[1] = PolyMesh->verts[p[0] * 3 + 1];
// 		pb[2] = PolyMesh->verts[p[0] * 3 + 2];

		for (j = 0; j < n; j ++)
		{
			unsigned short va0 = ploy[j];
			unsigned short va1 = ploy[(j+1) % n]; //��ֹԽ��
			pa[0] = PolyMesh->verts[va0 * 3 + 0];
			pa[1] = PolyMesh->verts[va0 * 3 + 1];
			pa[2] = PolyMesh->verts[va0 * 3 + 2];
			pb[0] = PolyMesh->verts[va1 * 3 + 0];
			pb[1] = PolyMesh->verts[va1 * 3 + 1];
			pb[2] = PolyMesh->verts[va1 * 3 + 2];
			if(!leftOn(pa, pb, pc))
				break;
		}
		if (j == n) return i;
	}

	return -1;
}

//�������ε����(��Խ���)
void CalculateArea(rcPolyMesh *PolyMesh, int *PolyArea)
{
	int minx, maxx, miny, maxy;
	//���㶥��Ŀ��
	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		unsigned short* poly = &PolyMesh->polys[i * PolyMesh->nvp * 2];
		minx = maxx = PolyMesh->verts[poly[0] * 3 + 0];
		miny = maxy = PolyMesh->verts[poly[0] * 3 + 2];
		for (int j = 0; j < PolyMesh->nvp; j ++)
		{
			if (poly[j] == RC_MESH_NULL_IDX)
				break;
			minx = minx > PolyMesh->verts[poly[j] * 3 + 0] ? PolyMesh->verts[poly[j] * 3 + 0] : minx;
			maxx = maxx < PolyMesh->verts[poly[j] * 3 + 0] ? PolyMesh->verts[poly[j] * 3 + 0] : maxx;
			miny = miny > PolyMesh->verts[poly[j] * 3 + 2] ? PolyMesh->verts[poly[j] * 3 + 2] : miny;
			maxy = maxy < PolyMesh->verts[poly[j] * 3 + 2] ? PolyMesh->verts[poly[j] * 3 + 2] : maxy;
		}

		PolyArea[i] = sqrt(double((maxx - minx) * (maxy - miny)));
	}
}

//��������
void CalculateCentre(rcPolyMesh *PolyMesh, vec3 *PolyCentre)
{
	int iSumX = 0, iSumZ = 0;
	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		iSumX = 0;
		iSumZ = 0;
		unsigned short* poly = &PolyMesh->polys[i * PolyMesh->nvp * 2];
		// ���㶥�������
		int n = countPolyVerts(poly, PolyMesh->nvp);
		for (int j = 0; j < n; j ++)
		{
			iSumX += PolyMesh->verts[poly[j] * 3 + 0];
			iSumZ += PolyMesh->verts[poly[j] * 3 + 2];
		}
		PolyCentre[i].x = iSumX / n;
		PolyCentre[i].y = 0.0f;
		PolyCentre[i].z = iSumZ / n;
	}
}

//��鹲ͬ�ı�
bool CheackShareEdge(unsigned short* pa, unsigned short* pb, int nvp, int& ea, int& eb)
{
	//��������na nb�������Ŀ
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);
	ea = -1;
	eb = -1;
	for (int i = 0; i < na; ++i)
	{
		unsigned short va0 = pa[i];
		unsigned short va1 = pa[(i+1) % na]; //��ֹԽ��
		if (va0 > va1)//��������ֵ
			rcSwap(va0, va1);
		for (int j = 0; j < nb; ++j)
		{
			unsigned short vb0 = pb[j];
			unsigned short vb1 = pb[(j+1) % nb];
			if (vb0 > vb1)
				rcSwap(vb0, vb1);
			if (va0 == vb0 && va1 == vb1)
			{
				ea = i;
				eb = j;
				break;
			}
		}
	}

	// û����ͬ�ı�
	if (ea == -1 || eb == -1)
		return false;

	return true;
}

//�����ڽ�
void CalculateContiguous(rcPolyMesh *PolyMesh, int ** con)
{
	int ea = -1, eb = -1;
	//�������ε����
	int *iArea = new int[PolyMesh->npolys];
	CalculateArea(PolyMesh, iArea);

	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		con[i][i] = 0;
		for (int j = i + 1; j < PolyMesh->npolys; j ++)
		{
			if (CheackShareEdge(&PolyMesh->polys[i * PolyMesh->nvp * 2],
				&PolyMesh->polys[j * PolyMesh->nvp * 2], PolyMesh->nvp, ea, eb))
			{
				con[i][j] = con[j][i] = iArea[j];
			}
		}
	}
	delete [] iArea;
}

// �����ڽӾ��������
void CalculateContiguousAndCentre(rcPolyMesh *PolyMesh, bool ** con, vec3* centre)
{
	int ea = -1, eb = -1;
	// �������ε�����
	CalculateCentre(PolyMesh, centre);

	for (int i = 0; i < PolyMesh->npolys; i ++)
	{
		con[i][i] = true;
		for (int j = i + 1; j < PolyMesh->npolys; j ++)
		{
			if (CheackShareEdge(&PolyMesh->polys[i * PolyMesh->nvp * 2],
				&PolyMesh->polys[j * PolyMesh->nvp * 2], PolyMesh->nvp, ea, eb))
			{
				con[i][j] = con[j][i] = true;
			}
		}
	}
}


//�Ͽ�˹�����㷨
void Dijkstra(int **con, int nploys, int v0, int *distance, int *path)
{
	int *s = new int[nploys];
	int minDis, i, j, u;
	//��ʼ��
	for (i = 0; i < nploys; ++i)
	{
		distance[i] = con[v0][i];
		s[i] = 0; //��ʼ���Ϊ0
		if(i != v0 && distance[i] < MAXWEIGHT)
		{
			path[i] = v0;
		}
		else 
		{
			path[i] = -1;
		}
	}
	s[v0] = 1; //��Ƕ���v0 �Ѿ�ʹ��

	//�ڵ�ǰ��û�ҵ����·���Ķ��㼯����ѡȡ������̾���Ķ���
	for (i = 1; i < nploys; i ++)
	{
		minDis = MAXWEIGHT;
		for (j = 0; j < nploys; j ++)
		{
			if (s[j] == 0 && distance[j] < minDis)
			{
				u = j;
				minDis = distance[j];
			}
		}

		//���Ѿ�������·��ʱ�㷨�����������Է���ͨͼ�Ǳ����
		if (minDis == MAXWEIGHT) return;

		s[u] = 1;

		for (j = 0; j < nploys; j ++)
		{
			if (s[j] == 0 && con[u][j] < MAXWEIGHT && 
				distance[u] + con[u][j] < distance[j])
			{
				distance[j] = distance[u] + con[u][j];
				path[j] = u;
			}
		}
	}
}



//debug
//�������� ��� �յ� ���ص����� �յ� �յ����ڵĶ�������� 
bool FindNextPoint(rcPolyMesh *PolyMesh,int *StartPoint, int StartIndex, int *EndPoint, int EndIndex, 
				   int *Path, int *NextPoint, int &NextIndex)
{
	int ea = -1, eb = -1;	
	int iArrLastEndPoint[3] = {StartPoint[0], StartPoint[1], StartPoint[2]};
	int iArrLeftPoint[3] = {0};
	int iArrRightPoint[3] = {0};
	int iArrNewLeftPoint[3] = {0};
	int iArrNewRightPoint[3] = {0};

	//ȡ����һ���ڱ� ��ʼ�������ҵ�
	unsigned short *pa = &PolyMesh->polys[Path[StartIndex] * PolyMesh->nvp * 2];
	unsigned short *pb = &PolyMesh->polys[StartIndex * PolyMesh->nvp * 2];
	CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);

	iArrLeftPoint[0] = PolyMesh->verts[pa[ea] * 3 + 0];
	iArrLeftPoint[1] = 0.0f;
	iArrLeftPoint[2] = PolyMesh->verts[pa[ea] * 3 + 2];
	iArrRightPoint[0] = PolyMesh->verts[pb[eb] * 3 + 0];
	iArrRightPoint[1] = 0.0f;
	iArrRightPoint[2] = PolyMesh->verts[pb[eb] * 3 + 2];

	StartIndex = Path[StartIndex];
	int iLeftIndex = StartIndex, iRightIndex = StartIndex;

	//�յ�����һ����ͬ�ߵĶ���ʱӦ������һ���ڽӶ�� ��Ȼ�᷵��false ���³������
	//�ȴ���Ӵ���
	//
	// code
	//

	while(Path[StartIndex] != -1)
	{
		//��ȡ��һ���ڽӱ�
		pa = &PolyMesh->polys[Path[StartIndex] * PolyMesh->nvp * 2];
		pb = &PolyMesh->polys[StartIndex * PolyMesh->nvp * 2];
		CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);

// 		iArrLastEndPoint[0] = Vect[Vect.size() - 1].x;
// 		iArrLastEndPoint[1] = Vect[Vect.size() - 1].y;
// 		iArrLastEndPoint[2] = Vect[Vect.size() - 1].z;
		iArrNewLeftPoint[0] = PolyMesh->verts[pa[ea] * 3 + 0];
		iArrNewLeftPoint[1] = 0.0f;
		iArrNewLeftPoint[2] = PolyMesh->verts[pa[ea] * 3 + 2];
		iArrNewRightPoint[0] = PolyMesh->verts[pb[eb] * 3 + 0];
		iArrNewRightPoint[1] = 0.0f;
		iArrNewRightPoint[2] = PolyMesh->verts[pb[eb] * 3 + 2];

		// �������ҵ���м� �������
		if( !left(iArrLastEndPoint, iArrLeftPoint, iArrNewLeftPoint)
			&& leftOn(iArrLastEndPoint, iArrRightPoint, iArrNewLeftPoint))
		{
			//leftPoint = newLeftPoint;
			iArrLeftPoint[0] = iArrNewLeftPoint[0];
			iArrLeftPoint[1] = iArrNewLeftPoint[1];
			iArrLeftPoint[2] = iArrNewLeftPoint[2];
			iLeftIndex = Path[StartIndex];//������������
		}
		// �����ҵ�֮�� �����µĹյ� �������ҵ�
		else if( !leftOn(iArrLastEndPoint, iArrLeftPoint, iArrNewLeftPoint)
			&& !leftOn(iArrLastEndPoint, iArrRightPoint, iArrNewLeftPoint))
		{
			//�ҵ��µĹյ�
			NextPoint[0] = iArrRightPoint[0];
			NextPoint[1] = iArrRightPoint[1];
			NextPoint[2] = iArrRightPoint[2];
			//��������
			NextIndex = iRightIndex;
			return true;
		}

		// ���ҵ��� ���ҵ��м� �����ҵ�
		if( !left(iArrLastEndPoint, iArrLeftPoint, iArrNewRightPoint)
			&& leftOn(iArrLastEndPoint, iArrRightPoint, iArrNewRightPoint))
		{
			//rightPoint = newRightPoint;
			for(int i = 0; i < 3; i ++)
			{
				iArrRightPoint[i] = iArrNewRightPoint[i];
			}
			iRightIndex = Path[StartIndex];//������������
		}
		// �����ҵ�֮�� �����µĹյ� �������ҵ�
		else if( left(iArrLastEndPoint, iArrLeftPoint, iArrNewRightPoint)
			&& left(iArrLastEndPoint, iArrRightPoint, iArrNewRightPoint))
		{
			//�ҵ��µĹյ�
			NextPoint[0] = iArrLeftPoint[0];
			NextPoint[1] = iArrLeftPoint[1];
			NextPoint[2] = iArrLeftPoint[2];
			//��������
			NextIndex = iLeftIndex;
			return true;
		}
		StartIndex = Path[StartIndex];
	}

	//�����һ�������Ƚ�
	// �������ҵ���м� �������
	if( !left(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& leftOn(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		NextIndex = EndIndex;
		return false;
	}
	// �����ҵ�֮�� �����µĹյ� �������ҵ�
	else if( !leftOn(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& !leftOn(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		//�ҵ��µĹյ�
		NextPoint[0] = iArrRightPoint[0];
		NextPoint[1] = iArrRightPoint[1];
		NextPoint[2] = iArrRightPoint[2];
		//��������
		NextIndex = iRightIndex;
		return true;
	}

	// debug����
	// ���ҵ��� ���ҵ��м� �����ҵ�
	if( !left(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& leftOn(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		NextIndex = EndIndex;
		return false;
	}
	// �����ҵ�֮�� �����µĹյ� �������ҵ�
	else if( left(iArrLastEndPoint, iArrLeftPoint, EndPoint)
		&& left(iArrLastEndPoint, iArrRightPoint, EndPoint))
	{
		//�ҵ��µĹյ�
		NextPoint[0] = iArrLeftPoint[0];
		NextPoint[1] = iArrLeftPoint[1];
		NextPoint[2] = iArrLeftPoint[2];
		//��������
		NextIndex = iLeftIndex;
		return true;
	}

	return false;
}

// ��ȡ���·
bool FindPach(rcPolyMesh *PolyMesh, int **con, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect)
{
	int iStart = GetPloyIndex(PolyMesh, StartPoint);
	int iEnd = GetPloyIndex(PolyMesh, EndPoint);
	printf("Start Index: %d  EndIndex: %d\n", iStart, iEnd);
	// 	//��ͬһ���������
	if (iStart == iEnd)
	{
		Vect.push_back(EndPoint);
		Vect.push_back(StartPoint);
	}
#if 1
	//debug 2 2013/10/18 14:33
	else 
	{
		//����·��
		int *distance = new int[PolyMesh->npolys];
		int *path = new int[PolyMesh->npolys];
		Dijkstra(con, PolyMesh->npolys, iStart, distance, path);
		//���ɴ�
		if (path[iEnd] == -1)
			return false; 
		//#debug ���δ�Ż�·��
		int temp = iEnd;
		unsigned short *pa, *pb;
		int ea = -1, eb = -1;
		printf("������%3d �գ�(%.2f %.2f %.2f)\n", iEnd, EndPoint.x, 0.0f, EndPoint.z);
		while(path[temp] != -1)
		{
			//ȡ����һ���ڱ� ��ʼ�������ҵ�
			pa = &PolyMesh->polys[path[temp] * PolyMesh->nvp * 2];
			pb = &PolyMesh->polys[temp * PolyMesh->nvp * 2];
			CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);
			int x = PolyMesh->verts[pa[ea] * 3 + 0] + PolyMesh->verts[pb[eb] * 3 + 0]; 
			int z = PolyMesh->verts[pa[ea] * 3 + 2] + PolyMesh->verts[pb[eb] * 3 + 2];
			printf("������%3d �У�(%3d %3d %3d) ", temp, x / 2, 0, z / 2);
			printf("��(%3d %3d %3d), �ң�(%3d %3d %3d)\n", 
				PolyMesh->verts[pa[ea] * 3 + 0], 
				PolyMesh->verts[pa[ea] * 3 + 1],
				PolyMesh->verts[pa[ea] * 3 + 2],
				PolyMesh->verts[pb[eb] * 3 + 0],
				PolyMesh->verts[pb[eb] * 3 + 1],
				PolyMesh->verts[pb[eb] * 3 + 2]
			);
			temp = path[temp];
		}
		printf("������%3d ��(%.2f %.2f %.2f)\n", iStart, StartPoint.x, 0.0f, StartPoint.z);

		//�Ż�·�� ���յ��ѻ����
		int iArrStartPoint[3] = {EndPoint.x, EndPoint.y, EndPoint.z};
		int iArrEndPoint[3] = {StartPoint.x, StartPoint.y, StartPoint.z};
		int iArrNextPoint[3] = {0};
		int StartIndex = iEnd;
		int EndIndex = iStart;
		int NextIndex = -1;
		//����յ�
		Vect.push_back(EndPoint);
		cout << Vect[Vect.size() - 1] << endl;
		while(path[StartIndex] != -1)
		{
			//�Ҳ����µĹյ�
			if (!FindNextPoint(PolyMesh, iArrStartPoint, StartIndex, iArrEndPoint, 
				EndIndex, path, iArrNextPoint, NextIndex))
			{
				break;
			}

			vec3 newPoint(iArrNextPoint[0], iArrNextPoint[1], iArrNextPoint[2]);
			Vect.push_back(newPoint);
			cout << Vect[Vect.size() - 1] << endl;
			//���¹յ�Ϊ�µ����
			for (int i = 0; i < 3; i ++)
			{
				iArrStartPoint[i] = iArrNextPoint[i];
			}
			StartIndex = NextIndex;
		}
// 		if(NextIndex != -1 && path[NextIndex] == -1)
// 		{
// 			Vect.push_back(StartPoint);
// 			cout << Vect[Vect.size() - 1] << endl;
// 		}
// 		else
// 		{
// 			return false;
// 		} 
		Vect.push_back(StartPoint);
		cout << Vect[Vect.size() - 1] << endl;

		delete [] path;
		delete [] distance;
	}
#endif

	return true;
}

static int Math_DistPointPoint(int dx, int dy)
{
	if (dx < 0)
	{
		dx = -dx;
	}
	if (dy < 0)
	{
		dy = -dy;
	}

	int min, max;
	if (dx < dy)
	{
		min = dx;
		max = dy;
	}
	else
	{
		min = dy;
		max = dx;
	}
	return ((max << 8) - (max << 3) - (max << 1) + (min << 6) + (min << 5)
		+ (min << 2) + (min << 1)) >> 8;
}


//��ֵ����
int HeuristicEstimateOfDistance(vec3 vStart, vec3 vEnd)
{
// 	int dx = fabs(vEnd.x - vStart.x);
// 	int dy = fabs(vEnd.z - vStart.z);
// 	return dx + dy;
	int dx = vEnd.x - vStart.x;
	int dy = vEnd.z - vStart.z;
	return Math_DistPointPoint(dx, dy);
}

// a b �������
int DistBetween(vec3 a, vec3 b)
{
	int len = sqrt((a.x - b.x)*(a.x - b.x) + (a.z - b.z)*(a.z - b.z));
	return len;
}

//����ȽϽṹ �������ȶ�������
struct cmp1{
	bool operator ()(AStarNode &a, AStarNode &b){
		//return a>b;//��Сֵ����
		return a.iF_score > b.iF_score;
	}
};

void UpdateFScore(priority_queue<AStarNode, vector<AStarNode>, cmp1> &que, int index, int len)
{
	queue<AStarNode> temp;
	AStarNode tempNode;
	while(!que.empty())
	{
		tempNode = que.top();
		que.pop();
		if (tempNode.iIndex == index)
		{
			tempNode.iF_score = len;
			que.push(tempNode);
			break;
		}
		temp.push(tempNode);
	}
	while(!temp.empty())
	{
		tempNode = temp.front();
		temp.pop();
		que.push(tempNode);
	}
}


// A*�㷨
void AStar(bool **con, int npolys, int iStart, int iEnd, vec3 *centre, int *came_from)
{
	priority_queue<AStarNode, vector<AStarNode>, cmp1> OpenList;      // Open ��
	//queue<AStarNode> CloseList;              // Close ��
	int* iG_score = new int[npolys];          // G
	int* iH_score = new int[npolys];          // H
	int* iInList = new int[npolys];           //����Ƿ�ʹ�� -1 ��close�� 0 δʹ�� 1 ��open��
	AStarNode* node = new AStarNode[npolys];
	AStarNode tempNode;
	//��ʼ����������
	for(int i = 0; i < npolys; i ++)
	{
		node[i].iIndex = i;
		node[i].iF_score = MAXWEIGHT;
		came_from[i] = -1;
		iInList[i] = 0;
	}
	iG_score[iStart] = 0;
	iH_score[iStart] = HeuristicEstimateOfDistance(centre[iStart], centre[iEnd]);
	node[iStart].iF_score = iH_score[iStart];	
	OpenList.push(node[iStart]);

// 	while openset is not empty
	while(!OpenList.empty())
	{
		tempNode = OpenList.top(); //ȡ����
		iInList[tempNode.iIndex] = -1; //�Ž�CloseList
		//�յ�
		if(tempNode.iIndex == iEnd)
		{
			return;
		}
		OpenList.pop();
		//CloseList.push(tempNode);//����ر��б�
		//����tempNode�ڽӵĶ����
		for (int i = 0; i < npolys; i ++)
		{
			if (con[tempNode.iIndex][i] && tempNode.iIndex != i)
			{
				//��CloseList
				if (iInList[i] == -1)
					continue;
				//
				//bool tentative_is_better = false; //�õ����ŵ�ֵ
				int tentative_g_score = iG_score[tempNode.iIndex] + 
					DistBetween(centre[tempNode.iIndex], centre[i]);
				// ����OpenList ֱ����ӵ�OpenList
				if(iInList[i] != 1)
				{
					came_from[i] = tempNode.iIndex; //���¸��׽ڵ�
					iG_score[i] = tentative_g_score;
					iH_score[i] = HeuristicEstimateOfDistance(centre[i], centre[iEnd]);
					AStarNode NewNode;
					NewNode.iIndex = i;
					NewNode.iF_score = iG_score[i] + iH_score[i];
					OpenList.push(NewNode);
					iInList[i] = 1; 
				}
				else if (tentative_g_score < iG_score[i])
				{
					//����Fֵ
					UpdateFScore(OpenList, i, tentative_g_score);
				}
			}
		}
	}
// x := the node in openset having the lowest f_score[] value
//    if x = goal
// 	   return reconstruct_path(came_from,goal)
// 	   remove x from openset
// 	   add x to closedset
// 	   foreach y in neighbor_nodes(x)  //foreach=for each
// 	   if y in closedset
// 		   continue
// tentative_g_score := g_score[x] + dist_between(x,y)
// 
// 				   if y not in openset
// 					   add y to openset
// 
// tentative_is_better := true
// 					 elseif tentative_g_score < g_score[y]
// tentative_is_better := true
// 				   else
// tentative_is_better := false
// 					 if tentative_is_better = true
// 						 came_from[y] := x
// 						 g_score[y] := tentative_g_score
// 						 h_score[y] := heuristic_estimate_of_distance(y, goal)
// 						 f_score[y] := g_score[y] + h_score[y]
// 					 return failure
}


// A* Ѱ·
bool FindPachOfAStar(rcPolyMesh *PolyMesh, bool **con, vec3 *centre, vec3 StartPoint, vec3 EndPoint, vector<vec3> &Vect)
{
	int iStart = GetPloyIndex(PolyMesh, StartPoint);
	int iEnd = GetPloyIndex(PolyMesh, EndPoint);
	printf("  A*\nStart Index: %d  EndIndex: %d\n", iStart, iEnd);
	// 	//��ͬһ���������
	if (iStart == iEnd)
	{
		Vect.push_back(EndPoint);
		Vect.push_back(StartPoint);
	}
#if 1
	else 
	{
		//����·��
		int *path = new int[PolyMesh->npolys];
		AStar(con, PolyMesh->npolys, iStart, iEnd, centre, path);
		//���ɴ�
		if (path[iEnd] == -1)
			return false; 
		//#debug ���δ�Ż�·��
		int temp = iEnd;
		unsigned short *pa, *pb;
		int ea = -1, eb = -1;
		printf("������%3d �գ�(%.2f %.2f %.2f)\n", iEnd, EndPoint.x, 0.0f, EndPoint.z);
		while(path[temp] != -1)
		{
			//ȡ����һ���ڱ� ��ʼ�������ҵ�
			pa = &PolyMesh->polys[path[temp] * PolyMesh->nvp * 2];
			pb = &PolyMesh->polys[temp * PolyMesh->nvp * 2];
			CheackShareEdge(pa, pb, PolyMesh->nvp, ea, eb);
			int x = PolyMesh->verts[pa[ea] * 3 + 0] + PolyMesh->verts[pb[eb] * 3 + 0]; 
			int z = PolyMesh->verts[pa[ea] * 3 + 2] + PolyMesh->verts[pb[eb] * 3 + 2];
			printf("������%3d �У�(%3d %3d %3d) ", temp, x / 2, 0, z / 2);
			printf("��(%3d %3d %3d), �ң�(%3d %3d %3d)\n", 
				PolyMesh->verts[pa[ea] * 3 + 0], 
				PolyMesh->verts[pa[ea] * 3 + 1],
				PolyMesh->verts[pa[ea] * 3 + 2],
				PolyMesh->verts[pb[eb] * 3 + 0],
				PolyMesh->verts[pb[eb] * 3 + 1],
				PolyMesh->verts[pb[eb] * 3 + 2]
			);
			temp = path[temp];
		}
		printf("������%3d ��(%.2f %.2f %.2f)\n", iStart, StartPoint.x, 0.0f, StartPoint.z);

		//�Ż�·�� ���յ��ѻ����
		int iArrStartPoint[3] = {EndPoint.x, EndPoint.y, EndPoint.z};
		int iArrEndPoint[3] = {StartPoint.x, StartPoint.y, StartPoint.z};
		int iArrNextPoint[3] = {0};
		int StartIndex = iEnd;
		int EndIndex = iStart;
		int NextIndex = -1;
		//����յ�
		Vect.push_back(EndPoint);
		cout << Vect[Vect.size() - 1] << endl;
		while(path[StartIndex] != -1)
		{
			//�Ҳ����µĹյ�
			if (!FindNextPoint(PolyMesh, iArrStartPoint, StartIndex, iArrEndPoint, 
				EndIndex, path, iArrNextPoint, NextIndex))
			{
				break;
			}

			vec3 newPoint(iArrNextPoint[0], iArrNextPoint[1], iArrNextPoint[2]);
			Vect.push_back(newPoint);
			cout << Vect[Vect.size() - 1] << endl;
			//���¹յ�Ϊ�µ����
			for (int i = 0; i < 3; i ++)
			{
				iArrStartPoint[i] = iArrNextPoint[i];
			}
			StartIndex = NextIndex;
		}
		// 		if(NextIndex != -1 && path[NextIndex] == -1)
		// 		{
		// 			Vect.push_back(StartPoint);
		// 			cout << Vect[Vect.size() - 1] << endl;
		// 		}
		// 		else
		// 		{
		// 			return false;
		// 		} 
		Vect.push_back(StartPoint);
		cout << Vect[Vect.size() - 1] << endl;

		delete [] path;
	}
#endif

	return true;
}