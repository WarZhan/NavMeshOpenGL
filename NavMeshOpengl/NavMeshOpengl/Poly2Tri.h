#pragma once

#ifndef _POLY2TRI_H_
#define _POLY2TRI_H_

#include "Poly2tri/geometry.h"
//#include "Poly2tri/parse.h"
#include "NavMesh.h"
#include <string>
using namespace std;

void BuildSet(string filename);

// �����ļ� ��װ��δ���߶��������
bool BuildSet(string filename, rcContourSet &rcConSet);

#endif