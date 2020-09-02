#include <fstream>
#include "Angel.h"
#include "NavMesh.h"
#include "BuildPach.h"
#include "Poly2Tri.h"
#include <iomanip>

using namespace std;

typedef Angel::vec3 point3;
typedef Angel::vec3 color3;
GLsizei wh = 700, ww = 700; //���ڴ�С

 /*shader�б�������*/
GLuint g_vPosition;  // shader��in����vPosition������
GLuint g_vColor;     // ��ɫ
GLuint MVPMatrix;    // shader��uniform����MVPMatrix������

//��������
rcContourSet *g_pConSet; 
int* g_iConIndex;       //��������������
int g_iNumOfCon;        //����������
int g_iNumOfSetVer;     //���϶��������
point3* g_pSetVer;      //���϶�������
color3* g_Ve3SetColors; //���϶�����ɫ
GLuint g_uiVaoSetCon;   //����vao

//���������
rcPolyMesh *g_pPolyMesh; 
int* g_iPolyIndex;         //��������������
int g_iNumOfPoly;          //���������
// int g_iNumOfSetVer;     //���϶��������
point3* g_pPolyVer;        //����ζ�������
color3* g_Ve3PolyColors;   //����ζ�����ɫ
GLuint g_uiVaoPloyMesh;    //�����vao
int g_iDrawPolyNum = 0;

//·��
GLuint g_uiVaoPath;     //·��vao
int g_NumOfPath = 1000; //Ĭ�����
int g_iDrawPathMaxSize = 0;
int g_iDrawAStarPathMaxSize = 0;


//�ڽӾ��� Dijkstra
int ** g_pContiguous;
//�����յ�
point3 g_p3Start = point3(-1.0f, -1.0f, -1.0f), g_p3End = point3(-1.0f, -1.0f, -1.0f);
//·������
vector<vec3> g_vetPachPoint;

//�ڽӾ��� A*
bool ** g_bContiguous;
point3 * g_Ve3Centre; //����ε�����

//
//MyKeyDownNum
enum { POLYADD, POLYSUB, NUM_KEY};
bool keyDown[NUM_KEY];

//����
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	
	//�����
	glBindVertexArray(g_uiVaoPloyMesh);
	int index = 0;
	for (int j = 0; j < g_iDrawPolyNum; j ++)
	{
		glDrawArrays(
			//GL_LINE_LOOP,
			GL_TRIANGLE_FAN,
			//GL_TRIANGLES,
			index, 
			g_iPolyIndex[j]
		);
		index += g_iPolyIndex[j];
	}

	//��������
	glBindVertexArray(g_uiVaoSetCon);//�󶨶�������
	index = 0;
	for (int i = 0; i < g_iNumOfCon; i ++)
		//for (int i = 0; i < 16; i ++)
	{
		glDrawArrays(
			GL_LINE_LOOP,
			//GL_TRIANGLE_FAN,
			//GL_TRIANGLES,
			index, 
			g_iConIndex[i]
		);
		index += g_iConIndex[i];
	}

	//·�� Dijkstar
	glBindVertexArray(g_uiVaoPath);
	glDrawArrays(
		GL_LINE_STRIP,
		0,
		g_iDrawPathMaxSize
		);

	//·�� A*
	glBindVertexArray(g_uiVaoPath);
	glDrawArrays(
		GL_LINE_STRIP,
		g_iDrawPathMaxSize,
		g_iDrawAStarPathMaxSize
		);

	glFlush();
}


void MyKeyDown(unsigned char key, int x, int y)
{

// 	if(key == 27 || key == 'q' || key == 'Q')
// 		exit(EXIT_SUCCESS);
	switch(key){
		case  27: //ESC
			exit(EXIT_SUCCESS);
			break;
		case '1':
			if(!keyDown[POLYADD] && g_iDrawPolyNum < g_iNumOfPoly) 
				g_iDrawPolyNum += 1;
			keyDown[POLYADD] = GL_TRUE;
			break;
		case '2':
			if(!keyDown[POLYSUB] && g_iDrawPolyNum > 0) 
				g_iDrawPolyNum -= 1;
			keyDown[POLYSUB] = GL_TRUE;
			break;
	}
	glutPostRedisplay();
}

void MyKeyUp(unsigned char key, int x, int y)
{
	if (key == '1')
	{
		keyDown[POLYADD] = GL_FALSE;
	}
	if (key == '2')
	{
		keyDown[POLYSUB] = GL_FALSE;
	}
	glutPostRedisplay();
}

void mymouse(int btn, int state, int x, int y)
{
	// ��ȡ�����յ�

	//��������ȡ��ʼ��
	if (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		g_p3Start.x = x;
		g_p3Start.z = y;
		g_p3Start.y = 0.0f;
		printf("start: %f %f %f\n", g_p3Start.x, g_p3Start.y, g_p3Start.z);
	}
	//����Ҽ���ȡ�յ�
	if (btn == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{
		g_p3End.x = x;
		g_p3End.z = y;
		g_p3End.y = 0.0f;
		printf("end: %f %f %f\n", g_p3End.x, g_p3End.y, g_p3End.z);
	}
	//����м���ʼѰ·
	if (btn == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
	{
		if (!(g_p3Start.x == -1 || g_p3Start.z == -1
			|| g_p3End.x == -1 || g_p3End.z == -1))
		{
#if 1
			//Dijkstra
			if(FindPach(g_pPolyMesh, g_pContiguous, g_p3Start, g_p3End, g_vetPachPoint))
			{
				int MaxSize = g_NumOfPath/2 < g_vetPachPoint.size() ? g_NumOfPath/2 : g_vetPachPoint.size();
				g_iDrawPathMaxSize = MaxSize;
				printf("MaxSize: %d\n", MaxSize);
				vec3 *data = new vec3[MaxSize * 2];
				for(int i = 0; i < MaxSize; i ++)
				{
					data[i * 2].x = g_vetPachPoint[i].x;
					data[i * 2].y = wh - g_vetPachPoint[i].z;
					data[i * 2].z = 0.0f;
					data[i * 2 + 1] = color3(1.0f, 0.0f, 0.0f);
				}
				//�����
				glBindVertexArray(g_uiVaoPath);
				glBufferSubData(GL_ARRAY_BUFFER,
					0,
					sizeof(vec3) * MaxSize * 2, 
					data);
				delete [] data;
				g_vetPachPoint.clear();
				glutPostRedisplay();//���¶���
			}
#endif

#if 1
			// A*
			if (FindPachOfAStar(g_pPolyMesh, g_bContiguous, g_Ve3Centre, g_p3Start, g_p3End, g_vetPachPoint))
			{
				int MaxSize = g_NumOfPath/2 < g_vetPachPoint.size() ? g_NumOfPath/2 : g_vetPachPoint.size();
				//g_iDrawPathMaxSize += MaxSize;
				g_iDrawAStarPathMaxSize = MaxSize;
				printf("MaxSize: %d\n", MaxSize);
				vec3 *data = new vec3[MaxSize * 2];
				for(int i = 0; i < MaxSize; i ++)
				{
					data[i * 2].x = g_vetPachPoint[i].x;
					data[i * 2].y = wh - g_vetPachPoint[i].z;
					data[i * 2].z = 0.0f;
					data[i * 2 + 1] = color3(0.0f, 1.0f, 0.0f);
				}
				//�����
				glBindVertexArray(g_uiVaoPath);
				glBufferSubData(GL_ARRAY_BUFFER,
					g_iDrawPathMaxSize * sizeof(vec3) * 2,
					sizeof(vec3) * g_iDrawAStarPathMaxSize * 2, 
					data);
				delete [] data;
				g_vetPachPoint.clear();
				glutPostRedisplay();//���¶���
			}
#endif
		}
	}
}


void myReshape(int w, int h)
{
	ww = w;
	wh = h;
	glViewport(0, 0, w, h);
	mat4 matProj = Ortho2D(0, w, 0, h);
	glUniformMatrix4fv(MVPMatrix, 1, true, matProj);
}

//��ʼ������
void initSet(fstream *f)
{

	//ע�⣺�ѽ����㱶���Ŵ� 3 ��
	//LoadConSet(f, g_pConSet, g_iNumOfSetVer);
	g_iNumOfSetVer = 0;
	for(int i = 0; i < g_pConSet->nconts; i ++)
	{
		g_iNumOfSetVer += g_pConSet->conts[i].nverts;
	}

#if 1
	g_pSetVer = new point3[g_iNumOfSetVer];
	g_Ve3SetColors = new point3[g_iNumOfSetVer];//��ɫ����
	g_iConIndex = new int[g_pConSet->nconts];
	g_iNumOfCon = g_pConSet->nconts;

	int iVerIndex = 0;

	for (int i = 0; i < g_pConSet->nconts; i ++)
	{
		g_iConIndex[i] = g_pConSet->conts[i].nverts;
		//�������һ����ɫ
		//color3 randColor = color3(rand() % 256, rand() % 256, rand() % 256) / 255;
		color3 randColor = color3(1.0f, 1.0f, 1.0f);
		for (int j = 0; j < g_iConIndex[i]; j ++)
		{
			g_pSetVer[iVerIndex].x = g_pConSet->conts[i].verts[j * 4 + 0];
			g_pSetVer[iVerIndex].y = wh - g_pConSet->conts[i].verts[j * 4 + 2];
// 			g_pSetVer[iVerIndex].x = g_pConSet->conts[i].verts[j * 4 + 0] * 3;
// 			g_pSetVer[iVerIndex].y = wh - g_pConSet->conts[i].verts[j * 4 + 2] * 3;
			g_pSetVer[iVerIndex].z = 0.0f;
			printf("%f %f\n",g_pSetVer[iVerIndex].x, g_pSetVer[iVerIndex].y);
			g_Ve3SetColors[iVerIndex ++] = randColor; 
		}
	}
#endif

	
	//GLuint vao;
	glGenVertexArrays(1, &g_uiVaoSetCon);
	glBindVertexArray(g_uiVaoSetCon);

	GLuint bufferSet;
	glGenBuffers(1, &bufferSet);
	glBindBuffer(GL_ARRAY_BUFFER, bufferSet);

	glBufferData(GL_ARRAY_BUFFER,
		sizeof(point3) * 2 * g_iNumOfSetVer,
		NULL,
		GL_STATIC_DRAW
		);
	//gai
	// 	glBufferData(GL_ARRAY_BUFFER,
	// 		(sizeof(point3) + sizeof(color3)) * g_iNumOfVer,
	// 		NULL,
	// 		GL_STATIC_DRAW
	// 		);

	/*�ֱ��������*/
	glBufferSubData(GL_ARRAY_BUFFER, 0, 
		sizeof(point3) * g_iNumOfSetVer,
		g_pSetVer);	// ���ض���λ������
	//sizeof(point3) * 3, g_pVer);	
	glBufferSubData(GL_ARRAY_BUFFER, 
		sizeof(point3) * g_iNumOfSetVer, 
		//sizeof(point3) * 3,
		sizeof(color3) * g_iNumOfSetVer,
		g_Ve3SetColors);  // ������ɫ����
	//sizeof(point3) * 3, g_Ve3Colors);  // ������ɫ����

	glEnableVertexAttribArray(g_vPosition);
	glVertexAttribPointer(
		g_vPosition,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		BUFFER_OFFSET(0)
		);


	glEnableVertexAttribArray(g_vColor);
	glVertexAttribPointer(
		g_vColor,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		//sizeof(color3) + sizeof(point3),
		BUFFER_OFFSET(sizeof(point3) * g_iNumOfSetVer));
	//BUFFER_OFFSET(sizeof(point3) * 3));

	//���տռ�
	delete [] g_pSetVer;
	delete [] g_Ve3SetColors;
}

//��ʼ���������� �������ߵ������Ϊ���ߵ����� ������ʱ��洢����
void initPoly2tri(string filename)
{
	//BuildSet(filename);
	BuildSet(filename, *g_pConSet);
}

//��ʼ�������
void initPloyMesh(fstream *f)
{
	g_pPolyMesh = rcAllocPolyMesh();
	// 6 Ϊ��󶥵���
	BuildPolyMesh(*g_pConSet, 6, *g_pPolyMesh);
	int MaxIndexNum = 0;
	OutputPolyMesh(f, g_pPolyMesh, MaxIndexNum);

	g_pPolyVer = new point3[MaxIndexNum];
	g_Ve3PolyColors = new color3[MaxIndexNum];
	g_iPolyIndex = new int[g_pPolyMesh->npolys];
	int index = 0;
	g_iNumOfPoly = g_pPolyMesh->npolys;
	for (int i = 0; i < g_pPolyMesh->npolys; i ++)
	{
		//�������һ����ɫ
		color3 randColor = color3(rand() % 256, rand() % 256, rand() % 256) / 255;
		g_iPolyIndex[i] = countPolyVerts(&g_pPolyMesh->polys[i * 2 * g_pPolyMesh->nvp], g_pPolyMesh->nvp);
		for (int j = 0; j < g_iPolyIndex[i]; j ++)
		{	
			int temp = g_pPolyMesh->polys[i * 2 * g_pPolyMesh->nvp + j];
			g_pPolyVer[index].x =  g_pPolyMesh->verts[3 * temp + 0];
			g_pPolyVer[index].y =  wh - g_pPolyMesh->verts[3 * temp + 2];
			g_pPolyVer[index].z =  0.0f;
			g_Ve3PolyColors[index ++] = randColor;
		}
	}

	glGenVertexArrays(1, &g_uiVaoPloyMesh);
	glBindVertexArray(g_uiVaoPloyMesh);

	GLuint bufferPoly;
	glGenBuffers(1, &bufferPoly);
	glBindBuffer(GL_ARRAY_BUFFER, bufferPoly);

	glBufferData(GL_ARRAY_BUFFER,
		sizeof(point3) * 2 * MaxIndexNum,
		NULL,
		GL_STATIC_DRAW
		);
	//gai
	// 	glBufferData(GL_ARRAY_BUFFER,
	// 		(sizeof(point3) + sizeof(color3)) * g_iNumOfVer,
	// 		NULL,
	// 		GL_STATIC_DRAW
	// 		);

	/*�ֱ��������*/
	glBufferSubData(GL_ARRAY_BUFFER, 0, 
		sizeof(point3) * MaxIndexNum,
		g_pPolyVer);	// ���ض���λ������
	//sizeof(point3) * 3, g_pVer);	
	glBufferSubData(GL_ARRAY_BUFFER, 
		sizeof(point3) * MaxIndexNum, 
		//sizeof(point3) * 3,
		sizeof(color3) * MaxIndexNum,
		g_Ve3PolyColors);  // ������ɫ����
	//sizeof(point3) * 3, g_Ve3Colors);  // ������ɫ����

	glEnableVertexAttribArray(g_vPosition);
	glVertexAttribPointer(
		g_vPosition,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		BUFFER_OFFSET(0)
		);


	glEnableVertexAttribArray(g_vColor);
	glVertexAttribPointer(
		g_vColor,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		//sizeof(color3) + sizeof(point3),
		BUFFER_OFFSET(sizeof(point3) * MaxIndexNum));
	//BUFFER_OFFSET(sizeof(point3) * 3));

	//���տռ�
	delete [] g_pPolyVer;
	delete [] g_Ve3PolyColors;
}

//��ʼ��·��Vao
void initPath()
{
	glGenVertexArrays(1, &g_uiVaoPath);
	glBindVertexArray(g_uiVaoPath);

	GLuint bufferPath;
	glGenBuffers(1, &bufferPath);
	glBindBuffer(GL_ARRAY_BUFFER, bufferPath);
	glBufferData(GL_ARRAY_BUFFER,
		(sizeof(point3) + sizeof(color3)) * g_NumOfPath * 2,
		NULL,
		GL_STATIC_DRAW
		);

	glEnableVertexAttribArray(g_vPosition);
	glVertexAttribPointer(
		g_vPosition,
		3,
		GL_FLOAT,
		GL_FALSE,
		sizeof(color3) + sizeof(point3),
		BUFFER_OFFSET(0)
		);


	glEnableVertexAttribArray(g_vColor);
	glVertexAttribPointer(
		g_vColor,
		3,
		GL_FLOAT,
		GL_FALSE,
		sizeof(color3) + sizeof(point3),
		//sizeof(color3) + sizeof(point3),
		BUFFER_OFFSET(sizeof(point3))
		);
}

void init(void)
{

	GLuint program = InitShader("vSquare.glsl", "fSquare.glsl");
	glUseProgram(program);

	g_vPosition = glGetAttribLocation(program, "vPosition");
	g_vColor = glGetAttribLocation(program, "vColor");

	MVPMatrix = glGetUniformLocation(program, "MVPMatrix");
	//mat4 matProj = Ortho2D(0, ww, 0, wh);
	//glUniformMatrix4fv(MVPMatrix, 1, true, matProj);


	//ת����������
	g_pConSet = rcAllocContourSet();
	if (!g_pConSet)
	{
		printf("��ʼ�� ConSet ����\n");
		//return false;
	}
	initPoly2tri("Sample.txt");

	//��ʼ������
	fstream MyFileStream;
	MyFileStream.open("test_in.txt", ios::in);
	
	if (MyFileStream)
	{	
		initSet(&MyFileStream);
	}
	else 
	{
		printf("��ȡConSet ʧ��\n");
	}
	MyFileStream.close();

	//��ʼ�����������
	MyFileStream.open("test_PolyMesh_out.txt", ios::out);
	initPloyMesh(&MyFileStream);
	MyFileStream.close();

	//��ʼ�����ƶ���ε�����
	g_iDrawPolyNum = g_iNumOfPoly;

// 	int *an = new int[g_pPolyMesh->npolys];
// 	CalculateArea(g_pPolyMesh, an);
//  Dijkstra���·
#if 1
	//�����ڽ�
	g_pContiguous = new int*[g_pPolyMesh->npolys];
	for (int i = 0; i < g_pPolyMesh->npolys; i ++)
	{
		g_pContiguous[i] = new int[g_pPolyMesh->npolys];
	}

	//test out con
	MyFileStream.open("Test_out_con.txt", ios::out);
	for (int j = 0; j < g_pPolyMesh->npolys; j ++)
	{
		for (int k = 0; k < g_pPolyMesh->npolys; k ++)
		{
			g_pContiguous[j][k] = 0xfffffff; //Ĭ�����ֵ ��ʾ���ɴ�
			MyFileStream << g_pContiguous[j][k] << " ";
		}
		MyFileStream << endl;
	}
	MyFileStream.close();

	CalculateContiguous(g_pPolyMesh, g_pContiguous);

	MyFileStream.open("Test_out_con2.txt", ios::out);
	for (int j = 0; j < g_pPolyMesh->npolys; j ++)
	{
		for (int k = 0; k < g_pPolyMesh->npolys; k ++)
		{
			MyFileStream << setw(9)  << g_pContiguous[j][k] << " ";
		}
		MyFileStream << endl;
	}
	MyFileStream.close();
#endif

// A*
#if 1
	//�����ڽ�
	g_bContiguous = new bool*[g_pPolyMesh->npolys];
	for (int i = 0; i < g_pPolyMesh->npolys; i ++)
	{
		g_bContiguous[i] = new bool[g_pPolyMesh->npolys];
	}
	//test out con
	MyFileStream.open("Test_out_AStar_con.txt", ios::out);
	for (int j = 0; j < g_pPolyMesh->npolys; j ++)
	{
		for (int k = 0; k < g_pPolyMesh->npolys; k ++)
		{
			g_bContiguous[j][k] = false; //Ĭ�����ֵ ��ʾ���ɴ�
			MyFileStream << g_bContiguous[j][k] << " ";
		}
		MyFileStream << endl;
	}
	MyFileStream.close();

	g_Ve3Centre = new vec3[g_pPolyMesh->npolys];
	CalculateContiguousAndCentre(g_pPolyMesh, g_bContiguous, g_Ve3Centre);

	MyFileStream.open("Test_out_AStar_con2.txt", ios::out);
	for (int j = 0; j < g_pPolyMesh->npolys; j ++)
	{
		for (int k = 0; k < g_pPolyMesh->npolys; k ++)
		{
			MyFileStream << g_bContiguous[j][k] << " ";
		}
		MyFileStream << endl;
	}
	MyFileStream.close();
	
	// �������
	MyFileStream.open("Test_out_AStar_Centre.txt", ios::out);
	for (int j = 0; j < g_pPolyMesh->npolys; j ++)
	{
		MyFileStream << g_Ve3Centre[j] << endl;
	}
	MyFileStream.close();

#endif


	initPath();
#if 0
	//����·��
	int *distance = new int[g_pPolyMesh->npolys];
	int *path = new int[g_pPolyMesh->npolys];
	Dijkstra(g_pContiguous, g_pPolyMesh->npolys, 0, distance, path);

	//test Dijkstra
	int ePoint = 4;
	while (path[ePoint] != -1)
	{	
		printf("%d\n", ePoint);
		if(path[ePoint] == ePoint) break;
		ePoint = path[ePoint];
	}
#endif

	glClearColor(0.0, 0.0, 0.0, 1.0);
}

// ������Դ
void MyEnd()
{ 
	delete[] g_iConIndex;       //��������������


	delete[] g_iPolyIndex;         //��������������

	//�ڽӱ�
	int n = g_pPolyMesh->npolys;
	for (int i = 0; i < n; i ++)
	{
		//g_pContiguous = new int*[g_pPolyMesh->npolys];
		delete [] g_pContiguous[i];
		delete [] g_bContiguous[i];
	}
	delete [] g_pContiguous;
	delete [] g_bContiguous;

	delete [] g_Ve3Centre; //����ε�����

	rcFreeContourSet(g_pConSet);
	rcFreePolyMesh(g_pPolyMesh);

}

int main(int argc, char ** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
	glutInitContextVersion(3, 1);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitWindowSize(ww, wh);
	glutCreateWindow("NavMeshOpengl");
	glewExperimental = GL_TRUE;//N��������� �������в���Opengl����
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		std::cout << "glewInit ʧ�ܣ��˳�����" << std::endl;
		exit(EXIT_FAILURE);
	}


	//ע��ص�����
	glutDisplayFunc(display);
	glutMouseFunc(mymouse);
	//glutMotionFunc(addSquare);
	glutKeyboardFunc(MyKeyDown);
	glutKeyboardUpFunc(MyKeyUp);
	glutReshapeFunc(myReshape);
	
	init();
	glutMainLoop();

	MyEnd();
}