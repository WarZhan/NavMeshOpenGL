//////////////////////////////////////////////////////////////////////////////
//
//  --- Angel.h ---
//
//   The main header file for all examples from Angel 6th Edition
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ANGEL_H__
#define __ANGEL_H__

//----------------------------------------------------------------------------
// 
// --- Include system headers ---
//

#include <cmath>	 // ����C++��ѧ��
#include <iostream>  // ����C++��׼���������

//  Define M_PI in the case it's not defined in the math header file
#ifndef M_PI
#  define M_PI  3.14159265358979323846
#endif

//----------------------------------------------------------------------------
//
// --- Include OpenGL header files and helpers ---
//
//   The location of these files vary by operating system.  We've included
//     copies of open-soruce project headers in the "GL" directory local
//     this this "include" directory.
//

#ifdef __APPLE__  // include Mac OS X verions of headers
#  include <OpenGL/OpenGL.h>
#  include <GLUT/glut.h>
#else // non-Mac OS X operating systems
#  include <GL/glew.h>
#  include <GL/freeglut.h>
#  include <GL/freeglut_ext.h>
#pragma comment (lib, "glew32.lib")  // Zou Kun�������ӣ���������glew��
#endif  // __APPLE__

// Define a helpful macro for handling offsets into buffer objects
// ����buffer����ƫ�����꣬��Ҫʵ������ת��
#define BUFFER_OFFSET( offset )   ((GLvoid*) (offset))

//----------------------------------------------------------------------------
//
//  --- Include our class libraries and constants ---
//

namespace Angel {

//  �������ض����ƬԪshader�ĺ������˺���������InitShader.cpp��
GLuint InitShader( const char* vertexShaderFile,
		   const char* fragmentShaderFile );

//  ������С����������ֹ��0��
const GLfloat  DivideByZeroTolerance = GLfloat(1.0e-07);

//  �Ƕ�ת���ȵ�ϵ�� 
const GLfloat  DegreesToRadians = M_PI / 180.0;

}  // namespace Angel

/*�����Զ����ͷ�ļ�*/
#include "vec.h"
#include "mat.h"

// ��ӡ����꣬����#x��ʾ��xתΪ�ַ�����
#define Print(x)  do { std::cerr << #x " = " << (x) << std::endl; } while(0)

//  ʹ��Angel�����ռ�
using namespace Angel;

#endif // __ANGEL_H__
