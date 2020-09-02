
#include "Angel.h"

namespace Angel {

// 返回从文件中读取的字符串(以NULL结尾)
static char*   // static修饰使得此函数仅在此文件可见
readShaderSource(const char* shaderFile)
{
    FILE* fp = fopen(shaderFile, "r"); // 以只读方式打开文件

    if ( fp == NULL ) { return NULL; } 

    fseek(fp, 0L, SEEK_END); // 将文件指针移到文件尾
    long size = ftell(fp);   // 返回当前文件指针位置，此时对应文件长度

    fseek(fp, 0L, SEEK_SET); // 将文件指针移到文件头
    char* buf = new char[size + 1]; // 根据size创建buffer，+1是为了留位置给'\0'
	memset(buf, 0, size+1);	 // 清空buf
    fread(buf, 1, size, fp); // 将文件内容一次全部读出

    buf[size] = '\0'; // 最后为空字符
    fclose(fp);		  // 关闭文件

    return buf;		  // 返回读取到的字符串
}


// 根据给定的顶点和片元shader文件创建GLSL程序对象
GLuint
InitShader(const char* vShaderFile, const char* fShaderFile)
{
    struct Shader {
		const char*  filename; // shader文件名
		GLenum       type;     // shader类型
		GLchar*      source;   // shader程序字符串
	} shaders[2] = {
		{ vShaderFile, GL_VERTEX_SHADER, NULL },
		{ fShaderFile, GL_FRAGMENT_SHADER, NULL }
	}; // 定义Shader结构体数组shaders

    GLuint program = glCreateProgram(); // 创建shader程序对象，返回其ID
    
    for ( int i = 0; i < 2; ++i ) { // 分别对顶点shader和片元shader进程处理
		Shader& s = shaders[i];		// s是shader[i]的引用
		s.source = readShaderSource( s.filename ); // 从文件读取程序内容(字符串)
		if ( shaders[i].source == NULL ) { // 读取失败？
			std::cerr << "Failed to read " << s.filename << std::endl;
			system("pause");
			exit( EXIT_FAILURE );
		}

		// 创建shader对象，参数为shader类型(GL_VERTEX_SHADER或GL_FRAGMENT_SHADER)，返回shader对象ID
		GLuint shader = glCreateShader( s.type ); 
		// 为shader对象指定shader源码
		glShaderSource( shader,	// shader对象ID 
					1,		// 参数3中含有的字符串个数
					(const GLchar**) &s.source, // 含有源码的字符串数组
					NULL	// 字符串长度数组(成员为参数3中各字符串长度)，为NULL表示各字符串均以NULL结束)
					);
		glCompileShader( shader ); // 编译shader程序

		GLint  compiled;
		// 获取编译状态信息
		glGetShaderiv( shader,	// shader对象ID
			GL_COMPILE_STATUS,  // 表明获取的是编译状态信息
			&compiled			// 输出参数，用于存储信息(这里可能返回GL_TRUE或GL_FALSE)
			);
		if ( !compiled ) {		// 如果编译失败
			std::cerr << s.filename << " failed to compile:" << std::endl; // 输出错误信息
			GLint  logSize;
			glGetShaderiv( shader, GL_INFO_LOG_LENGTH, &logSize ); // 获取错误信息的长度
			char* logMsg = new char[logSize];  // 根据信息长度创建buffer
			// 获取错误信息
			glGetShaderInfoLog( shader, // shader对象ID
				logSize,	// 用于存储信息的buffer(这里即logMsg)的大小
				NULL,       // 输出参数，存储获取到的信息长度的变量指针
				logMsg      // 用于保存信息的字符buffer
				); 
			std::cerr << logMsg << std::endl;	// 输出错误信息
			delete [] logMsg;
			system("pause");
			exit( EXIT_FAILURE );	// 退出程序
		}

		delete [] s.source;

		glAttachShader( program, shader ); // 将shader对象与program对象关联起来
    }

    /* 链接并检查错误信息 */
    glLinkProgram(program);	// 链接shader程序

    GLint  linked;
    glGetProgramiv( program, GL_LINK_STATUS, &linked ); // 获取链接信息，参数含义和glGetShaderiv类似
    if ( !linked ) {	// 链接失败？
		std::cerr << "Shader program failed to link" << std::endl;
		GLint  logSize;
		glGetProgramiv( program, GL_INFO_LOG_LENGTH, &logSize);	// 获取链接信息的长度
		char* logMsg = new char[logSize];
		glGetProgramInfoLog( program, logSize, NULL, logMsg );	// 获取链接信息
		std::cerr << logMsg << std::endl;
		delete [] logMsg;
		system("pause");
		exit( EXIT_FAILURE );
    }

    return program;
}

}  // 结束命名空间为Angel的代码块
