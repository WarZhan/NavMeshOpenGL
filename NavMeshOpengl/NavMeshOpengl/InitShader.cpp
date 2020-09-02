
#include "Angel.h"

namespace Angel {

// ���ش��ļ��ж�ȡ���ַ���(��NULL��β)
static char*   // static����ʹ�ô˺������ڴ��ļ��ɼ�
readShaderSource(const char* shaderFile)
{
    FILE* fp = fopen(shaderFile, "r"); // ��ֻ����ʽ���ļ�

    if ( fp == NULL ) { return NULL; } 

    fseek(fp, 0L, SEEK_END); // ���ļ�ָ���Ƶ��ļ�β
    long size = ftell(fp);   // ���ص�ǰ�ļ�ָ��λ�ã���ʱ��Ӧ�ļ�����

    fseek(fp, 0L, SEEK_SET); // ���ļ�ָ���Ƶ��ļ�ͷ
    char* buf = new char[size + 1]; // ����size����buffer��+1��Ϊ����λ�ø�'\0'
	memset(buf, 0, size+1);	 // ���buf
    fread(buf, 1, size, fp); // ���ļ�����һ��ȫ������

    buf[size] = '\0'; // ���Ϊ���ַ�
    fclose(fp);		  // �ر��ļ�

    return buf;		  // ���ض�ȡ�����ַ���
}


// ���ݸ����Ķ����ƬԪshader�ļ�����GLSL�������
GLuint
InitShader(const char* vShaderFile, const char* fShaderFile)
{
    struct Shader {
		const char*  filename; // shader�ļ���
		GLenum       type;     // shader����
		GLchar*      source;   // shader�����ַ���
	} shaders[2] = {
		{ vShaderFile, GL_VERTEX_SHADER, NULL },
		{ fShaderFile, GL_FRAGMENT_SHADER, NULL }
	}; // ����Shader�ṹ������shaders

    GLuint program = glCreateProgram(); // ����shader������󣬷�����ID
    
    for ( int i = 0; i < 2; ++i ) { // �ֱ�Զ���shader��ƬԪshader���̴���
		Shader& s = shaders[i];		// s��shader[i]������
		s.source = readShaderSource( s.filename ); // ���ļ���ȡ��������(�ַ���)
		if ( shaders[i].source == NULL ) { // ��ȡʧ�ܣ�
			std::cerr << "Failed to read " << s.filename << std::endl;
			system("pause");
			exit( EXIT_FAILURE );
		}

		// ����shader���󣬲���Ϊshader����(GL_VERTEX_SHADER��GL_FRAGMENT_SHADER)������shader����ID
		GLuint shader = glCreateShader( s.type ); 
		// Ϊshader����ָ��shaderԴ��
		glShaderSource( shader,	// shader����ID 
					1,		// ����3�к��е��ַ�������
					(const GLchar**) &s.source, // ����Դ����ַ�������
					NULL	// �ַ�����������(��ԱΪ����3�и��ַ�������)��ΪNULL��ʾ���ַ�������NULL����)
					);
		glCompileShader( shader ); // ����shader����

		GLint  compiled;
		// ��ȡ����״̬��Ϣ
		glGetShaderiv( shader,	// shader����ID
			GL_COMPILE_STATUS,  // ������ȡ���Ǳ���״̬��Ϣ
			&compiled			// ������������ڴ洢��Ϣ(������ܷ���GL_TRUE��GL_FALSE)
			);
		if ( !compiled ) {		// �������ʧ��
			std::cerr << s.filename << " failed to compile:" << std::endl; // ���������Ϣ
			GLint  logSize;
			glGetShaderiv( shader, GL_INFO_LOG_LENGTH, &logSize ); // ��ȡ������Ϣ�ĳ���
			char* logMsg = new char[logSize];  // ������Ϣ���ȴ���buffer
			// ��ȡ������Ϣ
			glGetShaderInfoLog( shader, // shader����ID
				logSize,	// ���ڴ洢��Ϣ��buffer(���ＴlogMsg)�Ĵ�С
				NULL,       // ����������洢��ȡ������Ϣ���ȵı���ָ��
				logMsg      // ���ڱ�����Ϣ���ַ�buffer
				); 
			std::cerr << logMsg << std::endl;	// ���������Ϣ
			delete [] logMsg;
			system("pause");
			exit( EXIT_FAILURE );	// �˳�����
		}

		delete [] s.source;

		glAttachShader( program, shader ); // ��shader������program�����������
    }

    /* ���Ӳ���������Ϣ */
    glLinkProgram(program);	// ����shader����

    GLint  linked;
    glGetProgramiv( program, GL_LINK_STATUS, &linked ); // ��ȡ������Ϣ�����������glGetShaderiv����
    if ( !linked ) {	// ����ʧ�ܣ�
		std::cerr << "Shader program failed to link" << std::endl;
		GLint  logSize;
		glGetProgramiv( program, GL_INFO_LOG_LENGTH, &logSize);	// ��ȡ������Ϣ�ĳ���
		char* logMsg = new char[logSize];
		glGetProgramInfoLog( program, logSize, NULL, logMsg );	// ��ȡ������Ϣ
		std::cerr << logMsg << std::endl;
		delete [] logMsg;
		system("pause");
		exit( EXIT_FAILURE );
    }

    return program;
}

}  // ���������ռ�ΪAngel�Ĵ����
