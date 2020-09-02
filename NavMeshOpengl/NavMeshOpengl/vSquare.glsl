#version 140

in vec3 vPosition;
in vec3 vColor;

uniform mat4 MVPMatrix;

out vec4 color;

void main()
{
	gl_Position = MVPMatrix * vec4(vPosition, 1.0);
	color = vec4(vColor, 1.0);
}