// OpenglSample1.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <GL\glut.h>  

void renderScene(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glBegin(GL_TRIANGLES);
	glVertex3f(-0.5, -0.5, 0.0);
	glVertex3f(0.5, 0.0, 0.0);
	glVertex3f(0.0, 0.5, 0.0);
	glEnd();
	glutSwapBuffers();
}

int _tmain(int argc, _TCHAR* argv[])
{
	glutInit(&argc, (char**)argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(320, 320);
	glutCreateWindow("Hello OpenGL");
	glutDisplayFunc(renderScene);
	glutMainLoop();
	return 0;
}
