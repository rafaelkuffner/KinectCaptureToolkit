#include "glut.h"
#include "main.h"
#include <stdio.h>
void draw() {
	CalculateFrameRate();
  // drawKinectData();
   drawImageData();
   glutSwapBuffers();
}

void execute() {
    glutMainLoop();
}
void CalculateFrameRate()
{

	static float framesPerSecond = 0.0f;       // This will store our fps
	static float lastTime = 0.0f;       // This will hold the time from the last frame
	float currentTime = GetTickCount() * 0.001f;
	++framesPerSecond;
	if (currentTime - lastTime > 1.0f)
	{
		lastTime = currentTime;
		printf("\nCurrent Frames Per Second: %d\n\n", (int)framesPerSecond);
		framesPerSecond = 0;
	}
}

void keyboard(unsigned char key,int x, int y){
	if (key == 'r'){
		record(true,false,false,false);
	}
	if (key == 's'){
		record(false, false,false,false);
	}
}
bool init(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(vwidth,vheight);
    glutCreateWindow("Kinect 2 Capture");
    glutDisplayFunc(draw);
	glutKeyboardUpFunc(keyboard);
    glutIdleFunc(draw);
	glewInit();
    return true;
}
