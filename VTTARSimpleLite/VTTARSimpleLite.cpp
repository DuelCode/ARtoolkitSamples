// VTTARSimpleLite.cpp : 定义控制台应用程序的入口点。
//


#include <stdio.h>
#include <string.h>
#include <stdlib.h>				
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <GL/glut.h>

#define VIEW_SCALEFACTOR		1.0         // Units received from ARToolKit tracking will be multiplied by this factor before being used in OpenGL drawing.
#define VIEW_DISTANCE_MIN		40.0        // Objects closer to the camera than this will not be displayed. OpenGL units.
#define VIEW_DISTANCE_MAX		10000.0     // Objects further away from the camera than this will not be displayed. OpenGL units.

// Preferences.
static int windowed = TRUE;                     // Use windowed (TRUE) or fullscreen mode (FALSE) on launch.
static int windowWidth = 640;					// Initial window width, also updated during program execution.
static int windowHeight = 480;                  // Initial window height, also updated during program execution.
static int windowDepth = 32;					// Fullscreen mode bit depth.
static int windowRefresh = 0;					// Fullscreen mode refresh rate. Set to 0 to use default rate.

// Image acquisition.
static ARUint8		*gARTImage = NULL;
static int          gARTImageSavePlease = FALSE;

// Marker detection.
static ARHandle		*gARHandle = NULL;
static ARPattHandle	*gARPattHandle = NULL;
static long			gCallCountMarkerDetect = 0;

// Transformation matrix retrieval.
static AR3DHandle	*gAR3DHandle = NULL;
static ARdouble		gPatt_width = 80.0;	// Per-marker, but we are using only 1 marker.
static ARdouble		gPatt_trans[3][4];		// Per-marker, but we are using only 1 marker.
static int			gPatt_found = FALSE;	// Per-marker, but we are using only 1 marker.
static int			gPatt_id;				// Per-marker, but we are using only 1 marker.

											// Drawing.
static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gShowHelp = 1;
static int gShowMode = 1;
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 0;			// For use in drawing.

static void drawBackground(const float width, const float height, const float x, const float y);


static void DrawCube(void)
{
	int i;
	float fSize = 40.0f;
	const GLfloat cube_vertices[8][3] = {
		/* +z */{ 0.5f, 0.5f, 0.5f },{ 0.5f, -0.5f, 0.5f },{ -0.5f, -0.5f, 0.5f },{ -0.5f, 0.5f, 0.5f },
		/* -z */{ 0.5f, 0.5f, -0.5f },{ 0.5f, -0.5f, -0.5f },{ -0.5f, -0.5f, -0.5f },{ -0.5f, 0.5f, -0.5f } };
	const GLubyte cube_vertex_colors[8][4] = {
		{ 255, 255, 255, 255 },{ 255, 255, 0, 255 },{ 0, 255, 0, 255 },{ 0, 255, 255, 255 },
		{ 255, 0, 255, 255 },{ 255, 0, 0, 255 },{ 0, 0, 0, 255 },{ 0, 0, 255, 255 } };
	const GLubyte cube_faces[6][4] = { /* ccw-winding */
		/* +z */{ 3, 2, 1, 0 }, /* -y */{ 2, 3, 7, 6 }, /* +y */{ 0, 1, 5, 4 },
		/* -x */{ 3, 0, 4, 7 }, /* +x */{ 1, 2, 6, 5 }, /* -z */{ 4, 5, 6, 7 } };

	glPushMatrix(); // Save world coordinate system.
	glRotatef(gDrawRotateAngle, 0.0f, 0.0f, 1.0f); // Rotate about z axis.
	glScalef(fSize, fSize, fSize);
	glTranslatef(0.0f, 0.0f, 0.5f); // Place base of cube on marker surface.
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, cube_vertex_colors);
	glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	for (i = 0; i < 6; i++) {
		glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
	}
	glDisableClientState(GL_COLOR_ARRAY);
	glColor4ub(0, 0, 0, 255);
	for (i = 0; i < 6; i++) {
		glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
	}
	glPopMatrix();    // Restore world coordinate system.
}

static void DrawCubeUpdate(float timeDelta)
{
	if (gDrawRotate) {
		gDrawRotateAngle += timeDelta * 45.0f; // Rotate cube at 45 degrees per second.
		if (gDrawRotateAngle > 360.0f) gDrawRotateAngle -= 360.0f;
	}
}

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle)
{
	ARParam			cparam;
	int				xsize, ysize;
	AR_PIXEL_FORMAT pixFormat;

	if (arVideoOpen(vconf) < 0) {
		ARLOGe("setupCamera(): Unable to open connection to camera.\n");
		return (FALSE);
	}

	if (arVideoGetSize(&xsize, &ysize) < 0) {
		ARLOGe("setupCamera(): Unable to determine camera frame size.\n");
		arVideoClose();
		return (FALSE);
	}
	ARLOGi("Camera image size (x,y) = (%d,%d)\n", xsize, ysize);

	pixFormat = arVideoGetPixelFormat();
	if (pixFormat == AR_PIXEL_FORMAT_INVALID) {
		ARLOGe("setupCamera(): Camera is using unsupported pixel format.\n");
		arVideoClose();
		return (FALSE);
	}

	if (arParamLoad(cparam_name, 1, &cparam) < 0) {
		ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
		arVideoClose();
		return (FALSE);
	}
	if (cparam.xsize != xsize || cparam.ysize != ysize) {
		ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
		arParamChangeSize(&cparam, xsize, ysize, &cparam);
	}

	if ((*cparamLT_p = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
		ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
		return (FALSE);
	}

	if ((*arhandle = arCreateHandle(*cparamLT_p)) == NULL) {
		ARLOGe("setupCamera(): Error: arCreateHandle.\n");
		return (FALSE);
	}
	if (arSetPixelFormat(*arhandle, pixFormat) < 0) {
		ARLOGe("setupCamera(): Error: arSetPixelFormat.\n");
		return (FALSE);
	}
	if (arSetDebugMode(*arhandle, AR_DEBUG_DISABLE) < 0) {
		ARLOGe("setupCamera(): Error: arSetDebugMode.\n");
		return (FALSE);
	}
	if ((*ar3dhandle = ar3DCreateHandle(&cparam)) == NULL) {
		ARLOGe("setupCamera(): Error: ar3DCreateHandle.\n");
		return (FALSE);
	}

	if (arVideoCapStart() != 0) {
		ARLOGe("setupCamera(): Unable to begin camera data capture.\n");
		return (FALSE);
	}

	return (TRUE);
}

static int setupMarker(const char *patt_name, int *patt_id, ARHandle *arhandle, ARPattHandle **pattHandle_p)
{
	if ((*pattHandle_p = arPattCreateHandle()) == NULL) {
		ARLOGe("setupMarker(): Error: arPattCreateHandle.\n");
		return (FALSE);
	}

	// Loading only 1 pattern in this example.
	if ((*patt_id = arPattLoad(*pattHandle_p, patt_name)) < 0) {
		ARLOGe("setupMarker(): Error loading pattern file %s.\n", patt_name);
		arPattDeleteHandle(*pattHandle_p);
		return (FALSE);
	}

	arPattAttach(arhandle, *pattHandle_p);

	return (TRUE);
}

static void cleanup(void)
{
	arglCleanup(gArglSettings);
	gArglSettings = NULL;
	arPattDetach(gARHandle);
	arPattDeleteHandle(gARPattHandle);
	arVideoCapStop();
	ar3DDeleteHandle(&gAR3DHandle);
	arDeleteHandle(gARHandle);
	arParamLTFree(&gCparamLT);
	arVideoClose();
}

static void Keyboard(unsigned char key, int x, int y)
{
	int mode, threshChange = 0;
	AR_LABELING_THRESH_MODE modea;

	switch (key) {
	case 0x1B:						// Quit.
	case 'Q':
	case 'q':
		cleanup();
		exit(0);
		break;
	case ' ':
		gDrawRotate = !gDrawRotate;
		break;
	case 'X':
	case 'x':
		arGetImageProcMode(gARHandle, &mode);
		switch (mode) {
		case AR_IMAGE_PROC_FRAME_IMAGE:  mode = AR_IMAGE_PROC_FIELD_IMAGE; break;
		case AR_IMAGE_PROC_FIELD_IMAGE:
		default: mode = AR_IMAGE_PROC_FRAME_IMAGE; break;
		}
		arSetImageProcMode(gARHandle, mode);
		break;
	case 'C':
	case 'c':
		ARLOGe("*** Camera - %f (frame/sec)\n", (double)gCallCountMarkerDetect / arUtilTimer());
		gCallCountMarkerDetect = 0;
		arUtilTimerReset();
		break;
	case 'a':
	case 'A':
		arGetLabelingThreshMode(gARHandle, &modea);
		switch (modea) {
		case AR_LABELING_THRESH_MODE_MANUAL:        modea = AR_LABELING_THRESH_MODE_AUTO_MEDIAN; break;
		case AR_LABELING_THRESH_MODE_AUTO_MEDIAN:   modea = AR_LABELING_THRESH_MODE_AUTO_OTSU; break;
		case AR_LABELING_THRESH_MODE_AUTO_OTSU:     modea = AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE; break;
		case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: modea = AR_LABELING_THRESH_MODE_AUTO_BRACKETING; break;
		case AR_LABELING_THRESH_MODE_AUTO_BRACKETING:
		default: modea = AR_LABELING_THRESH_MODE_MANUAL; break;
		}
		arSetLabelingThreshMode(gARHandle, modea);
		break;
	case '-':
		threshChange = -5;
		break;
	case '+':
	case '=':
		threshChange = +5;
		break;
	case 'D':
	case 'd':
		arGetDebugMode(gARHandle, &mode);
		arSetDebugMode(gARHandle, !mode);
		break;
	case 's':
	case 'S':
		if (!gARTImageSavePlease) gARTImageSavePlease = TRUE;
		break;
	case '?':
	case '/':
		gShowHelp++;
		if (gShowHelp > 1) gShowHelp = 0;
		break;
	case 'm':
	case 'M':
		gShowMode = !gShowMode;
		break;
	default:
		break;
	}
	if (threshChange) {
		int threshhold;
		arGetLabelingThresh(gARHandle, &threshhold);
		threshhold += threshChange;
		if (threshhold < 0) threshhold = 0;
		if (threshhold > 255) threshhold = 255;
		arSetLabelingThresh(gARHandle, threshhold);
	}

}

static void mainLoop(void)
{
	static int imageNumber = 0;
	static int ms_prev;
	int ms;
	float s_elapsed;
	ARUint8 *image;
	ARdouble err;

	int             j, k;

	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	if (s_elapsed < 0.01f) return; // Don't update more often than 100 Hz.
	ms_prev = ms;

	// Update drawing.
	DrawCubeUpdate(s_elapsed);

	// Grab a video frame.
	if ((image = arVideoGetImage()) != NULL) {
		gARTImage = image;	// Save the fetched image.

		if (gARTImageSavePlease) {
			char imageNumberText[15];
			sprintf(imageNumberText, "image-%04d.jpg", imageNumber++);
			if (arVideoSaveImageJPEG(gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, gARTImage, imageNumberText, 75, 0) < 0) {
				ARLOGe("Error saving video image.\n");
			}
			gARTImageSavePlease = FALSE;
		}

		gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

								  // Detect the markers in the video frame.
		if (arDetectMarker(gARHandle, gARTImage) < 0) {
			exit(-1);
		}

		// Check through the marker_info array for highest confidence
		// visible marker matching our preferred pattern.
		k = -1;
		for (j = 0; j < gARHandle->marker_num; j++) {
			if (gARHandle->markerInfo[j].id == gPatt_id) {
				if (k == -1) k = j; // First marker detected.
				else if (gARHandle->markerInfo[j].cf > gARHandle->markerInfo[k].cf) k = j; // Higher confidence marker detected.
			}
		}

		if (k != -1) {
			// Get the transformation between the marker and the real camera into gPatt_trans.
			err = arGetTransMatSquare(gAR3DHandle, &(gARHandle->markerInfo[k]), gPatt_width, gPatt_trans);
			gPatt_found = TRUE;
		}
		else {
			gPatt_found = FALSE;
		}

		// Tell GLUT the display has changed.
		glutPostRedisplay();
	}
}

static void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) {
		glutIdleFunc(mainLoop);
	}
	else {
		glutIdleFunc(NULL);
	}
}

static void Reshape(int w, int h)
{
	windowWidth = w;
	windowHeight = h;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
}

static void Display(void)
{
	ARdouble p[16];
	ARdouble m[16];

	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.

	arglPixelBufferDataUpload(gArglSettings, gARTImage);
	arglDispImage(gArglSettings);
	gARTImage = NULL; // Invalidate image data.

					  // Projection transformation.
	arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, p);
	glMatrixMode(GL_PROJECTION);
#ifdef ARDOUBLE_IS_FLOAT
	glLoadMatrixf(p);
#else
	glLoadMatrixd(p);
#endif
	glMatrixMode(GL_MODELVIEW);

	glEnable(GL_DEPTH_TEST);

	// Viewing transformation.
	glLoadIdentity();
	// Lighting and geometry that moves with the camera should go here.
	// (I.e. must be specified before viewing transformations.)
	//none

	if (gPatt_found) {

		// Calculate the camera position relative to the marker.
		// Replace VIEW_SCALEFACTOR with 1.0 to make one drawing unit equal to 1.0 ARToolKit units (usually millimeters).
		arglCameraViewRH((const ARdouble(*)[4])gPatt_trans, m, VIEW_SCALEFACTOR);
#ifdef ARDOUBLE_IS_FLOAT
		glLoadMatrixf(m);
#else
		glLoadMatrixd(m);
#endif

		// All lighting and geometry to be drawn relative to the marker goes here.
		DrawCube();

	} // gPatt_found

	  // Any 2D overlays go here.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, (GLdouble)windowWidth, 0, (GLdouble)windowHeight, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	glutSwapBuffers();
}

int main(int argc, char** argv)
{
	char glutGamemode[32];
	//char cparam_name[] = "Data/camera_para.dat";
	char cparam_name[] = "C:/WPFWorkspace/LiuHao/Demos/VTTARtoolkitSamples/Debug/Data/camera_para.dat";
	char vconf[] = "";
	//char patt_name[] = "Data/hiro.patt";
	char patt_name[] = "C:/WPFWorkspace/LiuHao/Demos/VTTARtoolkitSamples/Debug/Data/hiro.patt";

	glutInit(&argc, argv);

	if (!setupCamera(cparam_name, vconf, &gCparamLT, &gARHandle, &gAR3DHandle)) {
		ARLOGe("main(): Unable to set up AR camera.\n");

		exit(-1);
	}

	// Set up GL context(s) for OpenGL to draw into.
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	if (!windowed) {
		if (windowRefresh) sprintf(glutGamemode, "%ix%i:%i@%i", windowWidth, windowHeight, windowDepth, windowRefresh);
		else sprintf(glutGamemode, "%ix%i:%i", windowWidth, windowHeight, windowDepth);
		glutGameModeString(glutGamemode);
		glutEnterGameMode();
	}
	else {
		glutInitWindowSize(windowWidth, windowHeight);
		glutCreateWindow(argv[0]);
	}

	// Setup ARgsub_lite library for current OpenGL context.
	if ((gArglSettings = arglSetupForCurrentContext(&(gCparamLT->param), arVideoGetPixelFormat())) == NULL) {
		ARLOGe("main(): arglSetupForCurrentContext() returned error.\n");
		cleanup();
		exit(-1);
	}
	arglSetupDebugMode(gArglSettings, gARHandle);
	arUtilTimerReset();

	// Load marker(s).
	if (!setupMarker(patt_name, &gPatt_id, gARHandle, &gARPattHandle)) {
		ARLOGe("main(): Unable to set up AR marker.\n");
		cleanup();
		exit(-1);
	}

	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutVisibilityFunc(Visibility);
	glutKeyboardFunc(Keyboard);

	glutMainLoop();

	return (0);
}

static void drawBackground(const float width, const float height, const float x, const float y)
{
	GLfloat vertices[4][2];

	vertices[0][0] = x; vertices[0][1] = y;
	vertices[1][0] = width + x; vertices[1][1] = y;
	vertices[2][0] = width + x; vertices[2][1] = height + y;
	vertices[3][0] = x; vertices[3][1] = height + y;
	glLoadIdentity();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glVertexPointer(2, GL_FLOAT, 0, vertices);
	glEnableClientState(GL_VERTEX_ARRAY);
	glColor4f(0.0f, 0.0f, 0.0f, 0.5f);	// 50% transparent black.
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Opaque white.
									   //glLineWidth(1.0f);
									   //glDrawArrays(GL_LINE_LOOP, 0, 4);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisable(GL_BLEND);
}

