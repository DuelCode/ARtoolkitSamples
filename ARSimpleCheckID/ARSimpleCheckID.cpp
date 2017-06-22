/*
*  check_id.c
*  ARToolKit5
*
*  Copyright 2015 Daqri, LLC.
*  Copyright 2010-2015 ARToolworks, Inc.
*
*  Author(s): Philip Lamb.
*
*/


// ============================================================================
//	Includes
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>					// malloc(), free()
#include <GL/glut.h>
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <AR/arMulti.h>

// ============================================================================
//	Constants
// ============================================================================

#define VIEW_DISTANCE_MIN		1.0			// Objects closer to the camera than this will not be displayed.
#define VIEW_DISTANCE_MAX		10000.0		// Objects further away from the camera than this will not be displayed.
#  define snprintf _snprintf

typedef struct _cutoffPhaseColours {
	int cutoffPhase;
	GLubyte colour[3];
} cutoffPhaseColours_t;

const cutoffPhaseColours_t cutoffPhaseColours[AR_MARKER_INFO_CUTOFF_PHASE_DESCRIPTION_COUNT] = {
	{ AR_MARKER_INFO_CUTOFF_PHASE_NONE,{ 0xff, 0x0,  0x0 } },  // Red.
	{ AR_MARKER_INFO_CUTOFF_PHASE_PATTERN_EXTRACTION,{ 0x95, 0xd6, 0xf6 } },  // Light blue.
	{ AR_MARKER_INFO_CUTOFF_PHASE_MATCH_GENERIC,{ 0x0,  0x0,  0xff } },  // Blue.
	{ AR_MARKER_INFO_CUTOFF_PHASE_MATCH_CONTRAST,{ 0x99, 0x66, 0x33 } },  // Brown.
	{ AR_MARKER_INFO_CUTOFF_PHASE_MATCH_BARCODE_NOT_FOUND,{ 0x7f, 0x0,  0x7f } },  // Purple.
	{ AR_MARKER_INFO_CUTOFF_PHASE_MATCH_BARCODE_EDC_FAIL,{ 0xff, 0x0,  0xff } },  // Magenta.
	{ AR_MARKER_INFO_CUTOFF_PHASE_MATCH_CONFIDENCE,{ 0x0,  0xff, 0x0 } },  // Green.
	{ AR_MARKER_INFO_CUTOFF_PHASE_POSE_ERROR,{ 0xff, 0x7f, 0x0 } },  // Orange.
	{ AR_MARKER_INFO_CUTOFF_PHASE_POSE_ERROR_MULTI,{ 0xff, 0xff, 0x0 } },  // Yellow.
	{ AR_MARKER_INFO_CUTOFF_PHASE_HEURISTIC_TROUBLESOME_MATRIX_CODES,{ 0xc6, 0xdc, 0x6a } },  // Khaki.
};

// ============================================================================
//	Global variables
// ============================================================================

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
static int          gPattSize = AR_PATT_SIZE1;
static int          gPattCountMax = AR_PATT_NUM_MAX;

// Transformation matrix retrieval.
static AR3DHandle	*gAR3DHandle = NULL;
static int          gRobustFlag = TRUE;
#define CHECK_ID_MULTIMARKERS_MAX 16
static int gMultiConfigCount = 0;
static ARMultiMarkerInfoT *gMultiConfigs[CHECK_ID_MULTIMARKERS_MAX] = { NULL };
static ARdouble gMultiErrs[CHECK_ID_MULTIMARKERS_MAX];

// Drawing.
static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gShowHelp = 1;
static int gShowMode = 1;
static GLint gViewport[4];

// ============================================================================
//	Function prototypes.
// ============================================================================

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle);
static int setupMarkers(const int patt_count, const char *patt_names[], ARMultiMarkerInfoT *multiConfigs[], ARHandle *arhandle, ARPattHandle **pattHandle_p);
static void cleanup(void);
static void Visibility(int visible);
static void Reshape(int w, int h);
static void Display(void);
static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge);
static void drawBackground(const float width, const float height, const float x, const float y);

// ============================================================================
//	Functions
// ============================================================================

int main(int argc, char** argv)
{
	char glutGamemode[32];
	char *cpara = NULL;
	char cparaDefault[] = "C:/WPFWorkspace/LiuHao/Demos/VTTARtoolkitSamples/Debug/Data/camera_para.dat";
	//char cparaDefault[] = "Data/camera_para.dat";
	char *vconf = NULL;
	int patt_names_count = 0;
	char *patt_names[CHECK_ID_MULTIMARKERS_MAX] = { NULL };
	ARdouble pattRatio = (ARdouble)AR_PATT_RATIO;
	AR_MATRIX_CODE_TYPE matrixCodeType = AR_MATRIX_CODE_TYPE_DEFAULT;
	int labelingMode = AR_DEFAULT_LABELING_MODE;
	int patternDetectionMode = AR_DEFAULT_PATTERN_DETECTION_MODE;

	glutInit(&argc, argv);

	if (!cpara) cpara = cparaDefault;
	if (!setupCamera(cpara, vconf, &gCparamLT, &gARHandle, &gAR3DHandle)) {
		ARLOGe("main(): Unable to set up AR camera.\n");
		exit(-1);
	}

	//
	// AR init.
	//

	arSetPatternDetectionMode(gARHandle, patternDetectionMode);
	arSetLabelingMode(gARHandle, labelingMode);
	arSetPattRatio(gARHandle, pattRatio);
	arSetMatrixCodeType(gARHandle, matrixCodeType);

	//
	// Graphics setup.
	//

	// Set up GL context(s) for OpenGL to draw into.
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	if (!windowed) {
		if (windowRefresh) sprintf(glutGamemode, "%ix%i:%i@%i", windowWidth, windowHeight, windowDepth, windowRefresh);
		else sprintf(glutGamemode, "%ix%i:%i", windowWidth, windowHeight, windowDepth);
		glutGameModeString(glutGamemode);
		glutEnterGameMode();
	}
	else {
		glutInitWindowSize(gCparamLT->param.xsize, gCparamLT->param.ysize);
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
	if (!setupMarkers(patt_names_count, (const char **)patt_names, gMultiConfigs, gARHandle, &gARPattHandle)) {
		ARLOGe("main(): Unable to set up AR marker(s).\n");
		cleanup();
		exit(-1);
	}
	gMultiConfigCount = patt_names_count;

	// Register GLUT event-handling callbacks.
	// NB: mainLoop() is registered by Visibility.
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutVisibilityFunc(Visibility);
	glutMainLoop();

	return (0);
}

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle)
{
	ARParam			cparam;
	int				xsize, ysize;
	AR_PIXEL_FORMAT pixFormat;

	// Open the video path.
	if (arVideoOpen(vconf) < 0) {
		ARLOGe("setupCamera(): Unable to open connection to camera.\n");
		return (FALSE);
	}

	// Find the size of the window.
	if (arVideoGetSize(&xsize, &ysize) < 0) {
		ARLOGe("setupCamera(): Unable to determine camera frame size.\n");
		arVideoClose();
		return (FALSE);
	}
	ARLOGi("Camera image size (x,y) = (%d,%d)\n", xsize, ysize);

	// Get the format in which the camera is returning pixels.
	pixFormat = arVideoGetPixelFormat();
	if (pixFormat == AR_PIXEL_FORMAT_INVALID) {
		ARLOGe("setupCamera(): Camera is using unsupported pixel format.\n");
		arVideoClose();
		return (FALSE);
	}

	// Load the camera parameters, resize for the window and init.
	if (arParamLoad(cparam_name, 1, &cparam) < 0) {
		ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
		arVideoClose();
		return (FALSE);
	}
	if (cparam.xsize != xsize || cparam.ysize != ysize) {
		ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
		arParamChangeSize(&cparam, xsize, ysize, &cparam);
	}
#ifdef DEBUG
	ARLOG("*** Camera Parameter ***\n");
	arParamDisp(&cparam);
#endif
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

static int setupMarkers(const int patt_count, const char *patt_names[], ARMultiMarkerInfoT *multiConfigs[], ARHandle *arhandle, ARPattHandle **pattHandle_p)
{
	int i;

	if (!patt_count) {
		// Default behaviour is to default to matrix mode.
		*pattHandle_p = NULL;
		arSetPatternDetectionMode(arhandle, AR_MATRIX_CODE_DETECTION); // If no markers specified, default to matrix mode.
	}
	else {
		// If marker configs have been specified, attempt to load them.

		int mode = -1, nextMode;

		// Need a pattern handle because the config file could specify matrix or template markers.
		if ((*pattHandle_p = arPattCreateHandle2(gPattSize, gPattCountMax)) == NULL) {
			ARLOGe("setupMarkers(): Error: arPattCreateHandle2.\n");
			return (FALSE);
		}

		for (i = 0; i < patt_count; i++) {

			if (!(multiConfigs[i] = arMultiReadConfigFile(patt_names[i], *pattHandle_p))) {
				ARLOGe("setupMarkers(): Error reading multimarker config file '%s'.\n", patt_names[i]);
				for (i--; i >= 0; i--) {
					arMultiFreeConfig(multiConfigs[i]);
				}
				arPattDeleteHandle(*pattHandle_p);
				return (FALSE);
			}

			if (multiConfigs[i]->patt_type == AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE) {
				nextMode = AR_TEMPLATE_MATCHING_COLOR;
			}
			else if (multiConfigs[i]->patt_type == AR_MULTI_PATTERN_DETECTION_MODE_MATRIX) {
				nextMode = AR_MATRIX_CODE_DETECTION;
			}
			else { // AR_MULTI_PATTERN_DETECTION_MODE_TEMPLATE_AND_MATRIX or mixed.
				nextMode = AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX;
			}

			if (mode == -1) {
				mode = nextMode;
			}
			else if (mode != nextMode) {
				mode = AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX;
			}
		}
		arSetPatternDetectionMode(arhandle, mode);

		arPattAttach(arhandle, *pattHandle_p);
	}

	return (TRUE);
}

static void cleanup(void)
{
	int i;

	arglCleanup(gArglSettings);
	gArglSettings = NULL;

	arPattDetach(gARHandle);
	for (i = 0; i < gMultiConfigCount; i++) {
		arMultiFreeConfig(gMultiConfigs[i]);
	}
	if (gARPattHandle) arPattDeleteHandle(gARPattHandle);

	arVideoCapStop();
	arDeleteHandle(gARHandle);
	arParamLTFree(&gCparamLT);
	arVideoClose();
}

static void mainLoop(void)
{
	int i;
	static int imageNumber = 0;
	static int ms_prev;
	int ms;
	float s_elapsed;
	ARUint8 *image;

	// Find out how long since mainLoop() last ran.
	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	if (s_elapsed < 0.01f) return; // Don't update more often than 100 Hz.
	ms_prev = ms;

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

		// If  marker config files were specified, evaluate detected patterns against them now.
		for (i = 0; i < gMultiConfigCount; i++) {
			if (gRobustFlag) gMultiErrs[i] = arGetTransMatMultiSquareRobust(gAR3DHandle, arGetMarker(gARHandle), arGetMarkerNum(gARHandle), gMultiConfigs[i]);
			else gMultiErrs[i] = arGetTransMatMultiSquare(gAR3DHandle, arGetMarker(gARHandle), arGetMarkerNum(gARHandle), gMultiConfigs[i]);
		}

		// Tell GLUT the display has changed.
		glutPostRedisplay();
	}
}

//
//	This function is called on events when the visibility of the
//	GLUT window changes (including when it first becomes visible).
//
static void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) {
		glutIdleFunc(mainLoop);
	}
	else {
		glutIdleFunc(NULL);
	}
}

//
//	This function is called when the
//	GLUT window is resized.
//
static void Reshape(int w, int h)
{
	windowWidth = w;
	windowHeight = h;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	gViewport[0] = 0;
	gViewport[1] = 0;
	gViewport[2] = w;
	gViewport[3] = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// Call through to anyone else who needs to know about window sizing here.
}

static void drawAxes()
{
	GLfloat vertices[6][3] = {
		{ 0.0f, 0.0f, 0.0f },{ 10.0f,  0.0f,  0.0f },
		{ 0.0f, 0.0f, 0.0f },{ 0.0f, 10.0f,  0.0f },
		{ 0.0f, 0.0f, 0.0f },{ 0.0f,  0.0f, 10.0f }
	};
	GLubyte colours[6][4] = {
		{ 255,0,0,255 },{ 255,0,0,255 },
		{ 0,255,0,255 },{ 0,255,0,255 },
		{ 0,0,255,255 },{ 0,0,255,255 }
	};

	glVertexPointer(3, GL_FLOAT, 0, vertices);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, colours);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glLineWidth(2.0f);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDrawArrays(GL_LINES, 0, 6);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}

//
// This function is called when the window needs redrawing.
//
static void Display(void)
{
	ARdouble p[16];
	ARdouble m[16];
#ifdef ARDOUBLE_IS_FLOAT
	GLdouble p0[16];
	GLdouble m0[16];
#endif
	int i, j, k;
	GLfloat  w, bw, bh, vertices[6][2];
	GLubyte pixels[300];
	char text[256];
	GLdouble winX, winY, winZ;
	int showMErr[CHECK_ID_MULTIMARKERS_MAX];
	GLdouble MX[CHECK_ID_MULTIMARKERS_MAX];
	GLdouble MY[CHECK_ID_MULTIMARKERS_MAX];
	int pattDetectMode;
	AR_MATRIX_CODE_TYPE matrixCodeType;


	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.

	arglPixelBufferDataUpload(gArglSettings, gARTImage);
	arglDispImage(gArglSettings);

	if (gMultiConfigCount) {
		arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, p);
		glMatrixMode(GL_PROJECTION);
#ifdef ARDOUBLE_IS_FLOAT
		glLoadMatrixf(p);
#else
		glLoadMatrixd(p);
#endif
		glMatrixMode(GL_MODELVIEW);
		glEnable(GL_DEPTH_TEST);

		// If we have multi-configs, show their origin onscreen.
		for (k = 0; k < gMultiConfigCount; k++) {
			showMErr[k] = FALSE;
			if (gMultiConfigs[k]->prevF != 0) {
				arglCameraViewRH((const ARdouble(*)[4])gMultiConfigs[k]->trans, m, 1.0);
#ifdef ARDOUBLE_IS_FLOAT
				glLoadMatrixf(m);
#else
				glLoadMatrixd(m);
#endif
				drawAxes();
#ifdef ARDOUBLE_IS_FLOAT
				for (i = 0; i < 16; i++) m0[i] = (GLdouble)m[i];
				for (i = 0; i < 16; i++) p0[i] = (GLdouble)p[i];
				if (gluProject(0, 0, 0, m0, p0, gViewport, &winX, &winY, &winZ) == GL_TRUE)
#else
				if (gluProject(0, 0, 0, m, p, gViewport, &winX, &winY, &winZ) == GL_TRUE)
#endif
				{
					showMErr[k] = TRUE;
					MX[k] = winX; MY[k] = winY;
				}
			}

		} // for k
	}

	// Any 2D overlays go here.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, (GLdouble)windowWidth, 0, (GLdouble)windowHeight, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

	arGetPatternDetectionMode(gARHandle, &pattDetectMode);
	arGetMatrixCodeType(gARHandle, &matrixCodeType);

	// For all markers, draw onscreen position.
	// Colour based on cutoffPhase.
	glLoadIdentity();
	glVertexPointer(2, GL_FLOAT, 0, vertices);
	glEnableClientState(GL_VERTEX_ARRAY);
	glLineWidth(2.0f);
	for (j = 0; j < gARHandle->marker_num; j++) {
		glColor3ubv(cutoffPhaseColours[gARHandle->markerInfo[j].cutoffPhase].colour);
		for (i = 0; i < 5; i++) {
			int dir = gARHandle->markerInfo[j].dir;
			vertices[i][0] = (float)gARHandle->markerInfo[j].vertex[(i + 4 - dir) % 4][0] * (float)windowWidth / (float)gARHandle->xsize;
			vertices[i][1] = ((float)gARHandle->ysize - (float)gARHandle->markerInfo[j].vertex[(i + 4 - dir) % 4][1]) * (float)windowHeight / (float)gARHandle->ysize;
		}
		vertices[i][0] = (float)gARHandle->markerInfo[j].pos[0] * (float)windowWidth / (float)gARHandle->xsize;
		vertices[i][1] = ((float)gARHandle->ysize - (float)gARHandle->markerInfo[j].pos[1]) * (float)windowHeight / (float)gARHandle->ysize;
		glDrawArrays(GL_LINE_STRIP, 0, 6);
		// For markers that have been identified, draw the ID number.
		if (gARHandle->markerInfo[j].id >= 0) {
			glColor3ub(255, 0, 0);
			if (matrixCodeType == AR_MATRIX_CODE_GLOBAL_ID && (pattDetectMode == AR_MATRIX_CODE_DETECTION || pattDetectMode == AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX || pattDetectMode == AR_TEMPLATE_MATCHING_MONO_AND_MATRIX)) snprintf(text, sizeof(text), "%llu (err=%d)", gARHandle->markerInfo[j].globalID, gARHandle->markerInfo[j].errorCorrected);
			else snprintf(text, sizeof(text), "%d", gARHandle->markerInfo[j].id);
			print(text, (float)gARHandle->markerInfo[j].pos[0] * (float)windowWidth / (float)gARHandle->xsize, ((float)gARHandle->ysize - (float)gARHandle->markerInfo[j].pos[1]) * (float)windowHeight / (float)gARHandle->ysize, 0, 0);
		}
	}
	glDisableClientState(GL_VERTEX_ARRAY);

	// For matrix mode, draw the pattern image of the largest marker.
	if (pattDetectMode == AR_MATRIX_CODE_DETECTION || pattDetectMode == AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX || pattDetectMode == AR_TEMPLATE_MATCHING_MONO_AND_MATRIX) {

		int area = 0, biggestMarker = -1;

		for (j = 0; j < gARHandle->marker_num; j++) if (gARHandle->markerInfo[j].area > area) {
			area = gARHandle->markerInfo[j].area;
			biggestMarker = j;
		}
		if (area >= AR_AREA_MIN) {

			int imageProcMode;
			ARdouble pattRatio;
			ARUint8 ext_patt[AR_PATT_SIZE2_MAX*AR_PATT_SIZE2_MAX * 3]; // Holds unwarped pattern extracted from image.
			int size;
			int zoom = 4;
			ARdouble vertexUpright[4][2];

			// Reorder vertices based on dir.
			for (i = 0; i < 4; i++) {
				int dir = gARHandle->markerInfo[biggestMarker].dir;
				vertexUpright[i][0] = gARHandle->markerInfo[biggestMarker].vertex[(i + 4 - dir) % 4][0];
				vertexUpright[i][1] = gARHandle->markerInfo[biggestMarker].vertex[(i + 4 - dir) % 4][1];
			}
			arGetImageProcMode(gARHandle, &imageProcMode);
			arGetPattRatio(gARHandle, &pattRatio);
			if (matrixCodeType == AR_MATRIX_CODE_GLOBAL_ID) {
				size = 14;
				arPattGetImage2(imageProcMode, AR_MATRIX_CODE_DETECTION, size, size * AR_PATT_SAMPLE_FACTOR2,
					gARTImage, gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, &gCparamLT->paramLTf, vertexUpright, (ARdouble)14 / (ARdouble)(14 + 2), ext_patt);
			}
			else {
				size = matrixCodeType & AR_MATRIX_CODE_TYPE_SIZE_MASK;
				arPattGetImage2(imageProcMode, AR_MATRIX_CODE_DETECTION, size, size * AR_PATT_SAMPLE_FACTOR2,
					gARTImage, gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, &gCparamLT->paramLTf, vertexUpright, pattRatio, ext_patt);
			}
			glRasterPos2f((float)(windowWidth - size*zoom) - 4.0f, (float)(size*zoom) + 4.0f);
			glPixelZoom((float)zoom, (float)-zoom);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glDrawPixels(size, size, GL_LUMINANCE, GL_UNSIGNED_BYTE, ext_patt);
			glPixelZoom(1.0f, 1.0f);
		}
	}

	glLoadIdentity();
	glutSwapBuffers();
}

//
// The following functions provide the onscreen help text and mode info.
//

static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge)
{
	int i, len;
	GLfloat x0, y0;

	if (!text) return;

	if (calculateXFromRightEdge) {
		x0 = windowWidth - x - (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (const unsigned char *)text);
	}
	else {
		x0 = x;
	}
	if (calculateYFromTopEdge) {
		y0 = windowHeight - y - 10.0f;
	}
	else {
		y0 = y;
	}
	glRasterPos2f(x0, y0);

	len = (int)strlen(text);
	for (i = 0; i < len; i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
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

