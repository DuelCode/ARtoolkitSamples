/*
 *	simpleOSG.c
 *  ARToolKit5
 *
 *	Demonstration of ARToolKit with models rendered in OSG,
 *  and marker pose estimates filtered to reduce jitter.
 *
 *  Press '?' while running for help on available key commands.
 *
 *  Disclaimer: IMPORTANT:  This Daqri software is supplied to you by Daqri
 *  LLC ("Daqri") in consideration of your agreement to the following
 *  terms, and your use, installation, modification or redistribution of
 *  this Daqri software constitutes acceptance of these terms.  If you do
 *  not agree with these terms, please do not use, install, modify or
 *  redistribute this Daqri software.
 *
 *  In consideration of your agreement to abide by the following terms, and
 *  subject to these terms, Daqri grants you a personal, non-exclusive
 *  license, under Daqri's copyrights in this original Daqri software (the
 *  "Daqri Software"), to use, reproduce, modify and redistribute the Daqri
 *  Software, with or without modifications, in source and/or binary forms;
 *  provided that if you redistribute the Daqri Software in its entirety and
 *  without modifications, you must retain this notice and the following
 *  text and disclaimers in all such redistributions of the Daqri Software.
 *  Neither the name, trademarks, service marks or logos of Daqri LLC may
 *  be used to endorse or promote products derived from the Daqri Software
 *  without specific prior written permission from Daqri.  Except as
 *  expressly stated in this notice, no other rights or licenses, express or
 *  implied, are granted by Daqri herein, including but not limited to any
 *  patent rights that may be infringed by your derivative works or by other
 *  works in which the Daqri Software may be incorporated.
 *
 *  The Daqri Software is provided by Daqri on an "AS IS" basis.  DAQRI
 *  MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION
 *  THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE, REGARDING THE DAQRI SOFTWARE OR ITS USE AND
 *  OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS.
 *
 *  IN NO EVENT SHALL DAQRI BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL
 *  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION,
 *  MODIFICATION AND/OR DISTRIBUTION OF THE DAQRI SOFTWARE, HOWEVER CAUSED
 *  AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
 *  STRICT LIABILITY OR OTHERWISE, EVEN IF DAQRI HAS BEEN ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Copyright 2015 Daqri LLC. All Rights Reserved.
 *  Copyright 2009-2015 ARToolworks, Inc. All Rights Reserved.
 *
 *  Author(s): Philip Lamb.
 *
 */

// ============================================================================
//	Includes
// ============================================================================

#ifdef _WIN32
#  include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>					// malloc(), free()
#include <string.h>
#ifdef _WIN32
#  define snprintf _snprintf
#endif
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>

#include "ARMarkerSquare.h"
#include "VirtualEnvironment.h"

// ============================================================================
//	Constants
// ============================================================================

#define VIEW_SCALEFACTOR		1.0         // Units received from ARToolKit tracking will be multiplied by this factor before being used in OpenGL drawing.
#define VIEW_DISTANCE_MIN		40.0        // Objects closer to the camera than this will not be displayed. OpenGL units.
#define VIEW_DISTANCE_MAX		10000.0     // Objects further away from the camera than this will not be displayed. OpenGL units.

#define FONT_SIZE 10.0f
#define FONT_LINE_SPACING 2.0f

// ============================================================================
//	Global variables
// ============================================================================

// Preferences.
static int prefWindowed = TRUE;           // Use windowed (TRUE) or fullscreen mode (FALSE) on launch.
static int prefWidth = 640;               // Preferred initial window width.
static int prefHeight = 480;              // Preferred initial window height.
static int prefDepth = 32;                // Fullscreen mode bit depth. Set to 0 to use default depth.
static int prefRefresh = 0;				  // Fullscreen mode refresh rate. Set to 0 to use default rate.

// Image acquisition.
static ARUint8		*gARTImage = NULL;

// Markers.
static ARMarkerSquare *markersSquare = NULL;
static int markersSquareCount = 0;

// Marker detection.
static ARHandle		*gARHandle = NULL;
static long			 gCallCountMarkerDetect = 0;
static ARPattHandle	*gARPattHandle = NULL;
static int           gARPattDetectionMode;

// Transformation matrix retrieval.
static AR3DHandle	*gAR3DHandle = NULL;
static int           useContPoseEstimation = FALSE;

// Drawing.
static int gWindowW;
static int gWindowH;
static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static ARdouble cameraLens[16];
static ARdouble cameraPose[16];
static int cameraPoseValid;
static int gShowHelp = 1;
static int gShowMode = 1;


// ============================================================================
//	Function prototypes
// ============================================================================

static void usage(char *com);
static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p);
static void cleanup(void);
static void Keyboard(unsigned char key, int x, int y);
static void Visibility(int visible);
static void Reshape(int w, int h);
static void Display(void);
static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge);
static void drawBackground(const float width, const float height, const float x, const float y);
static void printHelpKeys();
static void printMode();

// ============================================================================
//	Functions
// ============================================================================

int main(int argc, char** argv)
{
	char    glutGamemode[32] = "";
    char   *vconf = NULL;
    //char    cparaDefault[] = "Data/camera_para.dat";
	char    cparaDefault[] = "C:/WPFWorkspace/LiuHao/Demos/VTTARtoolkitSamples/Debug/Data/camera_para.dat";
    char   *cpara = NULL;
    int     i;
    int     gotTwoPartOption;
    //const char markerConfigDataFilename[] = "Data/markers.dat";
	//const char objectDataFilename[] = "Data/objects.dat";
	const char markerConfigDataFilename[] = "C:/WPFWorkspace/LiuHao/Demos/VTTARtoolkitSamples/Debug/Data/markers.dat";
	const char objectDataFilename[] = "C:/WPFWorkspace/LiuHao/Demos/VTTARtoolkitSamples/Debug/Data/objects.dat";
	
    //
	// Process command-line options.
	//
    
	glutInit(&argc, argv);
    
    i = 1; // argv[0] is name of app, so start at 1.
    while (i < argc) {
        gotTwoPartOption = FALSE;
        // Look for two-part options first.
        if ((i + 1) < argc) {
            if (strcmp(argv[i], "--vconf") == 0) {
                i++;
                vconf = argv[i];
                gotTwoPartOption = TRUE;
            } else if (strcmp(argv[i], "--cpara") == 0) {
                i++;
                cpara = argv[i];
                gotTwoPartOption = TRUE;
            } else if (strcmp(argv[i],"--width") == 0) {
                i++;
                // Get width from second field.
                if (sscanf(argv[i], "%d", &prefWidth) != 1) {
                    ARLOGe("Error: --width option must be followed by desired width.\n");
                }
                gotTwoPartOption = TRUE;
            } else if (strcmp(argv[i],"--height") == 0) {
                i++;
                // Get height from second field.
                if (sscanf(argv[i], "%d", &prefHeight) != 1) {
                    ARLOGe("Error: --height option must be followed by desired height.\n");
                }
                gotTwoPartOption = TRUE;
            } else if (strcmp(argv[i],"--refresh") == 0) {
                i++;
                // Get refresh rate from second field.
                if (sscanf(argv[i], "%d", &prefRefresh) != 1) {
                    ARLOGe("Error: --refresh option must be followed by desired refresh rate.\n");
                }
                gotTwoPartOption = TRUE;
            }
        }
        if (!gotTwoPartOption) {
            // Look for single-part options.
            if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0) {
                usage(argv[0]);
            } else if (strncmp(argv[i], "-cpara=", 7) == 0) {
                cpara = &(argv[i][7]);
            } else if (strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-version") == 0 || strcmp(argv[i], "-v") == 0) {
                ARLOG("%s version %s\n", argv[0], AR_HEADER_VERSION_STRING);
                exit(0);
            } else if (strcmp(argv[i],"--windowed") == 0) {
                prefWindowed = TRUE;
            } else if (strcmp(argv[i],"--fullscreen") == 0) {
                prefWindowed = FALSE;
            } else {
                ARLOGe("Error: invalid command line argument '%s'.\n", argv[i]);
                usage(argv[0]);
            }
        }
        i++;
    }
    

	//
	// Video setup.
	//
    
	if (!setupCamera((cpara ? cpara : cparaDefault), vconf, &gCparamLT)) {
		ARLOGe("main(): Unable to set up AR camera.\n");
		exit(-1);
	}

    //
    // AR init.
    //
    
    // Init AR.
    gARPattHandle = arPattCreateHandle();
	if (!gARPattHandle) {
		ARLOGe("Error creating pattern handle.\n");
		exit(-1);
	}
    
    gARHandle = arCreateHandle(gCparamLT);
    if (!gARHandle) {
        ARLOGe("Error creating AR handle.\n");
		exit(-1);
    }
    arPattAttach(gARHandle, gARPattHandle);
    
    if (arSetPixelFormat(gARHandle, arVideoGetPixelFormat()) < 0) {
        ARLOGe("Error setting pixel format.\n");
		exit(-1);
    }
    
    gAR3DHandle = ar3DCreateHandle(&gCparamLT->param);
    if (!gAR3DHandle) {
        ARLOGe("Error creating 3D handle.\n");
		exit(-1);
    }
            
    //
    // Markers setup.
    //
    
    // Load marker(s).
    newMarkers(markerConfigDataFilename, gARPattHandle, &markersSquare, &markersSquareCount, &gARPattDetectionMode);
    ARLOGi("Marker count = %d\n", markersSquareCount);
    
    //
    // Other ARToolKit setup.
    //
    
    arSetMarkerExtractionMode(gARHandle, AR_USE_TRACKING_HISTORY_V2);
    //arSetMarkerExtractionMode(gARHandle, AR_NOUSE_TRACKING_HISTORY);
    //arSetLabelingThreshMode(gARHandle, AR_LABELING_THRESH_MODE_MANUAL); // Uncomment to force manual thresholding.
    
    // Set the pattern detection mode (template (pictorial) vs. matrix (barcode) based on
    // the marker types as defined in the marker config. file.
    arSetPatternDetectionMode(gARHandle, gARPattDetectionMode); // Default = AR_TEMPLATE_MATCHING_COLOR
    
    // Other application-wide marker options. Once set, these apply to all markers in use in the application.
    // If you are using standard ARToolKit picture (template) markers, leave commented to use the defaults.
    // If you are usign a different marker design (see http://www.artoolworks.com/support/app/marker.php )
    // then uncomment and edit as instructed by the marker design application.
    //arSetLabelingMode(gARHandle, AR_LABELING_BLACK_REGION); // Default = AR_LABELING_BLACK_REGION
    //arSetBorderSize(gARHandle, 0.25f); // Default = 0.25f
    //arSetMatrixCodeType(gARHandle, AR_MATRIX_CODE_3x3); // Default = AR_MATRIX_CODE_3x3
    
	//
	// Graphics setup.
	//

	// Set up GL context(s) for OpenGL to draw into.
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    if (prefWindowed) {
        if (prefWidth > 0 && prefHeight > 0) glutInitWindowSize(prefWidth, prefHeight);
        else glutInitWindowSize(gCparamLT->param.xsize, gCparamLT->param.ysize);
        glutCreateWindow(argv[0]);
    } else {
        if (glutGameModeGet(GLUT_GAME_MODE_POSSIBLE)) {
            if (prefWidth && prefHeight) {
                if (prefDepth) {
                    if (prefRefresh) snprintf(glutGamemode, sizeof(glutGamemode), "%ix%i:%i@%i", prefWidth, prefHeight, prefDepth, prefRefresh);
                    else snprintf(glutGamemode, sizeof(glutGamemode), "%ix%i:%i", prefWidth, prefHeight, prefDepth);
                } else {
                    if (prefRefresh) snprintf(glutGamemode, sizeof(glutGamemode), "%ix%i@%i", prefWidth, prefHeight, prefRefresh);
                    else snprintf(glutGamemode, sizeof(glutGamemode), "%ix%i", prefWidth, prefHeight);
                }
            } else {
                prefWidth = glutGameModeGet(GLUT_GAME_MODE_WIDTH);
                prefHeight = glutGameModeGet(GLUT_GAME_MODE_HEIGHT);
                snprintf(glutGamemode, sizeof(glutGamemode), "%ix%i", prefWidth, prefHeight);
            }
            glutGameModeString(glutGamemode);
            glutEnterGameMode();
        } else {
            if (prefWidth > 0 && prefHeight > 0) glutInitWindowSize(prefWidth, prefHeight);
            glutCreateWindow(argv[0]);
            glutFullScreen();
        }
    }
    
    // Create the OpenGL projection from the calibrated camera parameters.
    arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, cameraLens);
    cameraPoseValid = FALSE;
    
	// Setup ARgsub_lite library for current OpenGL context.
	if ((gArglSettings = arglSetupForCurrentContext(&(gCparamLT->param), arVideoGetPixelFormat())) == NULL) {
		ARLOGe("main(): arglSetupForCurrentContext() returned error.\n");
		cleanup();
		exit(-1);
	}
    arglSetupDebugMode(gArglSettings, gARHandle);
    
    // Load objects (i.e. OSG models).
    VirtualEnvironmentInit(objectDataFilename);
    VirtualEnvironmentHandleARViewUpdatedCameraLens(cameraLens);
    
    //
    // Setup complete. Start tracking.
    //
    
    // Start the video.
    if (arVideoCapStart() != 0) {
    	ARLOGe("setupCamera(): Unable to begin camera data capture.\n");
		return (FALSE);
	}
	arUtilTimerReset();
	
	// Register GLUT event-handling callbacks.
	// NB: mainLoop() is registered by Visibility.
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutVisibilityFunc(Visibility);
	glutKeyboardFunc(Keyboard);
	
	glutMainLoop();

	return (0);
}

static void usage(char *com)
{
    ARLOG("Usage: %s [options]\n", com);
    ARLOG("Options:\n");
    ARLOG("  --vconf <video parameter for the camera>\n");
    ARLOG("  --cpara <camera parameter file for the camera>\n");
    ARLOG("  -cpara=<camera parameter file for the camera>\n");
	ARLOG("  --width w     Use display/window width of w pixels.\n");
	ARLOG("  --height h    Use display/window height of h pixels.\n");
	ARLOG("  --refresh f   Use display refresh rate of f Hz.\n");
	ARLOG("  --windowed    Display in window, rather than fullscreen.\n");
	ARLOG("  --fullscreen  Display fullscreen, rather than in window.\n");
    ARLOG("  -h -help --help: show this message\n");
    exit(0);
}

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p)
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
        arVideoClose();
        return (FALSE);
    }
	
	return (TRUE);
}

static void cleanup(void)
{
	VirtualEnvironmentFinal();

    if (markersSquare) deleteMarkers(&markersSquare, &markersSquareCount, gARPattHandle);
    
    // Tracking cleanup.
    if (gARPattHandle) {
        arPattDetach(gARHandle);
		arPattDeleteHandle(gARPattHandle);
	}
	ar3DDeleteHandle(&gAR3DHandle);
	arDeleteHandle(gARHandle);
    arParamLTFree(&gCparamLT);

    // OpenGL cleanup.
    arglCleanup(gArglSettings);
    gArglSettings = NULL;
    
    // Camera cleanup.
	arVideoCapStop();
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
			ARLOGe("*** Camera - %f (frame/sec)\n", (double)gCallCountMarkerDetect/arUtilTimer());
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
	static int ms_prev;
	int ms;
	float s_elapsed;
	ARUint8 *image;
    ARMarkerInfo* markerInfo;
    int markerNum;
	ARdouble err;
    int             i, j, k;
	
	// Calculate time delta.
	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	ms_prev = ms;
	
	// Grab a video frame.
	if ((image = arVideoGetImage()) != NULL) {
		gARTImage = image;	// Save the fetched image.
		
		gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.
		
		// Detect the markers in the video frame.
		if (arDetectMarker(gARHandle, gARTImage) < 0) {
			exit(-1);
		}
		
		// Get detected markers
		markerInfo = arGetMarker(gARHandle);
		markerNum = arGetMarkerNum(gARHandle);
	
		// Update markers.
		for (i = 0; i < markersSquareCount; i++) {
			markersSquare[i].validPrev = markersSquare[i].valid;
            
            
			// Check through the marker_info array for highest confidence
			// visible marker matching our preferred pattern.
			k = -1;
			if (markersSquare[i].patt_type == AR_PATTERN_TYPE_TEMPLATE) {
				for (j = 0; j < markerNum; j++) {
					if (markersSquare[i].patt_id == markerInfo[j].idPatt) {
						if (k == -1) {
							if (markerInfo[j].cfPatt >= markersSquare[i].matchingThreshold) k = j; // First marker detected.
						} else if (markerInfo[j].cfPatt > markerInfo[k].cfPatt) k = j; // Higher confidence marker detected.
					}
				}
				if (k != -1) {
					markerInfo[k].id = markerInfo[k].idPatt;
					markerInfo[k].cf = markerInfo[k].cfPatt;
					markerInfo[k].dir = markerInfo[k].dirPatt;
				}
			} else {
				for (j = 0; j < markerNum; j++) {
					if (markersSquare[i].patt_id == markerInfo[j].idMatrix) {
						if (k == -1) {
							if (markerInfo[j].cfMatrix >= markersSquare[i].matchingThreshold) k = j; // First marker detected.
						} else if (markerInfo[j].cfMatrix > markerInfo[k].cfMatrix) k = j; // Higher confidence marker detected.
					}
				}
				if (k != -1) {
					markerInfo[k].id = markerInfo[k].idMatrix;
					markerInfo[k].cf = markerInfo[k].cfMatrix;
					markerInfo[k].dir = markerInfo[k].dirMatrix;
				}
			}

			if (k != -1) {
				markersSquare[i].valid = TRUE;
				ARLOGd("Marker %d matched pattern %d.\n", i, markerInfo[k].id);
				// Get the transformation between the marker and the real camera into trans.
				if (markersSquare[i].validPrev && useContPoseEstimation) {
					err = arGetTransMatSquareCont(gAR3DHandle, &(markerInfo[k]), markersSquare[i].trans, markersSquare[i].marker_width, markersSquare[i].trans);
				} else {
					err = arGetTransMatSquare(gAR3DHandle, &(markerInfo[k]), markersSquare[i].marker_width, markersSquare[i].trans);
				}
			} else {
				markersSquare[i].valid = FALSE;
			}
	   
			if (markersSquare[i].valid) {
			
				// Filter the pose estimate.
				if (markersSquare[i].ftmi) {
					if (arFilterTransMat(markersSquare[i].ftmi, markersSquare[i].trans, !markersSquare[i].validPrev) < 0) {
						ARLOGe("arFilterTransMat error with marker %d.\n", i);
					}
				}
			
				if (!markersSquare[i].validPrev) {
					// Marker has become visible, tell any dependent objects.
                    VirtualEnvironmentHandleARMarkerAppeared(i);
				}
	
				// We have a new pose, so set that.
				arglCameraViewRH((const ARdouble (*)[4])markersSquare[i].trans, markersSquare[i].pose.T, 1.0f /*VIEW_SCALEFACTOR*/);
				// Tell any dependent objects about the update.
				VirtualEnvironmentHandleARMarkerWasUpdated(i, markersSquare[i].pose);
			
			} else {
			
				if (markersSquare[i].validPrev) {
					// Marker has ceased to be visible, tell any dependent objects.
					VirtualEnvironmentHandleARMarkerDisappeared(i);
				}
			}                    
		}
		
		// Tell GLUT the display has changed.
		glutPostRedisplay();
	} else {
		arUtilSleep(2);
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
	} else {
		glutIdleFunc(NULL);
	}
}

//
//	This function is called when the
//	GLUT window is resized.
//
static void Reshape(int w, int h)
{
    GLint viewport[4];
    
    gWindowW = w;
    gWindowH = h;
    
	// Call through to anyone else who needs to know about window sizing here.
    viewport[0] = 0;
    viewport[1] = 0;
    viewport[2] = w;
    viewport[3] = h;
    VirtualEnvironmentHandleARViewUpdatedViewport(viewport);
}

//
// This function is called when the window needs redrawing.
//
static void Display(void)
{
	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.
    
    arglPixelBufferDataUpload(gArglSettings, gARTImage);
    arglDispImage(gArglSettings);
	gARTImage = NULL; // Invalidate image data.
				
    // Set up 3D mode.
	glMatrixMode(GL_PROJECTION);
#ifdef ARDOUBLE_IS_FLOAT
	glLoadMatrixf(cameraLens);
#else
	glLoadMatrixd(cameraLens);
#endif
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    glEnable(GL_DEPTH_TEST);

    // Set any initial per-frame GL state you require here.
    // --->
    
    // Lighting and geometry that moves with the camera should be added here.
    // (I.e. should be specified before camera pose transform.)
    // --->
    
    VirtualEnvironmentHandleARViewDrawPreCamera();
    
    if (cameraPoseValid) {
        
#ifdef ARDOUBLE_IS_FLOAT
        glMultMatrixf(cameraPose);
#else
        glMultMatrixd(cameraPose);
#endif
        
        // All lighting and geometry to be drawn in world coordinates goes here.
        // --->
        VirtualEnvironmentHandleARViewDrawPostCamera();
    }
    
    // Set up 2D mode.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, (GLdouble)gWindowW, 0, (GLdouble)gWindowH, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    // Add your own 2D overlays here.
    // --->
    
    VirtualEnvironmentHandleARViewDrawOverlay();

    //
    // Draw help text and mode.
    //
    if (gShowMode) {
        printMode();
    }
    if (gShowHelp) {
        if (gShowHelp == 1) {
            printHelpKeys();
        }
    }
	
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
        x0 = gWindowW - x - (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (const unsigned char *)text);
    } else {
        x0 = x;
    }
    if (calculateYFromTopEdge) {
        y0 = gWindowH - y - 10.0f;
    } else {
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
    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
    glVertexPointer(2, GL_FLOAT, 0, vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);	// 50% transparent black.
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Opaque white.
    //glLineWidth(1.0f);
    //glDrawArrays(GL_LINE_LOOP, 0, 4);
    glPopClientAttrib();
    glDisable(GL_BLEND);
}

static void printHelpKeys()
{
    int i;
    GLfloat  w, bw, bh;
    const char *helpText[] = {
        "Keys:\n",
        " ? or /        Show/hide this help.",
        " q or [esc]    Quit program.",
        " d             Activate / deactivate debug mode.",
        " m             Toggle display of mode info.",
        " a             Toggle between available threshold modes.",
        " - and +       Switch to manual threshold mode, and adjust threshhold up/down by 5.",
        " x             Change image processing mode.",
        " c             Calulcate frame rate.",
    };
#define helpTextLineCount (sizeof(helpText)/sizeof(char *))
	float hMargin = 2.0f;
	float vMargin = 2.0f;
    
    bw = 0.0f;
    for (i = 0; i < helpTextLineCount; i++) {
        w = (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (unsigned char *)helpText[i]);
        if (w > bw) bw = w;
    }
    bh = helpTextLineCount * FONT_SIZE + (helpTextLineCount - 1) * FONT_LINE_SPACING;
    drawBackground(bw, bh, hMargin, vMargin);
    
    for (i = 0; i < helpTextLineCount; i++) print(helpText[i], hMargin, vMargin + (helpTextLineCount - 1 - i)*(FONT_SIZE + FONT_LINE_SPACING), 0, 0);
}

static void printMode()
{
    int len, thresh, line, mode, xsize, ysize;
    AR_LABELING_THRESH_MODE threshMode;
    ARdouble tempF;
    char text[256], *text_p;
	float hMargin = 2.0f;
	float vMargin = 2.0f;
    
    glColor4ub(255, 255, 255, 255);
    line = 1;
    
    // Image size and processing mode.
    arVideoGetSize(&xsize, &ysize);
    arGetImageProcMode(gARHandle, &mode);
	if (mode == AR_IMAGE_PROC_FRAME_IMAGE) text_p = "full frame";
	else text_p = "even field only";
    snprintf(text, sizeof(text), "Processing %dx%d video frames %s", xsize, ysize, text_p);
    print(text, hMargin, (line - 1)*(FONT_SIZE + FONT_LINE_SPACING) + vMargin, 0, 1);
    line++;
    
    // Threshold mode, and threshold, if applicable.
    arGetLabelingThreshMode(gARHandle, &threshMode);
    switch (threshMode) {
        case AR_LABELING_THRESH_MODE_MANUAL: text_p = "MANUAL"; break;
        case AR_LABELING_THRESH_MODE_AUTO_MEDIAN: text_p = "AUTO_MEDIAN"; break;
        case AR_LABELING_THRESH_MODE_AUTO_OTSU: text_p = "AUTO_OTSU"; break;
        case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: text_p = "AUTO_ADAPTIVE"; break;
        case AR_LABELING_THRESH_MODE_AUTO_BRACKETING: text_p = "AUTO_BRACKETING"; break;
        default: text_p = "UNKNOWN"; break;
    }
    snprintf(text, sizeof(text), "Threshold mode: %s", text_p);
    if (threshMode != AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE) {
        arGetLabelingThresh(gARHandle, &thresh);
        len = (int)strlen(text);
        snprintf(text + len, sizeof(text) - len, ", thresh=%d", thresh);
    }
    print(text, hMargin, (line - 1)*(FONT_SIZE + FONT_LINE_SPACING) + vMargin, 0, 1);
    line++;
    
    // Border size, image processing mode, pattern detection mode.
    arGetBorderSize(gARHandle, &tempF);
    snprintf(text, sizeof(text), "Border: %0.1f%%", tempF*100.0);
    arGetPatternDetectionMode(gARHandle, &mode);
    switch (mode) {
        case AR_TEMPLATE_MATCHING_COLOR: text_p = "Colour template (pattern)"; break;
        case AR_TEMPLATE_MATCHING_MONO: text_p = "Mono template (pattern)"; break;
        case AR_MATRIX_CODE_DETECTION: text_p = "Matrix (barcode)"; break;
        case AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX: text_p = "Colour template + Matrix (2 pass, pattern + barcode)"; break;
        case AR_TEMPLATE_MATCHING_MONO_AND_MATRIX: text_p = "Mono template + Matrix (2 pass, pattern + barcode "; break;
        default: text_p = "UNKNOWN"; break;
    }
    len = (int)strlen(text);
    snprintf(text + len, sizeof(text) - len, ", Pattern detection mode: %s", text_p);
    print(text, hMargin,  (line - 1)*(FONT_SIZE + FONT_LINE_SPACING) + vMargin, 0, 1);
    line++;
    
    // Window size.
    snprintf(text, sizeof(text), "Drawing into %dx%d window", gWindowW, gWindowH);
    print(text, hMargin,  (line - 1)*(FONT_SIZE + FONT_LINE_SPACING) + vMargin, 0, 1);
    line++;
    
}
