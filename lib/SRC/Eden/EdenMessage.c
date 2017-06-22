//
//  EdenMessage.h
//
//  Copyright (c) 2001-2013 Philip Lamb (PRL) phil@eden.net.nz. All rights reserved.
//	Font loading code based on code by Jeff Molofee, 1999, http://nehe.gamedev.net/
//	
//	Rev		Date		Who		Changes
//	1.0.0	2001-12-04	PRL		Initial version for The SRMS simulator.
//	1.0.1	2005-09-28	PRL		Added headerDoc.
//  1.1.0   2013-02-19  PRL     Quick update for OpenGL ES.
//

// @@BEGIN_EDEN_LICENSE_HEADER@@
//
//  This file is part of The Eden Library.
//
//  The Eden Library is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  The Eden Library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with The Eden Library.  If not, see <http://www.gnu.org/licenses/>.
//
//  As a special exception, the copyright holders of this library give you
//  permission to link this library with independent modules to produce an
//  executable, regardless of the license terms of these independent modules, and to
//  copy and distribute the resulting executable under terms of your choice,
//  provided that you also meet, for each linked independent module, the terms and
//  conditions of the license of that module. An independent module is a module
//  which is neither derived from nor based on this library. If you modify this
//  library, you may extend this exception to your version of the library, but you
//  are not obligated to do so. If you do not wish to do so, delete this exception
//  statement from your version.
//
// @@END_EDEN_LICENSE_HEADER@@

// ============================================================================
//	Includes
// ============================================================================

#include <Eden/EdenMessage.h>
#include <Eden/EdenMath.h> // MIN()

#include <stdio.h>
#include <string.h>
#include <stdlib.h>				// malloc(), calloc(), free(), exit()
#ifndef _MSC_VER
#  include <stdbool.h>
#else
typedef unsigned char bool;
#  define false 0
#  define true 1
#  include <sys/timeb.h>    // struct _timeb
#endif
#include <pthread.h>
#include <errno.h>			// ETIMEDOUT
#include <Eden/EdenTime.h>	// EdenTimeAbsolutePlusOffset(), struct timespec, EdenTime_sleep()
#include <Eden/EdenSurfaces.h>	// TEXTURE_INFO_t, TEXTURE_INDEX_t, SurfacesTextureLoad(), SurfacesTextureSet(), SurfacesTextureUnload()
#include <Eden/EdenGLFont.h>
#ifndef EDEN_OPENGLES
#  define DISABLE_GL_STATE_CACHE
#endif
#include "glStateCache.h"


// ============================================================================
//  Types and constants
// ============================================================================

#define BOX_LINES_MAX 80
#define BOX_LINE_LENGTH_MAX 1023

typedef struct _boxSettings {
    pthread_mutex_t lock;
	float boxWidth;				// Pixel width of box.
    float boxHeight;            // Pixel height of box (calculated).
	float boxPaddingH;			// Pixels of padding between the left and right border and the text.
	float boxPaddingV;			// Pixels of padding between the top and bottom border and the text.
	float boxCornerRadius;		// Radius in pixels of the rounded corners. Typically set to MIN(boxPaddingH, boxPaddinV).
	float softwrapRatio;		// Percentage of the maximum text width at which to start soft-wrapping text.
    unsigned char *text;
	unsigned char *lines[BOX_LINES_MAX];
	int lineCount;
} boxSettings_t;

#define EDEN_MESSAGE_INPUT_MAX_LENGTH_DEFAULT 1023

//#define DEBUG_MESSAGE					// Uncomment to show extra debugging info.

#pragma mark -
#pragma mark [GLOBAL VARIABLES]
// ============================================================================
//	Global variables
// ============================================================================

// Sanity checks.
static EDEN_BOOL gMessageInited = FALSE; // Set to TRUE once EdenMessageInit() has succesfully completed.

static boxSettings_t *gBoxSettings = NULL;

// Screen size.
static float gScreenWidth = 640.0f;
static float gScreenHeight = 480.0f;
static float gScreenScale = 1.0f;

// Use of gDrawLock allows EdenMessageDraw() to be called in a separate thread
// by protecting the global static data (below) that it uses.
//static pthread_mutex_t gDrawLock;
EDEN_BOOL gEdenMessageDrawRequired = FALSE;						// EdenMessageDraw() should be called if this is set to TRUE;

static pthread_mutex_t gInputLock;      // Protects gInput.
static unsigned char *gInput; 			// Pointer to buffer for input string, including prompt.
static unsigned char *gInputPtr;        // Pointer into buffer to start of input string.
static unsigned int gInputLength;		// Holds length of input string including prompt.
static unsigned int gInputPromptLength; //
static int gInputCursorState;
static pthread_cond_t gInputCompleteCond;
static EDEN_BOOL gInputComplete;        // Set to TRUE once EdenMessageInputKeyboard() has captured a response.

// Input constraints.
static unsigned int gInputLengthMin;	// Minimum length of user input string.
static unsigned int gInputLengthMax;	// Maximum length of user input string.
static int gInputIntOnly;
static int gInputFPOnly;
static int gInputAlphaOnly;

EDEN_BOOL gEdenMessageKeyboardRequired = FALSE;		// EdenMessageInputKeyboard should be called with keystrokes if this is set to TRUE;

//
// Private functions.
//

// Can pass NULL for parameter 'text' in which case previous text is reused.
// Depends on settings 'text', 'boxWidth', 'boxPaddingH'.
// Sets settings 'text', 'lines', 'lineCount', 'boxHeight'.
static void boxSetText(boxSettings_t *settings, const unsigned char *text)
{
	int i;
	int textIndex;
	bool done;
 	float boxTextWidth;
	float boxTextWidthSoftwrap;
	float hyphenWidth;
    float interCharSpacingWidth;
    
	unsigned char c = '\0', c0, c1; // current, previous, next char.
	int lineLength = 0;
	unsigned char lineBuf[BOX_LINE_LENGTH_MAX + 1] = ""; // +1 for null terminator.
	float lineWidth = 0;
   
	if (!settings) return;
    
    pthread_mutex_lock(&settings->lock);
    
    if (text) {
        free(settings->text);
        settings->text = (unsigned char *)strdup((char *)text);
    }
    
	// Free old lines.
	if (settings->lineCount) {
        for (i = 0; i < settings->lineCount; i++) {
            free(settings->lines[i]);
            settings->lines[i] = NULL;
        }
		settings->lineCount = 0;
	}
    
	boxTextWidth = settings->boxWidth - 2*settings->boxPaddingH;
	boxTextWidthSoftwrap = boxTextWidth*settings->softwrapRatio;
	hyphenWidth = EdenGLFontGetCharacterWidth('-');
    interCharSpacingWidth = EdenGLFontGetLineWidth((const unsigned char *)"--") - 2.0f*hyphenWidth;
    
    if (settings->text) {
        // Split text into lines, softwrapping on whitespace if possible.
        textIndex = 0;
        done = false;
        do {
            
            bool newline = false;
            
            c0 = c;
            c = settings->text[textIndex];
            if (!c) {
                if (lineLength) newline = true;
                done = true;
            } else  if (c == '\n' || (c == ' ' && lineWidth >= boxTextWidthSoftwrap)) {
                textIndex++;
                newline = true;
            } else if (c < ' ') {
                textIndex++;
            } else {
                bool addChar = false;
                // Is there still room for a hyphen after this character?
                float predictedLineWidth = lineWidth + interCharSpacingWidth + EdenGLFontGetCharacterWidth(settings->text[textIndex]);
                if (predictedLineWidth < (boxTextWidth - hyphenWidth)) {
                    addChar = true;
                } else {
                    // No. But two exceptions:
                    // 1) this character is a space.
                    // 2) this character doesn't overflow and the next character is whitespace.
                    c1 = settings->text[textIndex + 1];
                    if (c == ' ') {
                        textIndex++;
                        newline = true;
                    } else if (predictedLineWidth <= boxTextWidth && (!c1 || c1 == ' ' || c1 == '\n')) {
                        addChar = true;
                    } else {
                        // Exception didn't apply, so insert hyphen, then newline, then continue with same char on next line (unless previous char was space, in which case no hyphen).
                        if (c0 != ' ') {
                            lineBuf[lineLength++] = '-';
                            lineBuf[lineLength] = '\0';
                        }
                        newline = true;
                    }
                }
                if (addChar) {
                    lineBuf[lineLength++] = c;
                    lineBuf[lineLength] = '\0';
                    lineWidth = EdenGLFontGetLineWidth(lineBuf);
                    if (lineLength == BOX_LINE_LENGTH_MAX) newline = true; // Next char would overflow buffer, so break now.
                    textIndex++;
                }
            }
            
            if (newline) {
                // Start a new line.
                settings->lines[settings->lineCount] = (unsigned char *)strdup((const char *)lineBuf);
                settings->lineCount++;
                if (settings->lineCount == BOX_LINES_MAX) done = true;
                lineLength = 0;
                lineBuf[0] = '\0';
                lineWidth = 0.0f;
            }
        } while (!done);
    }
    
    settings->boxHeight = EdenGLFontGetBlockHeight((const unsigned char **)settings->lines, settings->lineCount) + 2.0f*settings->boxPaddingV;

    pthread_mutex_unlock(&settings->lock);
}

static boxSettings_t *boxCreate(float width, float paddingH, float paddingV, float cornerRadius, float softwrapRatio)
{
    boxSettings_t *settings = (boxSettings_t *)calloc(1, sizeof(boxSettings_t));
    if (!settings) return (NULL);
    
    pthread_mutex_init(&settings->lock, NULL);
    settings->boxWidth = (width > 0.0f ? width : 400.0f)*gScreenScale;
    settings->boxPaddingH = (paddingH >= 0.0f ? paddingH : 20.0f)*gScreenScale;
    settings->boxPaddingV = (paddingV >= 0.0f ? paddingV : 20.0f)*gScreenScale;
    settings->boxCornerRadius = (cornerRadius >= 0.0f ? cornerRadius*gScreenScale : MIN(settings->boxPaddingH, settings->boxPaddingV));
    settings->softwrapRatio = (softwrapRatio > 0.0f ? softwrapRatio : 0.9f);
    return (settings);
}

static void boxDestroy(boxSettings_t **settings_p)
{
    int i;
    
    if (!settings_p || !*settings_p) return;
    
    pthread_mutex_destroy(&(*settings_p)->lock);
    
 	// Free lines.
	if ((*settings_p)->lineCount) {
		for (i = 0; i < (*settings_p)->lineCount; i++) free((*settings_p)->lines[i]);
		(*settings_p)->lineCount = 0;
	}
    free((*settings_p)->text);
    free(*settings_p);
    (*settings_p) = NULL;
}

// ============================================================================
//  Public functions
// ============================================================================

EDEN_BOOL EdenMessageInit(const int contextsActiveCount)
{
	if (gMessageInited) return (FALSE);

    gBoxSettings = boxCreate(-1.0f, -1.0f, -1.0f, -1.0f, -1.0f); // Use defaults.

    pthread_mutex_init(&gInputLock, NULL);
    gInput = gInputPtr = NULL;
    gInputLength = 0;
    gInputPromptLength = 0;
    pthread_cond_init(&gInputCompleteCond, NULL);
    gInputComplete = FALSE;
 
	gMessageInited = TRUE;
	return (TRUE);
}

EDEN_BOOL EdenMessageFinal(void)
{
	EDEN_BOOL ok = TRUE;

	if (!gMessageInited) return (FALSE);
    
	pthread_mutex_destroy(&gInputLock);
    free(gInput);
    gInput = gInputPtr = NULL;
    gInputLength = 0;
    gInputPromptLength = 0;
	pthread_cond_destroy(&gInputCompleteCond);
    gInputComplete = FALSE;

    boxDestroy(&gBoxSettings);
    
	gMessageInited = FALSE;
	return (ok);
}

unsigned char *EdenMessageInputGetInput(void)
{
    unsigned char *ret;
    
    pthread_mutex_lock(&gInputLock);
    if (gInputPtr && gInputComplete) {
        gInputPtr[gInputLength] = '\0'; // Overwrite any cursor character.
        ret = (unsigned char *)strdup((char *)gInputPtr);
    } else ret = NULL;
    pthread_mutex_unlock(&gInputLock);
    
    return (ret);
}


#pragma mark -
// ----------------------------------------------------------------------------
//	These functions should only be called directly in single- threaded apps.
//	In multi-threaded apps, they will be called indirectly.
// ----------------------------------------------------------------------------

EDEN_E_t EdenMessageShow(const unsigned char *msg)
{
	if (!gMessageInited) return (EDEN_E_INVALID_COMMAND);
    
	if (msg) {
        boxSetText(gBoxSettings, msg);
    }
    
	gEdenMessageDrawRequired = TRUE;

	return (EDEN_E_NONE);
}

EDEN_E_t EdenMessageHide(void)
{
	if (!gMessageInited) return (EDEN_E_INVALID_COMMAND);

	gEdenMessageDrawRequired = FALSE;
    
	return (EDEN_E_NONE);
}

EDEN_E_t EdenMessageInputShow(const unsigned char *prompt, const unsigned int minLength, const unsigned int maxLength, int intOnly, int fpOnly, int alphaOnly)
{
	EDEN_E_t messageErr = EDEN_E_NONE;
    
	if (!gMessageInited) return (EDEN_E_INVALID_COMMAND);

    pthread_mutex_lock(&gInputLock);
    
    if (maxLength == 0) gInputLengthMax = EDEN_MESSAGE_INPUT_MAX_LENGTH_DEFAULT;
    else gInputLengthMax = maxLength;
    
    if (minLength > gInputLengthMax) {
        messageErr = EDEN_E_OVERFLOW;
        goto done;
    } else gInputLengthMin = minLength;
    
    if (prompt) {
        gInputPromptLength = (unsigned int)strlen((const char *)prompt);
    }
    
    if (gInput) free(gInput);
    gInput = (unsigned char *)malloc(gInputPromptLength + gInputLengthMax + 2); // +1 for cursor and +1 for nul-terminator.
    if (!gInput) {
        messageErr = EDEN_E_OUT_OF_MEMORY;
        goto done;
    }
    if (prompt) {
        strncpy((char *)gInput, (const char *)prompt, gInputPromptLength);
    }
    
    gInputPtr = gInput + gInputPromptLength;
    gInputPtr[0] = '\0';
    gInputLength = 0;
    gInputCursorState = -1;
    gInputComplete = FALSE;
    
    gInputIntOnly = intOnly;
    gInputFPOnly = fpOnly;
    gInputAlphaOnly = alphaOnly;

    if ((messageErr = EdenMessageShow(gInput)) != EDEN_E_NONE) {
        goto done;
    }

	// This signals that keys should start being sent to EdenMessageInputKeyboard()
	gEdenMessageKeyboardRequired = TRUE;
    
done:
    pthread_mutex_unlock(&gInputLock);
    return (messageErr);
}

EDEN_E_t EdenMessageInputHide(void)
{
	if (!gMessageInited) return (EDEN_E_INVALID_COMMAND);

	gEdenMessageKeyboardRequired = FALSE;
    
	return (EdenMessageHide());
}

EDEN_BOOL EdenMessageInputIsComplete(void)
{
    int ret;
    
	if (!gMessageInited) return (FALSE);

    pthread_mutex_lock(&gInputLock);
    ret = gInputComplete;
    pthread_mutex_unlock(&gInputLock);
    return (ret);
}

#pragma mark -

// ----------------------------------------------------------------------------
//	Functions for use in multi-threaded apps only.
// ----------------------------------------------------------------------------
EDEN_E_t EdenMessage(unsigned char *msg, const unsigned int secs)
{
	EDEN_E_t err;
	
	err = EdenMessageShow(msg);
	if (err != EDEN_E_NONE) return (err);
	
	EdenTime_sleep(secs);

	err = EdenMessageHide();
	if (err != EDEN_E_NONE) return (err);

	return (EDEN_E_NONE);
}

EDEN_E_t EdenMessageInput(const unsigned char *prompt, const unsigned int minLength, const unsigned int maxLength, int intOnly, int fpOnly, int alphaOnly)
{
	EDEN_E_t messageErr = EDEN_E_NONE;

	if ((messageErr = EdenMessageInputShow(prompt, minLength, maxLength, intOnly, fpOnly, alphaOnly)) != EDEN_E_NONE) {
        goto done;
    }
	
	// Wait for signal from EdenMessageInputKeyboard that input is complete.
    pthread_mutex_lock(&gInputLock);
    if (!gInputComplete) {
        pthread_cond_wait(&gInputCompleteCond, &gInputLock);
    }
    pthread_mutex_unlock(&gInputLock);
	
	messageErr = EdenMessageInputHide();

done:
	return (messageErr);
}

#pragma mark -
// ----------------------------------------------------------------------------
//	Functions for use in single- and double-threaded apps.
// ----------------------------------------------------------------------------

void EdenMessageSetViewSize(const float width, const float height)
{
    gScreenWidth = width;
    gScreenHeight = height;
}

void EdenMessageSetBoxParams(const float width, const float padding)
{
    EDEN_BOOL changed = FALSE;
    if (gBoxSettings->boxWidth != width) {
        gBoxSettings->boxWidth = width;
        changed = TRUE;
    }
    if (gBoxSettings->boxPaddingH != padding || gBoxSettings->boxPaddingV != padding) {
        gBoxSettings->boxPaddingH = padding;
        gBoxSettings->boxPaddingV = padding;
        changed = TRUE;
    }
    if (changed) boxSetText(gBoxSettings, NULL);
}

void EdenMessageDraw(const int contextIndex)
{
    GLfloat boxcentrex, boxcentrey, boxwd2, boxhd2;
    GLfloat boxVertices[4][2];
    const GLubyte quadIndices[4] = {0, 1, 2, 3};
#ifdef _WIN32
    struct _timeb sys_time;
#else
    struct timeval time;
#endif
    int cursorState;

    if (!gBoxSettings) return;

    // Check if we need to show a blinking cursor, and if so, what state it is in.
    pthread_mutex_lock(&gInputLock);
    if (gInputPtr && !gInputComplete) {
#ifdef _WIN32
        _ftime(&sys_time);
        if (sys_time.millitm < 500ul) cursorState = 1;
        else cursorState = 0;
#else
#  if defined(__linux) || defined(__APPLE__)
        gettimeofday( &time, NULL );
#  else
        gettimeofday( &time );
#  endif
        if (time.tv_usec < 500000) cursorState = 1;
        else cursorState = 0;
#endif
        if (cursorState != gInputCursorState) {
            gInputCursorState = cursorState;
            if (cursorState == 1) {
                gInputPtr[gInputLength] = '|';
                gInputPtr[gInputLength + 1] = '\0';
            } else {
                gInputPtr[gInputLength] = ' ';
                gInputPtr[gInputLength + 1] = '\0';
            }
            EdenMessageShow(gInput);
        }
    }
    pthread_mutex_unlock(&gInputLock);
    
    boxcentrex = gScreenWidth / 2.0f;
    boxwd2 = gBoxSettings->boxWidth / 2;
    boxcentrey = gScreenHeight / 2.0f;
    boxhd2 = gBoxSettings->boxHeight / 2;
    boxVertices[0][0] = boxcentrex - boxwd2; boxVertices[0][1] = boxcentrey - boxhd2;
    boxVertices[1][0] = boxcentrex + boxwd2; boxVertices[1][1] = boxcentrey - boxhd2;
    boxVertices[2][0] = boxcentrex + boxwd2; boxVertices[2][1] = boxcentrey + boxhd2;
    boxVertices[3][0] = boxcentrex - boxwd2; boxVertices[3][1] = boxcentrey + boxhd2;

    // Draw box.
    pthread_mutex_lock(&gBoxSettings->lock);
	if (gBoxSettings->lineCount) {
        // Draw the semi-transparent black shaded box and white outline.
        glStateCacheBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glStateCacheEnableBlend();
        glVertexPointer(2, GL_FLOAT, 0, boxVertices);
        glStateCacheEnableClientStateVertexArray();
        glStateCacheDisableClientStateNormalArray();
        glStateCacheClientActiveTexture(GL_TEXTURE0);
        glStateCacheDisableClientStateTexCoordArray();
        glColor4f(0.0f, 0.0f, 0.0f, 0.5f);	// 50% transparent black.
        glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_BYTE, quadIndices);
        glStateCacheDisableBlend();
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Opaque white.
        glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_BYTE, quadIndices);
        
        EdenGLFontDrawBlock(contextIndex, (const unsigned char **)gBoxSettings->lines, gBoxSettings->lineCount, 0.0f, 0.0f, H_OFFSET_VIEW_CENTER_TO_TEXT_CENTER, V_OFFSET_VIEW_CENTER_TO_TEXT_CENTER);
    }
	pthread_mutex_unlock(&gBoxSettings->lock);

	return;
}

EDEN_BOOL EdenMessageInputKeyboard(const unsigned char keyAsciiCode)
{
    EDEN_BOOL ret = TRUE;
    
	pthread_mutex_lock(&gInputLock);
    if (gInputPtr && !gInputComplete) {
        switch (keyAsciiCode) {
            case EDEN_ASCII_ESC:
                free(gInput);
                gInput = gInputPtr = NULL;
                gInputLength = 0;
                gInputComplete = TRUE;
                pthread_cond_signal(&gInputCompleteCond);
                break;
            case EDEN_ASCII_CR:
                if (gInputLength >= gInputLengthMin) {
                    gInputComplete = TRUE;
                    pthread_cond_signal(&gInputCompleteCond);
                }
                break;
            case EDEN_ASCII_BS:
            case EDEN_ASCII_DEL:
                if (gInputLength > 0) {
                    gInputLength--;
                    gInput[gInputLength] = '\0';
                    if (EdenMessageShow(gInput) != EDEN_E_NONE) {
                        ret = FALSE;
                        goto done;
                    }
                }
                break;
            default:
                if (keyAsciiCode < ' ') break;	// Throw away all other control characters.
                if (gInputIntOnly && (keyAsciiCode < '0' || keyAsciiCode > '9')) break;
                if (gInputFPOnly && (keyAsciiCode < '0' || keyAsciiCode > '9') && keyAsciiCode != '.') break;
                if (gInputAlphaOnly && (keyAsciiCode < 'A' || keyAsciiCode > 'Z') && (keyAsciiCode < 'a' || keyAsciiCode > 'z')) break;
                if (gInputLength < gInputLengthMax) {
                    gInput[gInputLength] = keyAsciiCode;
                    gInputLength++;
                    gInput[gInputLength] = '\0';
                    if (EdenMessageShow(gInput) != EDEN_E_NONE) {
                        ret = FALSE;
                        goto done;
                    }
                }
                break;
        }
    }
done:
    pthread_mutex_unlock(&gInputLock);

	return (ret);
}
