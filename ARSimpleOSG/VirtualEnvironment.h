/*
 *	VirtualEnvironment.h
 *  ARToolKit5
 *
 *	Demonstration of ARToolKit NFT with models rendered in OSG,
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
 *  Copyright 2010-2015 ARToolworks, Inc. All Rights Reserved.
 *
 *  Author(s): Philip Lamb.
 *
 */

#ifndef __VirtualEnvironment_h__
#define __VirtualEnvironment_h__

#include <AR/ar.h>
#include "ARMarkerSquare.h"

#ifdef __cplusplus
extern "C" {
#endif

int VirtualEnvironmentInit(const char *objectListFile);
void VirtualEnvironmentFinal(void);

// ARMarker notification handlers.
void VirtualEnvironmentHandleARMarkerWasUpdated(int markerIndex, ARPose poseIn);
void VirtualEnvironmentHandleARMarkerAppeared(int markerIndex);
void VirtualEnvironmentHandleARMarkerDisappeared(int markerIndex);

// ARView notification handlers.
void VirtualEnvironmentHandleARViewUpdatedCameraLens(ARdouble *projection_in);
//void VirtualEnvironmentHandleARViewUpdatedCameraPose(GLfloat *modelview_in);
void VirtualEnvironmentHandleARViewUpdatedViewport(int *viewPort_in);
void VirtualEnvironmentHandleARViewDrawPreCamera(void);
void VirtualEnvironmentHandleARViewDrawPostCamera(void);
void VirtualEnvironmentHandleARViewDrawOverlay(void);

#ifdef __cplusplus
}
#endif

#endif // !__VirtualEnvironment_h__
