/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/
 
#include "utils/structs.glsl"

uniform sampler2D vfColor;
uniform ImageParameters vfParameters;
in vec3 texCoord_;


float passThrough(vec2 coord){
    return texture(vfColor,coord).x;
}

float magnitude( vec2 coord ){
    //TODO find the magnitude of the vectorfield at the position coords
	vec2 velo = texture2D(vfColor, coord).xy;
    float mag = sqrt(velo.x * velo.x + velo.y * velo.y);
	return mag;
}

float divergence(vec2 coord){
    //TODO find the divergence of the vectorfield at the position coords
	vec2 pixelSize = vfParameters.reciprocalDimensions;
	vec2 velo = texture2D(vfColor, coord).xy;

	vec2 v1x = texture2D(vfColor, vec2(coord.x + pixelSize.x , coord.y)).xy;
	vec2 v2x = texture2D(vfColor, vec2(coord.x - pixelSize.x , coord.y)).xy;
	vec2 dVdx = (v1x - v2x ) / 2*pixelSize.x;

	vec2 v1y = texture2D(vfColor, vec2(coord.x , coord.y + pixelSize.y )).xy;
	vec2 v2y = texture2D(vfColor, vec2(coord.x , coord.y  - pixelSize.y)).xy;
	vec2 dVdy = (v1y - v2y ) / 2*pixelSize.y;


    return dVdx.x + dVdy.y;
}

float rotation(vec2 coord){
    //TODO find the curl of the vectorfield at the position coords
    vec2 pixelSize = vfParameters.reciprocalDimensions;
	vec2 velo = texture2D(vfColor, coord).xy;

	vec2 v1x = texture2D(vfColor, vec2(coord.x + pixelSize.x , coord.y)).xy;
	vec2 v2x = texture2D(vfColor, vec2(coord.x - pixelSize.x , coord.y)).xy;
	vec2 dVdx = (v1x - v2x ) / 2*pixelSize.x;

	vec2 v1y = texture2D(vfColor, vec2(coord.x , coord.y + pixelSize.y )).xy;
	vec2 v2y = texture2D(vfColor, vec2(coord.x , coord.y  - pixelSize.y)).xy;
	vec2 dVdy = (v1y - v2y ) / 2*pixelSize.y;


    return dVdx.y - dVdy.x;
}


void main(void) {
    float v = OUTPUT(texCoord_.xy);
    FragData0 = vec4(v);
}
