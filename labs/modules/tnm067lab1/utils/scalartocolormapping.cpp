/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016-2017 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 *FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/tnm067lab1/utils/scalartocolormapping.h>

namespace inviwo {

ScalarToColorMapping::ScalarToColorMapping() {}

ScalarToColorMapping::~ScalarToColorMapping() {}

void ScalarToColorMapping::clearColors() { baseColors_.clear(); }

void ScalarToColorMapping::addBaseColors(vec4 color) { baseColors_.push_back(color); }

vec4 ScalarToColorMapping::sample(float t) {
    if (baseColors_.size() == 0) return vec4(t);
    if (baseColors_.size() == 1) return vec4(baseColors_[0]);


    // Implement here:
    // Interpolate colors in baseColors_
    // return the right values

    if(t<=0) return vec4(baseColors_.front());
    if(t>=1) return vec4(baseColors_.back()); 

    vec4 finalColor(t, t, t, 1); // dummy color

    size_t N = baseColors_.size();
	size_t mid = 0.5;
	float step = 1.0 / (N - 1);
	
	float T = t*(N-1);
	//Closest color to the left
	size_t idx_left = floor(T);
	vec4 a(baseColors_[idx_left]);

	//Closest color to the right
	size_t idx_right = ceil(T);
	vec4 b(baseColors_[idx_right]);

	finalColor = a*(1 - glm::fract(T)) + b*glm::fract(T);
	
	

    return finalColor;
}

}  // namespace inviwo
