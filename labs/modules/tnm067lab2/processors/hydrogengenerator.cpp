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

#include <modules/tnm067lab2/processors/hydrogengenerator.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/util/volumeramutils.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <modules/base/algorithm/dataminmax.h>

namespace inviwo {

    const ProcessorInfo HydrogenGenerator::processorInfo_{
        "org.inviwo.HydrogenGenerator",  // Class identifier
        "Hydrogen Generator",            // Display name
        "TNM067",                        // Category
        CodeState::Experimental,         // Code state
        Tags::None,                      // Tags
    };

    const ProcessorInfo HydrogenGenerator::getProcessorInfo() const { return processorInfo_; }

    HydrogenGenerator::HydrogenGenerator()
        : Processor()
        , volume_("volume")
        , size_("size_", "Volume Size", 16, 4, 256)
    {
        addPort(volume_);
        addProperty(size_);
    }

    void HydrogenGenerator::process() {
        auto vol = std::make_shared<Volume>(size3_t(size_), DataFloat32::get());

        auto ram = vol->getEditableRepresentation<VolumeRAM>();
        auto data = static_cast<float *>(ram->getData());
        util::IndexMapper3D index(ram->getDimensions());

        util::forEachVoxel(*ram, [&](const size3_t &pos) {
            vec3 cartesian = idTOCartesian(pos);
            data[index(pos)] = eval(cartesian);
        });

        auto minMax = util::volumeMinMax(ram);
        vol->dataMap_.dataRange = vol->dataMap_.valueRange = dvec2(minMax.first.x, minMax.second.x);

        volume_.setData(vol);
    }

    inviwo::vec3 HydrogenGenerator::cartesianToSphereical(vec3 cartesian) {
        vec3 sph;
		float r = sqrt(pow(cartesian[0], 2) + pow(cartesian[1], 2) + pow(cartesian[2], 2));
		if (r == 0.0)
			return vec3(0.0, 0.0, 0.0);

		float rad = cartesian[2] / r;
		float theta = (rad >= 1.0f || rad <= -1.0f) ? 0.0f : acos(rad);
		float phi = atan2(cartesian[1], cartesian[0]);  //(cartesian[0] == 0) ? atan2(M_PI , 2) : atan2(cartesian[1] , cartesian[0]);

		sph = vec3(r, theta, phi);
        return sph;
    }

    double HydrogenGenerator::eval(vec3 cartesian) {
		double Z = 1.0;
		double a0 = 1.0;
		vec3 sph = cartesianToSphereical(cartesian);

		double f1 = 1 / (81 * sqrt(6*M_PI));
		double f2 = pow ( Z/a0, 1.5 );
		double f3 = ( pow(Z,2) * pow(sph[0],2) )  / pow(a0,2);
		double f4 = exp( (-Z*sph[0]) / (3*a0) );
		double f5 = 3*pow(cos(sph[1]),2) - 1.0;

		double res = f1*f2*f3*f4*f5;
        return pow(res,2);
		
  
  }

    inviwo::vec3 HydrogenGenerator::idTOCartesian(size3_t pos) {
        vec3 p(pos);
        p /= size_ - 1;
        return p * (36.0f) - 18.0f;
    }

} // namespace

