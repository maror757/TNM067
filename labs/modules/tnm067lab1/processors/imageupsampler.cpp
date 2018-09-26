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

#include <inviwo/core/util/logcentral.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/tnm067lab1/processors/imageupsampler.h>
#include <modules/tnm067lab1/utils/interpolationmethods.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/util/imageramutils.h>

namespace inviwo {

    namespace detail{

        


        template<typename T>
        void upsample( ImageUpsampler::IntepolationMethod  method , 
            const LayerRAMPrecision<T> &inputImage, LayerRAMPrecision<T> &outputImage){
            using F = typename float_type<T>::type;
            
            size2_t inputSize = inputImage.getDimensions();
            size2_t outputSize = outputImage.getDimensions();

            const T* inPixels = inputImage.getDataTyped();
            T* outPixels = outputImage.getDataTyped();


            auto inIndex = [&inputSize](auto pos) -> size_t{
                pos = glm::clamp(pos, decltype(pos)(0) , decltype(pos)(inputSize - size2_t(1)));
                return pos.x + pos.y * inputSize.x;
            };
            auto outIndex = [&outputSize](auto pos) -> size_t{
                pos = glm::clamp(pos, decltype(pos)(0) , decltype(pos)(outputSize - size2_t(1)));
                return pos.x + pos.y * outputSize.x;
            };

            util::forEachPixel(outputImage, [&](ivec2 outImageCoords) {
                // outImageCoords: Exact pixel coordinates in the output image currently writing to
                // inImageCoords: Relative coordinates of outImageCoords in the input image, might be between pixels
                dvec2 inImageCoords = ImageUpsampler::convertCoordinate(outImageCoords, inputSize, outputSize);

                T finalColor(0);

                //DUMMY COLOR, remove or overwrite this bellow
                //finalColor = inPixels[inIndex( glm::clamp(size2_t(outImageCoords), size2_t(0), size2_t(inputSize - size2_t(1))) )];


                switch (method) {
                    case inviwo::ImageUpsampler::IntepolationMethod::PiecewiseConstant:
                    {
                        // Task 8
						//finalColor = inPixels[inIndex(size2_t(floor(inImageCoords.x), floor(inImageCoords.y)))];  
						//finalColor = inPixels[inIndex( size2_t(round(inImageCoords.x), round(inImageCoords.y)))];
						finalColor = inPixels[inIndex(floor(inImageCoords))];
						
                        break;
                    }
                    case inviwo::ImageUpsampler::IntepolationMethod::Bilinear:
                    {
						inImageCoords -= 0.5;
						size2_t originPos(floor(inImageCoords));
						std::array<T, 4> vals = {
							inPixels[inIndex(originPos)],
							inPixels[inIndex(originPos + size2_t(1, 0))],
							inPixels[inIndex(originPos + size2_t(0, 1))],
							inPixels[inIndex(originPos + size2_t(1, 1))]
						};

						double x = inImageCoords.x - originPos.x;
						double y = inImageCoords.y - originPos.y;
						finalColor = TNM067::Interpolation::bilinear(vals, x, y);
                        break;
                    }
                    case inviwo::ImageUpsampler::IntepolationMethod::Quadratic:
                    {
						//inImageCoords -= 0.5;
						size2_t originPos(floor(inImageCoords));

						std::array<T, 9> vals = {
							inPixels[inIndex(originPos)],
							inPixels[inIndex(originPos + size2_t(1, 0))],
							inPixels[inIndex(originPos + size2_t(2, 0))],
							inPixels[inIndex(originPos + size2_t(0, 1))],
							inPixels[inIndex(originPos + size2_t(1, 1))],
							inPixels[inIndex(originPos + size2_t(2, 1))],
							inPixels[inIndex(originPos + size2_t(0, 2))],
							inPixels[inIndex(originPos + size2_t(1, 2))],
							inPixels[inIndex(originPos + size2_t(2, 2))],
						};

						double x = (inImageCoords.x - originPos.x)/2.0;
						double y = (inImageCoords.y - originPos.y)/2.0;
						finalColor = TNM067::Interpolation::biQuadratic(vals, x, y);
						break;
                    }
                    case inviwo::ImageUpsampler::IntepolationMethod::Barycentric:
                    {
						inImageCoords -= 0.5;
						size2_t originPos(floor(inImageCoords));
						std::array<T, 4> vals = {
							inPixels[inIndex(originPos)],
							inPixels[inIndex(originPos + size2_t(1, 0))],
							inPixels[inIndex(originPos + size2_t(0, 1))],
							inPixels[inIndex(originPos + size2_t(1, 1))]
						};

						double x = inImageCoords.x - originPos.x;
						double y = inImageCoords.y - originPos.y;
						finalColor = TNM067::Interpolation::barycentric(vals, x, y);
						break;
                    }
                    default:
                        break;
                }

                outPixels[outIndex(outImageCoords)] = finalColor;
            });
        }
    
    }


const ProcessorInfo ImageUpsampler::processorInfo_{
    "org.inviwo.imageupsampler",  // Class identifier
    "Image Upsampler",            // Display name
    "TNM067",                     // Category
    CodeState::Experimental,      // Code state
    Tags::None,                   // Tags
};
const ProcessorInfo ImageUpsampler::getProcessorInfo() const { return processorInfo_; }

ImageUpsampler::ImageUpsampler()
    : Processor()
    , inport_("inport", true)
    , outport_("outport" , true)
    , interpolationMethod_(
          "interpolationMethod", "Interpolation Method",
          {
              {"piecewiseconstant", "Piecewise Constant (Nearest Neighbor)", IntepolationMethod::PiecewiseConstant},
              {"bilinear", "Bilinear", IntepolationMethod::Bilinear},
              {"quadratic", "Quadratic", IntepolationMethod::Quadratic},
              {"barycentric", "Barycentric", IntepolationMethod::Barycentric},
          }){
    addPort(inport_);
    addPort(outport_);
    addProperty(interpolationMethod_);
}

void ImageUpsampler::process() {
    auto inputImage = inport_.getData();
    if(inputImage->getDataFormat()->getComponents()!=1){
        LogError("The ImageUpsampler processor does only support single channel images");
    }

    auto inSize = inport_.getData()->getDimensions();
    auto outDim = outport_.getDimensions();

    auto outputImage = std::make_shared<Image>(outDim,inputImage->getDataFormat());
    outputImage->getColorLayer()->setSwizzleMask(inputImage->getColorLayer()->getSwizzleMask());
    outputImage->getColorLayer()->getEditableRepresentation<LayerRAM>()->dispatch<void,dispatching::filter::Scalars>([&](auto outRep){
        auto inRep = inputImage->getColorLayer()->getRepresentation<LayerRAM>();
        detail::upsample(interpolationMethod_.get(), *(const decltype(outRep))(inRep), *outRep);
    });

    outport_.setData(outputImage);
}

dvec2 ImageUpsampler::convertCoordinate(ivec2 outImageCoords, size2_t inputSize, size2_t outputSize) {
    // TODO implement

    dvec2 c(outImageCoords);
	

	c *= dvec2((double)inputSize.x / (double)outputSize.x, (double)inputSize.y / (double)outputSize.y);


    return c; 
}



}  // namespace inviwo
