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

#include <modules/tnm067lab2/processors/marchingtetrahedra.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>
#include <inviwo/core/network/networklock.h>

namespace inviwo {

size_t MarchingTetrahedra::HashFunc::max = 1;

const ProcessorInfo MarchingTetrahedra::processorInfo_{
    "org.inviwo.MarchingTetrahedra",  // Class identifier
    "Marching Tetrahedra",            // Display name
    "TNM067",                         // Category
    CodeState::Experimental,          // Code state
    Tags::None,                       // Tags
};
const ProcessorInfo MarchingTetrahedra::getProcessorInfo() const { return processorInfo_; }

MarchingTetrahedra::MarchingTetrahedra()
    : Processor()
    , volume_("volume")
    , mesh_("mesh")
    , isoValue_("isoValue", "ISO value", 0.5f, 0.0f, 1.0f) {

    addPort(volume_);
    addPort(mesh_);

    addProperty(isoValue_);

    isoValue_.setSerializationMode(PropertySerializationMode::All);

    volume_.onChange([&]() {
        if (!volume_.hasData()) {
            return;
        }
        NetworkLock lock(getNetwork());
        float iso = (isoValue_.get() - isoValue_.getMinValue()) /
                    (isoValue_.getMaxValue() - isoValue_.getMinValue());
        auto vr = volume_.getData()->dataMap_.valueRange;
        isoValue_.setMinValue(vr.x);
        isoValue_.setMaxValue(vr.y);
        isoValue_.setIncrement(glm::abs(vr.y - vr.x) / 50.0f);
        isoValue_.set(iso * (vr.y - vr.x) + vr.x);
        isoValue_.setCurrentStateAsDefault();
    });
}


void MarchingTetrahedra::process() {
    auto volume = volume_.getData()->getRepresentation<VolumeRAM>();
    MeshHelper mesh(volume_.getData());

    const auto dims = volume->getDimensions();
    MarchingTetrahedra::HashFunc::max = dims.x * dims.y * dims.z;

    float iso = isoValue_.get();

    util::IndexMapper3D index(dims);

    const static size_t tetrahedraIds[6][4] = {{0, 1, 2, 5}, {1, 3, 2, 5}, {3, 2, 5, 7},
                                               {0, 2, 4, 5}, {6, 4, 2, 5}, {6, 7, 5, 2}};

    size3_t pos;
    for (pos.z = 0; pos.z < dims.z - 1; ++pos.z) {
        for (pos.y = 0; pos.y < dims.y - 1; ++pos.y) {
            for (pos.x = 0; pos.x < dims.x - 1; ++pos.x) {
                // Step 1: create current cell
                // Use volume->getAsDouble to query values from the volume
                // Spatial position should be between 0 and 1
                // The voxel index should be the 1D-index for the voxel
                Cell c;

				Voxel vxl;
				int counter = 0;
				for (int z = 0; z < 2; ++z) {
					for (int y = 0; y < 2; ++y) {
						for (int x = 0; x < 2; ++x) {

							vec3 globalPos (pos.x + x , pos.y + y , pos.z + z);
							vxl.pos = vec3(globalPos.x / (dims.x - 1), globalPos.y / (dims.y - 1), globalPos.z / (dims.z - 1));
							vxl.index = index(globalPos);
							vxl.value = volume->getAsDouble(globalPos);	

							c.voxels[counter] = vxl;
							counter++;
						}
					}
				}


                // Step 2: Subdivide cell into tetrahedra (hint: use tetrahedraIds)
                std::vector<Tetrahedra> tetrahedras;
				Tetrahedra th;
				for (size_t i = 0; i < 6; i++)
				{
					for (size_t j = 0; j < 4; j++)
					{
						th.voxels[j] = c.voxels[tetrahedraIds[i][j]];
					}
					tetrahedras.push_back(th);
				}
				

                for (const Tetrahedra& tetrahedra : tetrahedras) {
                    // Step three: Calculate for tetra case index

					
					Voxel v0 = tetrahedra.voxels[0];
					Voxel v1 = tetrahedra.voxels[1];
					Voxel v2 = tetrahedra.voxels[2];
					Voxel v3 = tetrahedra.voxels[3];

					int caseId = 0;
					
					if (v0.value < iso) caseId += 1;
					if (v1.value < iso) caseId += 2;
					if (v2.value < iso) caseId += 4;
					if (v3.value < iso) caseId += 8;

					vec3 ipol0, ipol1, ipol2, ipol3;
					size_t idx0 = v0.index;
					size_t idx1 = v1.index;
					size_t idx2 = v2.index;
					size_t idx3 = v3.index;
					
					
					switch (caseId)
					{

					case 0: case 15:
						break;

					case 1: case 14: {

						// value = origin + dir * dist	// dist = (iso - min) / (max - min)
						ipol0 = v0.pos + (v1.pos - v0.pos) * (iso - v0.value) / (v1.value - v0.value); //0-1
						ipol1 = v0.pos + (v3.pos - v0.pos) * (iso - v0.value) / (v3.value - v0.value); //0-3
						ipol2 = v0.pos + (v2.pos - v0.pos) * (iso - v0.value) / (v2.value - v0.value); //0-2

						idx1 = mesh.addVertex(ipol0, v0.index, v1.index);
						idx3 = mesh.addVertex(ipol1, v0.index, v3.index);
						idx2 = mesh.addVertex(ipol2, v0.index, v2.index);
						if (caseId == 1)
						{
							mesh.addTriangle(idx1, idx3, idx2);
						}
						else
							mesh.addTriangle(idx1, idx2, idx3);
						break;
					}
					case 2: case 13: {

						ipol0 = v1.pos + (v3.pos - v1.pos) * (iso - v1.value) / (v3.value - v1.value); //1-3
						ipol1 = v1.pos + (v0.pos - v1.pos) * (iso - v1.value) / (v0.value - v1.value); //1-0
						ipol2 = v1.pos + (v2.pos - v1.pos) * (iso - v1.value) / (v2.value - v1.value); //1-2

						idx3 = mesh.addVertex(ipol0, v1.index, v3.index);
						idx0 = mesh.addVertex(ipol1, v1.index, v0.index);
						idx2 = mesh.addVertex(ipol2, v1.index, v2.index);
						if (caseId == 2)
						{
							mesh.addTriangle(idx3, idx0, idx2);
						}
						else
							mesh.addTriangle(idx3, idx2, idx0);

						break;
					}
					case 3: case 12: {

						ipol0 = v0.pos + (v3.pos - v0.pos) * (iso - v0.value) / (v3.value - v0.value); //0-3
						ipol1 = v0.pos + (v2.pos - v0.pos) * (iso - v0.value) / (v2.value - v0.value); //0-2
						ipol2 = v1.pos + (v2.pos - v1.pos) * (iso - v1.value) / (v2.value - v1.value); //1-2
						ipol3 = v1.pos + (v3.pos - v1.pos) * (iso - v1.value) / (v3.value - v1.value); //1-3

						idx0 = mesh.addVertex(ipol0, v0.index, v3.index);
						idx1 = mesh.addVertex(ipol1, v0.index, v2.index);
						idx2 = mesh.addVertex(ipol2, v1.index, v2.index);
						idx3 = mesh.addVertex(ipol3, v1.index, v3.index);
						if (caseId == 3) {
							mesh.addTriangle(idx0, idx1, idx2);
							mesh.addTriangle(idx0, idx2, idx3);
						}
						else {
							mesh.addTriangle(idx0, idx3, idx2);
							mesh.addTriangle(idx0, idx2, idx1);
						}

						break;
					}
					case 4: case 11: {

						ipol0 = v2.pos + (v0.pos - v2.pos) * (iso - v2.value) / (v0.value - v2.value); //2-0
						ipol1 = v2.pos + (v3.pos - v2.pos) * (iso - v2.value) / (v3.value - v2.value); //2-3
						ipol2 = v2.pos + (v1.pos - v2.pos) * (iso - v2.value) / (v1.value - v2.value); //2-1

						idx0 = mesh.addVertex(ipol0, v2.index, v0.index);
						idx3 = mesh.addVertex(ipol1, v2.index, v3.index);
						idx1 = mesh.addVertex(ipol2, v2.index, v1.index);

						if (caseId == 4)
						{
							mesh.addTriangle(idx0, idx3, idx1);
						}
						else
							mesh.addTriangle(idx0, idx1, idx3);

						break;
					}
					case 5: case 10: {
						ipol0 = v0.pos + (v3.pos - v0.pos) * (iso - v0.value) / (v3.value - v0.value); //0-3
						ipol1 = v2.pos + (v3.pos - v2.pos) * (iso - v2.value) / (v3.value - v2.value); //2-3
						ipol2 = v2.pos + (v1.pos - v2.pos) * (iso - v2.value) / (v1.value - v2.value); //2-1
						ipol3 = v0.pos + (v1.pos - v0.pos) * (iso - v0.value) / (v1.value - v0.value); //0-1

						idx0 = mesh.addVertex(ipol0, v0.index, v3.index);
						idx1 = mesh.addVertex(ipol1, v2.index, v3.index);
						idx2 = mesh.addVertex(ipol2, v2.index, v1.index);
						idx3 = mesh.addVertex(ipol3, v0.index, v1.index);
						if (caseId == 5) {
							mesh.addTriangle(idx0, idx1, idx2);
							mesh.addTriangle(idx0, idx2, idx3);
						}
						else {
							mesh.addTriangle(idx0, idx3, idx2);
							mesh.addTriangle(idx0, idx2, idx1);
						}

						break;
					}
					case 6: case 9: {
						ipol0 = v0.pos + (v2.pos - v0.pos) * (iso - v0.value) / (v2.value - v0.value); //0-2
						ipol1 = v0.pos + (v1.pos - v0.pos) * (iso - v0.value) / (v1.value - v0.value); //0-1
						ipol2 = v1.pos + (v3.pos - v1.pos) * (iso - v1.value) / (v3.value - v1.value); //1-3
						ipol3 = v2.pos + (v3.pos - v2.pos) * (iso - v2.value) / (v3.value - v2.value); //2-3

						idx0 = mesh.addVertex(ipol0, v0.index, v2.index);
						idx1 = mesh.addVertex(ipol1, v0.index, v1.index);
						idx2 = mesh.addVertex(ipol2, v1.index, v3.index);
						idx3 = mesh.addVertex(ipol3, v2.index, v3.index);
						if (caseId == 9) {
							mesh.addTriangle(idx0, idx1, idx2);
							mesh.addTriangle(idx0, idx2, idx3);
						}
						else {
							mesh.addTriangle(idx0, idx3, idx2);
							mesh.addTriangle(idx0, idx2, idx1);
						}

						break;
					}
					case 7: case 8: {

						ipol0 = v3.pos + (v0.pos - v3.pos) * (iso - v3.value) / (v0.value - v3.value); //3-0
						ipol1 = v3.pos + (v1.pos - v3.pos) * (iso - v3.value) / (v1.value - v3.value); //3-1
						ipol2 = v3.pos + (v2.pos - v3.pos) * (iso - v3.value) / (v2.value - v3.value); //3-2

						idx0 = mesh.addVertex(ipol0, v3.index, v0.index);
						idx1 = mesh.addVertex(ipol1, v3.index, v1.index);
						idx2 = mesh.addVertex(ipol2, v3.index, v2.index);


						if (caseId == 7)
						{
							mesh.addTriangle(idx0, idx2, idx1);
						}
						else
							mesh.addTriangle(idx0, idx1, idx2);

						break;
					}				
				}
               }
           }
       }
   }

    mesh_.setData(mesh.toBasicMesh());
}

MarchingTetrahedra::MeshHelper::MeshHelper(std::shared_ptr<const Volume> vol)
    : edgeToVertex_()
    , vertices_()
    , mesh_(std::make_shared<BasicMesh>())
    , indexBuffer_(mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None)) {
    mesh_->setModelMatrix(vol->getModelMatrix());
    mesh_->setWorldMatrix(vol->getWorldMatrix());
}

void MarchingTetrahedra::MeshHelper::addTriangle(size_t i0, size_t i1, size_t i2) {
    ivwAssert(i0 != i1, "i0 and i1 should not be the same value");
    ivwAssert(i0 != i2, "i0 and i2 should not be the same value");
    ivwAssert(i1 != i2, "i1 and i2 should not be the same value");

    indexBuffer_->add(static_cast<glm::uint32_t>(i0));
    indexBuffer_->add(static_cast<glm::uint32_t>(i1));
    indexBuffer_->add(static_cast<glm::uint32_t>(i2));

    auto a = vertices_[i0].pos;
    auto b = vertices_[i1].pos;
    auto c = vertices_[i2].pos;

    vec3 n = glm::normalize(glm::cross(b - a, c - a));
    vertices_[i0].normal += n;
    vertices_[i1].normal += n;
    vertices_[i2].normal += n;
}

std::shared_ptr<BasicMesh> MarchingTetrahedra::MeshHelper::toBasicMesh() {
    for (auto& vertex : vertices_) {
        vertex.normal = glm::normalize(vertex.normal);
    }
    mesh_->addVertices(vertices_);
    return mesh_;
}

std::uint32_t MarchingTetrahedra::MeshHelper::addVertex(vec3 pos, size_t i, size_t j) {
    ivwAssert(i != j, "i and j should not be the same value");
    if (j < i) {
        return addVertex(pos, j, i);
    }

    auto edge = std::make_pair(i, j);

    auto it = edgeToVertex_.find(edge);

    if (it == edgeToVertex_.end()) {
        edgeToVertex_[edge] = vertices_.size();
        vertices_.push_back({pos, vec3(0, 0, 0), pos, vec4(0.7f, 0.7f, 0.7f, 1.0f)});
        return static_cast<std::uint32_t>(vertices_.size() - 1);
    }

    return static_cast<std::uint32_t>(it->second);
}

}  // namespace inviwo
