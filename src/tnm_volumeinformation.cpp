#include "modules/tnm093/include/tnm_volumeinformation.h"
#include "voreen/core/datastructures/volume/volumeatomic.h"

namespace voreen {

	const std::string loggerCat_ = "TNMVolumeInformation";

namespace {
	// This ordering function allows us to sort the Data vector by the voxelIndex
	// The extraction *should* produce a sorted list, but you never know
	bool sortByIndex(const VoxelDataItem& lhs, const VoxelDataItem& rhs) {
		return lhs.voxelIndex < rhs.voxelIndex;
	}

}

TNMVolumeInformation::TNMVolumeInformation()
    : Processor()
    , _inport(Port::INPORT, "in.volume")
    , _outport(Port::OUTPORT, "out.data")
    , _data(0)
{
    addPort(_inport);
    addPort(_outport);
}

TNMVolumeInformation::~TNMVolumeInformation() {
    delete _data;
}

void TNMVolumeInformation::process() {
    const VolumeHandleBase* volumeHandle = _inport.getData();
    const Volume* baseVolume = volumeHandle->getRepresentation<Volume>();
    const VolumeUInt16* volume = dynamic_cast<const VolumeUInt16*>(baseVolume);
    if (volume == 0)
        return;
	// If we get this far, there actually is a volume to work with

	// If this is the first call, we will create the Data object
	if (_data == 0)
        _data = new Data;

	// Retrieve the size of the three dimensions of the volume
    const tgt::svec3 dimensions = volume->getDimensions();
	// Create as many data entries as there are voxels in the volume
    _data->resize(dimensions.x * dimensions.y * dimensions.z);

	// iX is the index running over the 'x' dimension
	// iY is the index running over the 'y' dimension
	// iZ is the index running over the 'z' dimension
    for (size_t iX = 1; iX < dimensions.x-1; ++iX) {
        for (size_t iY = 1; iY < dimensions.y-1; ++iY) {
            for (size_t iZ = 1; iZ < dimensions.z-1; ++iZ) {
				// i is a unique identifier for the voxel calculated by the following
				// (probably one of the most important) formulas:
				// iZ*dimensions.x*dimensions.y + iY*dimensions.x + iX;
                const size_t i = VolumeUInt16::calcPos(volume->getDimensions(), tgt::svec3(iX, iY, iZ));

				// Setting the unique identifier as the voxelIndex
				_data->at(i).voxelIndex = i;
                //
				// use iX, iY, iZ, i, and the VolumeUInt16::voxel method to derive the measures here
				int v111 = volume->voxel(i);

				int v000, v001, v002, v010, v011, v012, v020, v021, v022, v100, v101, v102, v110, v112, v120, v121, v122, v200, v201, v202, v210, v211, v212, v220, v221, v222 = 0;
				v000 = volume->voxel(iX-1, iY-1, iZ-1);
				v001 = volume->voxel(iX-1, iY-1, iZ);
				v002 = volume->voxel(iX-1, iY-1, iZ+1);
				v010 = volume->voxel(iX-1, iY, iZ);
				v011 = volume->voxel(iX-1, iY, iZ);
				v012 = volume->voxel(iX-1, iY, iZ+1);
				v020 = volume->voxel(iX-1, iY+1, iZ);
				v021 = volume->voxel(iX-1, iY+1, iZ);
				v022 = volume->voxel(iX-1, iY+1, iZ+1);
				v100 = volume->voxel(iX, iY-1, iZ-1);
				v101 = volume->voxel(iX, iY-1, iZ);
				v102 = volume->voxel(iX, iY-1, iZ+1);
				v110 = volume->voxel(iX, iY, iZ);
				v112 = volume->voxel(iX, iY, iZ+1);
				v120 = volume->voxel(iX, iY+1, iZ);
				v121 = volume->voxel(iX, iY+1, iZ);
				v122 = volume->voxel(iX, iY+1, iZ+1);
				v200 = volume->voxel(iX+1, iY-1, iZ-1);
				v201 = volume->voxel(iX+1, iY-1, iZ);
				v202 = volume->voxel(iX+1, iY-1, iZ+1);
				v210 = volume->voxel(iX+1, iY, iZ);
				v211 = volume->voxel(iX+1, iY, iZ);
				v212 = volume->voxel(iX+1, iY, iZ+1);
				v220 = volume->voxel(iX+1, iY+1, iZ);
				v221 = volume->voxel(iX+1, iY+1, iZ);
				v222 = volume->voxel(iX+1, iY+1, iZ+1);


				// Intensity
				//
				float intensity = -1.f;
				intensity = volume->voxel(i);
				// Retrieve the intensity using the 'VolumeUInt16's voxel method
				//
				_data->at(i).dataValues[0] = intensity;

				//
				// Average
				//
				float average = -1.0f;
 				// Compute the average; the voxel method accepts both a single parameter
 				average = v000 + v001 + v002 +
 						  v010 + v011 + v012 +
 						  v020 + v021 + v022 +
 						  v100 + v101 + v102 +
 						  v110 + v111 + v112 +
 						  v120 + v121 + v122 +
 						  v200 + v201 + v202 +
 						  v210 + v211 + v212 +
 						  v220 + v221 + v222;
 						average /= 27.0f;

				_data->at(i).dataValues[1] = average;

				//
				// Standard deviation
				//
				float stdDeviation = -1.f;
				// Compute the standard deviation


				_data->at(i).dataValues[2] = stdDeviation;

				//
				// Gradient magnitude
				//
				float gradientMagnitude = -1.f;
				// Compute the gradient direction using either forward, central, or backward
				// calculation and then take the magnitude (=length) of the vector.
				// Hint:  tgt::vec3 is a class that can calculate the length for you


				_data->at(i).dataValues[3] = gradientMagnitude;
            }
        }
    }

	// sort the data by the voxel index for faster processing later
	std::sort(_data->begin(), _data->end(), sortByIndex);

	// And provide access to the data using the outport
    _outport.setData(_data, false);
}

} // namespace
