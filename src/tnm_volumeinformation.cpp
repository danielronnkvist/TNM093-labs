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

		int v[3][3][3] = {0};
		v[0][0][0] = volume->voxel(iX-1, iY-1, iZ-1);
		v[0][0][1] = volume->voxel(iX-1, iY-1, iZ);
		v[0][0][2] = volume->voxel(iX-1, iY-1, iZ+1);
		v[0][1][0] = volume->voxel(iX-1, iY, iZ);
		v[0][1][1] = volume->voxel(iX-1, iY, iZ);
		v[0][1][2] = volume->voxel(iX-1, iY, iZ+1);
		v[0][2][0] = volume->voxel(iX-1, iY+1, iZ);
		v[0][2][1] = volume->voxel(iX-1, iY+1, iZ);
		v[0][2][2] = volume->voxel(iX-1, iY+1, iZ+1);
		v[1][0][0] = volume->voxel(iX, iY-1, iZ-1);
		v[1][0][1] = volume->voxel(iX, iY-1, iZ);
		v[1][0][2] = volume->voxel(iX, iY-1, iZ+1);
		v[1][1][0] = volume->voxel(iX, iY, iZ-1);
		v[1][1][1] = volume->voxel(iX, iY, iZ);
		v[1][1][2] = volume->voxel(iX, iY, iZ+1);
		v[1][2][0] = volume->voxel(iX, iY+1, iZ);
		v[1][2][1] = volume->voxel(iX, iY+1, iZ);
		v[1][2][2] = volume->voxel(iX, iY+1, iZ+1);
		v[2][0][0] = volume->voxel(iX+1, iY-1, iZ-1);
		v[2][0][1] = volume->voxel(iX+1, iY-1, iZ);
		v[2][0][2] = volume->voxel(iX+1, iY-1, iZ+1);
		v[2][1][0] = volume->voxel(iX+1, iY, iZ);
		v[2][1][1] = volume->voxel(iX+1, iY, iZ);
		v[2][1][2] = volume->voxel(iX+1, iY, iZ+1);
		v[2][2][0] = volume->voxel(iX+1, iY+1, iZ);
		v[2][2][1] = volume->voxel(iX+1, iY+1, iZ);
		v[2][2][2] = volume->voxel(iX+1, iY+1, iZ+1);


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
		float average = 0.0f;
		// Compute the average; the voxel method accepts both a single parameter

		
		for(int i = 0; i < 3;  i++)
		  for(int j = 0; j < 3;  j++)
		    for(int k = 0; k < 3; k++)
		      average += v[i][j][k];
    
		average /= 27.0f;

		_data->at(i).dataValues[1] = average;

		//
		// Standard deviation
		//
		float stdDeviation = 0.f;
		// Compute the standard deviation
		for(int i = 0; i < 3;  i++)
		  for(int j = 0; j < 3;  j++)
		    for(int k = 0; k < 3; k++)
		      stdDeviation += (pow(v[i][j][k]-average,2));
		stdDeviation = sqrt(stdDeviation);

		_data->at(i).dataValues[2] = stdDeviation;

		//
		// Gradient magnitude
		//
		float gradientMagnitude = -1.f;
		// Compute the gradient direction using either forward, central, or backward
		// calculation and then take the magnitude (=length) of the vector.
		// Hint:  tgt::vec3 is a class that can calculate the length for you
		float xVal = (v[2][1][1] - v[0][1][1])/2;
		float yVal = (v[1][2][1] - v[1][0][1])/2;
		float zVal = (v[1][1][2] - v[1][1][0])/2;

		gradientMagnitude = length(tgt::vec3(xVal*xVal + yVal*yVal + zVal*zVal));

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
