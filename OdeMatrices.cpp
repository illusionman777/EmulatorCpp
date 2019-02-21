#include "pch.h"
#include "OdeMatrices.h"
#include "Support.h"


OdeMatrices::OdeMatrices(Copter& copter,
                         CopterDynamicProperties& dynamicProperties) :
	copter_(copter), dynamicProperties_(dynamicProperties)
{
	matrixDimension_ = NUM_OF_SPACE_DIM +
		copter_.getNumOfEngines();
	coefMatrix_ = zeros(matrixDimension_, matrixDimension_);
	constantTermsVector_ = vec(matrixDimension_);
}

void OdeMatrices::compute(const ExternalForces& exForces,
                          const InternalForces& inForces)
{
	computeMatrix();
	computeVector(exForces, inForces);
}

mat OdeMatrices::getCoefMatrix() const
{
	return coefMatrix_;
}

vec OdeMatrices::getConstantTermsVector() const
{
	return constantTermsVector_;
}

//          |I.xx     I.xy     I.xz     Ie0.xz Ie1.xz ...|
//          |I.yx     I.yy     I.yz     Ie0.yz Ie1.yz ...|
//          |I.zx     I.zy     I.zz     Ie0.zz Ie1.zz ...|
//  mat A = |Ie0.zx   Ie0.zy   Ie0.zz   Ie0.zz 0      ...|
//          |Ie1.zx   Ie1.zy   Ie1.zz   0      Ie1.zz ...|
//          |...                                         |
//          |Same lines for every engine                 |
void OdeMatrices::computeMatrix()
{
	const mat copterInertiaMomentCur = dynamicProperties_.getCopterInertiaMomentCur();
	const auto engInertiaMomentsCur = dynamicProperties_.getEngInertiaMomentsCur();
	const auto engInertiaMomentsRelative = dynamicProperties_.getEngInertiaMomentsCurRelative();
	const auto engRotMatrices = dynamicProperties_.getEnginesRotMatrices();
	const auto engRotMatricesT = dynamicProperties_.getEnginesRotMatricesT();
	for (int i = 0; i < matrixDimension_; i++) {
		for (int j = 0; j < matrixDimension_; j++) {
			if (i < NUM_OF_SPACE_DIM && j < NUM_OF_SPACE_DIM) {
				coefMatrix_(i, j) = copterInertiaMomentCur(i, j);
				continue;
			}
			if (i >= NUM_OF_SPACE_DIM && j < NUM_OF_SPACE_DIM) {
				mat engInertiaMomentTmp = engInertiaMomentsCur[i - NUM_OF_SPACE_DIM] +
					engInertiaMomentsRelative[i - NUM_OF_SPACE_DIM];
				engInertiaMomentTmp = engRotMatricesT[i - NUM_OF_SPACE_DIM] *
					engInertiaMomentTmp * engRotMatrices[i - NUM_OF_SPACE_DIM];
				coefMatrix_(i, j) = engInertiaMomentTmp(2, j);
				continue;
			}
			if (i < NUM_OF_SPACE_DIM && j >= NUM_OF_SPACE_DIM) {
				mat engInertiaMomentTmp = engInertiaMomentsCur[j - NUM_OF_SPACE_DIM] *
					engRotMatrices[j - NUM_OF_SPACE_DIM];
				coefMatrix_(i, j) = engInertiaMomentTmp(i, 2);
				continue;
			}
			if (i == j) {
				coefMatrix_(i, j) = copter_.engines[i - NUM_OF_SPACE_DIM].inertiaMoment(2, 2);
			}
		}
	}
}

void OdeMatrices::computeVector(const ExternalForces& exForces,
                                const InternalForces& inForces)
{
	const auto engRotMatrices = dynamicProperties_.getEnginesRotMatrices();
	const vec mainMoment = exForces.getMainMoment();
	const auto enginesAeroMoments = exForces.getEnginesAeroMoments();
	const vec gyroscopicMoment = inForces.getGyroscopicMoment();
	const auto enginesGyroMoments = inForces.getEnginesGyroMoments();
	const auto enginesDriveMoments = inForces.getEnginesDriveMoments();
	for (int i = 0; i < matrixDimension_; i++) {
		if (i < NUM_OF_SPACE_DIM) {
			constantTermsVector_[i] = mainMoment[i] - gyroscopicMoment[i];
		}
		else {
			vec engineGyroMomentRelative = engRotMatrices[i - NUM_OF_SPACE_DIM] *
				enginesGyroMoments[i - NUM_OF_SPACE_DIM];
			constantTermsVector_[i] = enginesAeroMoments[i - NUM_OF_SPACE_DIM] +
				enginesDriveMoments[i - NUM_OF_SPACE_DIM] - engineGyroMomentRelative[2];
		}
	}
}
