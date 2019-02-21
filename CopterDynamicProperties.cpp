#include "pch.h"
#include "CopterDynamicProperties.h"
#include "Support.h"


CopterDynamicProperties::
CopterDynamicProperties(Copter& copter) : copter_(copter)
{
	fusRotMatrix_ = mat(NUM_OF_SPACE_DIM, NUM_OF_SPACE_DIM);
	fusInertiaMomentCur_ = mat(NUM_OF_SPACE_DIM, NUM_OF_SPACE_DIM);
	copterInertiaMomentCur_ = mat(NUM_OF_SPACE_DIM, NUM_OF_SPACE_DIM);
	const int numOfEngines = copter_.getNumOfEngines();
	enginesRotMatrices_.resize(numOfEngines);
	enginesRotMatricesT_.resize(numOfEngines);
	engInertiaMomentsCur_.resize(numOfEngines);
	engInertiaMomentsCurRelative_.resize(numOfEngines);
}

void CopterDynamicProperties::computeCurProperties()
{
	fusRotMatrix_ = copter_.fuselage.rotationQuat.rotationMatrix();
	fusInertiaMomentCur_ = fusRotMatrix_ * copter_.fuselage.inertiaMoment *
		fusRotMatrix_.t();
	copterInertiaMomentCur_ = fusInertiaMomentCur_;
	const mat matrixI(NUM_OF_SPACE_DIM, NUM_OF_SPACE_DIM, fill::eye);
	const int numOfEngines = copter_.getNumOfEngines();
	enginesRotMatrices_.resize(numOfEngines);
	enginesRotMatricesT_.resize(numOfEngines);
	engInertiaMomentsCur_.resize(numOfEngines);
	engInertiaMomentsCurRelative_.resize(numOfEngines);
	for (int i = 0; i < numOfEngines; i++) {
		const auto& engine = copter_.engines[i];
		auto engineRotationQuat = copter_.fuselage.rotationQuat *
			copter_.fusEngineQuaternions[i] *
			engine.rotationQuat;
		enginesRotMatrices_[i] = engineRotationQuat.rotationMatrix();
		enginesRotMatricesT_[i] = enginesRotMatrices_[i].t();
		engInertiaMomentsCur_[i] = enginesRotMatrices_[i] * engine.inertiaMoment *
			enginesRotMatricesT_[i];
		vec vectorFusEngineMcCur = enginesRotMatrices_[i] *
			copter_.getVectorFusEngineMc(i);
		auto vectorFusEngineMcCurT = rowvec(NUM_OF_SPACE_DIM);
		for (int j = 0; j < NUM_OF_SPACE_DIM; j++) {
			vectorFusEngineMcCurT[j] = vectorFusEngineMcCur(j);
		}
		engInertiaMomentsCurRelative_[i] = engine.mass *
		(matrixI * arma::norm(vectorFusEngineMcCur) -
			vectorFusEngineMcCur * vectorFusEngineMcCurT);
		copterInertiaMomentCur_ += engInertiaMomentsCur_[i] +
			engInertiaMomentsCurRelative_[i];
	}
}

mat CopterDynamicProperties::getFusRotMatrix() const
{
	return fusRotMatrix_;
}

mat CopterDynamicProperties::getFusInertiaMomentCur() const
{
	return fusInertiaMomentCur_;
}

mat CopterDynamicProperties::getCopterInertiaMomentCur() const
{
	return copterInertiaMomentCur_;
}

std::vector<mat> CopterDynamicProperties::getEnginesRotMatrices() const
{
	return enginesRotMatrices_;
}

std::vector<mat> CopterDynamicProperties::getEnginesRotMatricesT() const
{
	return enginesRotMatricesT_;
}

std::vector<mat> CopterDynamicProperties::getEngInertiaMomentsCur() const
{
	return engInertiaMomentsCur_;
}

std::vector<mat>
CopterDynamicProperties::getEngInertiaMomentsCurRelative() const
{
	return engInertiaMomentsCurRelative_;
}

