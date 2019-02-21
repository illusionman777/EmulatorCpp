#include "pch.h"
#include "RungeKuttaMethod.h"


RungeKuttaMethod::RungeKuttaMethod()
{
	function_ = nullptr;
	dt_ = 0.0;
	timeCur_ = 0.0;
}

RungeKuttaMethod::RungeKuttaMethod(PhysicalFunction* function, const double dt)
{
	function_ = function;
	dt_ = dt;
	timeCur_ = 0.0;
}

double RungeKuttaMethod::getCurrentTime() const
{
	return timeCur_;
}

mat RungeKuttaMethod::computeNextState(const mat& stateCurrent)
{
	const mat k1 = function_->compute(timeCur_, stateCurrent);
	const mat k2 = function_->compute(timeCur_ + dt_ / 2.0, stateCurrent + k1 * dt_ / 2.0);
	const mat k3 = function_->compute(timeCur_ + dt_ / 2.0, stateCurrent + k2 * dt_ / 2.0);
	const mat k4 = function_->compute(timeCur_ + dt_, stateCurrent + k3 * dt_);
	timeCur_ += dt_;
	mat result = stateCurrent + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0 * dt_;
	return result;
}

