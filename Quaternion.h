#pragma once
#include "pch.h"
#include <armadillo>

using namespace arma;


class Quaternion
{
public:
	Quaternion();
	Quaternion(double w, double x, double y, double z);
	Quaternion(double real, const vec& imaginary);
	explicit Quaternion(const vec& vector);

	double getReal() const;
	vec getImaginary() const;
	double at(int i) const;

	Quaternion conjugate() const;
	mat rotationMatrix() const;
	double norm() const;
	void normalize();
	Quaternion normalized() const;

	Quaternion operator +() const;
	Quaternion operator -();

	friend Quaternion operator
	+(const Quaternion& left, const Quaternion& right);
	friend Quaternion operator +(double left, const Quaternion& right);
	friend Quaternion operator +(const Quaternion& left, double right);
	friend Quaternion operator -(const Quaternion& left,
	                             const Quaternion& right);
	friend Quaternion operator -(double left, const Quaternion& right);
	friend Quaternion operator -(const Quaternion& left, double right);
	friend Quaternion operator *(const Quaternion& left,
	                             const Quaternion& right);
	friend Quaternion operator *(double left, const Quaternion& right);
	friend Quaternion operator *(const Quaternion& left, double right);
	friend Quaternion operator *(const vec& left, const Quaternion& right);
	friend Quaternion operator *(const Quaternion& left, const vec& right);
	friend Quaternion operator /(const Quaternion& left, double right);

	Quaternion operator +=(const Quaternion& right);
	Quaternion operator +=(double right);
	Quaternion operator -=(const Quaternion& right);
	Quaternion operator -=(double right);
	Quaternion operator *=(const Quaternion& right);
	Quaternion operator *=(double right);
	Quaternion operator *=(const vec& right);
	Quaternion operator /=(double right);
private:
	double real_;
	vec imaginary_;
};
