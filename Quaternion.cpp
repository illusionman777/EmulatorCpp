#include "pch.h"

#include <cmath>

#include "Quaternion.h"
#include "Support.h"


Quaternion::Quaternion()
{
	real_ = 1.0;
	imaginary_ = vec(3);
	imaginary_.fill(0.0);
}

Quaternion::Quaternion(const double w, const double x,
                       const double y, const double z)
{
	real_ = w;
	imaginary_ = vec(3);
	imaginary_ << x << y << z;
}

Quaternion::Quaternion(const double real, const vec& imaginary)
{
	real_ = real;
	imaginary_ = imaginary;
}

Quaternion::Quaternion(const vec& vector)
{
	real_ = 0.0;
	imaginary_ = vector;
}

double Quaternion::getReal() const
{
	return real_;
}

vec Quaternion::getImaginary() const
{
	return imaginary_;
}

double Quaternion::at(const int i) const
{
	if (i == 0)
		return real_;
	return imaginary_(i - 1);
}

Quaternion Quaternion::conjugate() const
{
	return Quaternion(real_, -imaginary_);
}

double Quaternion::norm() const
{
	const double result = sqrt(real_ * real_ + imaginary_[0] * imaginary_[0] +
		imaginary_[1] * imaginary_[1] + imaginary_[2] * imaginary_[2]);
	return result;
}

void Quaternion::normalize()
{
	const double norm = this->norm();
	if (isZero(norm))
		return;
	this->real_ /= norm;
	this->imaginary_ /= norm;
}

Quaternion Quaternion::normalized() const
{
	const double norm = this->norm();
	if (isZero(norm))
		return *this;
	return Quaternion(real_ / norm, imaginary_ / norm);
}


mat Quaternion::rotationMatrix() const
{
	auto result = mat(3, 3);
	double w = real_;
	double x = imaginary_[0];
	double y = imaginary_[1];
	double z = imaginary_[2];
	const double norm = this->norm();
	if (!isZero(norm) && !isZero(norm - 1.0)) {
		w /= norm;
		x /= norm;
		y /= norm;
		z /= norm;
	}
	const double x2 = 2.0 * x * x;
	const double y2 = 2.0 * y * y;
	const double z2 = 2.0 * z * z;
	const double xw = 2.0 * x * w;
	const double yw = 2.0 * y * w;
	const double zw = 2.0 * z * w;
	const double xy = 2.0 * x * y;
	const double xz = 2.0 * x * z;
	const double yz = 2.0 * y * z;
	result(0, 0) = 1.0 - y2 - z2;
	result(0, 1) = xy - zw;
	result(0, 2) = xz + yw;
	result(1, 0) = xy + zw;
	result(1, 1) = 1.0 - x2 - z2;
	result(1, 2) = yz - xw;
	result(2, 0) = xz - yw;
	result(2, 1) = yz + xw;
	result(2, 2) = 1.0 - x2 - y2;
	return result;
}

Quaternion Quaternion::operator+() const
{
	return *this;
}

Quaternion Quaternion::operator-()
{
	this->real_ *= -1;
	this->imaginary_ *= -1;
	return *this;
}

Quaternion operator+(const Quaternion& left, const Quaternion& right)
{
	return Quaternion(left.real_ + right.real_,
	                  left.imaginary_ + right.imaginary_);
}

Quaternion operator+(const Quaternion& left, const double right)
{
	return Quaternion(left.real_ + right, left.imaginary_ + right);
}

Quaternion operator+(const double left, const Quaternion& right)
{
	return Quaternion(left + right.real_, left + right.imaginary_);
}

Quaternion operator-(const Quaternion& left, const Quaternion& right)
{
	return Quaternion(left.real_ - right.real_,
	                  left.imaginary_ - right.imaginary_);
}

Quaternion operator-(const Quaternion& left, const double right)
{
	return Quaternion(left.real_ + right, left.imaginary_ + right);
}

Quaternion operator-(const double left, const Quaternion& right)
{
	return Quaternion(left + right.real_, left + right.imaginary_);
}

Quaternion operator*(const Quaternion& left, const Quaternion& right)
{
	const double real = left.real_ * right.real_ - 
		dot(left.imaginary_, right.imaginary_);
	const vec imaginary = left.real_ * right.imaginary_ +
		right.real_ * left.imaginary_ +
		cross(left.imaginary_, right.imaginary_);
	return Quaternion(real, imaginary);
}

Quaternion operator*(const Quaternion& left, const double right)
{
	return Quaternion(left.real_ * right, left.imaginary_ * right);
}

Quaternion operator*(const double left, const Quaternion& right)
{
	return Quaternion(right.real_ * left, right.imaginary_ * left);
}

Quaternion operator*(const vec& left, const Quaternion& right)
{
	const double real = -dot(left, right.imaginary_);
	const vec imaginary = right.real_ * left + cross(left, right.imaginary_);
	return Quaternion(real, imaginary);
}

Quaternion operator*(const Quaternion& left, const vec& right)
{
	const double real = -dot(left.imaginary_, right);
	const vec imaginary = left.real_ * right + cross(left.imaginary_, right);
	return Quaternion(real, imaginary);
}

Quaternion operator/(const Quaternion& left, const double right)
{
	return Quaternion(left.real_ / right, left.imaginary_ / right);
}

Quaternion Quaternion::operator+=(const Quaternion& right)
{
	this->real_ += right.real_;
	this->imaginary_ += right.imaginary_;
	return *this;
}

Quaternion Quaternion::operator+=(const double right)
{
	this->real_ += right;
	this->imaginary_ += right;
	return *this;
}

Quaternion Quaternion::operator-=(const Quaternion& right)
{
	this->real_ -= right.real_;
	this->imaginary_ -= right.imaginary_;
	return *this;
}

Quaternion Quaternion::operator-=(const double right)
{
	this->real_ -= right;
	this->imaginary_ -= right;
	return *this;
}

Quaternion Quaternion::operator*=(const Quaternion& right)
{
	const double real = this->real_ * right.real_ - 
		dot(this->imaginary_, right.imaginary_);
	const vec imaginary = this->real_ * right.imaginary_ +
		right.real_ * this->imaginary_ +
		cross(this->imaginary_, right.imaginary_);
	this->real_ = real;
	this->imaginary_ = imaginary;
	return *this;
}

Quaternion Quaternion::operator*=(const double right)
{
	this->real_ *= right;
	this->imaginary_ *= right;
	return *this;
}

Quaternion Quaternion::operator*=(const vec& right)
{
	const double real = -dot(this->imaginary_, right);
	const vec imaginary = this->real_ * right + cross(this->imaginary_, right);
	this->real_ = real;
	this->imaginary_ = imaginary;
	return *this;
}

Quaternion Quaternion::operator/=(const double right)
{
	this->real_ /= right;
	this->imaginary_ /= right;
	return *this;
}
