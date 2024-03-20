//////////
//////////Vector 3 files
//////////Written by Tanapat Somrid
/////////Starting 10/03/2024
//////// Most Recent Update 16/03/2024
//////// Most Recent change: 
//////// Adapted from my own Vector2 class from 2021


#pragma once
#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>
#include <iostream>

template <typename T> class Vector3 {
public:

	/// <summary>
/// Default Constructor
/// </summary>
	Vector3() {};
	/// <summary>
	/// Individual Coordinate Constructor
	/// </summary>
	Vector3(T x, T y, T z) : Vector3() { this->x = x; this->y = y; this->z = z; }
	/// <summary>
	/// Assignment with different typename
	/// </summary>
	template <typename U> explicit Vector3(const Vector3<U>& difPrimitive) {
		x = static_cast<T>(difPrimitive.x); y = static_cast<T>(difPrimitive.y); z = static_cast<T>(difPrimitive.z);
	}
	//Copy Constructors
	Vector3(const Vector3& rhs) { x = rhs.x; y = rhs.y; z = rhs.z; }//Required for assignment and required for below operatorr//	Vector3<int> dif = position;
	Vector3& operator =(const Vector3& copy) { x = copy.x; y = copy.y; z = copy.z; return *this; }//So this will copy/assign//		Node* endNode = new Node(endPos);
	T x, y, z;


	double Magnitude() const {
		return (double)(sqrt(abs(x * x) + abs(y * y) + abs(z * z)));
	}
	Vector3<double> Normalize() const {
		return Vector3<double>(*this / this->Magnitude());
	}

};

template <typename T> Vector3<T> operator +(const Vector3<T>& lhs, const Vector3<T>& rhs) {
	return Vector3<T>((lhs.x + rhs.x), (lhs.y + rhs.y), (lhs.z + rhs.z));
}
template <typename T> Vector3<T> operator -(const Vector3<T>& lhs, const Vector3<T>& rhs) {
	return Vector3<T>((lhs.x - rhs.x), (lhs.y - rhs.y), (lhs.z - rhs.z));
}
template <typename T> Vector3<float> operator -(const Vector3<T>& lhs, const Vector3<float>& rhs) {
	return Vector3<float>((lhs.x - rhs.x), (lhs.y - rhs.y), (lhs.z - rhs.z));
}
template <typename T, typename U> Vector3<float> operator -(const Vector3<T>& lhs, const Vector3<U>& rhs) {
	return Vector3<float>((lhs.x - rhs.x), (lhs.y - rhs.y), (lhs.z - rhs.z));
}
template <typename T> Vector3<T> operator /(const Vector3<T>& lhs, const T& rhs) {
	return Vector3<T>((lhs.x / rhs), (lhs.y / rhs), (lhs.z / rhs));
}
template <typename T> Vector3<T> operator *(const Vector3<T>& lhs, const T& rhs) {
	return Vector3<T>((lhs.x * rhs), (lhs.y * rhs), (lhs.z * rhs));
}

template <typename T> Vector3<T> operator *(const Vector3<T>& lhs, const double rhs) {
	return Vector3<T>((lhs.x * rhs), (lhs.y * rhs), (lhs.z * rhs));
}
template <typename T> Vector3<T> operator *(const Vector3<T>& lhs, const Vector3<T>& rhs) {
	return Vector3<T>((lhs.x * rhs.x), (lhs.y * rhs.y), (lhs.z * rhs.z));
}
template <typename T> void operator +=(Vector3<T>& lhs, const Vector3<T>& rhs) {
	lhs.x = lhs.x + rhs.x;
	lhs.y = lhs.y + rhs.y;
	lhs.z = lhs.z + rhs.z;
	return;
}

template <typename T> void operator -=(Vector3<T>& lhs, const Vector3<T>& rhs) {
	lhs.x = lhs.x - rhs.x;
	lhs.y = lhs.y - rhs.y;
	lhs.z = lhs.z - rhs.z;
	return;
}
template <typename T> void operator /=(const Vector3<T>& lhs, const T& rhs) {
	lhs.x = lhs.x / rhs.x;
	lhs.y = lhs.y / rhs.y;
	lhs.y = lhs.z / rhs.z;
	return;
}
template <typename T> bool operator ==(const Vector3<T>& lhs, const Vector3<T>& rhs) {
	return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}
template <typename T> bool operator !=(const Vector3<T>& lhs, const Vector3<T>& rhs) {
	return !(lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

template<typename T> std::ostream& operator<<(std::ostream& stream, Vector3<T> const& vector)
{
	return stream << "X:" << vector.x << " Y:" << vector.y << " Z:" << vector.z;
}

#endif // !VECTOR3_H
