#pragma once
#include "glm/geometric.hpp"
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/type_aligned.hpp>

// sourced from: https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
template<typename T, glm::qualifier P>
auto static approximate_normal(std::vector<glm::vec<4, T, P>>& points) -> glm::vec3 {
	// calculate centroid by through coefficient average
	glm::vec<4, T, P> centroid { 0, 0, 0, 0 };
	for (auto p = points.cbegin(); p != points.cend(); p++) {
		centroid += *p;
	}
	T recip = 1.0 / (T)points.size();
	centroid *= recip;

	// covariance matrix excluding symmetries
	T xx = 0.0; T xy = 0.0; T xz = 0.0;
	T yy = 0.0; T yz = 0.0; T zz = 0.0;
	for (auto point_it = points.cbegin(); point_it != points.cend(); point_it++) {
		auto r = *point_it - centroid;
		xx += r.x * r.x;
		xy += r.x * r.y;
		xz += r.x * r.z;
		yy += r.y * r.y;
		yz += r.y * r.z;
		zz += r.z * r.z;
	}
	xx *= recip;
	xy *= recip;
	xz *= recip;
	yy *= recip;
	yz *= recip;
	zz *= recip;

	// weighting linear regression based on square determinant
	glm::vec<4, T, P> weighted_dir = { 0, 0, 0, 0 };

	// determinant x
	{
		T det_x = yy*zz - yz*yz;
		glm::vec<4, T, P> axis_dir = {
			det_x,
			xz*yz - xy*zz,
			xy*yz - xz*yy,
			0.0
		};
		T weight = det_x * det_x;
		if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant y
	{
		T det_y = xx*zz - xz*xz;
		glm::vec<4, T, P> axis_dir = {
			xz*yz - xy*zz,
			det_y,
			xy*xz - yz*xx,
			0.0
		};
		T weight = det_y * det_y;
		if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}
	// determinant z
	{
		T det_z = xx*yy - xy*xy;
		glm::vec<4, T, P> axis_dir = {
			xy*yz - xz*yy,
			xy*xz - yz*xx,
			det_z,
			0.0
		};
		T weight = det_z * det_z;
		if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
		weighted_dir += axis_dir * weight;
	}

	// return normalized weighted direction as surface normal
    weighted_dir = glm::normalize(weighted_dir);
	return { (float)weighted_dir.x, (float)weighted_dir.y, (float)weighted_dir.z };
}