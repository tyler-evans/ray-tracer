// The JSON library allows you to reference JSON arrays like C++ vectors and JSON objects like C++ maps.

#include "raytracer.h"

#include <iostream>
#include <fstream>
#include <string>
#include <glm/glm.hpp>
#include <algorithm>
#include <glm/gtx/string_cast.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>


#include "json.hpp"

using json = nlohmann::json;

const char *PATH = "scenes/";
const float MAX_T = 1e6;
const float EPS = 0.001;

double fov = 60;
colour3 background_colour(0, 0, 0);

json scene;

/****************************************************************************/

// here are some potentially useful utility functions

json find(json &j, const std::string key, const std::string value) {
	json::iterator it;
	for (it = j.begin(); it != j.end(); ++it) {
		if (it->find(key) != it->end()) {
			if ((*it)[key] == value) {
				return *it;
			}
		}
	}
	return json();
}

glm::vec3 vector_to_vec3(const std::vector<float> &v) {
	return glm::vec3(v[0], v[1], v[2]);
}

/****************************************************************************/

void choose_scene(char const *fn) {
	if (fn == NULL) {
		std::cout << "Using default input file " << PATH << "c.json\n";
		fn = "c";
	}
	
	std::string fname = PATH + std::string(fn) + ".json";
	std::fstream in(fname);
	if (!in.is_open()) {
		std::cout << "Unable to open scene file " << fname << std::endl;
		exit(EXIT_FAILURE);
	}
	std::cout << "Loaded scene file " << fname << std::endl;
	
	in >> scene;
	
	json camera = scene["camera"];
	// these are optional parameters (otherwise they default to the values initialized earlier)
	if (camera.find("field") != camera.end()) {
		fov = camera["field"];
		std::cout << "Setting fov to " << fov << " degrees.\n";
	}
	if (camera.find("background") != camera.end()) {
		background_colour = vector_to_vec3(camera["background"]);
		std::cout << "Setting background colour to " << glm::to_string(background_colour) << std::endl;
	}
}


/************************************************************************************************/

struct RayCastSolution {
	float t;
	float further_t;
	json object;
	int triangle_index;
	bool intersection;
	colour3 colour;
};

struct Plane {
	point3 pos;
	point3 n;
};

struct Sphere {
	point3 center;
	float r;
};

struct Triangle {
	point3 a, b, c;
	point3 n;
};

Plane get_plane(json &object) {
	Plane result;
	result.pos = vector_to_vec3(object["position"]);
	result.n = vector_to_vec3(object["normal"]);
	return result;
}

Sphere get_sphere(json &object) {
	Sphere result;
	result.center = vector_to_vec3(object["position"]);
	result.r = float(object["radius"]);
	return result;
}

Triangle get_triangle(json &object, int index) {
	Triangle result;
	//std::vector<std::vector<std::vector<float>>> triangles = object["triangles"];
	std::vector<std::vector<float>> triangle = object["triangles"][index];

	result.a = vector_to_vec3(triangle[0]);
	result.b = vector_to_vec3(triangle[1]);
	result.c = vector_to_vec3(triangle[2]);

	result.n = glm::normalize(glm::cross(result.b - result.a, result.c - result.a));

	return result;
}

float plane_intersection(point3 eye, point3 direction, Plane plane) {
	float t = glm::dot(plane.n, plane.pos - eye) / glm::dot(plane.n, direction);
	return t > 0.0? t : MAX_T;
}

float triangle_intersetion(point3 e, point3 d, Triangle triangle){

	float t = glm::dot(triangle.n, triangle.a - e) / glm::dot(triangle.n, d);
	point3 x = e + t * d;

	bool triangle_test_0 = glm::dot(glm::cross(triangle.b - triangle.a, x - triangle.a), triangle.n) > 0.0;
	bool triangle_test_1 = glm::dot(glm::cross(triangle.c - triangle.b, x - triangle.b), triangle.n) > 0.0;
	bool triangle_test_2 = glm::dot(glm::cross(triangle.a - triangle.c, x - triangle.c), triangle.n) > 0.0;

	return triangle_test_0 && triangle_test_1 && triangle_test_2 ? t : MAX_T;
}

float sphere_intersection(point3 e, point3 d, Sphere sphere, float &store_further_t) {
	float a = glm::dot(d, d);
	float b = 2.0f*glm::dot(d, e - sphere.center);
	float c = glm::dot(e - sphere.center, e - sphere.center) - std::pow(sphere.r, 2);

	float discriminant = b * b - 4 * a*c;
	if (discriminant >= 0) {
		float solution_0 = (-1.0f*b + glm::sqrt(discriminant)) / (2.0f*a);
		float solution_1 = (-1.0f*b - glm::sqrt(discriminant)) / (2.0f*a);

		float t = glm::min(solution_0, solution_1);
		float further_t = glm::max(solution_0, solution_1);
		store_further_t = further_t;
		
		return t;
	}
	return MAX_T;
}


RayCastSolution cast_ray(point3 e, point3 d, bool find_any) {

	json &objects = scene["objects"];
	RayCastSolution solution;
	solution.t = MAX_T;
	solution.intersection = false;
	d = glm::normalize(d);

	for (json::iterator it = objects.begin(); it != objects.end(); ++it) {
		json &object = *it;

		if (object["type"] == "mesh") {
			std::vector<std::vector<std::vector<float>>> triangles = object["triangles"];
			for (int i = 0; i < triangles.size(); i++) {
				Triangle triangle = get_triangle(object, i);
				point3 a = triangle.a;
				point3 b = triangle.b;
				point3 c = triangle.c;
				point3 plane_n = triangle.n;

				float t = triangle_intersetion(e, d, triangle);

				if (t < solution.t && t>=0) { // intersection
					solution.t = t;
					solution.object = object;
					solution.triangle_index = i;
					solution.intersection = true;
					if (find_any)
						return solution;
				}

			}

		}
		else if (object["type"] == "sphere") {
			Sphere sphere = get_sphere(object);
			float further_t;
			float t = sphere_intersection(e, d, sphere, further_t);
			if (t < solution.t && t >= 0) {
				solution.t = t;
				solution.further_t = further_t;
				solution.object = object;
				solution.intersection = true;
				if (find_any)
					return solution;
			}
		}
		else if (object["type"] == "plane") {
			Plane plane = get_plane(object);
			float t = plane_intersection(e, d, plane);

			if (t < solution.t && t >= 0) {
				solution.t = t;
				solution.object = object;
				solution.intersection = true;
				if (find_any)
					return solution;
			}
		}

	}

	return solution;
}

point3 reflect(point3 V, point3 N) {
	return -glm::reflect(V, N);
}

point3 refract_vector(point3 N, point3 V_i, float n_i, float n_r) {
	float root_arg = 1.0f - (std::pow(n_i, 2) * (1.0f - std::pow(glm::dot(V_i, N), 2))) / std::pow(n_r, 2);

	if (root_arg < 0.0)
		return point3(0.0, 0.0, 0.0);

	return n_i * (V_i - N * glm::dot(V_i, N)) / n_r - N * std::pow(root_arg, 0.5f);
}

float calculate_I_d(point3 N, point3 L) {
	float I_d = glm::dot(glm::normalize(N), glm::normalize(L));
	return std::max(I_d, 0.0f);
}

float calculate_I_s(point3 N, point3 L, point3 V, float alpha) {
	N = glm::normalize(N);
	L = glm::normalize(L);
	V = glm::normalize(V);

	point3 R = reflect(L, N);

	float base = std::min(std::max(glm::dot(R, V), 0.0f), 1.0f);
	float I_s = std::pow(base, alpha);
	return I_s;
}

colour3 calculate_lighting(point3 normal, point3 x, point3 look, json &material) {
	json &lights = scene["lights"];

	colour3 ambient = material.find("ambient") != material.end() ? vector_to_vec3(material["ambient"]) : colour3(0.0, 0.0, 0.0);
	colour3 diffuse = material.find("diffuse") != material.end() ? vector_to_vec3(material["diffuse"]) : colour3(0.0, 0.0, 0.0);
	colour3 specular = material.find("specular") != material.end() ? vector_to_vec3(material["specular"]) : colour3(0.0, 0.0, 0.0);
	float shininess = material.find("shininess") != material.end() ? float(material["shininess"]) : 0.0f;

	colour3 total_colour = colour3(0.0, 0.0, 0.0);

	for (json::iterator it = lights.begin(); it != lights.end(); ++it) {
		json &light = *it;
		colour3 light_colour = vector_to_vec3(light["color"]);

		if (light["type"] == "ambient") {
			total_colour += light_colour * ambient;
		}
		else if (light["type"] == "directional") {
			point3 light_direction = vector_to_vec3(light["direction"]);
			float I_d = calculate_I_d(normal, -light_direction);
			float I_s = calculate_I_s(normal, -light_direction, look, shininess);

			float shadow_test = cast_ray(x, -light_direction, true).t;
			if (EPS < shadow_test && shadow_test < MAX_T) {
				I_d = 0.0;
				I_s = 0.0;
			}

			total_colour += light_colour * (I_d * diffuse + I_s * specular);
		}
		else if (light["type"] == "point") {
			point3 light_position = vector_to_vec3(light["position"]);
			float I_d = calculate_I_d(normal, light_position - x);
			float I_s = calculate_I_s(normal, light_position - x, look, shininess);

			float shadow_test = cast_ray(x, light_position - x, true).t;
			float distance_to_light = glm::length(light_position - x);
			if (EPS < shadow_test && shadow_test < MAX_T && shadow_test < distance_to_light){
				I_d = 0.0;
				I_s = 0.0;
			}

			total_colour += light_colour * (I_d * diffuse + I_s * specular);
		}
		else if (light["type"] == "spot") {
			point3 light_position = vector_to_vec3(light["position"]);
			point3 light_direction = vector_to_vec3(light["direction"]);
			float cutoff_angle = float(light["cutoff"]) * M_PI / 180.0;

			point3 point_to_light_vec = glm::normalize(light_position - x);

			float I_d = calculate_I_d(normal, point_to_light_vec);
			float I_s = calculate_I_s(normal, point_to_light_vec, look, shininess);

			float shadow_test = cast_ray(x, point_to_light_vec, true).t;
			float distance_to_light = glm::length(point_to_light_vec);
			float angle = acos(glm::dot(-point_to_light_vec, light_direction) / glm::length(light_direction));
			if ((EPS < shadow_test && shadow_test < MAX_T && shadow_test < distance_to_light) || (angle > 90 || angle > cutoff_angle)) {
				I_d = 0.0;
				I_s = 0.0;
			}

			total_colour += light_colour * (I_d * diffuse + I_s * specular);

		}
	}

	return total_colour;
}


const int MAX_LEVEL = 5;
colour3 recursive_trace(point3 e, point3 d, int level, bool &found_intersection) {

	colour3 colour = background_colour;

	RayCastSolution solution = cast_ray(e, d, false);
	json &object = solution.object;
	float t = solution.t;
	json &material = object["material"];

	if (t == MAX_T) 
		return colour;
	found_intersection = true;

	std::string object_type = object["type"];
	point3 x = e + t * d;
	point3 normal;
	Sphere sphere;

	if (object_type == "sphere") {
		sphere = get_sphere(object);
		normal = glm::normalize(x - sphere.center);
	}
	else if (object_type == "plane")
		normal = get_plane(object).n;
	else if (object_type == "mesh")
		normal = get_triangle(object, solution.triangle_index).n;

	point3 R = reflect(e - x, normal);
	colour = calculate_lighting(normal, x, e - x, material);

	if (material.find("reflective") != material.end() && level < MAX_LEVEL) {
			colour3 reflective = vector_to_vec3(material["reflective"]);
			colour += reflective * recursive_trace(x + EPS * R, R, level + 1, found_intersection);
	}

	if (material.find("transmissive") != material.end() && level < MAX_LEVEL) {
		colour3 transmissive = vector_to_vec3(material["transmissive"]);
		
		if (material.find("refraction") != material.end()) {
			float refraction = material["refraction"];
			d = refract_vector(normal, d, refraction, 1.0);
		}
		colour = (1.0f - transmissive)*colour + (transmissive)*recursive_trace(x + EPS * d, d, level + 1, found_intersection);
	}

	return colour;
}



bool trace(const point3 &e, const point3 &s, colour3 &colour, bool pick) {
	point3 d = glm::normalize(s - e);

	bool result = false;
	colour = recursive_trace(e, d, 0, result);
	return result;

}