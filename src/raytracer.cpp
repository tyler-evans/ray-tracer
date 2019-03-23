// The JSON library allows you to reference JSON arrays like C++ vectors and JSON objects like C++ maps.

#include "raytracer.h"

#include <iostream>
#include <fstream>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

#include "json.hpp"

using json = nlohmann::json;

const char *PATH = "scenes/";

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


float plane_intersection(point3 e, point3 d, point3 n, point3 a) {
	float t = glm::dot(n, a - e) / glm::dot(n, d);
	if (t > 0.0)
		return t;
	else
		return 1e6;
}

float triangle_intersetion(point3 e, point3 d, std::vector<std::vector<float>> triangle) {
	point3 a = vector_to_vec3(triangle[0]);
	point3 b = vector_to_vec3(triangle[1]);
	point3 c = vector_to_vec3(triangle[2]);

	point3 plane_n = glm::cross(b - a, c - a);
	plane_n = glm::normalize(plane_n);
	float t = glm::dot(plane_n, a - e) / glm::dot(plane_n, d);

	point3 x = e + t * d;

	bool triangle_test_0 = glm::dot(glm::cross(b - a, x - a), plane_n) > 0.0;
	bool triangle_test_1 = glm::dot(glm::cross(c - b, x - b), plane_n) > 0.0;
	bool triangle_test_2 = glm::dot(glm::cross(a - c, x - c), plane_n) > 0.0;

	if (triangle_test_0 && triangle_test_1 && triangle_test_2)
		return t;
	else
		return 1e6;
}

float sphere_intersection(point3 e, point3 d, point3 center, float r) {
	float a = glm::dot(d, d);
	float b = 2.0*glm::dot(d, e - center);
	float c = glm::dot(e - center, e - center) - r * r;

	float discriminant = b * b - 4 * a*c;
	if (discriminant >= 0) {
		float solution_0 = (-1.0*b + glm::sqrt(discriminant)) / (2.0*a);
		float solution_1 = (-1.0*b - glm::sqrt(discriminant)) / (2.0*a);

		float t = glm::min(solution_0, solution_1);
		return t;
	}
	return 1e6;
}


colour3 calculate_lighting(point3 normal, point3 x, colour3 ambient, colour3 diffuse, colour3 specular, float shininess) {

	float id = 1.0;
	float kd = 1.0;

	point3 light_source = glm::vec3(-1.0, 8.0, -5.0);
	float I_d = glm::dot(glm::normalize(normal), glm::normalize(light_source - x));

	return diffuse * I_d * colour3(0.8, 0.8, 0.0);

}


bool trace(const point3 &e, const point3 &s, colour3 &colour, bool pick) {

	float min_t = 1e6;
	json closest_object;
	std::vector<std::vector<float>> closest_triangle; //if any
	colour3 closest_colour;
	bool found = false;

	point3 d = s - e;
	d = glm::normalize(d);
	// traverse the objects
	json &objects = scene["objects"];
	for (json::iterator it = objects.begin(); it != objects.end(); ++it) {
		json &object = *it;

		// Every object will have a material
		json &material = object["material"];
		colour3 ambient = material.find("ambient") != material.end() ? vector_to_vec3(material["ambient"]) : colour3(0.0, 0.0, 0.0);
		colour3 diffuse = material.find("diffuse") != material.end() ? vector_to_vec3(material["diffuse"]) : colour3(0.0, 0.0, 0.0);
		colour3 specular = material.find("specular") != material.end() ? vector_to_vec3(material["specular"]) : colour3(0.0, 0.0, 0.0);
		float shininess = material.find("shininess") != material.end() ? float(material["shininess"]) : 0.0;


		if (object["type"] == "mesh") {
			std::vector<std::vector<std::vector<float>>> triangles = object["triangles"];

			//std::vector<std::vector<float>> triangle = triangles[0];
			for (std::vector<std::vector<std::vector<float>>>::size_type i = 0; i != triangles.size(); i++) {
				std::vector<std::vector<float>> triangle = triangles[i];

				float t = triangle_intersetion(e, d, triangle);

				if (t < min_t) {
					min_t = t;
					closest_object = object;
					closest_triangle = triangle;
					closest_colour = colour3(1.0, 0.0, 0.0);
					//colour = colour3(1.0, 0.0, 0.0);
					//return true;
					found = true;
				}
			}

		}
		else if (object["type"] == "sphere") {

			point3 center = vector_to_vec3(object["position"]);
			float r = float(object["radius"]);

			float t = sphere_intersection(e, d, center, r);
			if (t < min_t) {
				min_t = t;
				closest_object = object;
				point3 x = e + t * d;
				colour = calculate_lighting(x - center, x, ambient, diffuse, specular, shininess);
				closest_colour = colour;
				//return true;
				found = true;
			}
		}
		else if (object["type"] == "plane") {
			point3 a = vector_to_vec3(object["position"]);
			point3 n = vector_to_vec3(object["normal"]);

			//float t = glm::dot(n, a - e) / glm::dot(n, d);
			float t = plane_intersection(e, d, n, a);

			if (t < min_t) {
				colour = calculate_lighting(n, e+t*d, ambient, diffuse, specular, shininess);
				min_t = t;
				closest_colour = colour;
				//return true;
				found = true;
			}
		}

	}

	colour = closest_colour;
	return found;
}

/*
bool trace(const point3 &e, const point3 &s, colour3 &colour, bool pick) {
	// NOTE: This is a demo, not ray tracing code! You will need to replace all of this with your own code...

	// traverse the objects
	json &objects = scene["objects"];
	for (json::iterator it = objects.begin(); it != objects.end(); ++it) {
		json &object = *it;
		
		// every object in the scene will have a "type"
		if (object["type"] == "sphere") {
			// This is NOT ray-sphere intersection
			// Every sphere will have a position and a radius
			std::vector<float> pos = object["position"];
			point3 p = -(s - e) * pos[2];
			if (glm::length(glm::vec3(p.x - pos[0],p.y - pos[1],0)) < float(object["radius"])) {
				// Every object will have a material
				json &material = object["material"];
				std::vector<float> diffuse = material["diffuse"];
				colour = vector_to_vec3(diffuse);
				colour = colour3(1.0, 0.0, 0.0);
				// This is NOT correct: it finds the first hit, not the closest
				return true;
			}
		}
	}

	return false;
}
*/