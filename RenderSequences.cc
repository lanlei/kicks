#include "RenderSequences.hpp"
#include "ai.h"
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

void LoadObj(const std::string& filepath, Eigen::MatrixX3f& vertices, Eigen::MatrixX3i& face_topo) {
	tinyobj::ObjReader reader;
	tinyobj::ObjReaderConfig reader_config;
	if (!reader.ParseFromFile(filepath, reader_config)) {
		if (!reader.Error().empty()) {
			std::cerr << "TinyObjReader: " << reader.Error();
		}
		exit(1);
	}
	if (!reader.Warning().empty()) {
		std::cout << "TinyObjReader: " << reader.Warning();
	}

	auto& attrib = reader.GetAttrib();
	auto& shapes = reader.GetShapes();

	const size_t num_vertices = attrib.vertices.size() / 3;
	vertices.resize(num_vertices, 3);
	for (size_t vertex_index = 0, coord_index = 0; vertex_index < num_vertices; vertex_index++, coord_index += 3) {
		vertices.row(vertex_index) << attrib.vertices[coord_index],
										attrib.vertices[coord_index + 1],
										attrib.vertices[coord_index + 2];
	}

	// Loop over shapes
	size_t num_faces = 0;
	for (size_t group_index = 0; group_index < shapes.size(); group_index++) {
		// Loop over faces
		num_faces += shapes[group_index].mesh.num_face_vertices.size();
	}
	face_topo.resize(num_faces, 3);

	size_t face_index = 0;
	for (size_t group_index = 0; group_index < shapes.size(); group_index++) {
		const auto& mesh = shapes[group_index].mesh;
		const size_t cur_group_num_faces = mesh.num_face_vertices.size();

		for (size_t face_index_in_group = 0, index_offset = 0; face_index_in_group < cur_group_num_faces; face_index_in_group++, index_offset += 3) {
			// only triangle is currently supported
			assert(mesh.num_face_vertices[face_index_in_group] == 3);

			// Loop over vertices in the face.
			face_topo.row(face_index++) << mesh.indices[index_offset].vertex_index,
											  mesh.indices[index_offset + 1].vertex_index,
											  mesh.indices[index_offset + 2].vertex_index;
		}
	}
}

void CalculateSmoothedNormal(const Eigen::MatrixX3f& vertices, const Eigen::MatrixX3i& face_topo, Eigen::MatrixX3f& normals) {
	const int num_vertices = vertices.rows();
	normals = Eigen::MatrixX3f::Zero(num_vertices, 3);
	
	const int num_faces = face_topo.rows();
	for (int face_index = 0; face_index < num_faces; face_index++) {
		const Eigen::RowVector3i indices = face_topo.row(face_index);
		const Eigen::RowVector3f x0 = vertices.row(indices[0]);
		const Eigen::RowVector3f x1 = vertices.row(indices[1]);
		const Eigen::RowVector3f x2 = vertices.row(indices[2]);
		Eigen::RowVector3f normal = (x1 - x0).cross(x2 - x0).normalized();
		normals.row(indices[0]) += normal;
		normals.row(indices[1]) += normal;
		normals.row(indices[2]) += normal;
	}
	
	normals.rowwise().normalize();
}

std::string GetObjName(const std::string& shape_name) {
	auto start_pos = shape_name.find_first_of('/') + 1;
	auto end_pos = shape_name.find_first_of('/', start_pos);
	return shape_name.substr(start_pos, end_pos - start_pos);
}

std::string GetSuffix(const std::string& driver_name) {
	return driver_name.substr(driver_name.find_first_of('_') + 1);
}

std::string ToStringZeroPad(int num, int length) {
	std::string old_string = std::to_string(num);
	return std::string(std::max(0, length - (int)old_string.length()), '0') + old_string;
}

void RenderSequences(const fs::path& ass_path, const fs::path& obj_dir, const fs::path& output_dir, const std::string& demo_name, int begin_itr, int end_itr) {
	AiBegin();
	auto universe = AiUniverse();
	auto session = AiRenderSession(universe, AI_SESSION_BATCH);
	
	auto load_ass_param = AiParamValueMap();
	AiParamValueMapSetInt(load_ass_param, AtString("mask"), AI_NODE_ALL);

	auto load_change_param = AiParamValueMap();
	AiParamValueMapSetInt(load_change_param, AtString("mask"), AI_NODE_SHAPE);

	AiSceneLoad (universe, ass_path.c_str(), load_ass_param);

	std::vector<AtNode*> nodes_to_delete;

	/* preprocess */
	auto shape_itr = AiUniverseGetNodeIterator(universe, AI_NODE_SHAPE);
	while (!AiNodeIteratorFinished(shape_itr)) {
		auto node = AiNodeIteratorGetNext(shape_itr);
		std::string shape_name = AiNodeGetName(node);
		if (shape_name == "root") {
			// skipping arnold internal 'root' node
			continue;
		}
		nodes_to_delete.push_back(node);
	}
	AiNodeIteratorDestroy(shape_itr);

	for (int itr = begin_itr; itr <= end_itr; itr++) {
		for (auto node_to_delete : nodes_to_delete) {
			AiNodeDestroy(node_to_delete);
		}
		nodes_to_delete.clear();
		
		/* reload objects (assume only vertices change) */
		AiMsgInfo("Loading iteration %d", itr);
		AiSceneLoad (universe, ass_path.c_str(), load_change_param);
		auto shape_itr = AiUniverseGetNodeIterator(universe, AI_NODE_SHAPE);

		while (!AiNodeIteratorFinished(shape_itr)) {
			auto node = AiNodeIteratorGetNext(shape_itr);
			nodes_to_delete.push_back(node);
			std::string shape_name = AiNodeGetName(node);
			if (shape_name == "root") {
				// skipping arnold internal 'root' node
				continue;
			}

			std::string obj_name = GetObjName(shape_name);
			fs::path obj_path = obj_dir / (obj_name + "_itr" + ToStringZeroPad(itr, 4) + ".obj");
			if (!fs::is_regular_file(obj_path)) {
				AiMsgWarning("Iteration file for %s not found, use default instead", obj_name.c_str());
				continue;
			}

			Eigen::MatrixX3f vertices;
			Eigen::MatrixX3i face_topo;
			LoadObj(obj_path, vertices, face_topo);

			const int num_vertices = vertices.rows();
			const int num_faces = face_topo.rows();

			auto nidxs = AiArrayAllocate(num_faces * 3, 1, AI_TYPE_INT);
			for (int face_index = 0, point_index = 0; face_index < num_faces; face_index++, point_index += 3) {
				AiArraySetInt(nidxs, point_index, face_topo(face_index, 0));
				AiArraySetInt(nidxs, point_index + 1, face_topo(face_index, 1));
				AiArraySetInt(nidxs, point_index + 2, face_topo(face_index, 2));
			}
			AiNodeSetArray(node, AtString("nidxs"), nidxs);

			auto nlist = AiArrayAllocate(num_vertices * 3, 1, AI_TYPE_FLOAT);
			Eigen::MatrixX3f normals;
			CalculateSmoothedNormal(vertices, face_topo, normals);
			for (int vertex_index = 0; vertex_index < num_vertices; vertex_index++) {
				AiArraySetFlt(nlist, 3 * vertex_index, normals(vertex_index, 0));
				AiArraySetFlt(nlist, 3 * vertex_index + 1, normals(vertex_index, 1));
				AiArraySetFlt(nlist, 3 * vertex_index + 2, normals(vertex_index, 2));
			}
			AiNodeSetArray(node, AtString("nlist"), nlist);

			auto vlist = AiArrayAllocate(num_vertices * 3, 1, AI_TYPE_FLOAT);
			for (int vertex_index = 0; vertex_index < num_vertices; vertex_index++) {
				AiArraySetFlt(vlist, 3 * vertex_index, vertices(vertex_index, 0));
				AiArraySetFlt(vlist, 3 * vertex_index + 1, vertices(vertex_index, 1));
				AiArraySetFlt(vlist, 3 * vertex_index + 2, vertices(vertex_index, 2));
			}
			AiNodeSetArray(node, AtString("vlist"), vlist);
		}
		AiNodeIteratorDestroy(shape_itr);

		/* change output path */
		auto driver_itr = AiUniverseGetNodeIterator(universe, AI_NODE_DRIVER);
		while (!AiNodeIteratorFinished(driver_itr)) {
			auto node = AiNodeIteratorGetNext(driver_itr);
			auto node_entry = AiNodeGetNodeEntry(node);
			std::string node_name = AiNodeGetName(node);
			std::string entry_name = AiNodeEntryGetName(node_entry);
			if (node_name == "root") {
				// skipping arnold internal 'root' node
				continue;
			}

			if (entry_name.rfind("driver", 0) != 0) {
				continue;
			}
			std::string suffix = GetSuffix(AiNodeEntryGetName(node_entry));
			AiNodeSetStr(
				node, AtString("filename"),
				AtString((output_dir / (demo_name + ToStringZeroPad(itr, 4) + "." + suffix)).c_str())
			);
		}
		AiNodeIteratorDestroy(driver_itr);

		/* reset frame */
		auto options = AiUniverseGetOptions(universe);
		AiNodeSetFlt(options, AtString("frame"), itr);

		AiMsgInfo("Start rendering iteration %d", itr);
		AiRender(session, AI_RENDER_MODE_CAMERA);
	}

	AiRenderSessionDestroy(session);
	AiUniverseDestroy(universe);
	AiEnd();
}