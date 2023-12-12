#include <ai.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
   const std::string assdir = "/home/hansljy/maya/projects/textures/scenes/ass";

	AiBegin();
	auto universe = AiUniverse();
	auto session = AiRenderSession(universe, AI_SESSION_BATCH);
	
	std::vector<std::string> assfiles;

	for (const auto & entry : fs::directory_iterator(assdir)) {
		assfiles.push_back(entry.path());
	}

	std::sort(assfiles.begin(), assfiles.end());

	auto first_file = assfiles[0];

	auto loadFirstSceneParam = AiParamValueMap();
	AiParamValueMapSetInt(loadFirstSceneParam, AtString("mask"), AI_NODE_ALL);

	int change_mask = AI_NODE_ALL - AI_NODE_CAMERA - AI_NODE_LIGHT - AI_NODE_SHADER - AI_NODE_FILTER;
	auto loadSuccessiveSceneParam = AiParamValueMap();
	AiParamValueMapSetInt(loadSuccessiveSceneParam, AtString("mask"), change_mask);

	AiSceneLoad (universe, first_file.c_str(), loadFirstSceneParam);
	AiRender(session, AI_RENDER_MODE_CAMERA);

	std::vector<AtNode*> deletion_list;
	auto shape_itr = AiUniverseGetNodeIterator(universe, change_mask);
	while (!AiNodeIteratorFinished(shape_itr)) {
		auto node = AiNodeIteratorGetNext(shape_itr);
		deletion_list.push_back(node);
	}
	AiNodeIteratorDestroy(shape_itr);
	for (auto deletion_item : deletion_list) {
		AiNodeDestroy(deletion_item);
	}

	for (int i = 1; i < assfiles.size(); i++) {
		auto assfile = assfiles[i];
		std::cerr << "Loading " << assfile << std::endl;
		AiSceneLoad(universe, assfile.c_str(), loadSuccessiveSceneParam);
		AiRender(session, AI_RENDER_MODE_CAMERA);
		
		std::vector<AtNode* > deletion_list;
		auto shape_itr = AiUniverseGetNodeIterator(universe, change_mask);
		while (!AiNodeIteratorFinished(shape_itr)) {
			auto node = AiNodeIteratorGetNext(shape_itr);
			deletion_list.push_back(node);
		}
		AiNodeIteratorDestroy(shape_itr);
		for (auto deletion_item : deletion_list) {
			AiNodeDestroy(deletion_item);
		}
	}

	AiRenderSessionDestroy(session);
	AiUniverseDestroy(universe);
	AiEnd();

	return 0;
} 