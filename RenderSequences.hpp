#include <filesystem>

namespace fs = std::filesystem;

void RenderSequences(
	const fs::path& ass_path,
	const fs::path& obj_dir,
	const fs::path& output_dir,
	const std::string& demo_name,
	int begin_itr,
	int end_itr
);