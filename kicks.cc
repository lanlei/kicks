#include "RenderSequences.hpp"
#include <iostream>

void PrintUsage() {
	std::cout << "Usage: kicks ASS_PATH OBJ_DIR OUTPUT_DIR DEMO_NAME BEGIN_ITR END_ITR" << std::endl
			  << "ASS_PATH: path to ass file of the scene" << std::endl
			  << "OBJ_DIR: path to obj sequences" << std::endl
			  << "OUTPUT_DIR: output images will be placed here" << std::endl
			  << "DEMO_NAME: name of the demo, the output image sequences will be named as DEMO_NAMEXXXX.obj where XXXX is a zero-padded 4-digits iteration number" << std::endl
			  << "BEGIN_ITR, END_ITR: the range of obj in the sequence to be rendered" << std::endl;
}

int main(int argc, char* argv[]) {
	if (argc != 6) {
		PrintUsage();
	}

	fs::path ass_path(argv[0]);
	fs::path obj_dir(argv[1]);
	fs::path output_dir(argv[2]);
	std::string demo_name(argv[3]);
	int begin_itr = std::atoi(argv[4]);
	int end_itr = std::atoi(argv[5]);
	RenderSequences(ass_path, obj_dir, output_dir, demo_name, begin_itr, end_itr);

	return 0;
} 