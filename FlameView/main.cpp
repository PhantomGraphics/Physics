#include "FlameApp.h"

int main(int argc, char* argv[])
{
	FlameView::FlameApp app(1280, 720, "FlameView");
	app.run(argc, argv);
	return 0;
}
