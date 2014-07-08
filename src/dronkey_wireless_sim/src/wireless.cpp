#include <dronkey_wireless/wireless.h>

using namespace dronkey;

wireless::wireless(std::string world_name)
{
	gzbNode = transport::NodePtr(new transport::NodePtr());
	gzbNode->Init(world_name);


}

