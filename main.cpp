#include <Slam.h>
#include <getopt.h>
#include <iostream>

using namespace std;

struct option long_options[] =
    {
        {"validate",  no_argument, 0, 'v'},
        {"capture",  no_argument, 0, 'c'},
        {"saved",  no_argument, 0, 's'},
        {0, 0, 0, 0}
    };

int main(int argc, char ** argv)
{
	int demo = 0;
	int c;
	int option_index = 0;

	while ((c = getopt_long (argc, argv, "v:c:s",
                long_options, &option_index)  ) !=-1)
    {
    	switch(c)
    	{
    		case 'v':
    			demo = 0;
    			break;
    		case 'c':
    			demo = 1;
    			break;
    		case 's':
    			demo = 2;
    			break;
    	}
    }
    
    switch(demo)
    {
    	case 0:
    		runOnDataset();
    		break;
    	case 1:
    		runWithSensors();
    		break;
    	case 2:
    		runwithSavedData();
    		break;
    }

	return 0;
}