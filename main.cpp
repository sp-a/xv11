#include <Slam.h>
#include <getopt.h>
#include <iostream>
#include <string.h>

using namespace std;

struct option long_options[] =
    {
        {"validate",  no_argument, 0, 'p'},
        {"capture",  no_argument, 0, 'c'},
        {"saved",  no_argument, 0, 's'},
        {"file",  required_argument, 0, 'f'},
        {"vellocity",  required_argument, 0, 'v'},
        {0, 0, 0, 0}
    };

int main(int argc, char ** argv)
{
	int demo = 0;
	int vel_model = 0;
	int c;
	int option_index = 0;
	char *filename;

	while ((c = getopt_long (argc, argv, "f:pcsv",
                long_options, &option_index)  ) !=-1)
    {
    	switch(c)
    	{
    		case 'v':
    			vel_model = 0;
    			break;
    		case 'p':
    			demo = 0;
    			break;
    		case 'c':
    			demo = 1;
    			break;
    		case 's':
    			demo = 2;
    			break;
			case 'f':
				filename = (char*)malloc(strlen(optarg));
				strcpy(filename, optarg);
				break;
    	}
    }
    
    switch(demo)
    {
    	case 0:
			runOnDataset(filename,  vel_model);
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