#define EV3_PORT	50004
#define MAX_BUF 	1024

#include "ros/ros.h"
#include <sys/types.h>

struct param {
	
	char ip[16];
		
	int	outA;
	int outB;
	int outC;
	int	outD;
		
	bool isset_outA;
	bool isset_outB;
	bool isset_outC;
	bool isset_outD;
	
};

