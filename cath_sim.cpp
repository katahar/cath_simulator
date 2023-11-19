#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <iostream>
#include "fssimplewindow.h"



int main()
{
	std::cout <<" hello" << std::endl;
	FsOpenWindow(16,16,800,600,1);


	int key;

    while(FSKEY_ESC!=(key=FsInkey()))
    {
		FsPollDevice();
		// FsSwapBuffers();

        FsSleep(25);
	}

	return 0;
}