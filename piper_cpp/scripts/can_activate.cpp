#include "piper_cpp/can_utils.h"
#include <iostream>
int main(int argc, char *argv[])
{

    piper_cpp::activate({}, "can", 1000000);

    return 0;
}
