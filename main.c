#include <stdio.h>
#include "pid.h"

int main()
{
    printf("System begin \n");

    PID_position(100.0);
    // PID_incremental(100.0);
    // PID_separation(100.0);
    // PID_antiSaturation(100.0);
    // PID_antiAllergy(100.0);

    printf("System end \n");

    return 0;
}
