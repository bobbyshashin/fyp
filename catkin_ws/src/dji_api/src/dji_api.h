#include <iostream>
#include <fstream>
#include <sstream>

#define FMU_API_VERT_VEL 0
#define FMU_API_VERT_POS 1
#define FMU_API_VERT_THR 2

#define FMU_API_HORI_ATTI_TILT_ANG 0
#define FMU_API_HORI_VEL 	       1
#define FMU_API_HORI_POS 	       2

#define FMU_API_YAW_ANG  0
#define FMU_API_YAW_RATE 1

#define GROUND_LEVEL 0
#define BODY_LEVEL   1
#define REF_LEVEL    2

#define GROUND_TORSION 0
#define BODY_TORSION   1

using namespace std;

typedef struct {

    float                 roll_or_x = 0;
    float                 pitch_or_y = 0;
    float                 thr_z = 0; //Attention! when vert_ctrl flag is VERT_POS, set thr_z to zero  may let you confuse
    float                 yaw = 0;
    unsigned char         ctrl_flag = 0x53; //VERT_POS HORI_VEL YAW_ANGLE BODY_FRAME STABLE
    long                  seq = 0;
    double                stamp = 0;
    string                frame_id = "dji_api_ctrl_data";

} dji_drone_api_ctrl;


typedef struct {

    float         velocity_x = 0;
    float         velocity_y = 0;
    float         position_z = 0;
    float         yaw = 0;

    unsigned char ctrl_flag = 0x53;

} Ctrl_data;

