typedef enum {

    INIT               ,
    TAKEOFF            ,
    STAND_BY           ,
    LANDING            ,
    RELEASE_CONTROL    ,
    SEARCH_FOR_TAGS    ,
    UGV_TRACKING       ,


} MISSION_STATUS;

struct Triple {

    float vec[3];

};

struct Tuple {

    float vec[2];

};

