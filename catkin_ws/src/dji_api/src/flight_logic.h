typedef enum
{
    INIT               ,
    TAKEOFF            ,
    STAND_BY           ,
    LANDING            ,
    RELEASE_CONTROL    ,
    REACH_ALT          ,

    START_SEARCH       ,
    START_TRACK        ,
    VISION_START       ,
    LOCK_TARGET        ,
    TRACKING           ,
    LOCK_HOOK          ,
    RELEASE_HOOK       ,
    PLACE_BARREL       ,
    SEARCH_DESTINATION ,
    AIM_ARUCO          ,
    GO_BACK_TO_SOURCE  ,
    CLR                ,
    END_PROCESS        ,

    RETURN_HOME        ,
    MISSION_PROCESSING ,
    MISSION_ABORTED    ,
    MISSION_COMPLETED  ,
    EMERGENCY          ,

} MISSION_STATUS;
