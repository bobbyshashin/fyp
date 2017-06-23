#include "flight_logic.h"

void printMap(int waypointIndex) {

    switch(waypointIndex){
        case 0:
            cout << " --------------"  << endl;
            cout << "|"                << endl;
            cout << " --------------"  << endl;
            cout << "               |" << endl;
            cout << " --------------"  << endl;
            cout << "|"                << endl;
            cout << "X"                << endl;
            break;
        case 1:
            cout << " --------------"  << endl;
            cout << "|"                << endl;
            cout << " --------------"  << endl;
            cout << "               |" << endl;
            cout << "X--------------"  << endl;
            cout << "|"                << endl;
            break;        
        case 2:
            cout << " --------------"  << endl;
            cout << "|"                << endl;
            cout << " --------------"  << endl;
            cout << "               |" << endl;
            cout << " --------------X" << endl;
            cout << "|"                << endl;
            break;   
        case 3:
            cout << " --------------"  << endl;
            cout << "|"                << endl;
            cout << " --------------X"  << endl;
            cout << "               |" << endl;
            cout << " --------------" << endl;
            cout << "|"                << endl;
            break;   
        case 4:
            cout << " --------------"  << endl;
            cout << "|"                << endl;
            cout << "X--------------"  << endl;
            cout << "               |" << endl;
            cout << " --------------" << endl;
            cout << "|"                << endl;
            break;   
        case 5:
            cout << "X--------------"  << endl;
            cout << "|"                << endl;
            cout << " --------------"  << endl;
            cout << "               |" << endl;
            cout << " --------------" << endl;
            cout << "|"                << endl;
            break;   
        case 6:
            cout << " --------------X"  << endl;
            cout << "|"                << endl;
            cout << " --------------"  << endl;
            cout << "               |" << endl;
            cout << " --------------" << endl;
            cout << "|"                << endl;
            break;   
    }

}