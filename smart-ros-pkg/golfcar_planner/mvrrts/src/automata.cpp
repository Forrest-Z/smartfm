#include "automata.h"

Timed_Letter transition_label( const Subset_of_Sigma s_1, 
                                const Subset_of_Sigma s_2, 
                                const double duration) 
{
    Subset_of_Sigma label;
    
    // If any endpoint lies inside the sidewalk
    if ( s_1[SIDE_WALK] || s_2[SIDE_WALK] ) {
        label.insert_AP(SIDE_WALK);
    }
    // If both endpoints lie inside the correct lane
    if ( s_1[GOOD_DRCT] && s_2[GOOD_DRCT] ) {
        label.insert_AP(GOOD_DRCT);
    }
    // Otherwise, if any endpoint lies inside the correct lane
    else {
        if ( s_1[GOOD_DRCT] || s_2[GOOD_DRCT] ) {
            // If it is not the case that the transition is from the 
            // wrong direction towards the correct direction
            //if ( s_1[GOOD_DRCT] || !s_2[GOOD_DRCT] ) {
                label.insert_AP(CHNG_LANE);
            //}
        }
    }
    // If the endpoints lie on different sides of the bold line
    if ( (s_1[BOLD_RGHT] && s_2[BOLD_LEFT]) || 
         (s_1[BOLD_LEFT] && s_2[BOLD_RGHT]) ) {
        // If it is not the case that the transition is from the 
        // wrong direction towards the correct direction
        //if ( s_1[GOOD_DRCT] || !s_2[GOOD_DRCT] ) {
            label.insert_AP(CHNG_BOLD);
        //}
    }
    
    return Timed_Letter( label, (float) duration);
}
