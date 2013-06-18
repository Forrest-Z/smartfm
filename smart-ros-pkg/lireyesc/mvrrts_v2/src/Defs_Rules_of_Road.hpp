/*! 
 * @file 
 * @brief Atomic propositions and transition labeling function for implementing the 
 * rules of the road of Singapore. 
 */

#ifndef DEFS_RULES_OF_ROAD_HPP
#define	DEFS_RULES_OF_ROAD_HPP

// Header files: Custom [Luis]
#include "Defs_Automata.hpp"

/** True if driving on the left lane (the correct lane in Singapore). */

#define LANE_LEFT       0

/** Transition labeling function for the rules of the road in the Urban Driving examples. 
 * 
 * Currently, we're implementing the rule ``always drive in the left lane''. 
 * 
 * @param lab_start     Subset of atomic propositions holding at the starting state. 
 * 
 * @param lab_endPt     Subset of atomic propositions holding at the ending state. 
 * 
 * @return Subset of atomic propositions holding during the transition. */

inline Subset_of_Sigma rules_of_road( const Subset_of_Sigma lab_start, 
        
                                      const Subset_of_Sigma lab_endSt) {
    
    Subset_of_Sigma transition_label;
    
    if ( lab_start[LANE_LEFT] && lab_endSt[LANE_LEFT] ) { transition_label.insert_AP(LANE_LEFT); }
    
    return transition_label;
    
}

#endif	/* DEFS_RULES_OF_ROAD_HPP */
