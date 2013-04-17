/** 
 * Macros involved with the urban driving example. 
 */
#ifndef DEFS_DRIVING_EXAMPLE_HPP
#define	DEFS_DRIVING_EXAMPLE_HPP

/** True if driving on a sidewalk. */
#define SIDE_WALK   0
/** True if driving on the lane's direction. */
#define GOOD_DRCT   1
/** True if changing lanes. */
#define CHNG_LANE   2
/** True if on the right side of the bold line. */
#define BOLD_RGHT   3
/** True if on the left side of the bold line. */
#define BOLD_LEFT   4
/** True if changing lanes separated by a bold line. */
#define CHNG_BOLD   5

/** Number of atomic propositions in the alphabet. */
#define NUM_OF_APS  6

/** Number of rules. */
#define NUM_OF_RULES 3
/** Number of sub-rules (note that each one will 
 * correspond to a clocked automaton). */
#define NUM_OF_SUB_R 2

#endif	/* DEFS_DRIVING_EXAMPLE_HPP */
