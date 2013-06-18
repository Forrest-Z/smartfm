/*! 
 * @file 
 * @brief Subsets of atomic propositions and single-state safety-property automata. 
 */

#ifndef DEFS_AUTOMATA_HPP
#define	DEFS_AUTOMATA_HPP

// Header files: Standard C++
#include <cstdlib>
#include <iostream>

// Header files: Custom [Luis]
#include "Defs_Numbers.hpp"

/** Primitive integer type used for representing subsets of atomic propositions. */

#define symbol_t        UCH

/** Number of rules. Cardinality of the set of rules with priorities. */

#define NUM_OF_RULES    1

/** Number of sub-rules. Maximum cardinality of each individual set of rules. */

#define NUM_OF_SUB_R    1

/** 
 * Set of atomic propositions (symbols). 
 * 
 * Subset of an alphabet composed of the symbols zero to ( 8 * sizeof(SYMBOL_t) ) - 1. 
 */

class Subset_of_Sigma {
    
    private:
        
        /** Element of the power set of the alphabet. */
        
        symbol_t symbol;
        
    public:
        
        /** Initializes subset to the empty set. */
        
        Subset_of_Sigma() : symbol(0) {}
        
        /** Initializes subset to a singleton. 
         * 
         * @param index_of_AP   Index of atomic proposition (i.e. its 
         * position in the subset). Note that the position of each 
         * atomic proposition in the subset is defined by the user. */
        
        Subset_of_Sigma( const UCH index_of_AP) : 
            
            // Insert atomic proposition
        
            symbol( 0 | (1 << index_of_AP) ) {}
        
        /** Copies subset of atomic propositions. 
         * 
         * @param original      Original subset. */
        
        Subset_of_Sigma( const Subset_of_Sigma& original) : 
        
            symbol( original.symbol ) {}
        
        /** Does nothing. */
        
        ~Subset_of_Sigma() {}
        
        /** Re-initializes subset to the empty set.  */
        
        void clear_symbols() { symbol = 0; }
        
        /** Inserts atomic proposition into subset. 
         * 
         * @param index_of_AP   Index of atomic proposition (i.e. its 
         * position in the subset). Note that the position of each 
         * atomic proposition in the subset is defined by the user. */
        
        void insert_AP( const UCH index_of_AP) {
            
            // Bitwise-OR subset with unsigned integer equal 
            // to two to the power of the index of the atomic prop
            
            symbol = ( symbol | (1 << index_of_AP) );
            
        }
        
        /** Computes the union of this subset with the given one. 
         * 
         * @param rhs           Right-hand side subset. */
        
        void insert_Subset( const Subset_of_Sigma& rhs) {
            
            // Bitwise-OR the two subsets
            
            symbol = ( symbol | rhs.symbol );
            
        }
        
        /** Prints atomic propositions. */
        
        void print () const {
            
            std::cout << "SUBSET OF SIGMA:" << std::endl;
            
            for ( UCH ap = 0; ap < ( 8 * sizeof(symbol_t) ); ap++) {
                
                std::cout << "\tSymbol " << (USI) ap << ":\t" << operator[](ap) << std::endl;
                
            }
            
        }
        
        /** Copies subset of atomic propositions. 
         * 
         * @param rhs           Original subset. 
         * 
         * @return              Reference to this object. */
        
        Subset_of_Sigma& operator=( const Subset_of_Sigma& rhs) {
            
            symbol = rhs.symbol; return *this;
            
        }
        
        /** Checks whether atomic proposition is in the subset. 
         * 
         * @param index_of_AP   Index of atomic proposition. 
         * For more information, see Subset_of_Sigma::insert. 
         * 
         * @return              True if atomic proposition is in the subset. */
        
        bool operator[]( const UCH index_of_AP) const {
            
            // Bitwise-AND subset with unsigned integer equal 
            // to two to the power of the index of the atomic prop
            
            return ( symbol & (1 << index_of_AP) );
            
        }
        
        /** Checks whether subsets of atomic propositions are equal. 
         * 
         * @param rhs           Right-hand side subset. 
         * 
         * @return              True if subsets contain exactly the same atomic propositions. */
        
        bool operator==( const Subset_of_Sigma& rhs) const {
            
            // Subsets are equal if and only if their 
            // unsigned integer representations are the same
            
            return ( symbol == rhs.symbol );
            
        }
        
        /** Checks whether subsets of atomic propositions are different. 
         * 
         * @param rhs           Right-hand side subset. 
         * 
         * @return              False if subsets contain exactly the same atomic propositions. */
        
        bool operator!=( const Subset_of_Sigma& rhs) const {
            
            return ( symbol != rhs.symbol );
            
        }
        
};

/** 
 * Single-state safety-property automaton. 
 * 
 * Automaton for the specification of safety properties such as 
 * "always not wrong lane" or "always not change lane". 
 */

class Auto_SS_SP {
    
private:

    /** See Auto_SS_SP::Auto_SS_SP. */

    const bool      positive;

    /** See Auto_SS_SP::Auto_SS_SP. */

    const UCH       atomic_prop;

    /** See Auto_SS_SP::Auto_SS_SP. */

    const format_t  UST_weight_const;

    /** See Auto_SS_SP::Auto_SS_SP. */

    const format_t  UST_weight_fac;

public:

    /** Initializes automaton. 
     * 
     * @param positive          False if atomic proposition is to be negated. 
     * This flag indicates that the atomic proposition corresponding 
     * to safe transition is not to be negated before evaluation. 
     * 
     * @param atomic_prop       Index of atomic proposition corresponding 
     * to safe transition. For more information, see Subset_of_Sigma::insert. 
     * 
     * @param UST_weight_const  Weight constant of unsafe transition. 
     * 
     * @param UST_weight_fac    Weight factor of unsafe transition. */

    Auto_SS_SP( const bool positive, 

                const UCH atomic_prop, 

                const format_t UST_weight_const, 

                const format_t UST_weight_fac) : 

        // Copies arguments

        positive(positive), 

        atomic_prop(atomic_prop), 

        UST_weight_const(UST_weight_const), 

        UST_weight_fac(UST_weight_fac) {}

    /** Does nothing. */

    ~Auto_SS_SP() {}

    /** Given a transition label (a subset of atomic propositions that hold 
     * during the transition) and a time duration, computes weight of transition. 
     * 
     * @param label             Label of transition. 
     * 
     * @param duration          Time duration of transition. 
     * 
     * @return Zero, if transition is safe; sum of the weight constant and the product 
     * of the weight factor and the duration of the transition, if the transition is unsafe. */

    format_t weight( const Subset_of_Sigma label, 

                     const format_t duration) const {

        // If atomic proposition corresponding to safe transition is in the label 
        // of the transition then transition is safe and weight of transition is zero

        if ( (  positive &&  label[atomic_prop] ) || 
             ( !positive && !label[atomic_prop] ) ) { return 0;}

        // Otherwise, transition is unsafe and weight of transition is sum of 
        // weight constant and product of weight factor and duration of transition

        else { return UST_weight_const + (UST_weight_fac * duration); }

    }
        
};

/** 
 * Level of unsafety tuple. 
 * 
 * Array containing the level of unsafety of each rule, in addition to the metric cost. 
 */

struct Level_of_US {
    
    /** Metric cost (e.g. travel time, distance, etc.). */
    
    format_t metric_cost;
    
    /** Array of levels of unsafety. */
    
    format_t level_of_US [NUM_OF_RULES];
    
    /** Initializes tuple to given value. 
     * 
     * @param init_value    Initial value for the level of unsafety of each rule and 
     * for the metric cost. Normally, we would like to set this value to zero or infinity. */
    
    Level_of_US ( format_t init_value) :
    
    metric_cost(init_value) {
        
        for ( UCH k = 0; k < NUM_OF_RULES; k++) { level_of_US[k] = init_value; }
        
    }
    
    /** Copies a level of unsafety tuple. 
     * 
     * @param original      Original level of unsafety. */
    
    Level_of_US( const Level_of_US& original) : 
    
    metric_cost(original.metric_cost) {
        
        for ( UCH k = 0; k < NUM_OF_RULES; k++) { level_of_US[k] = original.level_of_US[k]; }
        
    }
    
    /** Does nothing. */
    
    ~Level_of_US () {}
    
    /** Prints level of unsafety. */
    
    void print() const {
        
        std::cout << "LEVEL OF UNSAFETY" << std::endl;
        
        for ( UCH k = 0; k < NUM_OF_RULES; k++) {
            
            std::cout << "\tof Rule " << (USI) k << ":\t";
            std::cout << level_of_US[k] << std::endl;
            
        }
        
        std::cout << "\t[Metric Cost]:\t" << metric_cost << std::endl;
        
    }
    
    /** Compares two levels of unsafety. 
     * 
     * @param rhs           Right-hand side level of unsafety. 
     * 
     * @return              True if LHS level of unsafety is smaller that RHS level, 
     * in the standard lexicographical ordering. */
    
    bool operator< ( const Level_of_US& rhs) const {
        
        // Iterates through rules
        
        for ( UCH k = 0; k < NUM_OF_RULES; k++) {
            
            // If LHS level of unsafety of rule is smaller, return true
            
            if ( (this -> level_of_US[k]) < rhs.level_of_US[k] ) { return true; }
            
            // If RHS level of unsafety of rule is smaller, return false
            
            else if ( (this -> level_of_US[k]) > rhs.level_of_US[k] ) { return false; }
            
        }
        
        // If LHS metric cost is smaller, return true
        
        if ( (this -> metric_cost) < rhs.metric_cost ) { return true; }
        
        // If RHS metric cost is smaller, return false
        
        else if ( (this -> metric_cost) > rhs.metric_cost ) { return false; }
        
        // If this line is reached (extremely unlikely) then 
        // the two levels of unsafety are equal, so return false
        
        else { return false; }
        
    }
    
    /** Resets every level of unsafety and the metric cost to the given value. 
     * 
     * @param value         Value. 
     * 
     * @return Reference to this object. */
    
    Level_of_US& operator= ( const format_t value) {
        
        metric_cost = value;
        
        for ( UCH k = 0; k < NUM_OF_RULES; k++) { level_of_US[k] = value; }
        
        return *this;
        
    }
    
    /** Copies a level of unsafety tuple. 
     * 
     * @param original      Original level of unsafety. 
     * 
     * @return Reference to this object. */
    
    Level_of_US& operator= ( const Level_of_US& original) {
        
        metric_cost =  original.metric_cost;
        
        for ( UCH k = 0; k < NUM_OF_RULES; k++) { level_of_US[k] = original.level_of_US[k]; }
        
        return *this;
        
    }
    
    /** Increases the level of unsafety by an amount equal to the RHS level. 
     * 
     * @param rhs           Right-hand side level of unsafety. 
     * 
     * @return Reference to this object. */
    
    Level_of_US& operator+=( const Level_of_US& rhs) {
        
        metric_cost += rhs.metric_cost;
        
        for ( UCH k = 0; k < NUM_OF_RULES; k++) { level_of_US[k] += rhs.level_of_US[k]; }
        
        return *this;
        
    }
    
};

/** 
 * Automaton A-bar. 
 * 
 * See Reyes.Chaudhari.ea.CDC13 for a detailed description of this object. 
 */

class Auto_AB {
    
private:

    /** Array of pointers to pointers to automata. */
    
    Auto_SS_SP *** rule;

public:

    /** Initialize automaton A-bar. 
     * 
     * Allocates memory for the automata necessary to model check all 
     * the desired rules and sub-rules. */
    
    Auto_AB() {

        // Allocate array of pointers to pointers to automata
        
        rule = new Auto_SS_SP ** [NUM_OF_RULES];

        // Iterate through rules
        
        for ( UCH i = 0; i < NUM_OF_RULES; i++) {

            // Allocate array of pointers to automata
            
            rule[i] = new Auto_SS_SP * [NUM_OF_SUB_R];

            // Iterate through sub-rules
            
            for ( UCH j = 0; j < NUM_OF_SUB_R; j++) {

                // Initialize pointer to automaton corresponding to this rule and sub-rule to null
                
                rule[i][j] = NULL;

            }

        }

    }

    /** De-allocates all automata. */
    
    ~Auto_AB() {

        for ( UCH i = 0; i < NUM_OF_RULES; i++) {

            for ( UCH j = 0; j < NUM_OF_SUB_R; j++) {

                if ( rule[i][j] ) { delete ( rule[i][j] ); }
            }
            
            delete [] ( rule[i] );

        }
        
        delete [] rule;
        
    }
    
    /** Inserts single-state safety-property automaton. 
     * 
     * @param auto_in   Single-state safety-property automaton to insert. 
     * 
     * @param i         Index of corresponding rule. 
     * 
     * @param j         Index of corresponding sub-rule. */
    
    void insert_Auto( Auto_SS_SP * auto_in, const UCH i, const UCH j) {

        if ( !rule[i][j] ) { rule[i][j] = auto_in; }

        else {

            std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
            std::cerr << "INVALID insertion of automaton" << std::endl;
            exit(EXIT_FAILURE);

        }

    }

    /** Given a transition label (a subset of atomic propositions that hold 
     * during the transition) and a time duration, computes level of unsafety 
     * of transition, and increases the given level by that amount. 
     * 
     * @param label             Label of transition. 
     * 
     * @param duration          Time duration of transition. 
     * 
     * @param level_to_update   Reference to level of unsafety object to which 
     * the level of this transition will be added to. 
     * 
     * @return Level of unsafety of transition, where the level of rule psi_i 
     * is the sum of the weights of the transitions of all automata associated 
     * with rule psi_i. See Reyes.Chaudhari.ea.CDC13 for more information. */
    
    Level_of_US& add_LUS( const Subset_of_Sigma label, 
    
                          const format_t duration, 
                          
                          Level_of_US& level_to_update) const {

        for ( UCH i = 0; i < NUM_OF_RULES; i++) {

            for ( UCH j = 0; j < NUM_OF_SUB_R; j++) {

                if ( rule[i][j] ) {

                    level_to_update.level_of_US[i] += rule[i][j] -> weight( label, duration);

                }

            }

        }

        level_to_update.metric_cost += duration;

        return level_to_update;

    }

    /** Given a transition label (a subset of atomic propositions that hold 
     * during the transition) and a time duration, computes level of unsafety 
     * of transition. 
     * 
     * @param label             Label of transition. 
     * 
     * @param duration          Time duration of transition. 
     * 
     * @return Level of unsafety of transition, where the level of rule psi_i 
     * is the sum of the weights of the transitions of all automata associated 
     * with rule psi_i. See Reyes.Chaudhari.ea.CDC13 for more information. */
    
    Level_of_US weight( const Subset_of_Sigma label, 
    
                        const format_t duration) const {

        Level_of_US trans_level (0);

        add_LUS( label, duration, trans_level);

        return trans_level;

    }
    
};

#endif	/* DEFS_AUTOMATA_HPP */
