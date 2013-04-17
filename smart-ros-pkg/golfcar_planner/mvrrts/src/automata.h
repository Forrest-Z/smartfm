/*! 
 * @file 
 * @brief Classes for handling to automata-related functions. 
 */
#ifndef _AUTOMATA_H_
#define	_AUTOMATA_H_

#include <cstdlib>
#include "typedefs_int.h"
#include "defs_driving.h"

/** 
 * Subset of atomic propositions. 
 * Subset of an alphabet of up to EIGHT atomic propositions. */
class Subset_of_Sigma {
  private:
    /** Element of the power set of the alphabet. */
    UCH symbol;
  public:
    Subset_of_Sigma() : symbol(0) {}

    /** Copies subset of atomic propositions. 
     * @param original      Original subset. */
    Subset_of_Sigma( const Subset_of_Sigma& original) : 
      // Copy subset
      symbol( original.symbol ) {}

    ~Subset_of_Sigma() {}

    /** Inserts atomic proposition into subset. 
     * @param index_of_AP   Index of atomic proposition (i.e. its 
     * position in the subset). Note that the position of each 
     * atomic proposition in the subset is defined by the user. */
    void insert_AP( const UCH index_of_AP) {
      // Bitwise-OR subset with unsigned integer equal 
      // to two to the power of the index of the atomic prop
      symbol = ( symbol | (1 << index_of_AP) );
    }

    /** Updates subset to the union of this subset and the given one. 
     * In other words, it inserts every atomic proposition present 
     * in the given subset but not in this one. 
     * @param rhs           Right-hand side subset. */
    void insert_Subset( const Subset_of_Sigma& rhs) {
      // Bitwise-OR the two subsets
      symbol = ( symbol | rhs.symbol );
    }

    /** Copies subset of atomic propositions. 
     * @param rhs           Original subset. 
     * @return              Reference to this object. */
    Subset_of_Sigma& operator=( const Subset_of_Sigma& rhs) {
      symbol = rhs.symbol;
      return *this;
    }

    /** Checks whether atomic proposition is in the subset. 
     * @param index_of_AP   Index of atomic proposition. 
     * For more information, see Subset_of_Sigma::insert. 
     * @return              True if atomic proposition is in the subset. */
    bool operator[]( const UCH index_of_AP) const {
      // Bitwise-AND subset with unsigned integer equal 
      // to two to the power of the index of the atomic prop
      return ( symbol & (1 << index_of_AP) );
    }

    /** Checks whether subsets of atomic propositions are equal. 
     * @param rhs           Right-hand side subset. 
     * @return              True if subsets contain 
     * exactly the same atomic propositions. */
    bool operator==( const Subset_of_Sigma& rhs) const {
      // Subsets are equal if and only if their 
      // unsigned integer representations are the same
      return ( symbol == rhs.symbol );
    }

    /** Checks whether subsets of atomic propositions are different. 
     * @param rhs           Right-hand side subset. 
     * @return              False if subsets contain 
     * exactly the same atomic propositions. */
    bool operator!=( const Subset_of_Sigma& rhs) const {
      // Subsets are equal if and only if their 
      // unsigned integer representations are the same
      return ( symbol != rhs.symbol );
    }
    void print()
    {
      std::cout<<(int)symbol<< std::endl;
    }
};

/** 
 * Timed Letter. 
 * Contains a subset of atomic propositions and a time duration. 
 */
struct Timed_Letter {

  /** Subset of atomic propositions. */
  const Subset_of_Sigma       subset;
  
  /** Duration of transition. */
  const float                 duration;
  
  /** Initializes timed word to a copy of the given one. 
   * @param original      Original timed word. */
  Timed_Letter( const Timed_Letter& original) : 
    subset(original.subset), duration(original.duration) {}
  
  /** Initializes timed word to the given subset of atomic 
   * propositions and duration of transition. 
   * @param subset        Subset of atomic propositions. 
   * @param duration      Duration of transition. */
  Timed_Letter( const Subset_of_Sigma subset, const float duration) : 
    subset(subset), duration(duration) {}
  
  ~Timed_Letter() {}
  
  /** Prints timed letter to standard console. */
  void print() const {
    std::cout << "TIMED LETTER" << std::endl;
    std::cout << "\tSubset of Sigma reads: " << std::endl;
    for ( UCH k = 0; k < NUM_OF_APS; k++) {
      std::cout << "\tSymbol " << (int) k << ":\t" 
        << subset[k] << std::endl;
    }
    std::cout << "\tDuration: " << duration << std::endl;
  }
};

/** 
 * Single-state safety-property automaton. 
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
    const float     UST_weight_const;
    /** See Auto_SS_SP::Auto_SS_SP. */
    const float     UST_weight_fac;
  public:
    /** Initializes automaton. 
     * @param positive          False if atomic proposition is to be negated. 
     * This flag indicates that the atomic proposition corresponding 
     * to safe transition is not to be negated before evaluation. 
     * @param atomic_prop       Index of atomic proposition corresponding 
     * to safe transition. For more information, see Subset_of_Sigma::insert. 
     * @param UST_weight_const  Weight constant of unsafe transition. 
     * @param UST_weight_fac    Weight factor of unsafe transition. */
    Auto_SS_SP( const bool positive, const UCH atomic_prop, 
        const float UST_weight_const, const float UST_weight_fac) : 
      // Copy values
      positive(positive), atomic_prop(atomic_prop), 
      UST_weight_const(UST_weight_const), UST_weight_fac(UST_weight_fac) {}
    
    ~Auto_SS_SP() {}
    
    /** Given a timed letter, computes weight of transition. 
     * @param timed_let         Timed letter of transition. 
     * @return                  Zero, if transition is safe; sum of the 
     * weight constant and the product of the weight factor and the 
     * duration of the transition, if the transition is unsafe. */
    float weight( const Timed_Letter timed_let) const {
      // If atomic proposition corresponding to safe transition 
      // is contained in the given timed word
      if ( (  positive &&  timed_let.subset[atomic_prop] ) || 
          ( !positive && !timed_let.subset[atomic_prop] ) ) {
        // Weight of transition is zero
        return 0.0;
      }
      else {
        // Weight of transition is sum of weight constant and 
        // product of weight factor and duration of transition

        return UST_weight_const + (UST_weight_fac * timed_let.duration);
      }
    }
};

/** 
 * Weight tuple. 
 * Array containing the level of unsafety 
 * of each rule, in addition to the metric cost. */
struct Level_of_US {
  /** Array of levels of unsafety. */
  float level_of_US [NUM_OF_RULES];
  /** Metric cost (e.g. travel time, distance, etc.). */
  float metric_cost;
  /** Initializes tuple to zero. */
  Level_of_US() : metric_cost(0.0) 
  {
    // Iterate through rules
    for ( UCH k = 0; k < NUM_OF_RULES; k++) {
      // Level of unsafety of rule is zero
      level_of_US[k] = 0.0;
    }
  }
  /** Copies a level of unsafety tuple. 
   * @param original      Original level of unsafety. */
  Level_of_US( const Level_of_US& original)
  {
    // Copy metric cost
    metric_cost =  original.metric_cost;
    // Iterate through rules
    for ( UCH k = 0; k < NUM_OF_RULES; k++) {
      // Copy level of unsafety of rule
      level_of_US[k] = original.level_of_US[k];
    }
  }
  /** Does nothing. */
  ~Level_of_US () {}
  /** Prints level of unsafety */
  void print() {
    // Print message to console
    std::cout << "LEVEL OF UNSAFETY" << std::endl;
    // Iterate through rules
    for ( UCH k = 0; k < NUM_OF_RULES; k++) {
      // Print level of unsafety of rule to console
      std::cout << "\tof Rule " << (int) k << ":\t";
      std::cout << level_of_US[k] << std::endl;
    }
    // Print metric cost to console
    std::cout << "\t[Metric Cost]:\t" << metric_cost << std::endl;
  }
  /** Increases the level of unsafety by 
   * an amount equal to the RHS level. 
   * @param rhs           Right-hand side level of unsafety. 
   * @return              Reference to this object. */
  Level_of_US& operator+=( const Level_of_US& rhs) {
    // Iterate through rules
    for ( UCH k = 0; k < NUM_OF_RULES; k++) {
      // Increase level of unsafety by RHS level
      level_of_US[k] += rhs.level_of_US[k];
    }
    // Increase metric cost by RHS level
    metric_cost += rhs.metric_cost;
    // Return reference to this object;
    return *this;
  }
  /** Compares two levels of unsafety. 
   * @param rhs           Right-hand side level of unsafety. 
   * @return              True if LHS level of unsafety is smaller that 
   * RHS level, in the lexicographical ordering. */
  bool operator< ( const Level_of_US& rhs) const {
    // Iterate through rules
    for ( UCH k = 0; k < NUM_OF_RULES; k++) {
      // If LHS level of unsafety of rule is smaller, return true
      if ( (this -> level_of_US[k]) < rhs.level_of_US[k] )
        return true;
      // If RHS level of unsafety of rule is smaller, return false
      else if ( (this -> level_of_US[k]) > rhs.level_of_US[k] )
      {
        //std::cout<<__LINE__<< "false"<<std::endl;
        return false;
      }
    }
    // If LHS metric cost is smaller, return true
    if ( (this -> metric_cost) < rhs.metric_cost )
      return true;
    // If RHS metric cost is smaller, return false
    else if ( (this -> metric_cost) > rhs.metric_cost )
    {
      //std::cout<<__LINE__<< "false"<<std::endl;
      return false;
    }
    // If this line is reached (extremely unlikely) then 
    // the two levels of unsafety are equal, so return false
    else
    {
      //std::cout<<__LINE__<< "false"<<std::endl;
      return false;
    }
  }
};

/** 
 * Automaton A-bar. 
 * See Reyes.Chaudhari.ea.CDC13 for a detailed description. */
class Auto_AB {
  private:
    /** Array of pointers to pointers to automata */
    Auto_SS_SP *** rule;
  public:
    /** Initialize automaton A-bar. 
     * Allocates memory for the automata necessary to 
     * model check all the desired rules and sub-rules. */
    Auto_AB() {
      // Allocate array of pointers to pointers to automata
      rule = new Auto_SS_SP ** [NUM_OF_RULES];
      // Iterate through rules
      for ( UCH i = 0; i < NUM_OF_RULES; i++) {
        // Allocate array of pointers to automata
        rule[i] = new Auto_SS_SP * [NUM_OF_SUB_R];
        // Iterate through sub-rules
        for ( UCH j = 0; j < NUM_OF_SUB_R; j++) {
          // Initialize pointer to automaton corresponding 
          // to this rule and sub-rule to null
          rule[i][j] = NULL;
        }
      }
    }
    /** De-allocates all automata. */
    ~Auto_AB() {
      // Iterate through rules
      for ( UCH i = 0; i < NUM_OF_RULES; i++) {
        // Iterate through sub-rules
        for ( UCH j = 0; j < NUM_OF_SUB_R; j++) {
          // If automaton was ever initialized
          if ( rule[i][j] ) {
            // Deallocate automaton corresponding 
            // to this rule and sub-rule
            delete ( rule[i][j] );
          }
        }
        // Deallocate array of pointers to 
        // automata corresponding to rule
        delete [] ( rule[i] );
      }
      // Deallocate array of pointers to pointers to automata
      delete [] rule;
    }
    /** Inserts single-state safety-property automaton. 
     * @param auto_in   Single-state safety-property automaton to insert. 
     * @param i         Index of corresponding rule. 
     * @param j         Index of corresponding sub-rule. */
    void insert_Auto( Auto_SS_SP * const auto_in, 
        const UCH i, const UCH j) {
      // If the automaton corresponding to this 
      // rule and sub-rule has not yet been inserted
      if ( !rule[i][j] ) {
        // Copy pointer to clocked automaton
        rule[i][j] = auto_in;
      }
      // Otherwise, throw error
      else {
        std::cerr << "INVALID INSERTION of automaton " << \
          "in object of class Auto_AB!" << std::endl;
        exit(EXIT_FAILURE);
      }
    }
    /** Given a timed letter, computes level of unsafety of transition. 
     * @param timed_let         Timed letter of transition. 
     * @return                  Level of unsafety of transition, where 
     * the level of rule psy_i is the sum of the weights of the transitions 
     * of all automata associated with this rule. */
    Level_of_US weight( const Timed_Letter timed_let) const {
      // Declare level of unsafety of transition
      Level_of_US trans_level;
      // Iterate through rules
      for ( UCH i = 0; i < NUM_OF_RULES; i++) {
        // Iterate through sub-rules
        for ( UCH j = 0; j < NUM_OF_SUB_R; j++) {
          // If the automaton corresponding 
          // to this rule and sub-rule was ever initialized
          if ( rule[i][j] ) 
          {
            // Add weight of transition of the automaton 
            // associated with this rule and sub-rule
            trans_level.level_of_US[i] += 
              rule[i][j] -> weight(timed_let);
            //std::cout<<"added weight: "<< (int)i<<" "<< (int)j<<" "<< (rule[i][j]->weight(timed_let))<< std::endl;
          }
        }
      }
      // Add metric cost
      trans_level.metric_cost += timed_let.duration;
      // Return level of unsafety of transition
      return trans_level;
    }
};

#endif	/* AUTOMATA_SAFETY_HPP */
