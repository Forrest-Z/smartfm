#ifndef INTERSECTIONPOLICY_H_
#define INTERSECTIONPOLICY_H_

/** Defines a behaviour at an intersection.
 *
 * It is instantiated when approaching the intersection by the
 * IntersectionHandler object. The is_clear_to_go method is called in a loop
 * until it returns true, upon which the object is deleted.
 */
class IntersectionPolicy
{
public:
    virtual ~IntersectionPolicy() { }

    /// Returns whether it is safe to go through the intersection
    virtual bool is_clear_to_go() = 0;
};


#endif /* INTERSECTIONPOLICY_H_ */
