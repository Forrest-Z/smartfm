#ifndef HISTORY_H
#define HISTORY_H

#include <vector>
#include <ostream>
#include <assert.h>

class ENTRY
{
public:
    ENTRY() { }

    ENTRY(int _Action, int obs)
    :   _Action(_Action), _Observation(obs)
    { }      

    
    int _Action;
    int _Observation;
};

class HISTORY
{
public:


    
    bool operator==(const HISTORY& history) const
    {
        if (history.History.size() != History.size())
            return false;
        for (int i = 0; i < History.size(); ++i)
            if (history.History[i]._Action != History[i]._Action
             || history.History[i]._Observation != History[i]._Observation)
                return false;
        return true;
    }
    
    void Add(int _Action, int obs = -1) 
    { 
        History.push_back(ENTRY(_Action, obs));
    }
    
    void Pop()
    {
        History.pop_back();
    }
    
    void Truncate(int t)
    {
        History.resize(t);
    }
    
    void Clear() 
    { 
        History.clear(); 
    }
    
    int Size() const
    {
        return History.size();
    }
    
    ENTRY& operator[](int t)
    {
        assert(t >= 0 && t < History.size());
        return History[t];
    }

    const ENTRY& operator[](int t) const
    {
        assert(t >= 0 && t < History.size());
        return History[t];
    }

    ENTRY& Back()
    {
        assert(History.size() > 0);
        return History.back();
    }

    const ENTRY& Back() const
    {
        assert(History.size() > 0);
        return History.back();
    }

    void Display(std::ostream& ostr) const
    {
        for (int t = 0; t < History.size(); ++t)
        {
            ostr << "a=" << History[t]._Action <<  " ";
            if (History[t]._Observation >= 0)
                ostr << "o=" << History[t]._Observation << " ";
        }
    }


private:

    std::vector<ENTRY> History;
};

#endif // HISTORY
