#include "exec_tracker.h"

ExecTracker::ExecTracker() {
}

void ExecTracker::Track(string addr, string position) {
	creation_loc_[addr] = position;
}

void ExecTracker::Untrack(string addr) {
	creation_loc_.erase(addr);
}

void ExecTracker::Print(ostream& out) const {
	for (auto& it : creation_loc_) {
		out << "(" << it.first << ", " << it.second << endl;
	}
}

void ExecTracker::PrintLocs(ostream& out) const {
	map<string, int> locs;
	for (auto& it : creation_loc_) {
		locs[it.second] ++;
	}
	out << "Locs:";
	for (auto& loc : locs)
		out << " (" << loc.first << ", " << loc.second << ")";
	out << endl;
}
