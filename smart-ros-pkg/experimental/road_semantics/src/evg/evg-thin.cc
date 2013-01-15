/*********************************************************************

  EVG-THIN v1.1: A Thinning Approximation to the Extended Voronoi Graph
  
  Copyright (C) 2006 - Patrick Beeson  (pbeeson@cs.utexas.edu)


  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301
  USA

*********************************************************************/

#include "evg-thin.hh"


evg_thin::evg_thin(const grid_type& curr_grid,
		   float distance_min, float distance_max, 
		   bool pruning, bool robot_dependent,
		   int loc_x, int loc_y) {

  original_grid=curr_grid;
  coastal_dist=distance_max;
  prune_dist=distance_min;
  prune=pruning;
  location=robot_dependent;


  grid_size_x=original_grid.size();
  grid_size_y=original_grid[0].size();

  robot_loc_x=loc_x;
  robot_loc_y=loc_y;


  vector <State> tmp(grid_size_y);
  _step1_grid.resize(grid_size_x,tmp);

  dist_col tmpcol(grid_size_y);
  distance_grid.resize(grid_size_x,tmpcol);
}


/**
   Resets data structures after a skeleton is found.  
**/
void evg_thin::reset() {
  // Use this before calling generate_skeleton() if you are looping
  // over a changing grid data structure.

  // Assumes the grid size is constant, if not, add code to resize
  // (and clear) _step1_grid and distance_grid based on new grid size.

  curr_skel.clear();
  _step1_queue.clear();
  _step2_queue.clear();
}


/**
   Public method that returns the skeleton of a grid
**/
skeleton_type evg_thin::generate_skeleton() {

  calculate_distances();
  find_skel();
  
  return curr_skel;
}


bool evg_thin::on_grid(int x, int y) {
  return ((x >=0 && x < grid_size_x) &&
	  (y >=0 && y < grid_size_y));
}



/**
   Calculate the distance of each free cell from the closest occupied
   cell.  Stores these (along with the location of the closest
   obstacle) in distance_grid.
**/
void evg_thin::calculate_distances() {

  heap<dist_cell> hdist;
  dist_cell current_cell;  

  for (int x=0;x<grid_size_x;x++)
    for (int y=0;y<grid_size_y;y++)
      if (original_grid[x][y]==Occupied) {
		distance_grid[x][y].distance=0;
		distance_grid[x][y].x=x;
		distance_grid[x][y].y=y;

		current_cell.x=x;
		current_cell.y=y;
		current_cell.distance=0;
		hdist.push(current_cell);
      }
      else {
		distance_grid[x][y].distance=FLT_MAX;
		distance_grid[x][y].x=-1;
		distance_grid[x][y].y=-1;
      }


  heap<dist_cell> hdist2;
  
	while (!hdist.empty()) {

	  while (!hdist.empty()) {

		current_cell=hdist.first();
		hdist.pop();
      
		// Look at neighbors to find a new free cell that needs its
		// distance updated
		for (int i=-1; i<=1; i++) {
			int nx=current_cell.x+i;
			for (int j=-1; j<=1; j++) {
				int ny=current_cell.y+j;
				if ((i!=0 || j!=0) && on_grid(nx,ny))
				{
					if (original_grid[nx][ny] != Occupied && distance_grid[nx][ny].distance==FLT_MAX)
					{
						float min_distance=FLT_MAX;

						//look at neighbors of freecell to find cells whose distance has already been found
						for (int k=-1;k<=1;k++) {
							int nxk=nx+k;
							for (int l=-1;l<=1;l++) {
								int nyl=ny+l;
								if ((k!=0 || l!=0) && on_grid(nxk,nyl))
								{
									if (distance_grid[nxk][nyl].x >=0)
									{
										//find distance to neighbor's closest cell
										float d=dist(nx,ny,
										distance_grid[nxk][nyl].x,
										distance_grid[nxk][nyl].y);
										if (d < min_distance) {
										min_distance=d;
										distance_grid[nx][ny].distance=min_distance;
										distance_grid[nx][ny].x=distance_grid[nxk][nyl].x;
										distance_grid[nx][ny].y=distance_grid[nxk][nyl].y;
										}
									}
								}
							}
						}
						if (min_distance<coastal_dist) {
						dist_cell me_cell;
						me_cell.x=nx;
						me_cell.y=ny;
						me_cell.distance=min_distance;
						hdist2.push(me_cell);
						}
					}
				}
			}
		}
    }
    hdist=hdist2;
    hdist2.clear();
  }
}


/**
   Basically, calls the procedures that build the skeleton once the
   distances to obstacles have been calcuated in calculate_distances()
**/
void evg_thin::find_skel() {
  
  //First initialize the grid by labeling cells
  initialize();
  
  //Then "thin" the grid by flipping free cells that border occupied
  //cells to occupied (when applicaple).
  thin();

  //Search for the actual skeleton cells after then thinning is done.
  find_skel_edge();

  // Convert from a grid to a skeleton_type data structure.
  build_skel();
}

/**
   Initialize _step1_grid to be "occupied" for all occupied grid
   cells.  Any free grid cells become either "free" (most free cells)
   or "processing" if they are next to obstacles.  Also, put any
   "processing" cells in the _step1_queue.
**/
void evg_thin::initialize() {  
  
  // All cells in _step1_grid need to be labeled based on occ grid.
	for (int i=0;i<grid_size_x;i++)
		for (int j=0;j<grid_size_y;j++)
		{
			if (original_grid[i][j]==Occupied) {
				_step1_grid[i][j]=occupied;
			}
			// "Bleed" obstacles out by the safety_radius amount
			else if (distance_grid[i][j].distance <= prune_dist)
				_step1_grid[i][j]=occupied;
			// If free
			else if (original_grid[i][j]!=Occupied)
			{
				bool border=false;
				for (int x=-1;x<=1;x++)
					for (int y=-1;y<=1;y++)
						if (x!=0 || y!=0)
							//if it's next to a "bleeded" obstacle
							if (on_grid(x+i,y+j) && distance_grid[x+i][y+j].distance <=prune_dist) {
								border=true;
								break;
							}
				//if bordering occupied, put on queue to be examined.
				if (border)
				{
					_step1_grid[i][j]=processing;
					edge tmp(i,j);
					_step1_queue.push_back(tmp);
				}
				else
					_step1_grid[i][j]=free;
			}
			//unknown cells are not "fuel" like occupied cells and free
			//cells next to occupied cells.
			else _step1_grid[i][j]=unknown;
		}
}


/**
   Alternate between step1 and step2 until _step1_grid==_step2_grid
   (i.e. until _step1_queue and _step2_queue no longer have any more
   "fuel" for the brush fire algorithm).
**/
void evg_thin::thin() {
  
  bool changing=true;

  while (changing)
  {
	  changing=false;
    
	  _step2_grid=_step1_grid;
    
    
	  // Keep _step1_grid constant, burning cells (when applicable) in
	  // _step2_grid.  Add neighbors to burned cells to _step2_queue.
	  while (!_step1_queue.empty())
	  {
		  edge current=_step1_queue.front();
		  _step1_queue.pop_front();
      
		  if (_step1_grid[current.x][current.y]==processing ||
				  _step1_grid[current.x][current.y]==processed)
		  {
			  //Not occupied && not skel.
			  //Shouldn't be "free" or "unknown" and on the queue.
			  State status=step(_step1_grid,current,true);

			  //status should be processing, processed, skel, or occupied.
			  _step2_grid[current.x][current.y]=status;
	
			  if (status==processing)
			  {
				  //If already on the heap, switch to "processed" so that it
				  //will only be looked at once more (by the next step),
				  //assuming a neighbor cell doesn't switch to occupied.
				  _step2_grid[current.x][current.y]=processed;
				  _step2_queue.push_back(current);
			  }
			  else if (status==occupied)
			  {
				  changing=true;
				  // Find neighboring cells to examine on the next step.
				  for (int i=-1; i<=1; i++)
					  for (int j=-1; j<=1; j++)
						  if (i!=0 || j!=0)
						  {
							  edge tmp(current.x+i,current.y+j);
							  if (on_grid(tmp.x,tmp.y) &&
									  (_step2_grid[tmp.x][tmp.y]==free ||
											  _step2_grid[tmp.x][tmp.y]==processed))
							  {
								  //If it is free or already processed, look at it
								  //(again)
								  if (_step2_grid[tmp.x][tmp.y]!=processed ||
										  _step1_grid[tmp.x][tmp.y]!=processing)
									  //If it just turned to processed, it's already
									  //on the queue
									  _step2_queue.push_back(tmp);

								  _step2_grid[tmp.x][tmp.y]=processing;
							  }
						  }
			  }
		  }
	  }
    
    _step1_grid=_step2_grid;
    
    // Keep _step2_grid constant, burning cells (when applicable) in
    // _step1_grid.  Add neighbors to burned cells to _step1_queue.
    while (!_step2_queue.empty())
    {
		edge current=_step2_queue.front();
		_step2_queue.pop_front();
      
		if (_step2_grid[current.x][current.y]==processing ||
				_step2_grid[current.x][current.y]==processed)
		{
			//Not occupied && not skel.  Shouldn't be "free" or "unknown"
			//and on the queue.
	
			State status=step(_step2_grid,current,false);
			//status should be processing, processed, skel, or occupied.
	
			_step1_grid[current.x][current.y]=status;
	
			if (status==processing)
			{
				//If already on the heap, switch to "processed" so that it
				//will only be looked at once more (by the next step),
				//assuming a neighbor cell doesn't switch to occupied.
				_step1_grid[current.x][current.y]=processed;
				_step1_queue.push_back(current);
			}
			else if (status==occupied)
			{
				changing=true;
				// Find neighboring cells to examine on the next step.
				for (int i=-1; i<=1; i++)
					for (int j=-1; j<=1; j++)
						if (i!=0 || j!=0)
						{
							edge tmp(current.x+i,current.y+j);
							if (on_grid(tmp.x,tmp.y) &&
									(_step1_grid[tmp.x][tmp.y]==free ||
											_step1_grid[tmp.x][tmp.y]==processed))
							{
								//If it is free or already processed, look at it
								//(again)
								if (_step1_grid[tmp.x][tmp.y]!=processed ||
								_step2_grid[tmp.x][tmp.y]!=processing)
								  //If it just turned to processed, it's already
								  //on the queue
								_step1_queue.push_back(tmp);
								_step1_grid[tmp.x][tmp.y]=processing;
							}
						}
			}
		}
      
    }
  }
}

/**
   Given a free cell in the grid, deterimine whether it can be
   switched to an occupied cell by looking at its' neighbors.  If not,
   it may be part of the skeleton.  Check for that.
**/

// the classic thinning algorithm;

evg_thin::State evg_thin::step(gridtype& grid, 
			       edge& current, bool step1) {

  // If there is a bound on the maximum distance (for coastal
  // navigation) stop if we reached that distance.
  if (distance_grid[current.x][current.y].distance >= coastal_dist)
    return skel;

  bool freecell[8];

  int nx,ny,np;
  
  int i,j;

  np=0;

  //Marked state of all neighbors (occupied or !occupied).
  for (i=-1;i<=1;i++)
  {
	  nx=current.x+i;
	  for (j=-1;j<=1;j++)
		  if (i!=0 || j!=0) {
			  ny=current.y+j;
			  if (on_grid(nx,ny) && grid[nx][ny]==occupied)
				  freecell[np]=false;
			  else freecell[np]=true;
			  np++;
		  }
  }
  

  int N=0;
  for (i=0;i<8;i++)
    if (freecell[i])
      N++;

  //if more tha n6 neighbors are occupied, this cell is definately
  //part of the skeleton.
  if (N < 2)
    return skel;
  
  if (N <= 6)
  {
    int count=0;
    if (!freecell[0] && freecell[1])
          count++;
    if (!freecell[1] && freecell[2])
      count++;
    if (!freecell[2] && freecell[4])
      count++;
    if (!freecell[4] && freecell[7])
      count++;
    if (!freecell[7] && freecell[6])
      count++;
    if (!freecell[6] && freecell[5])
      count++;
    if (!freecell[5] && freecell[3])
      count++;
    if (!freecell[3] && freecell[0])
      count++;
    
    if (count == 1) 
      if ((step1  && (!freecell[1] || !freecell[4] || !freecell[6]) &&
	   (!freecell[4] || !freecell[6] || !freecell[3])) ||
	  (!step1 && (!freecell[1] || !freecell[4] || !freecell[3]) &&
	   (!freecell[1] || !freecell[6] || !freecell[3])))
	return occupied;
  }

	return grid[current.x][current.y];
}

/**
   After thin() is run, _step1_grid has a bunch of cells marked
   occupied, free, processed, and skel.  Walk through the grid finding
   skel or processed cells that border occupied cells.  Those are part
   of the skeleton, otherwise, mark them occupied in _step2_grid.

   Also, find the closest skeleton point to the robot's current
   location.
**/
void evg_thin::find_skel_edge() {
  // Don't worry about making _step1_grid and _step2 equal.  After
  // thin(), if there is any difference it would only be that
  // _step1_grid had some cells marked "skel" that are marked
  // "processed" in _step2_grid.  In this function, all cells that are
  // "skel" or "procssed" in _step1_grid become either skel or
  // occupied in _step2_grid.  Then _step2 grid can be used for
  // pruning and making the skeleton data structure.

  
  float rdist=FLT_MAX;
  _closestx=-1;
  _closesty=-1;
  _distance=-1;

  for (int i=0;i<grid_size_x;i++)
    for (int j=0;j<grid_size_y;j++)
      if (_step1_grid[i][j]==free)
	_step2_grid[i][j]=occupied;
      else
	//state should never be "processing" at this point
	if (_step1_grid[i][j]==processed || 
	    _step1_grid[i][j]==skel)
	{
	  //We only need cells that have an occupied cell above,
	  //below, left, or right (no diagonals).
	  bool edge=false;
	  for (int x=-1;x<=1;x++)
	    for (int y=-1;y<=1;y++)
	      if (x!=y && (x==0 || y==0))
	    	  if (on_grid(i+x,j+y) && _step1_grid[i+x][j+y]==occupied)
	    		  edge=true;

	  if (!edge) 
	    _step2_grid[i][j]=occupied;
	  else
	  {
	    _step2_grid[i][j]=skel;
	    // Now find closest point to robot.  Make sure the robot
	    // is within the radius of that point.
	    float d=dist(robot_loc_x,robot_loc_y,i,j);
	    float d2=distance_grid[i][j].distance;
	    if (d < rdist && (!location || d<=d2))
	    {
	      rdist=d;
	      _closestx=i;
	      _closesty=j;
	      _distance=d2;
	    }
	    else if (d==rdist && (!location || d<=d2) && (d2>_distance))
	    {
	      _closestx=i;
	      _closesty=j;
	      _distance=d2;
	    }
	    
	  }
	}
}

//to thin the skeleton from possible multiple-pixel thickness to single pixel;
//the basic idea here is to remove redundant pixels without change skeleton topology;

void evg_thin::skel_thining_test(){

	gridtype _step_tmp;
	vector <State> tmp(3);
	_step_tmp.resize(3,tmp);
	_step_tmp[0][0]= occupied;
	_step_tmp[0][1]= skel;
	_step_tmp[0][2]= occupied;
	_step_tmp[1][0]= occupied;
	_step_tmp[1][1]= occupied;
	_step_tmp[1][2]= occupied;
	_step_tmp[2][0]= occupied;
	_step_tmp[2][1]= skel;
	_step_tmp[2][2]= occupied;

	int x =1; int y=1;

	bool case1 = (_step_tmp[x-1][y-1]==skel||_step_tmp[x-1][y]==skel||_step_tmp[x-1][y+1]==skel)
		    				&& (_step_tmp[x+1][y-1]==skel||_step_tmp[x+1][y]==skel||_step_tmp[x+1][y+1]==skel);

	bool case2 = (_step_tmp[x-1][y-1]==skel||_step_tmp[x][y-1]==skel||_step_tmp[x+1][y-1]==skel)
			&& (_step_tmp[x-1][y+1]==skel||_step_tmp[x][y+1]==skel||_step_tmp[x+1][y+1]==skel);

	int count = 0;
	for (int i=-1; i<=1; i++)
		for (int j=-1; j<=1; j++)
			if((i!=0 || j!=0) && (_step_tmp[x+i][y+j]==skel))
			{
				count++;
			}

	bool case3 = (count >1);

	/*
	if(!case1 && !case2)
	{
		printf("not satisfactory\n");
		return;
	}
	*/

	if(!case3)
	{
		printf("not satisfactory\n");
		return;
	}



	//do connected-component analysis;
	int connectivity_serial = 0;
	std::vector < std::vector<int> > connected_pairs;

	bool skel_box[3][3];
	skel_box[1][1]=false;
	for (int i=-1; i<=1; i++)
		for (int j=-1; j<=1; j++)
			if (i!=0 || j!=0)
			{
				if(_step_tmp[1+i][1+j]==skel)
					skel_box[1+i][1+j]=true;
			}

	//construct "label_box" for labeling
	int label_box[3][3];
	for(int i=-1; i<=1;  i++)
		for(int j=-1; j<=1; j++)
			label_box[1+i][1+j]=-1;

	for (int i=-1; i<=1; i++)
		for (int j=-1; j<=1; j++)
			if ((i!=0 || j!=0) && skel_box[1+i][1+j])
			{
				printf("pixel (%d, %d)\n", 1+i, 1+j);

				//check 2 cells: left and up-left;
				for (int m=-1; m<=0; m++)
				{
					int n=-1;
					if(1+i+m>=0 && 1+i+m<=2 && 1+j+n>=0 && 1+j+n<=2)
					{
						if(skel_box[1+i+m][1+j+n]&&label_box[1+i+m][1+j+n]!=-1)
						{
							printf("neibouring pixel (%d, %d): label %d\t", 1+i+m, 1+j+n, label_box[1+i+m][1+j+n]);
							label_box[1+i][1+j]=label_box[1+i+m][1+j+n];
						}
					}
				}

				//check 2 cells: up and up-right;
				for (int n=0; n<=1; n++)
				{
					int m=-1;
					if(1+i+m>=0 && 1+i+m<=2 && 1+j+n>=0 && 1+j+n<=2)
						if(skel_box[1+i+m][1+j+n] && label_box[1+i+m][1+j+n]!=-1)
						{
							printf("(%d, %d): label %d", 1+i+m, 1+j+n, label_box[1+i+m][1+j+n]);

							if(label_box[1+i][1+j]==-1) label_box[1+i][1+j]=label_box[1+i+m][1+j+n];
							else if(label_box[1+i][1+j]!=label_box[1+i+m][1+j+n])
							{
								//already get some serial, but different from the current one;
								std::vector<int> pair_tmp;
								pair_tmp.push_back(label_box[1+i][1+j]);
								pair_tmp.push_back(label_box[1+i+m][1+j+n]);
								printf("pair_tmp: %d, %d\t", label_box[1+i][1+j], label_box[1+i+m][1+j+n]);
								connected_pairs.push_back(pair_tmp);
							}
						}
				}

				if(label_box[1+i][1+j]==-1)
				{
					connectivity_serial ++;
					label_box[1+i][1+j]= connectivity_serial;
				}

				printf("pixel (%d, %d), label %d\n", 1+i, 1+j, label_box[1+i][1+j]);
			}

	//re-sorted the connected_pieces, substitute the label, and find how many disconnected pieces exist;
	std::vector < std::vector<int> > connected_unions;
	for(size_t i=0; i<connected_pairs.size(); i++)
	{
		bool absorbed = false;
		bool break_mid_loop = false;
		for(size_t j=0; j<connected_unions.size(); j++)
		{
			for(size_t k=0; k<connected_unions[j].size(); k++)
			{
				if(connected_unions[j][k]==connected_pairs[i][0]
					   ||connected_unions[j][k]==connected_pairs[i][1])
				{
					absorbed = true;
					break_mid_loop=true;
					break;
				}
			}
			if(break_mid_loop) break;
		}

		if(!absorbed)
		{
			connected_unions.push_back(connected_pairs[i]);
		}
	}

	std::vector < std::vector<int> > sorted_unions;
	for(size_t i=0; i<connected_unions.size(); i++)
	{
		//bubble sorting for each union, from small to big;
		for(size_t a=0;a<connected_unions[i].size();a++)
		{
			for(size_t b=0;b<a;b++)
			{
				if(connected_unions[i][a]<connected_unions[i][b])
				{
					int temp=connected_unions[i][a];
					connected_unions[i][a]=connected_unions[i][b];
					connected_unions[i][b]=temp;
				}
			}

		}

		for(size_t k=0; k<connected_unions[i].size(); k++) printf("%d\t", connected_unions[i][k]);
		printf("----");

		std::vector<int> union_tmp;
		union_tmp.push_back(connected_unions[i][0]);
		for(size_t a=1;a<connected_unions[i].size();a++)
		{
			if(connected_unions[i][a]==union_tmp.back())continue;
			else union_tmp.push_back(connected_unions[i][a]);
		}
		sorted_unions.push_back(union_tmp);
	}

	bool component_connected = true;

	if(sorted_unions.size()==0 )
	{
		printf("aaa\n");
		for(int i=-1; i<=1; i++)
		{
			for(int j=-1;j<=1; j++)
			{
				printf("pixel (%d, %d), label %d\n", 1+i, 1+j, label_box[1+i][1+j]);

				if(label_box[1+i][1+j]>1)
				{
					component_connected=false;
				}
			}
		}
	}
	else if(sorted_unions.size()>1 )
	{
		component_connected=false;
		printf("bbb\n");
		for(size_t i=0; i<sorted_unions.size(); i++)
			for(size_t k=0; k<sorted_unions[i].size(); k++)
				printf("%d\t", sorted_unions[i][k]);
	}
	else if(sorted_unions.size()==1 && sorted_unions[0].front()!=1)
	{
		component_connected=false;
		printf("ccc\n");
		for(size_t k=0; k<sorted_unions[0].size(); k++) printf("%d\t", sorted_unions[0][k]);
	}
	else if(sorted_unions.size()==1 && sorted_unions[0].front()==1)
	{
		printf("ddd\n");
		for(size_t k=0; k<sorted_unions[0].size(); k++) printf("%d\t", sorted_unions[0][k]);

		for(int i=-1; i<=1;  i++)
			for(int j=-1;j<=1; j++)
			{
				if(label_box[1+i][1+j]>1)
				{
					for(size_t k=0; k<sorted_unions[0].size(); k++)
					{
						if(label_box[1+i][1+j]==sorted_unions[0][k])
						{
							label_box[1+i][1+j]=1; //break;
						}
					}
				}
			}

		for(int i=-1; i<=1; i++)
		{
			for(int j=-1;j<=1; j++)
			{
				printf("pixel (%d, %d), label %d\n", 1+i, 1+j, label_box[1+i][1+j]);

				if(label_box[1+i][1+j]>1)
				{
					component_connected=false;
					//break;
				}
			}
		}
	}

	if(component_connected)
	{
		printf("able to erase\n");
	}
	else
	{
		printf("unable to erase\n");
	}
}

void evg_thin::skel_thining() {

	int total_skeleton = 0;
	int erased_skeleton = 0;
    
	//first round, using case1 and case2;
	for (int x=1;x<grid_size_x-1;x++)
	    for (int y=1;y<grid_size_y-1;y++)
	    	if (_step2_grid[x][y]==skel)
	    	{
	    		total_skeleton++;
	    		//printf("\t pixel (%d, %d) \t", x, y);

	    		bool case1 = (_step2_grid[x-1][y-1]==skel||_step2_grid[x-1][y]==skel||_step2_grid[x-1][y+1]==skel) && (_step2_grid[x+1][y-1]==skel||_step2_grid[x+1][y]==skel||_step2_grid[x+1][y+1]==skel);
	    		bool case2 = (_step2_grid[x-1][y-1]==skel||_step2_grid[x][y-1]==skel||_step2_grid[x+1][y-1]==skel) && (_step2_grid[x-1][y+1]==skel||_step2_grid[x][y+1]==skel||_step2_grid[x+1][y+1]==skel);


	    		if(!case1 && !case2 ) continue;

	    		if(remove_center_pixel(x, y))
	    		{
	    			_step2_grid[x][y]=occupied;
	    			erased_skeleton++;
	    			printf("erase\t");

	    		}
	    	}

	printf("1st round: total skeleton %d\t, erased skeleton pixel %d\t", total_skeleton, erased_skeleton);

	//second round, using case3
	total_skeleton = 0;
	erased_skeleton = 0;
	for (int x=1;x<grid_size_x-1;x++)
	    for (int y=1;y<grid_size_y-1;y++)
	    	if (_step2_grid[x][y]==skel)
	    	{
	    		total_skeleton++;
	    		//printf("\t pixel (%d, %d) \t", x, y);

	    		int count = 0;
	    		for (int i=-1; i<=1; i++)
	    			    for (int j=-1; j<=1; j++)
	    			    	if((i!=0 || j!=0) && (_step2_grid[x+i][y+j]==skel))
	    			    	{
	    			    		count++;
	    			    	}

	    		bool case3 = (count >1);

	    		if(!case3) continue;

	    		if(remove_center_pixel(x, y))
	    		{
	    			_step2_grid[x][y]=occupied;
	    			erased_skeleton++;
	    			printf("erase");

	    		}
	    	}

	printf("2nd round: total skeleton %d\t, erased skeleton pixel %d\t", total_skeleton, erased_skeleton);
}

bool evg_thin::remove_center_pixel (int x, int y)
{
	//do connected-component analysis;
	int connectivity_serial = 0;
	std::vector < std::vector<int> > connected_pairs;

	bool skel_box[3][3];
	skel_box[1][1]=false;
	for (int i=-1; i<=1; i++)
		for (int j=-1; j<=1; j++)
			if (i!=0 || j!=0)
			{
				int nx = x+i;
				int ny = y+j;
				if(_step2_grid[nx][ny]==skel)
				{
					skel_box[1+i][1+j]=true;
				}
				else
				{
					skel_box[1+i][1+j]=false;
				}
			}

	//construct "label_box" for labeling
	int label_box[3][3];
	for(int i=-1; i<=1;  i++)
		for(int j=-1; j<=1; j++)
			label_box[1+i][1+j]=-1;

	for (int i=-1; i<=1; i++)
		for (int j=-1; j<=1; j++)
			if ((i!=0 || j!=0) && skel_box[1+i][1+j])
			{
				//printf("pixel (%d, %d)\n", 1+i, 1+j);

				//check 2 cells: left and up-left;
				for (int m=-1; m<=0; m++)
				{
					int n=-1;
					if(1+i+m>=0 && 1+i+m<=2 && 1+j+n>=0 && 1+j+n<=2)
					{
						if(skel_box[1+i+m][1+j+n]&&label_box[1+i+m][1+j+n]!=-1)
						{
							//printf("neibouring pixel (%d, %d): label %d\t", 1+i+m, 1+j+n, label_box[1+i+m][1+j+n]);
							label_box[1+i][1+j]=label_box[1+i+m][1+j+n];
						}
					}
				}

				//check 2 cells: up and up-right;
				for (int n=0; n<=1; n++)
				{
					int m=-1;
					if(1+i+m>=0 && 1+i+m<=2 && 1+j+n>=0 && 1+j+n<=2)
						if(skel_box[1+i+m][1+j+n] && label_box[1+i+m][1+j+n]!=-1)
						{
							//printf("(%d, %d): label %d", 1+i+m, 1+j+n, label_box[1+i+m][1+j+n]);

							if(label_box[1+i][1+j]==-1) label_box[1+i][1+j]=label_box[1+i+m][1+j+n];
							else if(label_box[1+i][1+j]!=label_box[1+i+m][1+j+n])
							{
								//already get some serial, but different from the current one;
								std::vector<int> pair_tmp;
								pair_tmp.push_back(label_box[1+i][1+j]);
								pair_tmp.push_back(label_box[1+i+m][1+j+n]);
								//printf("pair_tmp: %d, %d\t", label_box[1+i][1+j], label_box[1+i+m][1+j+n]);
								connected_pairs.push_back(pair_tmp);
							}
						}
				}

				if(label_box[1+i][1+j]==-1)
				{
					connectivity_serial ++;
					label_box[1+i][1+j]= connectivity_serial;
				}

				//printf("pixel (%d, %d), label %d\n", 1+i, 1+j, label_box[1+i][1+j]);
			}

	//re-sorted the connected_pieces, substitute the label, and find how many disconnected pieces exist;
	std::vector < std::vector<int> > connected_unions;
	for(size_t i=0; i<connected_pairs.size(); i++)
	{
		bool absorbed = false;
		bool break_mid_loop = false;
		for(size_t j=0; j<connected_unions.size(); j++)
		{
			for(size_t k=0; k<connected_unions[j].size(); k++)
			{
				if(connected_unions[j][k]==connected_pairs[i][0]
					   ||connected_unions[j][k]==connected_pairs[i][1])
				{
					absorbed = true;
					break_mid_loop=true;
					break;
				}
			}
			if(break_mid_loop) break;
		}

		if(!absorbed)
		{
			connected_unions.push_back(connected_pairs[i]);
		}
	}


	std::vector < std::vector<int> > sorted_unions;
	for(size_t i=0; i<connected_unions.size(); i++)
	{
		//bubble sorting for each union, from small to big;
		for(size_t a=0;a<connected_unions[i].size();a++)
		{
			for(size_t b=0;b<a;b++)
			{
				if(connected_unions[i][a]<connected_unions[i][b])
				{
					int temp=connected_unions[i][a];
					connected_unions[i][a]=connected_unions[i][b];
					connected_unions[i][b]=temp;
				}
			}

		}

		//for(size_t k=0; k<connected_unions[i].size(); k++) printf("%d\t", connected_unions[i][k]);

		std::vector<int> union_tmp;
		union_tmp.push_back(connected_unions[i][0]);
		for(size_t a=1;a<connected_unions[i].size();a++)
		{
			if(connected_unions[i][a]==union_tmp.back())continue;
			else union_tmp.push_back(connected_unions[i][a]);
		}
		sorted_unions.push_back(union_tmp);
	}

	bool component_connected = true;

	if(sorted_unions.size()==0 )
	{
		//printf("aaa\n");
		for(int i=-1; i<=1; i++)
		{
			for(int j=-1;j<=1; j++)
			{
				//printf("pixel (%d, %d), label %d\t", 1+i, 1+j, label_box[1+i][1+j]);

				if(label_box[1+i][1+j]>1)
				{
					component_connected=false;
				}
			}
		}
	}
	else if(sorted_unions.size()>1 )
	{
		component_connected=false;
		printf("bbb\n");
		/*
		for(size_t i=0; i<sorted_unions.size(); i++)
			for(size_t k=0; k<sorted_unions[i].size(); k++)
				printf("%d\t", sorted_unions[i][k]);
		*/
	}
	else if(sorted_unions.size()==1 && sorted_unions[0].front()!=1)
	{
		component_connected=false;
		printf("ccc\n");
		//for(size_t k=0; k<sorted_unions[0].size(); k++) printf("%d\t", sorted_unions[0][k]);
	}
	else if(sorted_unions.size()==1 && sorted_unions[0].front()==1)
	{
		printf("ddd\n");
		//for(size_t k=0; k<sorted_unions[0].size(); k++) printf("%d\t", sorted_unions[0][k]);

		for(int i=-1; i<=1;  i++)
			for(int j=-1;j<=1; j++)
			{
				if(label_box[1+i][1+j]>1)
				{
					for(size_t k=0; k<sorted_unions[0].size(); k++)
					{
						if(label_box[1+i][1+j]==sorted_unions[0][k])
						{
							label_box[1+i][1+j]=1; //break;
						}
					}
				}
			}

		for(int i=-1; i<=1; i++)
		{
			for(int j=-1;j<=1; j++)
			{
				//printf("pixel (%d, %d), label %d\n", 1+i, 1+j, label_box[1+i][1+j]);

				if(label_box[1+i][1+j]>1)
				{
					component_connected=false;
					//break;
				}
			}
		}
	}

	if(component_connected)
	{
		/*
		for (int i=-1; i<=1; i++)
			for (int j=-1; j<=1; j++)
				if (i!=0 || j!=0)
				{
					int nx = x+i;
					int ny = y+j;
					if(_step2_grid[nx][ny]==skel)
					{
						printf("skel (%d, %d)\t", 1+i, 1+j);
					}
				}
		*/

		return true;
	}
	else
	{
		return false;
	}
}

void evg_thin::extract_topology(){

	//1st: to construct a skeleton image from the data "_step2_grid";
	IplImage *skel_image_ = cvCreateImage( cvSize(grid_size_x, grid_size_y),IPL_DEPTH_8U, 1);

	int skel_height 		= skel_image_ -> height;
	int skel_width  		= skel_image_ -> width;
	int skel_step	 		= skel_image_ -> widthStep/sizeof(uchar);
	uchar * skel_data 	= (uchar*)skel_image_ ->imageData;
	for(int ih=0; ih < skel_height; ih++)
	{
		for(int iw=0; iw < skel_width; iw++)
		{
			if(_step2_grid[iw][ih] == skel)
			{
				skel_data[ih*skel_step+iw]=255;
			}
			else
			{
				//sometimes this sentence is necessary;
				skel_data[ih*skel_step+iw]=0;
			}
		}
	}
	cvSaveImage( "/home/baoxing/skel_image.jpg", skel_image_ );

	IplImage *img = skel_image_;

    CvMat *mat = cvCreateMat( img->height, img->width, CV_32FC1);
    CvMat *node_mat = cvCreateMat( img->height, img->width, CV_32FC1);
    CvMat *edge_mat = cvCreateMat( img->height, img->width, CV_32FC1);

    IplImage  *node =0, *edge = 0, *color_edge = 0;
    node = cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U, 1);
    edge = cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U, 1);
    color_edge = cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U, 3);

	 cvConvert( img, mat);

	 std::vector<CvPoint2D32f> node_points;
	 Graph_Extraction(mat, node_mat, edge_mat, node_points);

	 //do simple clustering for the node points;
	 std::vector <std::vector<CvPoint2D32f> > node_clusters;
	 for(size_t i=0; i<node_points.size(); i++)
	 {
		 CvPoint2D32f point_tmp = node_points[i];
		 int cluster_serial = -1;

		 for(size_t j=0; j<node_clusters.size(); j++)
		 {
			   if(cluster_serial !=-1) break;
				for(size_t k=0; k<node_clusters[j].size(); k++)
				{
					CvPoint2D32f point_tmpp;
					if(fmutil::distance(point_tmp, point_tmpp) < 5.0)
					{
						cluster_serial = j;
						break;
					}
				}
		 }
		 if(cluster_serial == -1)
		 {
				std::vector<CvPoint2D32f> cluster_tmp;
				cluster_tmp.push_back(point_tmp);
				node_clusters.push_back(cluster_tmp);
		 }
		 else
		 {
				node_clusters[cluster_serial].push_back(point_tmp);
		 }
	 }

	 std::vector<CvPoint>  merged_nodes;
	 for(size_t i=0; i<node_clusters.size(); i++)
	 {
		 float points_number = (float)node_clusters[i].size();
		 float x_tmp =0.0; float  y_tmp = 0.0;
		 for(size_t j=0; j<node_clusters[i].size(); j++)
		 {
			 x_tmp = x_tmp + node_clusters[i][j].x;
			 y_tmp = y_tmp + node_clusters[i][j].y;
		 }
		 x_tmp = x_tmp / points_number;
		 y_tmp = y_tmp / points_number;
		 merged_nodes.push_back(cvPoint(x_tmp, y_tmp));

		 cvCircle( color_edge, cvPoint(x_tmp, y_tmp), 5, CV_RGB(0,255,0), 2);
	 }

	 std::vector < std::vector<CvPoint> > edges;
     Extract_Edges(edge_mat, edges);

	 for(size_t i=0; i < edges.size(); i++)
	 {
		CvScalar ext_color;
		ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
		ext_color = CV_RGB( 0, 0, 255 );

        for(size_t j=0; j < edges[i].size(); j++) {
            int x = edges[i][j].x;
            int y = edges[i][j].y;
				cvSet2D(color_edge, y, x, ext_color);
        }
    }

    cvConvert( node_mat, node);
    cvConvert( edge_mat, edge);

    cvSaveImage( "/home/baoxing/node.jpg", node );
	 cvNamedWindow("node",1);
    cvShowImage("node", node);

    cvSaveImage( "/home/baoxing/edge.jpg", edge );
	 cvNamedWindow("edge",1);
    cvShowImage("edge", edge);

    cvSaveImage( "/home/baoxing/color_edge.jpg", color_edge );
	 cvNamedWindow("color_edge",1);
    cvShowImage("color_edge", color_edge);
}


void evg_thin::Graph_Extraction(CvMat *pSrc, CvMat *pDst, CvMat *pDst2, std::vector<CvPoint2D32f> & node_points)
{
		node_points.clear();
        int rows = pSrc->rows;
        int cols = pSrc->cols;

        for(int i = 1; i < rows-1; i++) {
                for(int j = 1; j < cols-1; j++) {
                        if ( CV_MAT_ELEM(*pSrc, float, i, j) == 255.0) {
                                /// get 8 neighbors
                                /// calculate C(p)
                                int neighbor0 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j-1);
                                int neighbor1 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j);
                                int neighbor2 = (int) CV_MAT_ELEM(*pSrc, float, i-1, j+1);
                                int neighbor3 = (int) CV_MAT_ELEM(*pSrc, float, i, j+1);
                                int neighbor4 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j+1);
                                int neighbor5 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j);
                                int neighbor6 = (int) CV_MAT_ELEM(*pSrc, float, i+1, j-1);
                                int neighbor7 = (int) CV_MAT_ELEM(*pSrc, float, i, j-1);

                                int Count = neighbor0 + neighbor1 + neighbor2 + neighbor3 + neighbor4 +
															neighbor5 + neighbor6 + neighbor7;

										  if(Count > 2*255 )
										  {
											  CV_MAT_ELEM(*pDst, float, i, j) = 255.0;
											  node_points.push_back(cvPoint2D32f(j, i));
										  }
										  else
										  {
											  //only record the remained edge points;
											  CV_MAT_ELEM(*pDst2, float, i, j) = 255.0;
										  }
                        }
                }
        }
}

//Quick and easy connected component (blob) using OpenCV
//http://nghiaho.com/?p=1102
void evg_thin::Extract_Edges(CvMat* pSrc, std::vector < std::vector<CvPoint> > & edges)
{
    edges.clear();
    CvMat *label_image = cvCloneMat( pSrc );

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image->rows; y++) {
        for(int x=0; x < label_image->cols; x++) {
            if((int)CV_MAT_ELEM(*label_image, float, y, x) != 1) {
                continue;
            }

            cvFloodFill(label_image, cvPoint(x,y), cvScalar(label_count), cvScalar(0), cvScalar(0), NULL, 8, NULL);

            std::vector <CvPoint> edge;

            for(int i= 0 ; i < label_image->rows; i++) {
                for(int j=0; j < label_image->cols; j++) {

                    if((int)CV_MAT_ELEM(*label_image, float, i, j)!= label_count)
                    {
                        continue;
                    }

                    edge.push_back(cvPoint(j,i));
                    //CV_MAT_ELEM(*label_image, float, i, j) = 0.0;
                }
            }

            edges.push_back(edge);

            label_count++;
        }
    }
}

/** 
    Build the final data structure of the skeleton from the grid with
    cells marked as skeleton or occupied.
**/
void evg_thin::build_skel() {
  // Use _step2_grid because after find_skel_edge(), it will have only
  // skel, occupied, and unknown cells.  prune_skel just changes
  // things in _step2_grid.


  // Walk along the skeleton away from the closest point to the robot.
  // Walk is best-first (using total distance from start).  If a
  // branch terminates, but is not next to an unknown cell or the
  // edge, that branch must be pruned.
  if (_distance>=0)
  {
    //if _distance==-1, then there was no skeleton found near the
    //robot.

	//skel_thining_test();
	skel_thining();
	//to extract and visualize the topology of the skeleton;
	extract_topology();

	crawl_grid();

	remove_final_spur();

	best_to_depth_first();
  }  
}

void evg_thin::calc_nearest_skeleton(){

	float rdist=FLT_MAX;
	_closestx=-1;
	_closesty=-1;
	_distance=-1;
	for (int i=0;i<grid_size_x;i++)
		for (int j=0;j<grid_size_y;j++)
			if(_step2_grid[i][j]==skel)
			{
				// Now find closest point to robot.  Make sure the robot
				// is within the radius of that point.
				float d=dist(robot_loc_x,robot_loc_y,i,j);
				float d2=distance_grid[i][j].distance;
				if (d < rdist && (!location || d<=d2))
				{
				  rdist=d;
				  _closestx=i;
				  _closesty=j;
				  _distance=d2;
				}
				else if (d==rdist && (!location || d<=d2) && (d2>_distance))
				{
				  _closestx=i;
				  _closesty=j;
				  _distance=d2;
				}
			}
}

/**
   Starting with the closest skeleton point to the robot, walk along
   the skeleton in the grid, building an intermediate skeleton data
   structure.  Do this in a best-first fashion, using cell distance as
   the cost function.  Also, if a branch ends but is not an exit (next
   to the edge or next to unknown cells in the occupancy grid), prune
   that branch back (basically, make the distance field of nodes in
   that branch equal to -1).
**/
void evg_thin::crawl_grid() {

	//re-calculate the nearest skeleton to the robot, since the one calculated before pruning maybe removed;
	calc_nearest_skeleton();

    _tmp_skel.clear();
  
	node new_node;
	new_node.x=_closestx;
	new_node.y=_closesty;
	new_node.distance=0;
	new_node.parent=-1;

	heap<node> open_list;
	//Closest point is root of tree.
	open_list.push(new_node);

	_num_exits=0;
	_root_index=0;

	bool cont;
	vector<node> children;
  
  while (!open_list.empty())
  {
    cont=true;
    node curr_node=open_list.first();
    open_list.pop();
    // If the current node has not yet been processed
    if (_step2_grid[curr_node.x][curr_node.y]==skel)
    {
    	//Mark the node as begin processed.
    	_step2_grid[curr_node.x][curr_node.y]=occupied;
    	//If exit (and not the tree root) stop searching this branch
    	if (is_exit(curr_node) && curr_node.parent>=0)
    	{
			_num_exits++;
			curr_node.num_children=0;
			curr_node.radius=distance_grid[curr_node.x][curr_node.y].distance;
			//update parent with the index of this node in _tmp_skel
			_tmp_skel[curr_node.parent].children.push_back(_tmp_skel.size());
			//update _tmp_skel with this node
			_tmp_skel.push_back(curr_node);
			cont=false;
    	}
    	//For non-exits
    	if (cont)
    	{
			//Find neighbors
			children=find_neighbors(curr_node.x,curr_node.y);
			curr_node.num_children=children.size();
			//If there are no children (no neighbors left to process),
			//this branch is a "spur" and needs to be removed.  Remove
			//from it's parent's children.
			if (curr_node.num_children==0)
			{
				//Called on parent because this node was never added to
				//_tmp_skel.
				remove_branch(curr_node.parent);
			}
			//For non-exits with children
			else
			{
				curr_node.radius=distance_grid[curr_node.x][curr_node.y].distance;
				int curr_index=_tmp_skel.size();
				//If not the root, update the parent to know the index of this node.
				if (curr_node.parent>=0) _tmp_skel[curr_node.parent].children.push_back(curr_index);
				//Add this node to the tree

				_tmp_skel.push_back(curr_node);

				//Now add children to the heap
				for (uint i=0;i<curr_node.num_children;i++)
				{
					new_node.x=children[i].x;
					new_node.y=children[i].y;
					//cost is distance (in skeleton nodes) to the root.
					new_node.distance= dist(new_node.x,new_node.y, curr_node.x,curr_node.y) + curr_node.distance;
					new_node.parent=curr_index;
					open_list.push(new_node);
				}
			}
    	}
    }
    //If no longer available for processing, remove this node from
    //it's parent's children.
    else 
      remove_branch(curr_node.parent);
  }
}


/**
   If a node is removed, because it terminates and is not an exit,
   mark it's distance = -1, and recurse on its parent, which may now
   need to be removed as well.
**/
void evg_thin::remove_branch(int index, int child_index) {
  if (prune) 
    if (index >=0)
    {
      //Remove unneccessary child from list of children.
      if (child_index>=0)
      {
    	  vector<uint> new_children;
    	  for (uint i=0;i<_tmp_skel[index].children.size();i++)
    		  if (_tmp_skel[index].children[i]!=uint(child_index))
    			  new_children.push_back(_tmp_skel[index].children[i]);
    	  _tmp_skel[index].children=new_children;
      }
      //Reduce number of children.  Different from children.size()
      //because not all children may have been processed (thus added to
      //the tree and updated their parent's children list).
      _tmp_skel[index].num_children--;
      
      //If no more children, prune this node from it's parent's child
      //list.
      if (_tmp_skel[index].num_children==0)
      {
		//Set distance to -1 so that later, we'll know this is a pruned
		//node (instead of having to take it out of the tree, which may
		//be hairy---requires updating indexes of all parents and
		//chilren which _tmp_skel is rearranged).
		_tmp_skel[index].distance=-1;
		remove_branch(_tmp_skel[index].parent, index);
      }
    }
}

/**
   If a node is next to unknown nodes or an edge of the grid, it is an
   exit.
**/
bool evg_thin::is_exit(const node& curr_node) {

  int i,j,nx,ny;

  for (i=-1;i<=1;i++) {
    nx=curr_node.x+i;
    for (j=-1;j<=1;j++)
      if (i!=0 || j!=0) {
	ny=curr_node.y+j;
	if (!on_grid(nx,ny) ||
	    original_grid[nx][ny]==Unknown)
	  return true;
      }
  }
  
  return false;
}

/**
   Find the immediate neighbors, save their grid coordinates, and push
   them all into a list.
 **/
vector<node> evg_thin::find_neighbors(int x, int y) {
  vector<node> neighbors;
  
  node c1;
  int i,j,newx,newy;
  
  for (i=-1;i<=1;i++) {
    newx=x+i;
    for (j=-1;j<=1;j++) 
      if (i!=0 || j!=0) {
	newy=y+j;
	if (on_grid(newx,newy) &&
	    _step2_grid[newx][newy]==skel) {
	  c1.x=newx;
	  c1.y=newy;
	  neighbors.push_back(c1);
	}
      }
  }
  return neighbors;
}


/**
   We used the closest skeleton point to the robot to know which
   skeleton (thinning may find many) is the correct one to use.
   Sometimes, the branch that includes the closest point to the robot
   needs to be pruned.  This does that (essentially reordering the
   tree to have a new root).
 **/
void evg_thin::remove_final_spur()
{
  if (prune)
    if (!_tmp_skel.empty())
      if (_num_exits > 1 && !is_exit(_tmp_skel[0]) && _tmp_skel[0].num_children==1)
      {
		//If not an exit and only 1 child (terminal node) and more than
		//1 exit exists (this isn't the only branch) then prune the
		//branch.
		_tmp_skel[0].distance=-1;
		remove_branch2(_tmp_skel[0].children[0]);
      }
}


/**
   Similar to remove_branch, but going down the branch instead of up
   (to prune the root of the skeleton tree).  

   If a node is removed, because it terminates and is not an exit,
   mark it's distance = -1, and recurse on its child, which may now
   need to be removed as well.
**/
void evg_thin::remove_branch2(int index) {
  if (_tmp_skel[index].num_children==1) {
    _tmp_skel[index].distance=-1;
    remove_branch2(_tmp_skel[index].children[0]);
  }
  else {
    _tmp_skel[index].parent=-1;
    _root_index=index;
  }
}

/**
   Go from the intermediate skeleton (from crawl_grid) to a fully
   pruned data structure.  While doing this, its convient (for other
   modules that may use the skeleton) to make this depth first.  Also,
   when we see exits, add them to the internal or external exists list
   (to be saved in the local topology data structure).
 **/
void evg_thin::best_to_depth_first() {
  if (!_tmp_skel.empty())
    best_to_depth_first_helper(_root_index,-1);
}


/**
   The recursive function that does the actual work of best_to_depth_first().
 **/
void evg_thin::best_to_depth_first_helper(int myindex, int parent_index){

  // Check distance (it's set to -1 for pruned nodes).  Only add
  // non-pruned cells to the final skeleton.

  if (_tmp_skel[myindex].distance>=0)
  {
    //Curr node is somewhere (given a depth first search) in the _tmp_skel
    node curr_node;
    curr_node.x=_tmp_skel[myindex].x;
    curr_node.y=_tmp_skel[myindex].y;
    curr_node.radius=_tmp_skel[myindex].radius;
    curr_node.distance=_tmp_skel[myindex].distance;
    curr_node.num_children=_tmp_skel[myindex].children.size();

    //parent has a new index in the depth first tree.
    curr_node.parent=parent_index;
    // Delete children indexes.  They will recreate this list when
    // they get added.
    
    uint new_index=curr_skel.size();

    //Update parents list of children to include the new index of the
    //node.
    if (parent_index>=0)
      curr_skel[parent_index].children.push_back(new_index);

    //Add to the new, depth-first tree.
    curr_skel.push_back(curr_node);

    //Do a depth first traversal of the skeleton.
    for (uint i=0;i<curr_node.num_children;i++) 
      best_to_depth_first_helper(_tmp_skel[myindex].children[i],new_index);
    
  }
  
}

