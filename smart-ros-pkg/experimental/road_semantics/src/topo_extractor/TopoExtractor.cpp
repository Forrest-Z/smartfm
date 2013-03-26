#include "TopoExtractor.h"

topo_extractor::topo_extractor(const grid_type& curr_grid,
		   float distance_min, float distance_max, 
		   bool pruning) {

  original_grid=curr_grid;
  coastal_dist=distance_max;
  prune_dist=distance_min;
  prune=pruning;

  grid_size_x=original_grid.size();
  grid_size_y=original_grid[0].size();

  vector <State> tmp(grid_size_y);
  _step1_grid.resize(grid_size_x,tmp);

  dist_col tmpcol(grid_size_y);
  distance_grid.resize(grid_size_x,tmpcol);
}

//the core function of "topo_extractor";
void topo_extractor::extract_topology()
{
	//this 2 functions are from evg-thin;
	printf("calculate_distances\n");
	calculate_distances();

	printf("find_skel\n");
	find_skel();

	//to further thining the skeleton to single pixel;
	printf("skel_thining\n");
	skel_thining();

	//to extract and visualize the topology of the skeleton;
	printf("\n extract_node_edge\n");
	extract_node_edge();
}

bool topo_extractor::on_grid(int x, int y) {
  return ((x >=0 && x < grid_size_x) &&
	  (y >=0 && y < grid_size_y));
}

/**
   Calculate the distance of each free cell from the closest occupied
   cell.  Stores these (along with the location of the closest
   obstacle) in distance_grid.
**/
void topo_extractor::calculate_distances() {

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
  this function comes from evg-thin;
**/
void topo_extractor::find_skel()
{
	//First initialize the grid by labeling cells
	initialize();
	//Then "thin" the grid by flipping free cells that border occupied
	//cells to occupied (when applicaple).
	thin();
	//Search for the actual skeleton cells after then thinning is done.
	find_skel_edge();
}

/**
   Initialize _step1_grid to be "occupied" for all occupied grid
   cells.  Any free grid cells become either "free" (most free cells)
   or "processing" if they are next to obstacles.  Also, put any
   "processing" cells in the _step1_queue.
**/
void topo_extractor::initialize() {
  
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
void topo_extractor::thin() {
  
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

topo_extractor::State topo_extractor::step(gridtype& grid,
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

void topo_extractor::find_skel_edge() {
  // Don't worry about making _step1_grid and _step2 equal.  After
  // thin(), if there is any difference it would only be that
  // _step1_grid had some cells marked "skel" that are marked
  // "processed" in _step2_grid.  In this function, all cells that are
  // "skel" or "procssed" in _step1_grid become either skel or
  // occupied in _step2_grid.  Then _step2 grid can be used for
  // pruning and making the skeleton data structure.


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
	  }
	}
}


void topo_extractor::skel_thining() {

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
	    			//printf("erase\t");

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
	    			//printf("erase");
	    		}
	    	}

	printf("2nd round: total skeleton %d\t, erased skeleton pixel %d\t", total_skeleton, erased_skeleton);
}


//this function can also be easily achieved by using "cvFloodFill"(or its basic idea);
//just to check whether after removing the center pixel, the connectivity inside the box is changed or not.
bool topo_extractor::remove_center_pixel (int x, int y)
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
		/*
		for(size_t i=0; i<sorted_unions.size(); i++)
			for(size_t k=0; k<sorted_unions[i].size(); k++)
				printf("%d\t", sorted_unions[i][k]);
		*/
	}
	else if(sorted_unions.size()==1 && sorted_unions[0].front()!=1)
	{
		component_connected=false;
		//for(size_t k=0; k<sorted_unions[0].size(); k++) printf("%d\t", sorted_unions[0][k]);
	}
	else if(sorted_unions.size()==1 && sorted_unions[0].front()==1)
	{
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

//to be continued:
//a. use uniform data-type of IplImage;
//b. generate the result in "road_graph_";

void topo_extractor::extract_node_edge(){

	/////////////////////////////////////////////////////////////////////
	//1st: to construct a skeleton image from the data "_step2_grid";
	/////////////////////////////////////////////////////////////////////

	IplImage *skel_image_ 	= cvCreateImage( cvSize(grid_size_x, grid_size_y),IPL_DEPTH_8U, 1);
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
	cvSaveImage( "/home/baoxing/raw_skel.jpg", skel_image_ );

	///////////////////////////////////////////////////////////////////
	//2nd: to extract nodes, node clusters, and edges;
	///////////////////////////////////////////////////////////////////
	IplImage *img = skel_image_;
    CvMat *mat = cvCreateMat( img->height, img->width, CV_32FC1);
	cvConvert( img, mat);
	Graph_Extraction(mat);


    //////////////////////////////////////////////////////////////////////////////////////
    //3rd: build the relationship of "node and edge" in "road_graph_";
    //////////////////////////////////////////////////////////////////////////////////////
	build_topoloty();

	//////////////////////////////////////////////////////////////////////////////////////
	//4th: add some filtering algorithm to erase irrelevant edges;
	//////////////////////////////////////////////////////////////////////////////////////
	topo_filtering();

	//for visualization purposes, to check the extracted nodes and edges;
    IplImage *color_edge = 0;
    color_edge = cvCreateImage(cvSize(img->width,img->height),IPL_DEPTH_8U, 3);
    cvZero(color_edge);

	for(size_t i=0; i<road_graph_.nodeClusters.size(); i++)
	 {
		 float points_number = (float)road_graph_.nodeClusters[i].nodeIDs.size();
		 float x_tmp =0.0; float  y_tmp = 0.0;

		 for(size_t j=0; j<road_graph_.nodeClusters[i].nodeIDs.size(); j++)
		 {
			 x_tmp = x_tmp + road_graph_.nodes[(road_graph_.nodeClusters[i].nodeIDs[j])].position.x;
			 y_tmp = y_tmp + road_graph_.nodes[(road_graph_.nodeClusters[i].nodeIDs[j])].position.y;
		 }
		 x_tmp = x_tmp / points_number;
		 y_tmp = y_tmp / points_number;
		 road_graph_.nodeClusters[i].cluster_center = cvPoint2D32f(x_tmp, y_tmp);
		 cvCircle( color_edge, cvPoint(x_tmp, y_tmp), 5, CV_RGB(0,255,0), 2);
	 }

	 for(size_t i=0; i<road_graph_.edges.size(); i++)
	 {
		 CvScalar ext_color;
		 ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 );

		 for(size_t j=0; j<road_graph_.edges[i].points.size(); j++)
		 {
			 int x = road_graph_.edges[i].points[j].x;
			 int y = road_graph_.edges[i].points[j].y;
			 cvSet2D(color_edge, y, x, ext_color);
		 }
	 }

    cvSaveImage( "/home/baoxing/color_edge.jpg", color_edge );

    cvReleaseMat(&mat);
    cvReleaseImage(&skel_image_);
    cvReleaseImage(&color_edge);

}

void topo_extractor::Graph_Extraction(CvMat *pSrc)
{
	int rows = pSrc->rows;
	int cols = pSrc->cols;

    CvMat *pDst = cvCreateMat( pSrc->height, pSrc->width, CV_32FC1);
    CvMat *pDst2 = cvCreateMat( pSrc->height, pSrc->width, CV_32FC1);

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

				int Count = neighbor0 + neighbor1 + neighbor2 + neighbor3 + neighbor4 + neighbor5 + neighbor6 + neighbor7;

				//if at least 3 neighborhood skel points;
				if(Count >= 3*255 )
				{
					CV_MAT_ELEM(*pDst, float, i, j) = 255.0;
					CV_MAT_ELEM(*pDst2, float, i, j) = 0.0;
				}
				else
				{
					//only record the remained edge points;
					CV_MAT_ELEM(*pDst, float, i, j) = 0.0;
					CV_MAT_ELEM(*pDst2, float, i, j) = 255.0;
				}
			}
		}
	}

	CvMat *node_label_image = cvCloneMat( pDst );
	int label_count = 256; // starts at 256 because 0,255 are used already
	int node_serial = 0;
	int cluster_serial = 0;
	int edge_serial = 0;

	for(int y=0; y < node_label_image->rows; y++) {
		for(int x=0; x < node_label_image->cols; x++) {

			if((int)CV_MAT_ELEM(*node_label_image, float, y, x) != 255) continue;

			cvFloodFill(node_label_image, cvPoint(x,y), cvScalar(label_count), cvScalar(0), cvScalar(0), NULL, 8, NULL);

			topo_graph::node_cluster cluster_tmp;
			cluster_tmp.ID = cluster_serial;
			cluster_serial++;

			for(int i= 0 ; i < node_label_image->rows; i++) {
				for(int j=0; j < node_label_image->cols; j++)
				{
					if((int)CV_MAT_ELEM(*node_label_image, float, i, j)!= label_count)
					{
						continue;
					}
					else
					{
						topo_graph::node node_tmp;
						node_tmp.ID = node_serial;
						node_tmp.position.x = j;
						node_tmp.position.y = i;
						road_graph_.nodes.push_back(node_tmp);
						cluster_tmp.nodeIDs.push_back(node_serial);
						node_serial++;
					}
				}
			}
			road_graph_.nodeClusters.push_back(cluster_tmp);
			label_count++;
		}
	}

	CvMat *edge_label_image = cvCloneMat( pDst2);
	label_count = 256;
	for(int y=0; y < edge_label_image->rows; y++) {
		for(int x=0; x < edge_label_image->cols; x++) {
			if((int)CV_MAT_ELEM(*edge_label_image, float, y, x) != 255) continue;
			cvFloodFill(edge_label_image, cvPoint(x,y), cvScalar(label_count), cvScalar(0), cvScalar(0), NULL, 8, NULL);

			topo_graph::edge edge_tmp;
			edge_tmp.ID = edge_serial;
			edge_serial++;
			edge_tmp.head_nodeCluster = -1;
			edge_tmp.end_nodeCluster = -1;

			for(int i= 0 ; i < edge_label_image->rows; i++) {
				for(int j=0; j < edge_label_image->cols; j++)
				{
					if((int)CV_MAT_ELEM(*edge_label_image, float, i, j)!= label_count)
					{
						continue;
					}
					else
					{
						CvPoint edge_point_tmp;
						//pay attention to the x, y coordinates;
						edge_point_tmp.x = j;
						edge_point_tmp.y = i;
						edge_tmp.points.push_back(edge_point_tmp);
					}
				}
			}
			road_graph_.edges.push_back(edge_tmp);
			label_count++;
		}
	}
}

//-------------------------------------------IMPORTANT---------------------------------------------------
//pay attention to the IDs of nodes, node_clusters, and edges;
//before any filtering process, their IDs correspond to their position in the storing vector;
//we can safely use their position in the storing vector as their IDs;

void topo_extractor::build_topoloty()
{
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //pay attention to this bar of code: the logic is a little subtle;
    //a.assumption:
    // 	one edge has at most two node (one head and one end) and at least one node (one head); and each edge is single-pixel thick;
    // 	when the edge is a trunk, it has two node; when the edge is some branch, it has only one head node;
    // 	before this processing, it is possible that some small broken pieces of edge is not linked to the trunk, they have no nodes, and will be discarded automatically after this process;
    //b.logic flow
    //	One nodeCluster tries to find its connecting edge, if this edge hasn't been linked to some node, then this node is its heading node; the connecting point in this edge will be the
    //	first point of this edge; then this edge will be re-arranged by connecting other points from this first point, one-by-one;
    //	if this edge has been linked, then it is a end node;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	vector <bool> edge_flags(road_graph_.edges.size(), false);

	for(size_t i=0; i<road_graph_.nodeClusters.size(); i++)
	{
		printf("\n ---node cluster serial %ld ---\n", i);

		for(size_t j=0; j<road_graph_.nodeClusters[i].nodeIDs.size(); j++)
		{
			//While there are multiple nodes in one single nodeCluster, it is not possible for two nodes in the same nodeCluster to connect one same edge;

			size_t node_position_tmp = road_graph_.nodeClusters[i].nodeIDs[j];
			topo_graph::node node_tmp = road_graph_.nodes[node_position_tmp];

			for(size_t k=0; k<road_graph_.edges.size(); k++)
			{
				for(size_t p=0; p<road_graph_.edges[k].points.size(); p++)
				{
					CvPoint edge_pt_tmp = road_graph_.edges[k].points[p];
					bool connecting = check_8connectivity(node_tmp.position, edge_pt_tmp);
					if(connecting)
					{
						printf("\t---edge serial %ld\t", k);

						if(!edge_flags[k])
						{
							//this edge hasn't been linked to some node; find the edge's headCluster serial;
							edge_flags[k] = true;
							topo_graph::edge edge_rearrange;
							edge_rearrange.ID = road_graph_.edges[k].ID;
							edge_rearrange.head_nodeCluster = (int)i;
							edge_rearrange.end_nodeCluster = road_graph_.edges[k].end_nodeCluster;

							printf("head_nodeCluster %ld\t", i);

							//nodeCluster records its connecting edge (ID);
							road_graph_.nodeClusters[i].edgeIDs.push_back(k);

							//this edge_pt_tmp will be the first point in the newly re-arranged edge;
							//the rearrangement is used to sort the order of the points in the edge; in the original edges, sequences of points are sort of random, due to the flood-fill labelling;
							//after rearrangement, we can calculate the length of this edge for some filtering purposes;
							edge_rearrange.points.push_back(edge_pt_tmp);
							vector<CvPoint> points_tmp = road_graph_.edges[k].points;
							points_tmp.erase (points_tmp.begin()+p);

							while(points_tmp.size()!=0)
							{
								for(size_t ipp= 0; ipp < points_tmp.size(); ipp++)
								{
									if(check_8connectivity(points_tmp[ipp], edge_rearrange.points.back()))
									{
										edge_rearrange.points.push_back(points_tmp[ipp]);
										points_tmp.erase(points_tmp.begin()+ipp);
										//re-search from the beginning of this vector;
										ipp = 0;
									}
								}
							}

							road_graph_.edges[k] = edge_rearrange;
							break;
						}
						else
						{
							printf("end_nodeCluster %ld\t", i);
							//the edge has been linked to one node and re-arranged;
							//the only possibility is that this nodeCluster is its "end_nodeCluster";
							road_graph_.edges[k].end_nodeCluster = (int)i;
							road_graph_.nodeClusters[i].edgeIDs.push_back(k);
							break;
						}

						break;
					}
				}
			}


		}
	}

	printf("\n full topology \n");
	printf_topology();
}

inline bool topo_extractor::check_8connectivity(CvPoint pt1, CvPoint pt2)
{
	bool connecting = false;
	for(int a=-1; a<=1;  a++)
		for(int b=-1;b<=1; b++)
			if(a!=0 || b!=0)
			{
				CvPoint p1_neighbor;
				p1_neighbor.x = pt1.x + a;
				p1_neighbor.y = pt1.y + b;
				if(p1_neighbor.x==pt2.x && p1_neighbor.y==pt2.y) connecting = true;
			}
	return connecting;
}

void topo_extractor::printf_topology()
{
	printf("\n-----------------------------------------------------\n");
	printf("\n-----------------------------------------------------\n");

	for(size_t i=0; i<road_graph_.nodeClusters.size(); i++)
	{
		printf("\n nodeCluster serial %ld, ID %d\t", i, road_graph_.nodeClusters[i].ID);
		for(size_t j=0; j<road_graph_.nodeClusters[i].edgeIDs.size(); j++)
		{
			printf("edge %d\t", road_graph_.nodeClusters[i].edgeIDs[j]);
		}
	}

	for(size_t i=0; i<road_graph_.edges.size(); i++)
	{
		printf("\n edge serial %ld, ID %d\t", i, road_graph_.edges[i].ID);
		printf("head_cluster %d, end_cluster %d", road_graph_.edges[i].head_nodeCluster, road_graph_.edges[i].end_nodeCluster);
		if(road_graph_.edges[i].head_nodeCluster ==road_graph_.edges[i].end_nodeCluster) printf("\t loop!!!");
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//pay special attention to the filtering process, i.e. the erase of some edges;
//it may change the serial number of each edge, then the connectivity between the node and edges;
////////////////////////////////////////////////////////////////////////////////////////////////////
void topo_extractor::topo_filtering()
{
	IplImage *final_image 	= cvCreateImage( cvSize(grid_size_x, grid_size_y), IPL_DEPTH_8U, 1);
	cvZero(final_image);
	CvScalar gray_value; gray_value.val[0]=255;

	printf("\n");
	for(size_t k=0; k<road_graph_.edges.size(); k++)
	{
		road_graph_.edges[k].edge_length = 0.0;
		for(size_t i=1; i<road_graph_.edges[k].points.size(); i++)
		{
			road_graph_.edges[k].edge_length = road_graph_.edges[k].edge_length +
					fmutil::distance(road_graph_.edges[k].points[i], road_graph_.edges[k].points[i-1]);
		}
		printf("edge ID %ld, length %5f", k, road_graph_.edges[k].edge_length);
	}
	printf("\n");

	//filter the broken pieces of edge segment have no connecting node;
	//or filter those too-short pieces that have only one connecting node;

	//filter the edges;
	vector<topo_graph::edge> edges_tmp;
	for(size_t k=0; k<road_graph_.edges.size(); k++)
	{
		printf("head nodeCluster %d, end nodeCluster %d\t", road_graph_.edges[k].head_nodeCluster, road_graph_.edges[k].end_nodeCluster);
		if(road_graph_.edges[k].end_nodeCluster!=-1)
		{
			edges_tmp.push_back(road_graph_.edges[k]);
		}
		else if (road_graph_.edges[k].head_nodeCluster !=-1 && road_graph_.edges[k].end_nodeCluster ==-1)
		{
			if(road_graph_.edges[k].edge_length < EDGE_LENGTH_THRESHOLD)
			{
				//"node_cluster" (in nodeClusters) hasn't been filtered, so we can still use its ID as its position in vector;
				topo_graph::node_cluster & cluster_tmp = road_graph_.nodeClusters[road_graph_.edges[k].head_nodeCluster];
				for(size_t i=0; i<cluster_tmp.edgeIDs.size(); i++)
				{
					if(cluster_tmp.edgeIDs[i]==road_graph_.edges[k].ID)
					{
						cluster_tmp.edgeIDs.erase(cluster_tmp.edgeIDs.begin()+i);
						break;
					}
				}
			}
			else
			{
				edges_tmp.push_back(road_graph_.edges[k]);
			}
		}
	}
	road_graph_.edges = edges_tmp;

	//filter the nodeCluster;
	vector<topo_graph::node_cluster> clusters_tmp;
	for(size_t k=0; k<road_graph_.nodeClusters.size(); k++)
	{
		if(road_graph_.nodeClusters[k].edgeIDs.size() == 1)
		{

		}
		else if(road_graph_.nodeClusters[k].edgeIDs.size() == 2)
		{
			CvPoint pts[2];
			for(size_t i=0; i<2; i++)
			{
				int edge_ID = road_graph_.nodeClusters[k].edgeIDs[i];
				int edge_position = road_graph_.find_ID_position <topo_graph::edge> (road_graph_.edges, edge_ID);
				if(edge_position==-1) {printf("cannot find the queried edge ID, some bug happens"); break;}

				if(road_graph_.edges[edge_position].end_nodeCluster == (int)k) {pts[i] = road_graph_.edges[edge_position].points.back();}
				else if(road_graph_.edges[edge_position].head_nodeCluster == (int)k) {pts[i] = road_graph_.edges[edge_position].points.front();}
			}

			//bug-once-here: this step may lead to some redundant pixels, which need to be thinned in one more thinning step;
			cvLine(final_image, pts[0], pts[1], gray_value);
		}
		else
		{
			clusters_tmp.push_back(road_graph_.nodeClusters[k]);
		}

	}
	road_graph_.nodeClusters = clusters_tmp;

	topo_extractor::printf_topology();

	//re-build topology with the "final_image";
	 for(size_t i=0; i<road_graph_.edges.size(); i++)
	 {
		 for(size_t j=0; j<road_graph_.edges[i].points.size(); j++)
		 {
			 int x = road_graph_.edges[i].points[j].x;
			 int y = road_graph_.edges[i].points[j].y;
			 cvSet2D(final_image, y, x, gray_value);
		 }
	 }

	 for(size_t i=0; i<road_graph_.nodeClusters.size(); i++)
	 {

		 for(size_t j=0; j<road_graph_.nodeClusters[i].nodeIDs.size(); j++)
		 {
			 size_t node_ID_tmp = (size_t) road_graph_.nodeClusters[i].nodeIDs[j];
			 int x = road_graph_.nodes[node_ID_tmp].position.x;
			 int y = road_graph_.nodes[node_ID_tmp].position.y;
			 cvSet2D(final_image, y, x, gray_value);
		 }
	 }

	//add the "one-more" thinning operation caused by the previous "cvLine" step;
	int img_height 		= final_image -> height;
	int img_width  		= final_image -> width;
	int img_step	 	= final_image -> widthStep/sizeof(uchar);
	uchar * img_data 	= (uchar*)final_image ->imageData;
	for(int ih=0; ih < img_height; ih++)
	{
		for(int iw=0; iw < img_width; iw++)
		{
			if(img_data[ih*img_step+iw]==gray_value.val[0])
			{
				//free means bright color;
				_step2_grid[iw][ih]=skel;
			}
			else
			{
				_step2_grid[iw][ih]=occupied;
			}
		}
	}
	skel_thining();
	cvZero(final_image);
	for(int ih=0; ih < img_height; ih++)
	{
		for(int iw=0; iw < img_width; iw++)
		{
			if(_step2_grid[iw][ih]==skel)
			{
				img_data[ih*img_step+iw]=gray_value.val[0];

			}
		}
	}
	cvSaveImage( "/home/baoxing/final_image.jpg", final_image );

	CvMat *mat = cvCreateMat( final_image->height, final_image->width, CV_32FC1);
	road_graph_.clear();
	cvConvert( final_image, mat);
	Graph_Extraction(mat);
	build_topoloty();
}




