#include "TopoSemantic.h"

topo_semantic::topo_semantic(topo_extractor& topo_extractor_object)
{
	topo_extractor_pt_ = new topo_extractor(topo_extractor_object);
	cvNamedWindow("cycle_window");
	cvNamedWindow("grow_window");
	cvNamedWindow("cycle_filter");
}

void topo_semantic::analyze_semantic()
{
	//analyse semantics from "nodeClusters", to find "places" like intersections and roundabout;
	place_analyze();
	link_analyze();
}

void topo_semantic::place_analyze()
{
	roundabout_analyze();
	intersection_analyze();
}

//to recognize roundabout from road network;
void topo_semantic::roundabout_analyze()
{
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//// 1st step: loop through all of the node-clusters, to find cycles;
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	cycle_extraction();

	//2nd step: perform classification for the cycles, based on the network features;

	//3rd step: determine the area of extracted roundabouts;
}

void topo_semantic::cycle_extraction()
{
	//to store the growing paths;
	vector < vector <path_segment> > growing_paths;

	vector <path_segment> virtual_origin_path;
	path_segment virtual_origin_seg;
	virtual_origin_seg.head_nodeID = -1;
	virtual_origin_seg.tail_nodeID = 0;
	virtual_origin_path.push_back(virtual_origin_seg);

	path_growing_branching(virtual_origin_path, growing_paths);
	for(size_t i=0; i<growing_paths.size(); i++)
	{
		//remove the virtual original path;
		growing_paths[i].erase(growing_paths[i].begin());
	}
	recursive_path_search(growing_paths);

	//remove the outer cycles when some cycles are nested inside;
	cycle_filter();
	visualization();
}

void topo_semantic::recursive_path_search(vector <vector <path_segment> > &growing_paths)
{

	ROS_INFO("recursive path search");

	//1st: check all the growing paths:
	//extract cycle paths; remove cycle paths and stopping paths from the growing_paths;
	visualize_growing(growing_paths);
	path_check_remove(growing_paths);
	visualization();

	ROS_INFO("growing_path: %ld, cycle size(): %ld", growing_paths.size(), extracted_cycles_.size());

	//2nd: recursively perform path search until all the growing paths are removed from the vector;
	if(growing_paths.size()==0)return;
	else
	{
		vector < vector <path_segment> > growing_paths_new;
		for(size_t i=0; i<growing_paths.size(); i++)
		{
			path_growing_branching(growing_paths[i], growing_paths_new);
		}

		ROS_INFO("path_growing done");
		growing_paths = growing_paths_new;
		recursive_path_search(growing_paths);
	}
}

void topo_semantic::path_growing_branching(vector <path_segment> &path_father, vector < vector <path_segment> > & path_sons)
{
	int father_edgeID = path_father.back().edgeID;
	int tail_nodeID = path_father.back().tail_nodeID;
	//ROS_INFO("tail_nodeID %d", tail_nodeID);

	topo_graph::node_cluster seedNode = topo_extractor_pt_->road_graph_.nodeClusters[tail_nodeID];


	for(size_t ie=0; ie<seedNode.edgeIDs.size();ie++)
	{
		int edgeID_tmp = seedNode.edgeIDs[ie];

		//to prevent backward searching;
		if(father_edgeID == edgeID_tmp) continue;

		vector <path_segment> path_son = path_father;
		path_segment segment_tmp;
		topo_graph::edge edge_segment = topo_extractor_pt_->road_graph_.edges[size_t(edgeID_tmp)];
		segment_tmp.edgeID = edgeID_tmp;

		if(edge_segment.head_nodeCluster == tail_nodeID)
		{
			segment_tmp.head_nodeID = edge_segment.head_nodeCluster;
			segment_tmp.tail_nodeID = edge_segment.end_nodeCluster;
		}
		else if(edge_segment.end_nodeCluster == tail_nodeID)
		{
			segment_tmp.head_nodeID = edge_segment.end_nodeCluster;
			segment_tmp.tail_nodeID = edge_segment.head_nodeCluster;
		}
		else
		{
			ROS_ERROR("BUG: edge connection error!");
			return;
		}
		path_son.push_back(segment_tmp);
		path_sons.push_back(path_son);
	}
}

void topo_semantic::path_check_remove(vector < vector <path_segment> > & growing_paths)
{
	ROS_INFO("path_check_remove before: %ld", growing_paths.size());

	for(size_t i=0; i<growing_paths.size(); i++)
	{
		int path_tail_nodeID = growing_paths[i].back().tail_nodeID;
		int seg_tail_nodeID = growing_paths[i].back().head_nodeID;
		ROS_INFO("----------------path_tail_nodeID %d, segment_headID: %d----------", path_tail_nodeID, seg_tail_nodeID);

		if(path_tail_nodeID == -1)
		{
			growing_paths.erase(growing_paths.begin()+i);
			i=i-1;
		}
		else
		{
			for(size_t j=0; j<growing_paths[i].size(); j++)
			{
				int seg_tail_headID = growing_paths[i][j].head_nodeID;
				//ROS_INFO("along this path, head nodeID %d", seg_tail_headID);
				if(seg_tail_headID == path_tail_nodeID)
				{
					//only keep the loop part, by erasing the irrelevant head part;
					growing_paths[i].erase(growing_paths[i].begin(), growing_paths[i].begin()+j);

					if(!cycle_duplicate(growing_paths[i])) extracted_cycles_.push_back(growing_paths[i]);
					growing_paths.erase(growing_paths.begin()+i);
					i=i-1;
					break;
				}
			}
		}
	}

	ROS_INFO("path_check_remove after: %ld", growing_paths.size());
}

bool topo_semantic::cycle_duplicate(vector <path_segment> &cycle_under_check)
{
	for(size_t i=0; i<extracted_cycles_.size(); i++)
	{
		vector <path_segment> cycle_component = extracted_cycles_[i];

		bool cycle_duplicate = true;
		//check every segment to find possible exactly same cycle;
		if(cycle_component.size()==cycle_under_check.size())
		{
			for(size_t j=0; j<cycle_under_check.size(); j++)
			{
				path_segment segment_under_check = cycle_under_check[j];
				bool segment_equal = false;
				for(size_t a=0; a<cycle_component.size(); a++)
				{
					if(cycle_component[a].edgeID == segment_under_check.edgeID)
					{
						segment_equal = true;
						break;
					}
				}
				if(!segment_equal) cycle_duplicate = false;
			}
		}
		else
		{
			cycle_duplicate = false;
		}

		if(cycle_duplicate)
		{
			return true;
		}
	}
	return false;
}

//to filter those cycles nesting some small cycles;
void topo_semantic::cycle_filter()
{
	CvSize image_size = cvSize(topo_extractor_pt_->grid_size_x, topo_extractor_pt_->grid_size_y);
	IplImage *cycle_image = cvCreateImage(image_size, 8,3);

	//1st: find shared path segment between different cycles;
	map<int,int> edges_number;
	vector<size_t> shared_edgeIDs;

	for(size_t i=0; i<extracted_cycles_.size(); i++)
	{
		for(size_t j=0; j<extracted_cycles_[i].size(); j++)
		{
			path_segment edge_segment_tmp = extracted_cycles_[i][j];
			map<int, int>::iterator it = edges_number.find(edge_segment_tmp.edgeID);
			if(it != edges_number.end()){it->second = it->second+1;}
			else {edges_number.insert(make_pair(edge_segment_tmp.edgeID, 1));}
		}
	}

	for(map <int, int>::iterator it = edges_number.begin(); it != edges_number.end(); it++ )
	{
		if(it->second>=2)shared_edgeIDs.push_back(it->first);
	}
	ROS_INFO("shared_edgeIDs number %ld", shared_edgeIDs.size());

	//2nd: check a cycle (without this edge), whether this edge (its center point) is inside the cycle polygon;

	vector<CvPoint2D32f> centerPt_vector;
	for(size_t i=0; i<shared_edgeIDs.size(); i++)
	{
		//find the center point of shared edges;
		topo_graph::edge edge_tmp = topo_extractor_pt_->road_graph_.edges[size_t(shared_edgeIDs[i])];
		int center_serial = int(edge_tmp.points.size())/2-1; //each edge has at least 2 points;
		CvPoint center_tmp = edge_tmp.points[center_serial];
		centerPt_vector.push_back(cvPoint2D32f(center_tmp.x, center_tmp.y));
	}


	for(size_t i=0; i<extracted_cycles_.size(); i++)
	{
		cvZero(cycle_image);

		//construct the cycle_polygon;
		vector <path_segment> cycle_component = extracted_cycles_[i];
		vector<CvPoint2D32f> cycle_polygon;
		for(size_t j=0; j<cycle_component.size(); j++)
		{
			topo_graph::edge edge_tmp = topo_extractor_pt_->road_graph_.edges[size_t(cycle_component[j].edgeID)];
			for(size_t p=0; p<edge_tmp.points.size(); p++)
			{
				cycle_polygon.push_back(cvPoint2D32f(edge_tmp.points[p].x, edge_tmp.points[p].y));

				//cvSet2D(cycle_image, edge_tmp.points[p].y,  edge_tmp.points[p].x, CV_RGB(0,255,0));
				cvCircle( cycle_image, edge_tmp.points[p], 1, CV_RGB(0,255,0), 1);
			}
		}

		bool erase_cycle = false;
		for(size_t a=0; a<shared_edgeIDs.size(); a++)
		{
			bool contain_shared_edge = false;
			for(size_t j=0; j<cycle_component.size(); j++)
			{
				if(cycle_component[j].edgeID == shared_edgeIDs[a])
				{
					contain_shared_edge = true;
					break;
				}
			}

			if(!contain_shared_edge)
			{
				if(pointInPolygon(centerPt_vector[a], cycle_polygon))
				{
					erase_cycle = true;
					cvCircle( cycle_image, cvPointFrom32f(centerPt_vector[a]), 3, CV_RGB(255,0,0), 2);
					extracted_cycles_.erase(extracted_cycles_.begin()+i);
					i=i-1;
					break;
				}
			}
		}

		if(!erase_cycle) cycle_polygons_.push_back(cycle_polygon);

		cvShowImage("cycle_filter", cycle_image);
		cvWaitKey(100);
	}

	ROS_INFO("cycle number after filtering %ld", extracted_cycles_.size());
	cvReleaseImage(&cycle_image);
}


void topo_semantic::intersection_analyze()
{

}

void topo_semantic::link_analyze()
{

}

void topo_semantic::visualization()
{
	CvSize image_size = cvSize(topo_extractor_pt_->grid_size_x, topo_extractor_pt_->grid_size_y);
	IplImage *cycle_image = cvCreateImage(image_size, 8,3);
	cvZero(cycle_image);

	for(size_t i=0; i<extracted_cycles_.size(); i++)
	{
		CvScalar ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
		for(size_t j=0; j<extracted_cycles_[i].size(); j++)
		{
			topo_graph::edge cycle_edge = topo_extractor_pt_->road_graph_.edges[(extracted_cycles_[i][j].edgeID)];
			for(size_t ip=0; ip<cycle_edge.points.size(); ip++)
			{
				cvSet2D(cycle_image, cycle_edge.points[ip].y,  cycle_edge.points[ip].x, ext_color);
			}
		}
		cvShowImage("cycle_window", cycle_image);
	}
	cvSaveImage( "/home/baoxing/cycle_image.jpg", cycle_image );
	cvWaitKey(100);
	cvReleaseImage(&cycle_image);
}

void topo_semantic::visualize_growing(vector < vector <path_segment> > & growing_paths)
{
	CvSize image_size = cvSize(topo_extractor_pt_->grid_size_x, topo_extractor_pt_->grid_size_y);
	IplImage *cycle_image = cvCreateImage(image_size, 8,3);
	cvZero(cycle_image);

	for(size_t i=0; i<growing_paths.size(); i++)
	{
		CvScalar ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
		CvPoint branch_head = cvPointFrom32f(topo_extractor_pt_->road_graph_.nodeClusters[growing_paths[i].back().head_nodeID].cluster_center);
		cvCircle( cycle_image, branch_head, 3, CV_RGB(0,255,0), 2);

		CvFont font;
		double hScale=0.7;
		double vScale=0.7;
		int lineWidth=1;
		stringstream  node_string;
		node_string<< growing_paths[i].back().head_nodeID;
		const char *node_name = node_string.str().c_str();
		cvInitFont(&font,CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
		cvPutText(cycle_image, node_name, branch_head, &font, CV_RGB(255 ,255, 255));

		for(size_t j=0; j<growing_paths[i].size(); j++)
		{
			topo_graph::edge cycle_edge = topo_extractor_pt_->road_graph_.edges[(growing_paths[i][j].edgeID)];
			for(size_t ip=0; ip<cycle_edge.points.size(); ip++)
			{
				cvSet2D(cycle_image, cycle_edge.points[ip].y,  cycle_edge.points[ip].x, ext_color);
			}
		}
		cvShowImage("grow_window", cycle_image);
	}
	cvWaitKey(100);
	cvReleaseImage(&cycle_image);
}


