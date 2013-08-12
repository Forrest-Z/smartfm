Overall Structure
-----------------------------------------------------------------------------------------------------------------------------------------------------------
There are 3 classes inside this package: 
1. "TopoExtractor" to perform skeletonization on the binary image, and extract the topo-metric graph of the road network;
2. "TopoSemantics" is a preliminary class to extract topology-induced semantics, such as "round-about", "3-connection intersection", etc.
3. "road_semantics" process grayscale image to get a binary image of road network, and nest the above 2 classes;
Besides, "road_semantics_node" provides an example of using the library of "road_semantics", where all the needed parameters are stored in one yaml file;
-----------------------------------------------------------------------------------------------------------------------------------------------------------
