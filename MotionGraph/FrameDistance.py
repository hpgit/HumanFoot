def AllPointKNearest(size, dimension, num_neighbors, input_file, output_file, result):
    #TODO:
    # use scipy kdtree

    initGlobals()

    # numOfJoint
    dim = dimension

    # numOfTotalFrame
    data_size = size
    query_size = size

    # TODO:
    # result should be call by reference
    # result is ndarray
    result.setSize(size, size)
    result *= 0.

    bucket_size = 5
    epsilon = 0.

    # numOfnearFrameFind
    near_neigh = num_neighbors
    true_nn = num_neighbors

