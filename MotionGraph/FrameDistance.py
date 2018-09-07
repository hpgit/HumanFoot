from scipy.spatial import kdtree
import numpy as np
from annoy import AnnoyIndex


def AllPointKNearest(size, dimension, num_neighbors, input_file, output_file):
    """

    :param size:
    :type size: int
    :param dimension:
    :type dimension: int
    :param num_neighbors:
    :type num_neighbors: int
    :param input_file:
    :type input_file: str
    :param output_file:
    :type output_file: str
    :return:
    :rtype: np.ndarray
    """

    result = np.zeros((size, size))

    #TODO:
    # use flann

    # numOfTotalFrame
    data_size = size
    query_size = size

    # numOfJoint
    dim = dimension

    data = readPts(data_size, dim, input_file)
    query = readPts(query_size, dim, input_file)

    bucket_size = 5
    epsilon = 0.

    # numOfnearFrameFind
    near_neigh = num_neighbors
    true_nn = num_neighbors

    t = AnnoyIndex(dim, metric='euclidean')
    for i in range(data_size):
        t.add_item(i, data[i])

    t.build(20)
    with open(output_file, 'w') as out_file:
        for i in range(query_size):
            res, dist = t.get_nns_by_vector(query[i], near_neigh, include_distances=True)

            for j in range(near_neigh):
                out_file.write('{0} {1} {2:0.6f}\n'.format(i, res[j], dist[j]))
                result[i, res[j]] = dist[j]

    return result


def readPts(n, dim, filename):
    print('\n'+filename)
    data = []
    with open(filename, 'r') as in_file:
        s = in_file.read()
        _s = s.split()
        for i in range(n):
            _data = []
            for d in range(dim):
                _data.append(float(_s.pop(0)))
            data.append(_data)
    return data
