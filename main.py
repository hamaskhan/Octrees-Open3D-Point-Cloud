# Author: Muhammad Hamas Khan
# Please add this reference if you want to use part of this code:  https://github.com/hamaskhan
# Email: m.hamaskhan@gmail.com
# ---------------------------------------------------------------------------------------------

# Libraries and dependencies
# ---------------------------------------
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import open3d as o3d
import time

# Useful Resources, References, and Links------------------------------------------------------
# PCL Libraries
# https://pointclouds.org/documentation/tutorials/
# https://pointclouds.org/
# Open3D libraries
# http://www.open3d.org/
# ---------------------------------------------------------------------------------------------
'''
# The format of data in the text file is as below
//  X  ;   Y   ;   Z  ;Value;    Nx  ;     Ny  ;    Nz
385.888;145.156;44.359;1.504;0.272890;-0.422294;0.864407
409.526;176.741;22.387;0.020;0.956552;-0.021037;0.290802
388.059;145.798;42.530;0.953;0.426310;-0.423093;0.799533
398.934;167.566;38.444;1.196;0.734180;-0.292712;0.612617
'''
# Suppresing scientific notation for easy readability
np.set_printoptions(suppress=True)

# 3D plotting of original, sampled and nearest points
# --------------------------------------------------------------------
def plot_3D():
    fig=plt.figure()
    ax = plt.axes(projection='3d')

    #print("X axis of sample test points are ",test_sample[:,0])
    #print("Y axis of sample test points are ",test_sample[:,1])
    #print("Z axis of sample test points are ",test_sample[:,2])

    ax.scatter3D(pointsxyz[:,0], pointsxyz[:,1], pointsxyz[:,2], label="Points", marker=".")
    ax.scatter3D(referencexyz[:,0], referencexyz[:,1], referencexyz[:,2], label="Reference Cloud Points", marker=".")

    # Select for which points you want to show neighbors
    # ---------------------------------------------------------------------
    ax.scatter3D(nearest[0][:,0], nearest[0][:,1], nearest[0][:,2], label="Nearest 2 Points for point at index[0,0,0]", marker="x")

    # Uncomment below line to see nearest neighbors for all the points
    #ax.scatter3D(nearest[:, :, 0], nearest[:, :, 1], nearest[:, :, 2], label="Nearest 2 Reference Points for all the points", marker="x")

    plt.title("3D scatter plot for Nearest Neighbors")
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.legend(loc="upper right")
    plt.show()


def Octree():

# Create Octree as in https://github.com/isl-org/Open3D/blob/master/examples/python/geometry/octree_point_cloud.py    
    octree_points = o3d.geometry.Octree(max_depth=4)     # Octree for Points Data
    octree_points.convert_from_point_cloud(pcd_points, size_expand=0.01)
#    print("The Points Octree results are ", octree_points)

    print('\nDisplaying Octree of the Points Cloud')
    o3d.visualization.draw_geometries([octree_points])
    #o3d.visualization.draw([octree_points])    #---- This requires OpenGL 4.1 version minimum

    octree_ref = o3d.geometry.Octree(max_depth=4)     # Octree for Reference Data
    octree_ref.convert_from_point_cloud(pcd_ref, size_expand=0.01)
#    print("The Reference Octree results are ", octree_ref)

    print('\nDisplaying Octree of the Reference Cloud')
    o3d.visualization.draw_geometries([octree_ref])
    #o3d.visualization.draw([octree_ref])    #---- This requires OpenGL 4.1 version minimum
    
    print("\nTraversing Points Octree. Node threshold set to 250000 points\n")
    octree_points.traverse(octree_traverse)


def octree_traverse(node, node_info):
    early_stop = False

    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0

            for child in node.children:
                if child is not None:
                    n += 1
            print("{} Processing Node {} at depth {} with {} points."
                .format('    ' * node_info.depth,
                        node_info.child_index, node_info.depth,
                        len(node.indices), node_info.origin))

            # we only want to process nodes / spatial regions with enough points
            early_stop = len(node.indices) < 250000

            if early_stop == True:

                #print("Processing Node indices ", node.indices)
                dist,points=kdtree.query(pointsxyz[node.indices],2)   # 2 means find nearest 2 neighbors
                #print("The nearest 2 neighbor point indices in this Octree are ", points)

                # Use below if Normals already present in file
                Oct_points=points2d[node.indices]

                #print("Displaying points: ", Oct_points_o)
                nearest = reference2d[points]
                print("The nearest 2 neighbor points in this Octree are ", nearest)
                points_norms=Oct_points[:, [4, 5,6]]
                nearest_norms= nearest[:,0, [3, 4,5]]
                # ------------------------------------------------------------------------------

                print("Normals of the points data are ", points_norms)
                print("Normals of the nearest data (taking only first neighbor) are ", nearest_norms)

    else:
        raise NotImplementedError('Node type not recognized!')

    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop



if __name__ == "__main__":

    # Data loading and pre-processing

    # Data is loaded in to an NxM matrix using the delimiter arguments
    # ------------------------------------------------------------
    print("\nLoading Points")
    points = np.genfromtxt("points.txt", skip_header=1, dtype=str, delimiter=';') #Open3D File IO can also be used
    reference = np.genfromtxt("reference.txt", skip_header=1, dtype=str, delimiter=';') #Open3D File IO can also be used
    print("No. of points are ", points.shape)
    print("Size of reference Point Cloud is ", reference.shape)

    # Convert string matrix to int
    points2d=points.astype(np.float32)
    reference2d=reference.astype(np.float32)

    # Select first three columns for all rows to get only x, y,z data if more columns are present
    pointsxyz=points2d[:, [0, 1,2]]
    referencexyz=reference2d[:, [0, 1,2]]

    print("\nData Loaded")

    # Initializing KD Tree and nearest neighbor return
    # ----------------------------------------------------------------------------
    print("\nInitializing KD Tree with reference point cloud")
    kdtree=KDTree(referencexyz)          # This is the reference point cloud

    dist,pts=kdtree.query(pointsxyz,2)   # Forward data here for the points to be queried for nearest neighbors. 2 represents the nearest 2 neighbor.

    print("The distance for nearest 2 neighbors are ", dist)
    print("The nearest 2 neighbor point indices are ", pts)
    nearest = referencexyz[pts]
    print("The nearest 2 neighbor points are ", nearest)


# Create Point Cloud as in https://github.com/isl-org/Open3D/blob/master/examples/python/geometry/point_cloud_with_numpy.py
    pcd_points = o3d.geometry.PointCloud()      # Open3d PointCloud object of originalxyz data
    pcd_points.points = o3d.utility.Vector3dVector(pointsxyz)
    pcd_points.paint_uniform_color([0, 0, 1])

    pcd_ref = o3d.geometry.PointCloud()      # Open3d PointCloud object of sampledxyz data
    pcd_ref.points = o3d.utility.Vector3dVector(referencexyz)
    pcd_ref.paint_uniform_color([1, 0.706, 0])

# ---------------------------------------------------------------------------------------------
# Below processing is with Octree Implementation-----------------------------------------------
    start_time = time.time()

    Octree()

    print("Execution time with Octrees: %s seconds" % (time.time() - start_time))
    #plot_3D()