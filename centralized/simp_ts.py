import networkx
from networkx import shortest_path, has_path, shortest_path_length

from itertools import product

from math import sqrt

import pickle
import time


def distance(pose1, pose2):
    return (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001)


def shortest_path_exclude(G, s, g, exclude):
    new_G = G.copy()
    for n in exclude:
        new_G.remove_node(n)
    if has_path(new_G, s, g):
        return shortest_path(new_G, s, g), shortest_path_length(new_G, s, g, 'weight')
    else:
        return False, 0


if __name__ == "__main__":
    print 'Simplify fts starts'
    t0 = time.time()
    #----------
    [ws, tris, wps] = pickle.load(open('ws_model.p','rb'))
    # --- construct roadmap as fts ---    
    roadmap = networkx.Graph()
    roadmap = networkx.Graph()    
    for wp in wps:
        roadmap.add_edge(wp[0], wp[1], weight = distance(wp[0], wp[1]))
        roadmap.add_edge(wp[0], wp[2], weight = distance(wp[0], wp[2]))
        roadmap.add_edge(wp[0], wp[3], weight = distance(wp[0], wp[3]))
        roadmap.add_edge(wp[1], wp[2], weight = distance(wp[1], wp[2]))
        roadmap.add_edge(wp[1], wp[3], weight = distance(wp[1], wp[3]))
        roadmap.add_edge(wp[2], wp[3], weight = distance(wp[2], wp[3]))

    # ----- c1 motion ----
    c1_init_pose = [(5.666666666666667, 5.0), (4.666666666666667, 4.333333333333333),(6.5, 6.666666666666667)]
    c1_regions = {(6.666666666666667, 9.5): set(['r1',]),
                  (1.0, 8.833333333333334): set(['r2',]),
                (9.333333333333334, 9.0): set(['r3',]),
                  c1_init_pose[0]: set(['r0',]),
                  c1_init_pose[1]: set(['r0',]),
                  c1_init_pose[2]: set(['r0',]),              
                  }
    c1_rs = set(c1_regions.keys())
    for init in c1_init_pose:
        c1_rs.remove(init)
    c1_paths = dict()
    for p1 in c1_regions.iterkeys():
        exclude = c1_rs
        exclude.discard(p1)
        for p2 in c1_regions.iterkeys():
            if p1 != p2:
                exclude.discard(p2)
                path, length = shortest_path_exclude(roadmap, p1, p2, exclude)
                if path:
                    c1_paths[(p1, p2)] = [path, length]

    # ----- c2 motion ----
    c2_init_pose = [(5.666666666666667, 5.0), (6.5, 6.666666666666667),  (4.666666666666667, 4.333333333333333)]
    c2_symbols = ['r4', 'r5', 'r6']
    c2_regions = {(9.666666666666666, 6.0): set(['r4',]),
                  (8.666666666666666, 1.1666666666666667): set(['r5',]),
                  (4.666666666666667, 0.5): set(['r6',]),
                  c2_init_pose[0]: set(['r0',]),
                  c2_init_pose[1]: set(['r0',]),
                  c2_init_pose[2]: set(['r0',]),              
                  }
    c2_rs = set(c2_regions.keys())
    for init in c2_init_pose:
        c2_rs.remove(init)
    c2_paths = dict()
    for p1 in c2_regions.iterkeys():
        exclude = c2_rs
        exclude.discard(p1)
        for p2 in c2_regions.iterkeys():
            if p1 != p2:
                exclude.discard(p2)
                path, length = shortest_path_exclude(roadmap, p1, p2, exclude)
                if path:
                    c2_paths[(p1, p2)] = [path, length]
    # ----- c3 motion ----
    c3_init_pose = [(6.5, 6.666666666666667), (5.666666666666667, 5.0), (4.666666666666667, 4.333333333333333)]
    c3_symbols = ['r7', 'r8', 'r9']
    c3_regions = {(0.3333333333333333, 5.666666666666667): set(['r7',]),
                  (1.0, 3.3333333333333335): set(['r8',]),
                  (4.333333333333333, 2.3333333333333335): set(['r9',]),              
                  c3_init_pose[0]: set(['r0',]),
                  c3_init_pose[1]: set(['r0',]),
                  c3_init_pose[2]: set(['r0',]),              
                  }
    c3_rs = set(c3_regions.keys())
    for init in c3_init_pose:
        c3_rs.remove(init)
    c3_paths = dict()
    for p1 in c3_regions.iterkeys():
        exclude = c3_rs
        exclude.discard(p1)
        for p2 in c3_regions.iterkeys():
            if p1 != p2:
                exclude.discard(p2)
                path, length = shortest_path_exclude(roadmap, p1, p2, exclude)
                if path:
                    c3_paths[(p1, p2)] = [path, length]
    #-------------------- l motion
    l_paths = dict()
    for p,l in c1_paths.iteritems():
        l_paths[p] = l
    for p,l in c2_paths.iteritems():
        l_paths[p] = l
    for p,l in c3_paths.iteritems():
        l_paths[p] = l    
    #
    print 'Simplified roadmap construction done in %.2f' %(time.time()-t0)
    pickle.dump([c1_paths, c2_paths, c3_paths, l_paths], open('simp_ts.p','wb'))
    print 'Simplified roadmap saved to simp_ts.p'

