import networkx
from networkx import shortest_path, has_path, shortest_path_length

from itertools import product

import pickle
import time


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
    a11_init_pose = c1_init_pose[0]
    a11_linear_speed = c1_linear_speed[0]
    a11_angular_speed = c1_angular_speed[0]
    a11_symbols = c1_symbols[:]
    a11_regions = c1_regions.copy()
    del a11_regions[c1_init_pose[2]]
    del a11_regions[c1_init_pose[1]]
    a11_motion = MotionFts(a11_regions, a11_symbols, 'a11_FTS')
    # ---construct motion fts ---
    for pair, route in c1_paths.iteritems():
        if pair[0] in a11_regions:
            if pair[1] in a11_regions:
                a11_motion.add_edge(pair[0], pair[1], weight = route[1]/a11_linear_speed, path = route[0])
    a11_motion.set_initial(a11_init_pose)
    

