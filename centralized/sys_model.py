from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel, distance

import networkx
from networkx import shortest_path, has_path, shortest_path_length

from itertools import product

import pickle
import time


#----------------------------
#----------------------------
def construct_sys_model(N):
    #----------------------------
    print 'Construct_agent_model starts'
    t0 = time.time()
    sys_models = []
    #----------
    [ws, tris, wps] = pickle.load(open('ws_model.p','rb'))
    # --- construct roadmap as fts ---    
    roadmap = networkx.Graph()
    # --- use only center point, to reduce the number of nodes
    wpc = dict()
    print 'wps number', len(wps)
    for k,wp in enumerate(wps):
        roadmap.add_node(wp[0])
        wpc[wp[0]] = k
    for f_n in roadmap.nodes():
        for t_n in roadmap.nodes():
            if f_n != t_n:
                f_k = wpc[f_n]
                t_k = wpc[t_n]
                f_s = set(wps[f_k])
                t_s = set(wps[t_k])
                if f_s.intersection(t_s):
                    roadmap.add_edge(f_n, t_n, weight = distance(f_n, t_n))
    # if complete wps are used
    # for wp in wps:
    #     roadmap.add_edge(wp[0], wp[1], weight = distance(wp[0], wp[1]))
    #     roadmap.add_edge(wp[0], wp[2], weight = distance(wp[0], wp[2]))
    #     roadmap.add_edge(wp[0], wp[3], weight = distance(wp[0], wp[3]))
        # roadmap.add_edge(wp[1], wp[2], weight = distance(wp[1], wp[2]))
        # roadmap.add_edge(wp[1], wp[3], weight = distance(wp[1], wp[3]))
        # roadmap.add_edge(wp[2], wp[3], weight = distance(wp[2], wp[3]))
    #----------
    # N c1 agents, N c2 agents, N c3 agents, N leaders
    #----------
    A_start_pose = [(5.666666666666667, 5.0),
                  (4.666666666666667, 4.333333333333333),
                  (6.5, 6.666666666666667),
                  (5.666666666666667, 5.0),
                  (4.666666666666667, 4.333333333333333),
                  (6.5, 6.666666666666667)]
    A_linear_speed = [ 0.45, 0.65, 1.35, 0.45, 0.65, 1.35] #m/s
    A_angular_speed = [ 0.75, 0.9, 1.05, 0.75, 0.9, 1.05] #rad/s
    COST = 1
    #----------
    add_data = {}
    act_time = {}
    symbols = []
    #----------
    #----------
    #----------
    # group ---**c1**---
    for n in range(0, N):
        r_name  = 'a1%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_symbols = ['r%d0' %n, 'r%d1' %n, 'r%d2' %n, 'r%d3' %n]
        symbols.append(r_symbols)
        r_regs = {r_init_pose: set([r_symbols[0],]),
                  (6.666666666666667, 9.5): set([r_symbols[1],]),
                  (1.0, 8.833333333333334): set([r_symbols[2],]),
                  (9.333333333333334, 9.0): set([r_symbols[3],]),
        }
        r_regions = {}
        for nd in roadmap.nodes():
            if nd in r_regs:
                r_regions[nd] = r_regs[nd]
            else:
                r_regions[nd] = set()                
        r_motion = MotionFts(r_regions, r_symbols, '%s_FTS' %r_name)
        r_motion.add_un_edges(roadmap.edges(), unit_cost = 1/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        
        # action fts
        add_data['g%d1'%n] = 1
        add_data['g%d2' %n] = 2
        add_data['g%d3' %n] = 2

        act_time['g%d1'%n] = COST
        act_time['g%d2'%n] = 2*COST
        act_time['g%d3'%n] = COST
        symbols.append(['g%d1'%n, 'g%d2'%n, 'g%d3'%n])

        r_action_dict = {
            'g%d1'%n: (act_time['g%d1'%n], '1', set(['g%d1'%n,])),
            'g%d2'%n: (act_time['g%d2'%n], '1', set(['g%d2'%n,])),
            'g%d3'%n: (act_time['g%d3'%n], '1', set(['g%d3'%n,])),        
        }
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = '([] <> (r%d1 && g%d1)) && ([] <> (r%d2 && g%d2)) && ([] <> (r%d3 && g%d3))' %tuple((n,)*6)
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #----------
    #----------
    #----------
    # group ---**c2**---
    for n in range(0, N):
        r_name  = 'a2%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_symbols = ['r%d0' %n, 'r%d4' %n, 'r%d5' %n, 'r%d6' %n]
        symbols.append(r_symbols)
        r_regs = {r_init_pose: set([r_symbols[0],]),
                  (9.666666666666666, 6.0): set([r_symbols[1],]),
                (8.666666666666666, 1.1666666666666667): set([r_symbols[2],]),
                  (4.666666666666667, 0.5): set([r_symbols[3],]),
        }
        r_regions = {}
        for nd in roadmap.nodes():
            if nd in r_regs:
                r_regions[nd] = r_regs[nd]
            else:
                r_regions[nd] = set()                
        r_motion = MotionFts(r_regions, r_symbols, '%s_FTS' %r_name)
        r_motion.add_un_edges(roadmap.edges(), unit_cost = 1/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        # action fts
        add_data['g%d4'%n] = 2
        add_data['g%d5' %n] = 1

        act_time['g%d4'%n] = COST
        act_time['g%d5'%n] = COST
        symbols.append(['g%d4'%n, 'g%d5'%n])

        r_action_dict = {
            'g%d4'%n: (act_time['g%d4'%n], '1', set(['g%d4'%n,])),
            'g%d5'%n: (act_time['g%d5'%n], '1', set(['g%d5'%n,])),
        }
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = '([] <> (r%d4 && g%d4)) && ([] <> (r%d6 && g%d4)) && ([] <> (r%d5 && g%d5))' %tuple((n,)*6)
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #----------
    #----------
    #----------
    # group ---**c3**---
    for n in range(0, N):
        r_name  = 'a3%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_symbols = ['r%d0' %n, 'r%d7' %n, 'r%d8' %n, 'r%d9' %n]
        symbols.append(r_symbols)
        r_regs = {r_init_pose: set([r_symbols[0],]),
                  (0.3333333333333333, 5.666666666666667): set([r_symbols[1],]),
                (1.0, 3.3333333333333335): set([r_symbols[2],]),
                (4.333333333333333, 2.3333333333333335): set([r_symbols[3],]),
        }
        r_regions = {}
        for nd in roadmap.nodes():
            if nd in r_regs:
                r_regions[nd] = r_regs[nd]
            else:
                r_regions[nd] = set()                
        r_motion = MotionFts(r_regions, r_symbols, '%s_FTS' %r_name)
        r_motion.add_un_edges(roadmap.edges(), unit_cost = 1/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        # action fts
        add_data['g%d6'%n] = 2
        add_data['g%d7' %n] = 1

        act_time['g%d6'%n] = COST
        act_time['g%d7'%n] = 2*COST
        symbols.append(['g%d6'%n, 'g%d7'%n])

        r_action_dict = {
            'g%d6'%n: (act_time['g%d6'%n], '1', set(['g%d6'%n,])),
            'g%d7'%n: (act_time['g%d7'%n], '1', set(['g%d7'%n,])),
        }
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = '([] <> (r%d7 && g%d6)) && ([] <> (r%d8 && g%d7)) && ([] <> (r%d9 && g%d6))' %tuple((n,)*6)
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #----------
    #----------
    #----------
    # group ---**leader**---
    for n in range(0, N):
        r_name  = 'l%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_regions = {}
        for nd in roadmap.nodes():
            r_regions[nd] = set()                
        r_motion = MotionFts(r_regions, [], '%s_FTS' %r_name)
        r_motion.add_un_edges(roadmap.edges(), unit_cost = 1/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        # action fts
        act_time['ul'] = COST
        symbols.append(['ul',])
        r_action_dict = {'ul': (act_time['ul'], '1', set(['ul',]))}
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = 'true'
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #--------------------
    print 'Agent model construction done in %.2f' %(time.time()-t0)
    print '%d source robots and %d relay robots' %(N, 3*N)
    return sys_models, add_data, symbols

#----------------------------
#----------------------------
def construct_sys_model_small(N):
    #----------------------------
    print 'Construct_agent_model_small starts'
    t0 = time.time()
    sys_models = []
    #---------- load simplified roadmap
    [c1_paths, c2_paths, c3_paths, l_paths] = pickle.load(open('simp_ts.p','rb'))
    #---------- 
    # N c1 agents, N c2 agents, N c3 agents, N leaders
    #----------
    A_start_pose = [(5.666666666666667, 5.0),
                  (4.666666666666667, 4.333333333333333),
                  (6.5, 6.666666666666667),
                  (5.666666666666667, 5.0),
                  (4.666666666666667, 4.333333333333333),
                  (6.5, 6.666666666666667)]
    A_linear_speed = [ 0.45, 0.65, 1.35, 0.45, 0.65, 1.35] #m/s
    A_angular_speed = [ 0.75, 0.9, 1.05, 0.75, 0.9, 1.05] #rad/s
    COST = 1
    #----------
    add_data = {}
    act_time = {}
    symbols = []
    l_regions = set()
    #----------
    #----------
    #----------
    # group ---**c1**---
    for n in range(0, N):
        r_name  = 'a1%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_symbols = ['r%d0' %n, 'r%d1' %n, 'r%d2' %n, 'r%d3' %n]
        symbols.append(r_symbols)
        r_regs = {r_init_pose: set([r_symbols[0],]),
                  (6.666666666666667, 9.5): set([r_symbols[1],]),
                  (1.0, 8.833333333333334): set([r_symbols[2],]),
                  (9.333333333333334, 9.0): set([r_symbols[3],]),
        }
        r_motion = MotionFts(r_regs, r_symbols, '%s_FTS' %r_name)
        l_regions.update(set(r_regs.keys()))
        for pair, route in c1_paths.iteritems():
            if pair[0] in r_regs.keys():
                if pair[1] in r_regs.keys():
                    r_motion.add_edge(pair[0], pair[1], weight = route[1]/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        
        # action fts
        add_data['g%d1'%n] = 2

        act_time['g%d1'%n] = 2*COST
        symbols.append(['g%d1'%n,])

        r_action_dict = {
            'g%d1'%n: (act_time['g%d1'%n], '1', set(['g%d1'%n,])),        
        }
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = '([] <> (r%d1 && g%d1)) && ([] <> (r%d2 && g%d1)) && ([] <> (r%d3 && g%d1))' %tuple((n,)*6)
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #----------
    #----------
    #----------
    # group ---**c2**---
    for n in range(0, N):
        r_name  = 'a2%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_symbols = ['r%d0' %n, 'r%d4' %n, 'r%d5' %n, 'r%d6' %n]
        symbols.append(r_symbols)
        r_regs = {r_init_pose: set([r_symbols[0],]),
                  (9.666666666666666, 6.0): set([r_symbols[1],]),
                (8.666666666666666, 1.1666666666666667): set([r_symbols[2],]),
                  (4.666666666666667, 0.5): set([r_symbols[3],]),
        }
        r_motion = MotionFts(r_regs, r_symbols, '%s_FTS' %r_name)
        l_regions.update(set(r_regs.keys()))
        for pair, route in c2_paths.iteritems():
            if pair[0] in r_regs.keys():
                if pair[1] in r_regs.keys():
                    r_motion.add_edge(pair[0], pair[1], weight = route[1]/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        # action fts
        add_data['g%d4'%n] = 2
        act_time['g%d4'%n] = COST
        symbols.append(['g%d4'%n,])

        r_action_dict = {
            'g%d4'%n: (act_time['g%d4'%n], '1', set(['g%d4'%n,])),
        }
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = '([] <> (r%d4 && g%d4)) && ([] <> (r%d6 && g%d4)) && ([] <> (r%d5 && g%d4))' %tuple((n,)*6)
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #----------
    #----------
    #----------
    # group ---**c3**---
    for n in range(0, N):
        r_name  = 'a3%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_symbols = ['r%d0' %n, 'r%d7' %n, 'r%d8' %n, 'r%d9' %n]
        symbols.append(r_symbols)
        r_regs = {r_init_pose: set([r_symbols[0],]),
                  (0.3333333333333333, 5.666666666666667): set([r_symbols[1],]),
                (1.0, 3.3333333333333335): set([r_symbols[2],]),
                (4.333333333333333, 2.3333333333333335): set([r_symbols[3],]),
        }
        r_motion = MotionFts(r_regs, r_symbols, '%s_FTS' %r_name)
        l_regions.update(set(r_regs.keys()))
        for pair, route in c3_paths.iteritems():
            if pair[0] in r_regs.keys():
                if pair[1] in r_regs.keys():
                    r_motion.add_edge(pair[0], pair[1], weight = route[1]/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        # action fts
        add_data['g%d6'%n] = 2
        
        act_time['g%d6'%n] = COST
        symbols.append(['g%d6'%n, ])

        r_action_dict = {
            'g%d6'%n: (act_time['g%d6'%n], '1', set(['g%d6'%n,])),
        }
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = '([] <> (r%d7 && g%d6)) && ([] <> (r%d8 && g%d6)) && ([] <> (r%d9 && g%d6))' %tuple((n,)*6)
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #----------
    #----------
    #----------
    # group ---**leader**---
    for n in range(0, N):
        r_name  = 'l%d' %n
        r_init_pose = A_start_pose[n]
        r_linear_speed = A_linear_speed[n]
        r_angular_speed = A_angular_speed[n]
        # motion fts
        r_regs = dict()
        for nd in l_regions:
            r_regs[nd] = set()                
        r_motion = MotionFts(r_regs, [], '%s_FTS' %r_name)
        for pair, route in l_paths.iteritems():
            if pair[0] in r_regs.keys():
                if pair[1] in r_regs.keys():
                    r_motion.add_edge(pair[0], pair[1], weight = route[1]/r_linear_speed)
        r_motion.set_initial(r_init_pose)
        # action fts
        act_time['ul'] = COST
        symbols.append(['ul',])
        r_action_dict = {'ul': (act_time['ul'], '1', set(['ul',]))}
        r_action = ActionModel(r_action_dict)
        r_model = MotActModel(r_motion, r_action)
        r_model.build_full()
        r_hard_task = 'true'
        r_soft_task = None
        sys_models.append([r_model, r_hard_task, r_soft_task])
    #--------------------
    print 'Agent model construction done in %.2f' %(time.time()-t0)
    print '%d source robots and %d relay robots' %(N, 3*N)
    return sys_models, add_data, symbols


def compose_sys_fts(sys_models, add_data, symbols, buffer_size, com_rad = 1):
    #----------------------------
    print 'compose_fts starts'
    t0 = time.time()
    #--------------------
    nodes = set()
    # construct nodes
    N = len(sys_models)
    N_f = int(round(0.75*N))
    ind_nodes = []
    for k in range(0, N):
        regs = sys_models[k][0].nodes()
        bufs = range(buffer_size[k]+1)
        new_s = []
        for item in product(regs, bufs):
            # (reg,d)
            new_s.append(item)
        ind_nodes.append(new_s)
    #--------------------
    comp_nodes = dict()
    for items in product(*ind_nodes):
        prop = set()
        for k in range(0, N):
            fts = sys_models[k][0]
            reg = items[k][0]
            label = fts.node[reg]['label']
            prop.update(label)
        comp_nodes[tuple(items)] = prop
    comp_symbols = symbols
    #------------------------------
    comp_motion = MotionFts(comp_nodes, comp_symbols, 'comp_FTS')
    # build transitions
    for f_n in comp_nodes:
        # [(reg1,d1),(reg2,d2)...]
        for t_n in comp_nodes:
            if (f_n != t_n):
                allowed = True
                a_weight = 0
                for i in range(0, N):
                    f_n_i = f_n[i][0]
                    t_n_i = t_n[i][0]
                    fts_i = sys_modes[i][0]
                    if (t_n_i in fts_i.successors(f_n_i)):
                        label = fts_i.edge[f_n_i][t_n_i]['label']
                        f_d_i = f_n[i][1]
                        t_d_i = t_n[i][1]
                        if (label != 'goto') and (label != 'ul'):
                            extra_data = add_data[label]
                            if (t_d_i == f_d_i + extra_data):
                                e_weight = fts_i.edge[f_n_i][t_n_i]['weight']
                                if e_weight > a_weight:
                                    a_weight = e_weight
                            else:
                                allowed = False
                                break
                                # data gather actions
                        elif (label == 'goto'):
                            one_ul = False
                            for j in range(N_f, N):
                                if f_n[j][0] == t_n[j][0]:
                                    one_ul = True
                                    break
                            if ((t_d_i == f_d_i)
                                or ((t_d_i != f_d_i) and (t_d_i == 0) and (one_ul) and (i in range(0, N_f)))):
                                e_weight = fts_i.edge[f_n_i][t_n_i]['weight']
                                if e_weight > a_weight:
                                    a_weight = e_weight
                            else:
                                allowed = False
                                break
                        elif (label == 'ul'):
                            t_reg_i = t_n_i[0]
                            all_empty = True
                            for j in range(0, N_f):
                                t_reg_j = t_n[j][0][0]
                                t_d_j = t_n[j][1]
                                f_d_j = f_n[j][1]
                                if distance(t_reg_i, t_reg_j) <= com_rad:
                                    if t_d_j != 0:
                                        all_empty = False
                                elif distance(t_reg_i, t_reg_j) > com_rad:
                                    if t_d_j != f_d_j:
                                        all_empty = False
                            if ((all_empty) and (t_d_i == 0)):
                                e_weight = fts_i.edge[f_n_i][t_n_i]['weight']
                                if e_weight > a_weight:
                                    a_weight = e_weight
                            else:
                                 allowed = False
                                 break
                    else:
                        allowed = False
                        break
                if allowed:
                    comp_motion.add_edge(f_n,t_n,weight=a_weight)
    #------------------------------
    comp_action_dict = dict()
    comp_action = ActionModel(comp_action_dict)
    comp_fts = MotActModel(comp_motion, comp_action)
    print '----------'
    print 'comp_fts generated in %.2f' %(time.time()-t0)
    print 'Composed fts: nodes %d, edges %d' %(len(comp_fts.nodes()),len(comp_fts.edges()))
    print '----------'
    return comp_fts
        
        

def compose_sys_ltl(sys_models):
    task_ltl = sys_models[0][1]
    for k in range(1, len(sys_models)):
        task_ltl = '(%s) && (%s)' %(task_ltl, sys_models[k][1])
    print 'composed ltl formula:', task_ltl
    return task_ltl

    
